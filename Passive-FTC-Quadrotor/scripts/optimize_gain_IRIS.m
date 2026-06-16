%% optimize_gain_IRIS.m — 화물별 최적 게인 vs 무게추정 피드포워드
% 질문(연구원 제안): 각 화물에 대해 게인(AgilityAtti/Pos)을 최적화하면,
%   그냥 무게추정→질량·관성 피드포워드(C1)보다 나은가, 못한가?
% 비교:
%   ① 최적게인만 : 마른질량 제어기 + 게인 격자탐색 (순수 게인 스케줄링 = 원래 아이디어)
%   ② C1         : 추정 질량·관성 피드포워드, 게인 0.5/0.5 고정
%   ③ C1+최적게인 : 질량보정 + 게인 격자탐색 (둘 다)
% 비용 J = min(회복,30) + 말기틸트 + 30·최종오차   (낮을수록 좋음; 발산/추락은 J↑로 자동배제)
%
% 사용법:  optimize_gain_IRIS            % 기본 340g(가장 어려운 점)
%          PAYLOAD=0.3; optimize_gain_IRIS

if ~exist('PAYLOAD','var') || isempty(PAYLOAD), PAYLOAD = 0.34; end
clc; addPathFtc();
poff = 0.10;

%% 공통 설정 (base 워크스페이스 = sim이 읽음)
failure_time_mot_1=1000; failure_time_mot_2=1; failure_time_mot_3=1000; failure_time_mot_4=1000;
simin = automatedStickCommands();
copter = copterLoadParams('copter_params_IRIS');
envir = envirLoadParams('params_envir','envir',0);
IC.omega_Kb=[0;0;0]; IC.q_bg=euler2Quat([0;0;0]); IC.V_Kb=[0;0;0]; IC.s_Kg=[0;0;0];
IC.omega_mot=[1;1;1;1]*500;
grnd=groundLoadParams('params_ground_default');
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=50;
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;

copter_dry  = copter;  m_dry = copter_dry.body.m;
copter_pl   = mkplant(copter_dry, PAYLOAD, poff, true);     % plant=진짜 화물(고정)
copter_ctrl = mkplant(copter_dry, PAYLOAD, poff, false);    % 제어기용(질량+관성)

m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');

aAttiList = [0.3 0.5 0.7];
aPosList  = [0.3 0.5 0.7 0.9];

fprintf('\n========== 화물 %.0f g : 최적 게인 탐색 ==========\n', PAYLOAD*1000);

%% ① 순수 게인 스케줄링 (마른질량 제어기)
fprintf('\n[①] 최적게인만 (마른 제어기 + 게인탐색) — 비용 J 격자\n');
best1 = gridsearch(copter_dry, copter_pl, aAttiList, aPosList, m);

%% ② C1 (질량·관성 피드포워드, 게인 0.5/0.5)
[r,et,ze] = runcase(copter_ctrl, copter_pl, 0.5, 0.5, m);
c1 = struct('aA',0.5,'aP',0.5,'rec',r,'et',et,'ze',ze,'J',costJ(r,et,ze));

%% ③ C1 + 최적게인
fprintf('\n[③] C1+게인탐색 (질량보정 제어기 + 게인탐색) — 비용 J 격자\n');
best3 = gridsearch(copter_ctrl, copter_pl, aAttiList, aPosList, m);

%% 요약
fprintf('\n================= 요약 (화물 %.0f g) =================\n', PAYLOAD*1000);
fprintf(' 방식             | 게인 A/P | 회복[s] | 말기틸트[°] | 최종오차[m] | 비용 J\n');
fprintf('----------------------------------------------------------------------------\n');
prow('① 최적게인만',     best1);
prow('② C1(피드포워드)',  c1);
prow('③ C1+최적게인',     best3);
fprintf('----------------------------------------------------------------------------\n');
fprintf('>> ①J<②J = 게인스케줄링 우세. ②J<①J = 피드포워드 우세. ③ 최소 = 둘 다.\n\n');

%% ===== 보조 =====
function J = costJ(rec,et,ze),  J = min(rec,30) + et + 30*ze;  end

function best = gridsearch(cctrl, cpl, aA, aP, m)
    fprintf('  aAtti\\aPos '); fprintf('| %4.1f  ', aP); fprintf('\n');
    best.J = inf;
    for i = 1:numel(aA)
        fprintf('   %4.1f      ', aA(i));
        for j = 1:numel(aP)
            [r,et,ze] = runcase(cctrl, cpl, aA(i), aP(j), m);
            jj = costJ(r,et,ze);
            fprintf('| %5.1f ', jj);
            if jj < best.J
                best = struct('J',jj,'aA',aA(i),'aP',aP(j),'rec',r,'et',et,'ze',ze);
            end
        end
        fprintf('\n');
    end
    fprintf('   → 최적: A=%.1f P=%.1f (J=%.1f)\n', best.aA, best.aP, best.J);
end

function [rec,endt,zerr] = runcase(cctrl, cpl, aA, aP, m)
    fm = mkctrl(cctrl, aA, aP);
    assignin('base','fm_loiter',fm);
    assignin('base','copter',cpl);
    out = sim(m,'StopTime','25');           % base 워크스페이스에서 파라미터 읽음
    [~,rec,~,zerr,endt] = perfmetrics(out);
end

function fm = mkctrl(cdes, aA, aP)
    copter = cdes; %#ok<NASGU>
    fm = lindiCopterAutoCreate(copter,'AgilityAtti',aA,'AgilityPos',aP,'FilterStrength',0,'CntrlEffectScaling',1);
    fm.psc.inditype = 1;
end
function c = mkplant(cdry, pm, poff, withCoG)
    M = cdry.body.m + pm;  mu = cdry.body.m*pm/M;
    c = cdry;  c.body.m = M;
    c.body.I = cdry.body.I + mu*poff^2*diag([1 1 0]);
    if withCoG, c.config.CoG_Pos_c = c.config.CoG_Pos_c + [0;0;(pm/M)*poff]; end
end
function [te,tilt] = tiltsig(out)
    te=out.Euler_angles.Time(:); E=sq3(out.Euler_angles.Data,3);
    tilt = acosd(max(-1,min(1,cos(E(:,1)).*cos(E(:,2)))));
end
function [drop,trec,mxt,zerr,endt] = perfmetrics(out)
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    post=ts>=1; upp=up(post); tp=ts(post); [mn,idx]=min(upp); drop=-mn;
    after=idx:numel(upp); rec=find(abs(upp(after))<0.5,1);
    if isempty(rec), trec=inf; else, trec=tp(after(rec))-1; end
    zerr=abs(up(end)); [te,tl]=tiltsig(out); mxt=max(tl(te>=1)); endt=max(tl(te>=te(end)-3));
end
function prow(nm, b)
    fprintf('  %-16s | %.1f/%.1f | %6.2f  |   %6.1f    |   %6.2f    | %6.1f\n', ...
            nm, b.aA, b.aP, b.rec, b.et, b.ze, b.J);
end
function X=sq3(D,nch), X=squeeze(D); if size(X,2)~=nch && size(X,1)==nch, X=X.'; end, end
