%% compare_perf_IRIS.m — 성능개선 적응전략 비교 (스톡 IRIS, 화물 ≤25%)
% 목표: 생존범위(≤340g) 안에서 고장후 성능(회복시간·말기안정·강하)을
%        payload-aware 적응으로 개선. 게인-업은 기체불문 악화(검증됨)라 제외.
% 비교 전략 (제어기만 교체, plant=진짜 화물):
%   B    = baseline           : 마른 IRIS, Agility 0.5/0.5 (화물 모름)
%   C1   = 질량·관성 피드포워드: 추정질량 반영, 0.5/0.5
%   C1_dA= C1 + 자세게인 다운  : atti 0.35 (스핀 자유롭게)
%   C1_uP= C1 + 위치게인 업    : pos 0.8  (고도회복 빠르게, 자세는 유지)
% 지표: 최대강하 / 회복 / 최대틸트 / 말기틸트(말기3s) / 최종오차  (낮을수록 좋음)
%   ※ 질량은 추정정확도 별도검증됨 → 여기선 제어효과만 분리(진짜질량 사용)
%
% 사용법:  compare_perf_IRIS

clc; addPathFtc();
poffset = 0.10;

%% 공통 설정
failure_time_mot_1=1000; failure_time_mot_2=1; failure_time_mot_3=1000; failure_time_mot_4=1000;
simin = automatedStickCommands();
copter = copterLoadParams('copter_params_IRIS');
envir = envirLoadParams('params_envir','envir',0);
IC.omega_Kb=[0;0;0]; IC.q_bg=euler2Quat([0;0;0]); IC.V_Kb=[0;0;0]; IC.s_Kg=[0;0;0];
IC.omega_mot=[1;1;1;1]*500;
grnd=groundLoadParams('params_ground_default');
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=50;
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;

copter_dry = copter;  m_dry = copter_dry.body.m;

m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');

%% 전략 정의: {이름, 제어기설계copter함수, atti, pos}
payloads = [0.2 0.3 0.34];
strat = { 'B (baseline)',  @(pm) copter_dry,                       0.5, 0.5 ; ...
          'C1 (질량보정)',  @(pm) mkplant(copter_dry,pm,0.10,false),0.5, 0.5 ; ...
          'C1+자세다운',    @(pm) mkplant(copter_dry,pm,0.10,false),0.35,0.5 ; ...
          'C1+위치업',      @(pm) mkplant(copter_dry,pm,0.10,false),0.5, 0.8 };

for p = payloads
    fprintf('\n========== 화물 %.0f g (%.1f%%) ==========\n', p*1000, p/m_dry*100);
    fprintf(' 전략           | 강하[m] | 회복[s] | 최대틸트[°] | 말기틸트[°] | 최종오차[m]\n');
    fprintf('---------------------------------------------------------------------------\n');
    for s = 1:size(strat,1)
        cdes = strat{s,2}(p);
        fm_loiter = mkctrl(cdes, strat{s,3}, strat{s,4});
        copter = mkplant(copter_dry, p, poffset, true);     % plant=진짜 화물
        out = sim(m,'StopTime','25');
        [drop,trec,mxtilt,zerr,endtilt] = perfmetrics(out);
        fprintf('  %-13s | %6.2f  | %6.2f  |   %6.1f    |   %6.1f    |  %6.2f\n', ...
                strat{s,1}, drop, trec, mxtilt, endtilt, zerr);
    end
    fprintf('---------------------------------------------------------------------------\n');
end
fprintf('\n>> B 대비 회복↓·말기틸트↓·최종오차↓ 되는 전략 = 성능개선 적응. 그게 채택안.\n\n');

%% ===== 보조 =====
function fm = mkctrl(cdes, aAtti, aPos)
    copter = cdes;
    fm = lindiCopterAutoCreate(copter,'AgilityAtti',aAtti,'AgilityPos',aPos, ...
                               'FilterStrength',0,'CntrlEffectScaling',1);
    fm.psc.inditype = 1;
end
function c = mkplant(cdry, pm, poff, withCoG)
    % 중심 하부 점질량 화물: 질량 + 롤/피치 관성(환산질량, 물리정확).
    M = cdry.body.m + pm;  mu = cdry.body.m*pm/M;          % 환산질량
    c = cdry;  c.body.m = M;
    c.body.I = cdry.body.I + mu*poff^2*diag([1 1 0]);
    if withCoG, c.config.CoG_Pos_c = c.config.CoG_Pos_c + [0;0;(pm/M)*poff]; end
end
function [te,tilt] = tiltsig(out)
    te=out.Euler_angles.Time(:); E=sq3(out.Euler_angles.Data,3);
    tilt = acosd( max(-1,min(1, cos(E(:,1)).*cos(E(:,2)) )) );
end
function [drop,trec,mxtilt,zerr,endtilt] = perfmetrics(out)
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    post=ts>=1; upp=up(post); tp=ts(post);
    [mn,idx]=min(upp); drop=-mn;
    after=idx:numel(upp); rec=find(abs(upp(after))<0.5,1);
    if isempty(rec), trec=inf; else, trec=tp(after(rec))-1; end
    zerr = abs(up(end));
    [te,tilt]=tiltsig(out);
    mxtilt  = max(tilt(te>=1));
    endtilt = max(tilt(te>=te(end)-3));     % 말기 3초 최대틸트(말기 안정성)
end
function X = sq3(D,nch)
    X=squeeze(D);  if size(X,2)~=nch && size(X,1)==nch, X=X.'; end
end
