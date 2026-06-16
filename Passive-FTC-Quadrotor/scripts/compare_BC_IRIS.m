%% compare_BC_IRIS.m — 적응형 FTC 비교 (B vs C1 vs C2)
% 한 화물 조건(기본 500g = baseline B가 추락하는 임계점)에서 세 제어기 비교.
%   B  = baseline           : 마른 IRIS 기준 제어기 (화물 모름)
%   C1 = 적응(질량+관성)    : 추정 화물의 질량/관성을 제어기에 반영
%   C2 = 적응(질량+관성+CoG): C1 + 무게중심 하강까지 반영
% ★ plant(진짜 드론)은 항상 '진짜 화물'(질량+관성+CoG) 그대로. 제어기만 교체.
% ★ 추정값은 무게추정기를 실제로 돌려서 얻음 → "추정→적응" 전체 파이프라인 시연.
%
% 가설: baseline B가 추락하는 화물에서, 제어기에 추정 화물을 반영(C1/C2)하면
%       추력배분(G행렬)이 올바르게 보정되어 단일고장 후에도 생존한다.
%       H1=질량만(C1)으로 충분?  H2=무게중심(C2)까지 필요?  → 이 실험이 가린다.
%
% 사용법:  compare_BC_IRIS                 % 기본 500g
%          TRUE_PAYLOAD=1.0; compare_BC_IRIS   % 화물 직접 지정(먼저 변수 설정)

if ~exist('TRUE_PAYLOAD','var') || isempty(TRUE_PAYLOAD), TRUE_PAYLOAD = 0.5; end
clc; addPathFtc();
poffset = 0.10;                 % [m] 화물 장착 오프셋 (알려진 장착 위치 = 설계값)

%% 공통 설정
failure_time_mot_1=1000; failure_time_mot_2=1; failure_time_mot_3=1000; failure_time_mot_4=1000;
simin = automatedStickCommands();
copter = copterLoadParams('copter_params_IRIS');     % 이름 'copter' 필수
envir = envirLoadParams('params_envir','envir',0);
IC.omega_Kb=[0;0;0]; IC.q_bg=euler2Quat([0;0;0]); IC.V_Kb=[0;0;0]; IC.s_Kg=[0;0;0];
IC.omega_mot=[1;1;1;1]*500;
grnd=groundLoadParams('params_ground_default');
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=15;
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;

copter_dry = copter;  m_dry = copter_dry.body.m;
copter_plant = mkplant(copter_dry, TRUE_PAYLOAD, poffset, true);   % 진짜 드론(plant)

m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');

%% 0) 화물 무게 추정 (호버, 고장 전) — 추정→적응 파이프라인
fm_loiter = mkctrl(copter_dry);                        % 추정 시뮬용(아무 제어기나)
copter = copter_dry;    out0 = sim(m,'StopTime','2');  ss_dry = hover_sumsq(out0);
copter = copter_plant;  outp = sim(m,'StopTime','2');  ss_p   = hover_sumsq(outp);
m_est = m_dry*ss_p/ss_dry - m_dry;                     % 추정 화물 [kg]
fprintf('\n[추정] 진짜 화물 %.0f g → 추정 화물 %.0f g (오차 %+.0f g)\n', ...
        TRUE_PAYLOAD*1000, m_est*1000, (m_est-TRUE_PAYLOAD)*1000);

%% 1) 세 제어기 설계 (추정 화물 m_est 사용 — 진짜값 아님)
designs = { copter_dry, ...                                  % B  : 화물 모름
            mkplant(copter_dry, m_est, poffset, false), ...  % C1 : 질량+관성
            mkplant(copter_dry, m_est, poffset, true ) };    % C2 : +무게중심
names = {'B (baseline·화물모름)','C1 (질량+관성)','C2 (+무게중심)'};

%% 2) 비교 시뮬 (plant은 항상 진짜 화물)
figure(50); clf; hold on; grid on; cols=lines(3);
fprintf('\n 제어기                | 최대강하[m] | 회복시간[s] | 최종고도[m] | 생존\n');
fprintf('----------------------------------------------------------------------\n');
for k=1:3
    fm_loiter = mkctrl(designs{k});      % 제어기 = 설계 화물 기준 (G행렬 baked-in)
    copter    = copter_plant;            % plant = 진짜 화물
    out = sim(m,'StopTime','18');
    [drop,trec,zf,ok] = pmetrics(out);
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    plot(ts,up,'Color',cols(k,:),'LineWidth',1.6,'DisplayName',names{k});
    v='추락 ✗'; if ok, v='생존 ✓'; end
    fprintf('  %-20s|  %8.2f  |  %8.2f  |  %8.2f  | %s\n', names{k}, drop, trec, zf, v);
end
fprintf('----------------------------------------------------------------------\n');
fprintf('>> B 추락 & C1/C2 생존 = 적응 효과 입증. C1만으로 생존=H1, C2 필요=H2.\n\n');
xline(1,'k--','고장'); xlabel('시간 [s]'); ylabel('고도 [m]');
title(sprintf('적응형 FTC 비교 (화물 %.0f g): B vs C1 vs C2', TRUE_PAYLOAD*1000));
legend('show','Location','southwest');

%% ===== 보조 함수 =====
function fm = mkctrl(cdes)
    copter = cdes;   % lindiCopterAutoCreate가 evalin('caller','copter')로 읽음
    fm = lindiCopterAutoCreate(copter,'AgilityAtti',0.5,'AgilityPos',0.5, ...
                               'FilterStrength',0,'CntrlEffectScaling',1);
    fm.psc.inditype = 1;
end

function c = mkplant(cdry, pm, poff, withCoG)
    c = cdry;
    c.body.m = c.body.m + pm;                               % 총질량
    c.body.I = c.body.I + pm*poff^2*diag([1 1 0]);          % 평행축(롤/피치)
    if withCoG
        c.config.CoG_Pos_c = c.config.CoG_Pos_c + [0;0;poff];   % 무게중심 아래로
    end
end

function s = hover_sumsq(out)
    t=out.motor_speed.Time(:); W=sq(out.motor_speed.Data,4);
    idx = t>=-0.5 & t<=0.8;  s = mean(sum(W(idx,:).^2,2));
end

function [drop,trec,zf,ok] = pmetrics(out)
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    post=ts>=1; upp=up(post); tp=ts(post);
    [mn,idx]=min(upp); drop=-mn;
    after=idx:numel(upp); rec=find(abs(upp(after))<0.5,1);
    if isempty(rec), trec=inf; else, trec=tp(after(rec))-1; end
    zf = up(end);
    ss=ts>=(ts(end)-3); ok=isfinite(trec) && max(abs(up(ss)))<2;
end

function X = sq(D,nch)
    X=squeeze(D);
    if size(X,2)~=nch && size(X,1)==nch, X=X.'; end
end
