%% diag_altitude_IRIS.m — 500g 추락: "영구 불가" vs "과도 고도부족" 판별
% 배경: diag_thrust_IRIS — 500g는 추력 포화. 근데 일단 스핀 안정되면 상승함(정적 호버 가능).
%        → 추락이 "시작고도(15m) < 과도구간 강하(15m)" 때문일 수 있음.
% 방법: 500g 고정, 시작고도(pos_ref.alt)만 15→150m로 올려가며 강하/회복 관찰.
%   - 강하가 어느 값에서 멈추고 공중에서 회복 = 과도 고도부족 (충분히 높으면 생존)
%   - 강하가 끝없이 커짐 = 영구 추락 (정적으로도 못 버팀)
%
% 사용법:  diag_altitude_IRIS              % 기본 500g
%          TRUE_PAYLOAD=0.75; diag_altitude_IRIS

if ~exist('TRUE_PAYLOAD','var') || isempty(TRUE_PAYLOAD), TRUE_PAYLOAD = 0.5; end
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
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=15;
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;

copter_dry = copter;  m_dry = copter_dry.body.m;

% 제어기: 질량보정(C1), plant: 진짜 화물
copter_ctrl  = mkplant(copter_dry, TRUE_PAYLOAD, poffset, false);
copter_plant = mkplant(copter_dry, TRUE_PAYLOAD, poffset, true);

m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');

fm_loiter = mkctrl(copter_ctrl);

%% 시작고도 스윕
altList = [15 30 50 80 120 150];
figure(52); clf; hold on; grid on; cols=lines(numel(altList));
fprintf('\n=== 화물 %.0f g: 시작고도별 강하/회복 ===\n', TRUE_PAYLOAD*1000);
fprintf(' 시작고도[m] | 최대강하[m] | 최저점고도[m] | 최종고도[m] | 공중회복 | 생존\n');
fprintf('--------------------------------------------------------------------------\n');
for k=1:numel(altList)
    pos_ref.alt = altList(k);
    copter = copter_plant;
    out = sim(m,'StopTime','25');
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));  % 시작점 기준 상대고도(0에서 시작)
    post=ts>=1; [mn,~]=min(up(post)); drop=-mn;
    absLow = altList(k) + mn;             % 지면 기준 최저 고도(절대)
    zfRel  = up(end);                     % 최종 상대고도
    airRec = (absLow > 1) && (zfRel > mn+1);   % 지면 안 찍고(>1m) 다시 올라왔나
    surv   = airRec && (zfRel > -altList(k)+1);
    plot(ts, altList(k)+up, 'Color',cols(k,:),'LineWidth',1.4, ...
         'DisplayName',sprintf('시작 %dm',altList(k)));
    fprintf('   %4.0f      |   %7.2f   |    %7.2f    |   %7.2f   |   %s   | %s\n', ...
        altList(k), drop, absLow, zfRel, tf(airRec), sv(surv));
end
fprintf('--------------------------------------------------------------------------\n');
fprintf('>> 강하가 어느 값에서 포화하고 공중회복=O 뜨면 = 과도 고도부족(높이면 생존).\n');
fprintf('   강하가 시작고도 따라 끝없이 커지면 = 영구 추락(정적으로도 불가).\n\n');
yline(0,'r--','지면'); xline(1,'k--','고장');
xlabel('시간 [s]'); ylabel('지면 기준 고도 [m]');
title(sprintf('화물 %.0f g: 시작고도별 고도 궤적', TRUE_PAYLOAD*1000));
legend('show','Location','best');

%% ===== 보조 =====
function fm = mkctrl(cdes)
    copter = cdes;
    fm = lindiCopterAutoCreate(copter,'AgilityAtti',0.5,'AgilityPos',0.5, ...
                               'FilterStrength',0,'CntrlEffectScaling',1);
    fm.psc.inditype = 1;
end
function c = mkplant(cdry, pm, poff, withCoG)
    c = cdry;  c.body.m = c.body.m + pm;
    c.body.I = c.body.I + pm*poff^2*diag([1 1 0]);
    if withCoG, c.config.CoG_Pos_c = c.config.CoG_Pos_c + [0;0;poff]; end
end
function s=tf(b), if b, s='O'; else, s='X'; end, end
function s=sv(ok), if ok, s='생존 ✓'; else, s='추락 ✗'; end, end
