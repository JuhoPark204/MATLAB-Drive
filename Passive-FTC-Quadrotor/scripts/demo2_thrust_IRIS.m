%% demo2_thrust_IRIS — 실험② 추력 물리한계 (생존 vs 영구추락)
% 화물별 고장 후 고도 궤적. 500g↑ = 추력 포화 → 어떤 제어로도 못 살림(물리한계).
% 사용법:  demo2_thrust_IRIS
clear; clc; addPathFtc();
poffset = 0.10;

%% 공통 설정
failure_time_mot_1=1000; failure_time_mot_2=1; failure_time_mot_3=1000; failure_time_mot_4=1000;
simin=automatedStickCommands();
copter=copterLoadParams('copter_params_IRIS');
envir=envirLoadParams('params_envir','envir',0);
IC.omega_Kb=[0;0;0]; IC.q_bg=euler2Quat([0;0;0]); IC.V_Kb=[0;0;0]; IC.s_Kg=[0;0;0];
IC.omega_mot=[1;1;1;1]*500;
grnd=groundLoadParams('params_ground_default');
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=50;
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;
copter_dry=copter; m_dry=copter_dry.body.m;
m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');

%% 화물 스윕
fprintf('추력 한계 진단 중 (화물별 비행 4회)...\n');
P=[0 0.35 0.5 1.0];
figure('Name','실험2: 추력 물리한계','Color','w','Position',[100 120 820 520]); clf;
hold on; grid on; cols=lines(numel(P));
fm_loiter=mkctrl(copter_dry,0.5,0.5);
for i=1:numel(P)
    copter=mkplant(copter_dry,P(i),poffset,true);
    o=sim(m,'StopTime','30');
    ts=o.s_g.Time(:); up=-squeeze(o.s_g.Data(3,1,:));
    surv=abs(up(end))<2;
    if surv, sty='-'; tag='(생존)'; else, sty='--'; tag='(추락)'; end
    plot(ts,up,'Color',cols(i,:),'LineWidth',2.0,'LineStyle',sty,...
         'DisplayName',sprintf('%dg %s',round(P(i)*1000),tag));
end
xline(1,'k--','고장','LabelVerticalAlignment','bottom','FontSize',12);
yline(0,'k:'); set(gca,'FontSize',13);
xlabel('시간 [s]','FontSize',13); ylabel('고도(상대) [m]','FontSize',13);
legend('Location','southwest','FontSize',12);
title('실험 ② 추력 한계 — 500g↑ 영구추락 (제어 아닌 물리 한계)','FontSize',15);
fprintf('완료 — 실선=생존, 점선=추락.\n');

%% ===== 보조 =====
function fm=mkctrl(cdes,aA,aP)
    copter=cdes; %#ok<NASGU>
    fm=lindiCopterAutoCreate(copter,'AgilityAtti',aA,'AgilityPos',aP,'FilterStrength',0,'CntrlEffectScaling',1);
    fm.psc.inditype=1;
end
function c=mkplant(cdry,pm,poff,withCoG)
    M=cdry.body.m+pm; mu=cdry.body.m*pm/M;
    c=cdry; c.body.m=M; c.body.I=cdry.body.I+mu*poff^2*diag([1 1 0]);
    if withCoG, c.config.CoG_Pos_c=c.config.CoG_Pos_c+[0;0;(pm/M)*poff]; end
end
