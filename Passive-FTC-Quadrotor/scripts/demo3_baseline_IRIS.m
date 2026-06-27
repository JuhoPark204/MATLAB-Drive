%% demo3_baseline_IRIS — 실험④ baseline 화물 영향 (개선 대상)
% 화물 모르는 기존 제어기. 화물 0~340g 고도·틸트 궤적. 클수록 회복 악화·말기 발산.
% 사용법:  demo3_baseline_IRIS
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

%% 화물 스윕 (0~340g)
fprintf('baseline 화물 영향 측정 중 (5회)...\n');
P=[0 0.1 0.2 0.3 0.34];
figure('Name','실험3: baseline 화물 영향','Color','w','Position',[90 80 880 640]); clf;
ax1=subplot(2,1,1); hold(ax1,'on'); grid(ax1,'on');
ax2=subplot(2,1,2); hold(ax2,'on'); grid(ax2,'on');
cols=lines(numel(P));
fm_loiter=mkctrl(copter_dry,0.5,0.5);
for i=1:numel(P)
    copter=mkplant(copter_dry,P(i),poffset,true);
    o=sim(m,'StopTime','25');
    ts=o.s_g.Time(:); up=-squeeze(o.s_g.Data(3,1,:));
    [te,tl]=tiltsig(o); nm=sprintf('%dg',round(P(i)*1000));
    plot(ax1,ts,up,'Color',cols(i,:),'LineWidth',1.6,'DisplayName',nm);
    plot(ax2,te,tl,'Color',cols(i,:),'LineWidth',1.4,'DisplayName',nm);
end
xline(ax1,1,'k--'); xline(ax2,1,'k--');
set(ax1,'FontSize',12); set(ax2,'FontSize',12);
ylabel(ax1,'고도 [m]','FontSize',12); legend(ax1,'Location','southeast','FontSize',11);
title(ax1,'실험 ④ baseline — 화물 클수록 회복 악화','FontSize',14);
ylabel(ax2,'틸트 [°]','FontSize',12); xlabel(ax2,'시간 [s]','FontSize',12);
legend(ax2,'Location','northeast','FontSize',11);
title(ax2,'340g: 말기 자세(틸트) 발산','FontSize',13);
fprintf('완료.\n');

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
function [te,tilt]=tiltsig(o)
    te=o.Euler_angles.Time(:); E=sq(o.Euler_angles.Data,3);
    tilt=acosd(max(-1,min(1, cos(E(:,1)).*cos(E(:,2)))));
end
function X=sq(D,nch), X=squeeze(D); if size(X,2)~=nch && size(X,1)==nch, X=X.'; end, end
