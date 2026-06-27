%% demo5_BvsC_IRIS — 핵심 시연: 기존 vs 적응 궤적 @340g (추정→적응 end-to-end)
% B: 기존(화물 미반영). C: 호버에서 화물 추정 → 질량·관성 반영 + 화물별 최적게인.
% 고도·틸트 궤적 비교 → 기존 말기 발산 vs 적응 안정.
% 사용법:  demo5_BvsC_IRIS
clear; clc; addPathFtc();
poffset = 0.10;
PAYLOAD = 0.34;            % 시연 화물[kg]
BESTGAIN = [0.437 0.724];  % [AgilityAtti AgilityPos] — optimize_gain_bo_IRIS(BO) 340g 결과

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

%% 0) 무화물 보정
fprintf('기존 vs 적응 시연 중 (%dg)...\n', round(PAYLOAD*1000));
copter=copter_dry; fm_loiter=mkctrl(copter_dry,0.5,0.5);
out0=sim(m,'StopTime','2'); sumsq_dry=hover_sumsq(out0);

%% 1) 화물 장착 + 추정
cpl=mkplant(copter_dry,PAYLOAD,poffset,true);
copter=cpl; fm_loiter=mkctrl(copter_dry,0.5,0.5);
outh=sim(m,'StopTime','2'); m_est=m_dry*hover_sumsq(outh)/sumsq_dry-m_dry;
fprintf('  추정 화물 = %.0fg\n', m_est*1000);

%% 2) B(기존) vs C(적응)
copter=cpl; fm_loiter=mkctrl(copter_dry,0.5,0.5);
outB=sim(m,'StopTime','25');
copter=cpl; fm_loiter=mkctrl(mkplant(copter_dry,m_est,poffset,false), BESTGAIN(1),BESTGAIN(2));
outC=sim(m,'StopTime','25');

%% 3) 그림
figure('Name','핵심 시연: 기존 vs 적응 @340g','Color','w','Position',[80 70 900 660]); clf;
ax1=subplot(2,1,1); hold(ax1,'on'); grid(ax1,'on');
plotalt(ax1,outB,'기존 (화물 미반영)',[.85 .33 .10]);
plotalt(ax1,outC,'적응 (추정→질량반영+게인)',[0 .45 .74]);
xline(ax1,1,'k--','고장'); set(ax1,'FontSize',12);
ylabel(ax1,'고도 [m]','FontSize',12); legend(ax1,'Location','southeast','FontSize',12);
title(ax1,sprintf('%dg (추정 %.0fg): 기존 vs 적응 — 고도',round(PAYLOAD*1000),m_est*1000),'FontSize',14);
ax2=subplot(2,1,2); hold(ax2,'on'); grid(ax2,'on');
plottilt(ax2,outB,'기존',[.85 .33 .10]);
plottilt(ax2,outC,'적응',[0 .45 .74]);
xline(ax2,1,'k--'); set(ax2,'FontSize',12);
ylabel(ax2,'틸트 [°]','FontSize',12); xlabel(ax2,'시간 [s]','FontSize',12);
legend(ax2,'Location','northeast','FontSize',12);
title(ax2,'틸트 — 기존 말기 발산 vs 적응 안정','FontSize',13);
fprintf('완료 — 적응이 말기 틸트·회복에서 우세.\n');

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
function s=hover_sumsq(o)
    t=o.motor_speed.Time(:); W=sq(o.motor_speed.Data,4);
    idx=t>=-0.5 & t<=0.8; s=mean(sum(W(idx,:).^2,2));
end
function [te,tilt]=tiltsig(o)
    te=o.Euler_angles.Time(:); E=sq(o.Euler_angles.Data,3);
    tilt=acosd(max(-1,min(1, cos(E(:,1)).*cos(E(:,2)))));
end
function plotalt(ax,o,nm,c)
    ts=o.s_g.Time(:); up=-squeeze(o.s_g.Data(3,1,:));
    plot(ax,ts,up,'Color',c,'LineWidth',2.0,'DisplayName',nm);
end
function plottilt(ax,o,nm,c)
    [te,tl]=tiltsig(o); plot(ax,te,tl,'Color',c,'LineWidth',1.6,'DisplayName',nm);
end
function X=sq(D,nch), X=squeeze(D); if size(X,2)~=nch && size(X,1)==nch, X=X.'; end, end
