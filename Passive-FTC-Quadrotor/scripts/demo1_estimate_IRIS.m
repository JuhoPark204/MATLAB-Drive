%% demo1_estimate_IRIS — 실험① 화물 질량 추정 (실제 vs 추정 막대그래프)
% 호버 로터 회전수 제곱합 비율로 총질량 역산 → 화물 질량 추정. 오차 ≤2%.
% 사용법:  demo1_estimate_IRIS
clear; clc; addPathFtc();
poffset = 0.10;

%% 공통 설정 (base 워크스페이스 = sim이 읽음)
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

%% 무화물 보정 + 화물별 추정
fprintf('화물 질량 추정 중 (호버 5회)...\n');
copter=copter_dry; fm_loiter=mkctrl(copter_dry,0.5,0.5);
out0=sim(m,'StopTime','2'); sumsq_dry=hover_sumsq(out0);

P=[0 0.2 0.5 1.0]; est=zeros(size(P));
for i=1:numel(P)
    copter=mkplant(copter_dry,P(i),poffset,true);
    fm_loiter=mkctrl(copter_dry,0.5,0.5);
    o=sim(m,'StopTime','2');
    est(i)=m_dry*hover_sumsq(o)/sumsq_dry-m_dry;
end

%% 그림
figure('Name','실험1: 화물 질량 추정','Color','w','Position',[100 120 760 480]); clf;
b=bar([P(:) est(:)]*1000,'grouped'); grid on;
b(1).FaceColor=[.6 .6 .65]; b(2).FaceColor=[0 .45 .74];
set(gca,'XTickLabel',compose('%dg',round(P*1000)),'FontSize',13);
ylabel('화물 질량 [g]','FontSize',13);
legend({'실제','추정'},'Location','northwest','FontSize',13);
title('실험 ① 화물 질량 추정 — 오차 ≤ 2%','FontSize',15);
for i=1:numel(P)
    errp=abs(est(i)-P(i))/max(P(i),eps)*100;
    txt=sprintf('%.0fg\n(오차 %.0f%%)',est(i)*1000,errp);
    if P(i)==0, txt=sprintf('%.0fg',est(i)*1000); end
    text(i+0.15,est(i)*1000+25,txt,'HorizontalAlignment','center','FontSize',11,'Color',[0 .3 .6]);
end
fprintf('완료 — 추정[g]: %s\n', mat2str(round(est*1000)));

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
function X=sq(D,nch), X=squeeze(D); if size(X,2)~=nch && size(X,1)==nch, X=X.'; end, end
