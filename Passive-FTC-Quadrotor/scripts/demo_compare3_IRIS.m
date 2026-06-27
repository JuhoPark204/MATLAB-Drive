%% demo_compare3_IRIS — 가/나/다 3제어기 비교 (강하 포함 4지표 + 종합 J)
%   가: 기존 FTC (화물 미반영, 게인 0.5/0.5)
%   나: 기존 + 무게추정 (추정질량·관성 반영, 게인 0.5/0.5)
%   다: 기존 + 무게추정 + 게인전환 (추정 반영 + 화물별 최적게인)
% 지표(모두 낮을수록 우수): 최대강하[m] / 회복시간[s] / 말기틸트[°] / 최종오차[m]
% 종합 J = 각 지표를 (가) 기준 정규화 후 가중평균 → 가=1.0, 낮을수록 개선.
% 사용법:  demo_compare3_IRIS
clear; clc; addPathFtc();
poffset = 0.10;
PAYLOAD  = 0.34;            % 시연 화물[kg]
BESTGAIN = [0.437 0.724];   % '다' 최적게인 [AgilityAtti AgilityPos] — optimize_gain_bo_IRIS(BO) 340g 결과
WEIGHTS  = [1 1 1 1];       % [강하 회복 틸트 오차] 종합 J 가중치 (강하 안전중시면 ↑)

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

%% 0) 무화물 보정 + 화물 추정
fprintf('가/나/다 비교 중 (%dg)...\n', round(PAYLOAD*1000));
copter=copter_dry; fm_loiter=mkctrl(copter_dry,0.5,0.5);
out0=sim(m,'StopTime','2'); sumsq_dry=hover_sumsq(out0);
cpl=mkplant(copter_dry,PAYLOAD,poffset,true);
copter=cpl; fm_loiter=mkctrl(copter_dry,0.5,0.5);
outh=sim(m,'StopTime','2'); m_est=m_dry*hover_sumsq(outh)/sumsq_dry-m_dry;
fprintf('  추정 화물 = %.0fg\n', m_est*1000);

%% 1) 3제어기 비행
names={'가: 기존','나: +무게추정','다: +추정+게인'};
cols=[.85 .33 .10; .93 .69 .13; 0 .45 .74];
ctrl{1}=mkctrl(copter_dry,0.5,0.5);                                  % 가
ctrl{2}=mkctrl(mkplant(copter_dry,m_est,poffset,false),0.5,0.5);     % 나
ctrl{3}=mkctrl(mkplant(copter_dry,m_est,poffset,false),BESTGAIN(1),BESTGAIN(2)); % 다
O=cell(1,3); MET=zeros(3,4);   % [강하 회복 틸트 오차]
for i=1:3
    copter=cpl; fm_loiter=ctrl{i};
    O{i}=sim(m,'StopTime','25');
    [dr,rc,~,ze,et]=perfmetrics(O{i});
    MET(i,:)=[dr, min(rc,30), et, ze];
end

%% 2) 종합 J (가 기준 정규화 + 가중평균)
base=MET(1,:); base(base==0)=eps;
R=MET./base;                          % 비율 (가=1)
Jn=(R*WEIGHTS(:))/sum(WEIGHTS);       % 종합 J (가=1.0)

%% 3) 표 출력
fprintf('\n 제어기        | 강하[m] | 회복[s] | 말기틸트[°] | 최종오차[m] | 종합J(가=1)\n');
fprintf('------------------------------------------------------------------------------\n');
for i=1:3
    fprintf(' %-13s | %6.2f  | %6.2f  |   %6.1f    |   %6.3f    |   %.3f\n', ...
            names{i}, MET(i,1),MET(i,2),MET(i,3),MET(i,4), Jn(i));
end
fprintf('------------------------------------------------------------------------------\n');
fprintf('>> 종합J 가장 낮은 제어기 = 최우수. 강하 트레이드오프는 Fig2서 확인.\n\n');

%% Fig1) 궤적 (고도 + 틸트)
figure('Name','가/나/다: 궤적','Color','w','Position',[60 70 920 660]); clf;
ax1=subplot(2,1,1); hold(ax1,'on'); grid(ax1,'on');
ax2=subplot(2,1,2); hold(ax2,'on'); grid(ax2,'on');
for i=1:3
    ts=O{i}.s_g.Time(:); up=-squeeze(O{i}.s_g.Data(3,1,:));
    [te,tl]=tiltsig(O{i});
    plot(ax1,ts,up,'Color',cols(i,:),'LineWidth',2.0,'DisplayName',names{i});
    plot(ax2,te,tl,'Color',cols(i,:),'LineWidth',1.7,'DisplayName',names{i});
end
xline(ax1,1,'k--','고장'); xline(ax2,1,'k--'); set(ax1,'FontSize',12); set(ax2,'FontSize',12);
ylabel(ax1,'고도 [m]','FontSize',12); legend(ax1,'Location','southeast','FontSize',12);
title(ax1,sprintf('%dg (추정 %.0fg): 고도 — 강하·회복 비교',round(PAYLOAD*1000),m_est*1000),'FontSize',14);
ylabel(ax2,'틸트 [°]','FontSize',12); xlabel(ax2,'시간 [s]','FontSize',12);
legend(ax2,'Location','northeast','FontSize',12); title(ax2,'틸트 — 말기 안정성','FontSize',13);

%% Fig2) 4지표 막대 (절대값)
figure('Name','가/나/다: 4지표','Color','w','Position',[120 90 900 560]); clf;
labs={'최대강하 [m]','회복시간 [s]','말기틸트 [°]','최종오차 [m]'};
for k=1:4
    subplot(2,2,k);
    b=bar(MET(:,k)); b.FaceColor='flat'; for i=1:3, b.CData(i,:)=cols(i,:); end
    set(gca,'XTickLabel',{'가','나','다'},'FontSize',12); grid on;
    title(labs{k},'FontSize',13);
    for i=1:3, text(i,MET(i,k),sprintf('  %.2f',MET(i,k)),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',10); end
end
sgtitle('지표별 비교 (낮을수록 우수) — 강하 트레이드오프 확인','FontSize',14);

%% Fig3) 종합 J (정규화)
figure('Name','가/나/다: 종합 J','Color','w','Position',[180 120 720 460]); clf;
b=bar(Jn); b.FaceColor='flat'; for i=1:3, b.CData(i,:)=cols(i,:); end
set(gca,'XTickLabel',names,'FontSize',12); grid on; ylabel('종합 J (가=1.0, 낮을수록 우수)','FontSize',12);
title('종합 비용 J — 4지표 정규화 가중평균','FontSize',14); yline(1,'k:','가 기준');
for i=1:3
    imp=(1-Jn(i))*100;
    txt=sprintf('%.2f',Jn(i)); if i>1, txt=sprintf('%.2f\n(%.0f%% 개선)',Jn(i),imp); end
    text(i,Jn(i),['  ' txt],'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',11);
end

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
function [drop,trec,mxtilt,zerr,endtilt]=perfmetrics(o)
    ts=o.s_g.Time(:); up=-squeeze(o.s_g.Data(3,1,:));
    post=ts>=1; upp=up(post); tp=ts(post);
    [mn,idx]=min(upp); drop=-mn;
    after=idx:numel(upp); rec=find(abs(upp(after))<0.5,1);
    if isempty(rec), trec=inf; else, trec=tp(after(rec))-1; end
    zerr=abs(up(end)); [te,tl]=tiltsig(o);
    mxtilt=max(tl(te>=1)); endtilt=max(tl(te>=te(end)-3));
end
function X=sq(D,nch), X=squeeze(D); if size(X,2)~=nch && size(X,1)==nch, X=X.'; end, end
