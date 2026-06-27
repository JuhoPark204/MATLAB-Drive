%% demo_estnoise_IRIS — 센서(RPM) 노이즈 하 무게추정 정확도 (계획서 요구)
% 추정기는 호버 모터회전수 제곱합(Σω²) 비율로 총질량 역산.
% RPM 측정에 가우시안 노이즈(% 단위)를 주입하고 노이즈레벨별 추정오차를 검증.
% 호버구간 다중샘플 평균이 노이즈를 억제하는지 확인 → 추정 강건성.
% 사용법:  demo_estnoise_IRIS
clear; clc; addPathFtc(); rng(0);
poffset = 0.10;
SIGMAS = [0 0.01 0.02 0.05 0.10];   % RPM 노이즈 표준편차 (값의 비율: 1%,2%,5%,10%)
NSEED  = 50;                         % 노이즈 실현 횟수(통계)
PAY    = [0.2 0.5 1.0];              % 검증 화물[kg]

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

%% 깨끗한 호버 회전수 1회씩 확보 (이후 노이즈는 후처리)
fprintf('호버 회전수 수집 중 (무화물 + 화물 %d종)...\n', numel(PAY));
fm_loiter=mkctrl(copter_dry,0.5,0.5);
copter=copter_dry; Wdry=hoverW(sim(m,'StopTime','2'));
Wpl=cell(1,numel(PAY));
for i=1:numel(PAY)
    copter=mkplant(copter_dry,PAY(i),poffset,true);
    Wpl{i}=hoverW(sim(m,'StopTime','2'));
end

%% 노이즈레벨 × 시드 추정
ns=numel(SIGMAS); np=numel(PAY);
ERR=zeros(np,ns,NSEED);   % 추정 화물오차[g]
for i=1:np
    for s=1:ns
        for k=1:NSEED
            ss_dry=sumsq_noisy(Wdry, SIGMAS(s));
            ss_pl =sumsq_noisy(Wpl{i},SIGMAS(s));
            mest = m_dry*ss_pl/ss_dry - m_dry;
            ERR(i,s,k) = (mest-PAY(i))*1000;   % g
        end
    end
end
MU = mean(ERR,3); SD = std(ERR,0,3);          % np×ns

%% 표
fprintf('\n=== RPM 노이즈별 추정 화물오차 [g] (평균±표준편차, %d시드) ===\n', NSEED);
fprintf(' 화물[g] |'); fprintf(' %4.0f%%노이즈   |', SIGMAS*100); fprintf('\n');
fprintf('---------------------------------------------------------------------\n');
for i=1:np
    fprintf('  %4.0f   |', PAY(i)*1000);
    for s=1:ns, fprintf(' %+5.0f±%-4.0f |', MU(i,s), SD(i,s)); end
    fprintf('\n');
end
fprintf('---------------------------------------------------------------------\n');
fprintf('>> 노이즈 커져도 오차 작게 유지되면 = 호버 다중샘플 평균이 노이즈 억제 = 추정 강건.\n\n');

%% 그림: 노이즈 vs 추정오차 (화물별, 평균±표준편차)
figure('Name','추정 강건성: RPM 노이즈','Color','w','Position',[120 110 800 500]); clf;
cols=lines(np); hold on; grid on;
for i=1:np
    errorbar(SIGMAS*100, MU(i,:), SD(i,:), '-o','Color',cols(i,:),'LineWidth',1.8, ...
             'DisplayName',sprintf('%dg',round(PAY(i)*1000)),'CapSize',8);
end
yline(0,'k:'); set(gca,'FontSize',12);
xlabel('RPM 측정 노이즈 [%]','FontSize',12); ylabel('추정 화물오차 [g]','FontSize',12);
legend('Location','northwest','FontSize',12);
title('센서(RPM) 노이즈 하 무게추정 오차 — 다중샘플 평균이 노이즈 억제','FontSize',13);

%% 백분율 오차 막대 (대표: 최대노이즈)
figure('Name','추정 강건성: %오차','Color','w','Position',[160 90 720 460]); clf;
pe = abs(MU(:,end))./(PAY(:)*1000)*100;     % 최대노이즈서 |평균오차|%
b=bar(pe); b.FaceColor='flat'; for i=1:np, b.CData(i,:)=cols(i,:); end
set(gca,'XTickLabel',compose('%dg',round(PAY*1000)),'FontSize',12); grid on;
ylabel(sprintf('추정오차 [%%] @ %d%% 노이즈',round(SIGMAS(end)*100)),'FontSize',12);
title('최대 노이즈서도 추정오차 유지','FontSize',13);
for i=1:np, text(i,pe(i),sprintf('  %.1f%%',pe(i)),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',11); end

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
function W=hoverW(o)
    t=o.motor_speed.Time(:); X=sq(o.motor_speed.Data,4);
    idx=t>=-0.5 & t<=0.8; W=X(idx,:);          % 호버구간 N×4 회전수
end
function s=sumsq_noisy(W,sigma)
    Wn = W .* (1 + sigma*randn(size(W)));        % RPM 측정노이즈(비례)
    s  = mean(sum(Wn.^2,2));                      % 구간 평균 Σω²
end
function X=sq(D,nch), X=squeeze(D); if size(X,2)~=nch && size(X,1)==nch, X=X.'; end, end
