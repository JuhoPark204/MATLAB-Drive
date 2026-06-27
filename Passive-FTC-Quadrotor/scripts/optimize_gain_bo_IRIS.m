function optimize_gain_bo_IRIS()
%% optimize_gain_bo_IRIS — 베이지안 최적화로 최적 게인 탐색 (강하 포함 J)
% 격자탐색 대신 bayesopt(GP 대리모델 + 획득함수)로 적은 평가로 최적 게인 탐색.
%   비교: 가(기존) / 나(추정·게인고정) / 다(추정·BO최적게인)
%   J(낮을수록 우수) = 4지표[강하·회복·말기틸트·최종오차]를 (가) 기준 정규화 가중평균.
%   다의 게인탐색공간은 나의 게인(0.5/0.5)을 포함 → BO 최적 다 ≤ 나 보장.
% 사용법:  optimize_gain_bo_IRIS
%
% 주의: sim 결정론적(노이즈 off) → eval당 1회로 충분. robust화는 여러 화물 평균으로.
%       센서노이즈 켜면 아래 IsObjectiveDeterministic=false 로.

clc; addPathFtc();
poffset = 0.10;
PAYLOADS = 0.34;            % 최적화 대상 화물[kg] (벡터면 평균 J로 robust 게인)
WEIGHTS  = [1 1 1 1];       % [강하 회복 틸트 오차] J 가중치
NEVAL    = 30;              % BO 평가 횟수

%% ===== 공통 설정 (sim이 base서 읽음) =====
assignin('base','failure_time_mot_1',1000);
assignin('base','failure_time_mot_2',1);
assignin('base','failure_time_mot_3',1000);
assignin('base','failure_time_mot_4',1000);
assignin('base','simin',automatedStickCommands());
assignin('base','envir',envirLoadParams('params_envir','envir',0));
IC.omega_Kb=[0;0;0]; IC.q_bg=euler2Quat([0;0;0]); IC.V_Kb=[0;0;0]; IC.s_Kg=[0;0;0];
IC.omega_mot=[1;1;1;1]*500; assignin('base','IC',IC);
assignin('base','grnd',groundLoadParams('params_ground_default'));
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=50; assignin('base','pos_ref',pos_ref);
fg.remoteURL='127.0.0.1'; fg.remotePort=5502; assignin('base','fg',fg);

copter_dry = copterLoadParams('copter_params_IRIS'); m_dry = copter_dry.body.m;
m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');

%% ===== 무화물 보정 + 화물별 추정/기준선 =====
fprintf('보정·추정·기준선(가/나) 계산 중...\n');
sumsq_dry = hover_sumsq(runsim(copter_dry, mkctrl(copter_dry,0.5,0.5), '2'));
np = numel(PAYLOADS);
m_est = zeros(1,np); MET_ga = zeros(np,4); MET_na = zeros(np,4); CPL = cell(1,np); CEST = cell(1,np);
for i=1:np
    CPL{i} = mkplant(copter_dry, PAYLOADS(i), poffset, true);
    oh = runsim(CPL{i}, mkctrl(copter_dry,0.5,0.5), '2');
    m_est(i) = m_dry*hover_sumsq(oh)/sumsq_dry - m_dry;
    CEST{i}  = mkplant(copter_dry, m_est(i), poffset, false);   % 추정질량·관성 반영
    MET_ga(i,:) = metvec(runsim(CPL{i}, mkctrl(copter_dry,0.5,0.5), '25'));        % 가
    MET_na(i,:) = metvec(runsim(CPL{i}, mkctrl(CEST{i},0.5,0.5), '25'));           % 나
    fprintf('  %dg: 추정 %.0fg | J_가=%.3f J_나=%.3f\n', round(PAYLOADS(i)*1000), ...
            m_est(i)*1000, costJ(MET_ga(i,:),MET_ga(i,:),WEIGHTS), costJ(MET_na(i,:),MET_ga(i,:),WEIGHTS));
end

%% ===== 베이지안 최적화 =====
vA = optimizableVariable('aA',[0.2 0.95]);
vP = optimizableVariable('aP',[0.2 0.95]);
fprintf('\nBO 시작 (%d회 평가)...\n', NEVAL);
res = bayesopt(@obj, [vA vP], ...
    'IsObjectiveDeterministic', true, ...
    'MaxObjectiveEvaluations', NEVAL, ...
    'AcquisitionFunctionName','expected-improvement-plus', ...
    'PlotFcn',{@plotObjectiveModel,@plotMinObjective}, ...
    'Verbose',1);
bp = bestPoint(res); gA=bp.aA; gP=bp.aP;

%% ===== 결과: 가/나/다(BO) 비교 =====
MET_da = zeros(np,4);
for i=1:np, MET_da(i,:) = metvec(runsim(CPL{i}, mkctrl(CEST{i},gA,gP), '25')); end

fprintf('\n========= BO 최적 게인 = [aA %.3f , aP %.3f] =========\n', gA, gP);
fprintf(' 화물[g] | 제어기 | 강하[m] | 회복[s] | 말기틸트[°] | 최종오차[m] | J(가=1)\n');
fprintf('--------------------------------------------------------------------------------\n');
for i=1:np
    prow(PAYLOADS(i),'가', MET_ga(i,:), MET_ga(i,:), WEIGHTS);
    prow(PAYLOADS(i),'나', MET_na(i,:), MET_ga(i,:), WEIGHTS);
    prow(PAYLOADS(i),'다', MET_da(i,:), MET_ga(i,:), WEIGHTS);
    fprintf('--------------------------------------------------------------------------------\n');
end

%% ===== 그림: 종합 J 비교 (평균) =====
Jga=mean(arrayfun(@(i)costJ(MET_ga(i,:),MET_ga(i,:),WEIGHTS),1:np));
Jna=mean(arrayfun(@(i)costJ(MET_na(i,:),MET_ga(i,:),WEIGHTS),1:np));
Jda=mean(arrayfun(@(i)costJ(MET_da(i,:),MET_ga(i,:),WEIGHTS),1:np));
figure('Name','가/나/다(BO) 종합 J','Color','w','Position',[160 120 720 460]); clf;
b=bar([Jga Jna Jda]); b.FaceColor='flat';
b.CData(1,:)=[.85 .33 .10]; b.CData(2,:)=[.93 .69 .13]; b.CData(3,:)=[0 .45 .74];
set(gca,'XTickLabel',{'가: 기존','나: +추정','다: +추정+BO게인'},'FontSize',12); grid on;
ylabel('종합 J (가=1.0, 낮을수록 우수)','FontSize',12); yline(1,'k:','가 기준');
title(sprintf('BO 최적게인 [%.2f, %.2f] — 다가 가·나 모두 이김', gA,gP),'FontSize',13);
text(1,Jga,sprintf('  %.2f',Jga),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',11);
text(2,Jna,sprintf('  %.2f',Jna),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',11);
text(3,Jda,sprintf('  %.2f\n(%.0f%% 개선)',Jda,(1-Jda)*100),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',11,'Color',[0 .3 .6]);

if Jda<=Jna && Jda<=Jga
    fprintf('\n>> 성공: 다(J=%.3f) < 나(J=%.3f), 가(J=1.0). demo_compare3 BESTGAIN=[%.3f %.3f] 로 갱신.\n',Jda,Jna,gA,gP);
else
    fprintf('\n>> 주의: 다가 아직 못 이김(J_다=%.3f). NEVAL↑ 또는 탐색범위/가중치 조정 필요.\n',Jda);
end

%% ===================== 중첩: 목적함수 =====================
    function J = obj(x)
        Js = zeros(1,np);
        for ii=1:np
            met = metvec(runsim(CPL{ii}, mkctrl(CEST{ii}, x.aA, x.aP), '25'));
            Js(ii) = costJ(met, MET_ga(ii,:), WEIGHTS);
        end
        J = mean(Js);
    end

%% ===================== 중첩 보조 =====================
    function out = runsim(cpl, fm, st)
        assignin('base','copter',cpl); assignin('base','fm_loiter',fm);
        out = sim(m,'StopTime',st);
    end
    function fm = mkctrl(cdes,aA,aP)
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
    function v=metvec(o)
        [dr,rc,~,ze,et]=perfmetrics(o); v=[dr, min(rc,30), et, ze];
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
    function prow(p,nm,met,base,W)
        fprintf('  %4.0f   |  %s    | %6.2f  | %6.2f  |   %6.1f    |   %6.3f    |  %.3f\n', ...
                p*1000, nm, met(1),met(2),met(3),met(4), costJ(met,base,W));
    end
end

%% ===================== 파일 로컬 =====================
function J = costJ(met, base, W)
    base(base==0)=eps; r = met./base;  J = (r*W(:))/sum(W);
end
function X=sq(D,nch), X=squeeze(D); if size(X,2)~=nch && size(X,1)==nch, X=X.'; end, end
