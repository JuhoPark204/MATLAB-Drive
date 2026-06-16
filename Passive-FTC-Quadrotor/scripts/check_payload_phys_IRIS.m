%% check_payload_phys_IRIS.m — 하부화물 물리모델 검증 (rigor check)
% 질문: plant이 진짜 '하부 고정 화물' 물리를 반영하나, 아니면 사실상 '무게만'인가?
%        그리고 우리가 쓴 CoG/관성 파라미터가 물리적으로 맞나?
% 방법: 같은 화물(300g)·같은 제어기(마른,0.5/0.5)로 plant 물리모델만 4가지로 바꿔 비교.
%   M1 질량만        : m+pm                          (관성·CoG 없음)
%   M2 질량+관성      : + pm·poff²                    (CoG 없음)
%   M3 질량+관성+CoG  : + CoG +poff (★현재 코드, CoG 과대 가능)
%   M4 물리정확       : 환산질량 관성 μ·poff² + CoG (pm/M)·poff  (정확)
% 판독:
%   - M1 vs M2/M3/M4 다르면 = 관성/CoG가 동역학에 진짜 작용(무게만 아님)
%   - M3 vs M4 다르면 = 우리가 쓴 과대 파라미터가 결과를 왜곡 → M4로 재현 필요
%   - M2 vs M3/M4 차이 = plant이 CoG_Pos_c를 실제로 쓰는지 여부
%
% 사용법:  check_payload_phys_IRIS

clc; addPathFtc();
pm = 0.3; poff = 0.10;     % 화물 300g, 장착 0.1m 아래

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
M = m_dry+pm;  mu = m_dry*pm/M;
fprintf('\n화물 %.0fg, 오프셋 %.2fm | 정확 CoG이동=%.4fm(현재코드는 %.2fm), 환산질량 mu=%.4f(현재는 pm=%.2f)\n', ...
        pm*1000, poff, (pm/M)*poff, poff, mu, pm);

m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');

% 제어기: 마른 IRIS 고정(모든 모델 동일) → plant 물리 차이만 분리
fm_loiter = mkctrl(copter_dry, 0.5, 0.5);

%% 4가지 plant 물리모델
models = { ...
 'M1 질량만',       set_mass(copter_dry,pm); ...
 'M2 질량+관성',     set_massI(copter_dry,pm,poff); ...
 'M3 +CoG(현재)',    set_full_now(copter_dry,pm,poff); ...
 'M4 물리정확',      set_correct(copter_dry,pm,poff) };

figure(55); clf;
ax1=subplot(2,1,1); hold(ax1,'on'); grid(ax1,'on'); title(ax1,'고도'); ylabel(ax1,'m');
ax2=subplot(2,1,2); hold(ax2,'on'); grid(ax2,'on'); title(ax2,'틸트'); ylabel(ax2,'deg'); xlabel(ax2,'s');
cols=lines(4);
fprintf('\n plant 물리      | 강하[m] | 회복[s] | 최대틸트[°] | 말기틸트[°] | 최종오차[m]\n');
fprintf('-----------------------------------------------------------------------\n');
for k=1:size(models,1)
    copter = models{k,2};
    out = sim(m,'StopTime','25');
    [drop,trec,mxt,zerr,endt] = perfmetrics(out);
    fprintf('  %-13s | %6.2f  | %6.2f  |   %6.1f    |   %6.1f    |  %6.2f\n', ...
            models{k,1}, drop, trec, mxt, endt, zerr);
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    plot(ax1,ts,up,'Color',cols(k,:),'LineWidth',1.4,'DisplayName',models{k,1});
    [te,tl]=tiltsig(out); plot(ax2,te,tl,'Color',cols(k,:),'LineWidth',1.1,'DisplayName',models{k,1});
end
fprintf('-----------------------------------------------------------------------\n');
fprintf('>> M1 vs 나머지 다르면=관성/CoG가 작용(무게만 아님). M3 vs M4 다르면=과대파라미터 왜곡.\n\n');
xline(ax1,1,'k--'); legend(ax1,'show','Location','southeast');
xline(ax2,1,'k--'); legend(ax2,'show','Location','northeast');

%% ===== plant 물리모델 빌더 =====
function c=set_mass(c0,pm),  c=c0; c.body.m=c0.body.m+pm; end
function c=set_massI(c0,pm,poff)
    c=c0; c.body.m=c0.body.m+pm; c.body.I=c0.body.I+pm*poff^2*diag([1 1 0]);
end
function c=set_full_now(c0,pm,poff)   % 현재 코드와 동일(과대 가능)
    c=c0; c.body.m=c0.body.m+pm; c.body.I=c0.body.I+pm*poff^2*diag([1 1 0]);
    c.config.CoG_Pos_c=c0.config.CoG_Pos_c+[0;0;poff];
end
function c=set_correct(c0,pm,poff)    % 물리정확: 환산질량 + 질량비 CoG
    M=c0.body.m+pm; mu=c0.body.m*pm/M;
    c=c0; c.body.m=M; c.body.I=c0.body.I+mu*poff^2*diag([1 1 0]);
    c.config.CoG_Pos_c=c0.config.CoG_Pos_c+[0;0;(pm/M)*poff];
end

%% ===== 보조 =====
function fm=mkctrl(cdes,aA,aP)
    copter=cdes;
    fm=lindiCopterAutoCreate(copter,'AgilityAtti',aA,'AgilityPos',aP,'FilterStrength',0,'CntrlEffectScaling',1);
    fm.psc.inditype=1;
end
function [te,tilt]=tiltsig(out)
    te=out.Euler_angles.Time(:); E=sq3(out.Euler_angles.Data,3);
    tilt=acosd(max(-1,min(1,cos(E(:,1)).*cos(E(:,2)))));
end
function [drop,trec,mxt,zerr,endt]=perfmetrics(out)
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    post=ts>=1; upp=up(post); tp=ts(post); [mn,idx]=min(upp); drop=-mn;
    after=idx:numel(upp); rec=find(abs(upp(after))<0.5,1);
    if isempty(rec), trec=inf; else, trec=tp(after(rec))-1; end
    zerr=abs(up(end)); [te,tl]=tiltsig(out); mxt=max(tl(te>=1)); endt=max(tl(te>=te(end)-3));
end
function X=sq3(D,nch), X=squeeze(D); if size(X,2)~=nch && size(X,1)==nch, X=X.'; end, end
