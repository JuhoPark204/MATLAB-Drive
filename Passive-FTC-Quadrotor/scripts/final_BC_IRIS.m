%% final_BC_IRIS.m — 헤드라인 결과: B vs C (추정→적응 전체 파이프라인)
% 스톡 IRIS, 화물 ≤25%(생존범위). C = 무게추정 → 질량·관성 피드포워드.
%   B : baseline, 마른 IRIS 기준 (화물 모름), 0.5/0.5
%   C : 호버에서 화물무게 추정 → 추정질량·관성 제어기에 반영 (적응), 0.5/0.5
% ★ C는 진짜질량이 아니라 '추정값'을 씀 = 추정→적응 end-to-end 시연.
% 결과: 화물별 B vs C 성능표 + 최대화물에서 B(발산) vs C(안정) 궤적.
%
% 사용법:  final_BC_IRIS

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
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=50;
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;

copter_dry = copter;  m_dry = copter_dry.body.m;

m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');

%% 0) 무화물 호버 보정 (추정 기준)
fm_loiter = mkctrl(copter_dry, 0.5, 0.5);
copter = copter_dry;  out0 = sim(m,'StopTime','2');  sumsq_dry = hover_sumsq(out0);

%% 1) 화물별 B vs C
payloads = [0 0.2 0.3 0.34];
plotP = 0.34;                       % 궤적 그릴 화물(최대=임계)
store = struct();

fprintf('\n=== 헤드라인: B vs C (추정→적응), 스톡 IRIS ===\n');
fprintf(' 화물[g] | 추정[g] | 전략 | 강하[m] | 회복[s] | 말기틸트[°] | 최종오차[m]\n');
fprintf('--------------------------------------------------------------------------\n');
for p = payloads
    cpl = mkplant(copter_dry, p, poffset, true);     % plant=진짜 화물
    % --- 무게추정 (고장 전 호버) ---
    fm_loiter = mkctrl(copter_dry,0.5,0.5); copter = cpl;
    outh = sim(m,'StopTime','2');  m_est = m_dry*hover_sumsq(outh)/sumsq_dry - m_dry;
    % --- B: 마른 제어기 ---
    fm_loiter = mkctrl(copter_dry, 0.5, 0.5);  copter = cpl;
    outB = sim(m,'StopTime','25');  [dB,rB,~,zB,eB] = perfmetrics(outB);
    % --- C: 추정질량 피드포워드 ---
    fm_loiter = mkctrl(mkplant(copter_dry,m_est,poffset,false), 0.5, 0.5);  copter = cpl;
    outC = sim(m,'StopTime','25');  [dC,rC,~,zC,eC] = perfmetrics(outC);
    fprintf('  %4.0f   |  %4.0f   |  B   | %6.2f  | %6.2f  |   %6.1f    |  %6.2f\n', p*1000,m_est*1000,dB,rB,eB,zB);
    fprintf('         |         |  C   | %6.2f  | %6.2f  |   %6.1f    |  %6.2f\n', dC,rC,eC,zC);
    fprintf('--------------------------------------------------------------------------\n');
    if abs(p-plotP)<1e-9, store.B=outB; store.C=outC; store.m_est=m_est; end
end
fprintf('>> C가 회복·말기틸트·최종오차에서 B를 이기면(특히 %.0fg) = 적응 성능개선 입증.\n\n', plotP*1000);

%% 2) 헤드라인 그림: 최대화물에서 B vs C
if isfield(store,'B')
    figure(54); clf;
    sp1=subplot(2,1,1); hold on; grid on;
    plotalt(store.B,'B (화물 모름)',[0.85 0.33 0.10]);
    plotalt(store.C,'C (추정→적응)',[0 0.45 0.74]);
    xline(1,'k--','고장'); ylabel('고도[m]'); legend('Location','southeast');
    title(sprintf('화물 %.0fg(추정 %.0fg): B vs C — 고도', plotP*1000, store.m_est*1000));
    sp2=subplot(2,1,2); hold on; grid on;
    plottilt(store.B,'B',[0.85 0.33 0.10]);
    plottilt(store.C,'C',[0 0.45 0.74]);
    xline(1,'k--'); ylabel('틸트[°]'); xlabel('시간[s]'); legend('Location','northeast');
    title('B vs C — 틸트 (B 말기 발산 vs C 안정)');
end

%% ===== 보조 =====
function fm = mkctrl(cdes, aAtti, aPos)
    copter = cdes;
    fm = lindiCopterAutoCreate(copter,'AgilityAtti',aAtti,'AgilityPos',aPos, ...
                               'FilterStrength',0,'CntrlEffectScaling',1);
    fm.psc.inditype = 1;
end
function c = mkplant(cdry, pm, poff, withCoG)
    % 중심 하부 점질량 화물: 질량 + 롤/피치 관성(환산질량, 물리정확).
    % z-CoG 이동은 회전동역학 무영향(추력∥z)이라 동역학엔 불필요하나, 물리정확값으로만 표기.
    M = cdry.body.m + pm;  mu = cdry.body.m*pm/M;          % 환산질량
    c = cdry;  c.body.m = M;
    c.body.I = cdry.body.I + mu*poff^2*diag([1 1 0]);       % 평행축(환산질량)
    if withCoG, c.config.CoG_Pos_c = c.config.CoG_Pos_c + [0;0;(pm/M)*poff]; end
end
function s = hover_sumsq(out)
    t=out.motor_speed.Time(:); W=sq3(out.motor_speed.Data,4);
    idx=t>=-0.5 & t<=0.8;  s=mean(sum(W(idx,:).^2,2));
end
function [te,tilt] = tiltsig(out)
    te=out.Euler_angles.Time(:); E=sq3(out.Euler_angles.Data,3);
    tilt = acosd( max(-1,min(1, cos(E(:,1)).*cos(E(:,2)) )) );
end
function [drop,trec,mxtilt,zerr,endtilt] = perfmetrics(out)
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    post=ts>=1; upp=up(post); tp=ts(post);
    [mn,idx]=min(upp); drop=-mn;
    after=idx:numel(upp); rec=find(abs(upp(after))<0.5,1);
    if isempty(rec), trec=inf; else, trec=tp(after(rec))-1; end
    zerr=abs(up(end)); [te,tilt]=tiltsig(out);
    mxtilt=max(tilt(te>=1)); endtilt=max(tilt(te>=te(end)-3));
end
function plotalt(out,nm,c)
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    plot(ts,up,'Color',c,'LineWidth',1.6,'DisplayName',nm);
end
function plottilt(out,nm,c)
    [te,tilt]=tiltsig(out); plot(te,tilt,'Color',c,'LineWidth',1.3,'DisplayName',nm);
end
function X = sq3(D,nch)
    X=squeeze(D);  if size(X,2)~=nch && size(X,1)==nch, X=X.'; end
end
