%% bench_perf_IRIS.m — baseline 성능 벤치마크 (스톡 IRIS, 화물 ≤25%)
% 방향 확정: 실제 배송드론 비율대로 화물 ≤25%(≤~340g, IRIS 생존범위 내)로 제한,
%            그 구간에서 '적응으로 고장후 성능을 개선'하는 게 목표.
% 본 스크립트 = 개선 대상인 baseline B(화물 모름, 0.5/0.5)의 성능을 정량화.
% 지표: 최대강하 / 회복시간 / 최대틸트 / 최종고도오차.  (낮을수록 좋음)
%
% 사용법:  bench_perf_IRIS

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
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=50;   % 여유고도(과도구간 관측용)
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;

copter_dry = copter;  m_dry = copter_dry.body.m;

m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');

% baseline 제어기 = 마른 IRIS 기준(화물 모름), 0.5/0.5
fm_loiter = mkctrl(copter_dry, 0.5, 0.5);

%% 화물 스윕 (0 ~ 25% = 0~342g)
payloads = [0 0.1 0.2 0.3 0.34];   % kg  (25% of 1.37 ≈ 0.342)
figure(53); clf;
ax1=subplot(2,1,1); hold(ax1,'on'); grid(ax1,'on'); title(ax1,'고도(상대)');
ax2=subplot(2,1,2); hold(ax2,'on'); grid(ax2,'on'); title(ax2,'틸트[deg]');
cols=lines(numel(payloads));

fprintf('\n=== baseline B 성능 (스톡 IRIS, 화물 ≤25%%) ===\n');
fprintf(' 화물[g] | 화물비%% | 최대강하[m] | 회복[s] | 최대틸트[°] | 최종고도오차[m]\n');
fprintf('------------------------------------------------------------------------\n');
for k=1:numel(payloads)
    pm = payloads(k);
    copter = mkplant(copter_dry, pm, poffset, true);    % plant=진짜 화물
    out = sim(m,'StopTime','25');
    [drop,trec,mxtilt,zerr] = perfmetrics(out);
    fprintf('  %4.0f   |  %4.1f   |  %8.2f   | %6.2f  |   %6.1f    |   %6.2f\n', ...
            pm*1000, pm/m_dry*100, drop, trec, mxtilt, zerr);
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    plot(ax1, ts, up, 'Color',cols(k,:),'LineWidth',1.4,'DisplayName',sprintf('%dg',pm*1000));
    [te,tilt]=tiltsig(out);
    plot(ax2, te, tilt, 'Color',cols(k,:),'LineWidth',1.2,'DisplayName',sprintf('%dg',pm*1000));
end
fprintf('------------------------------------------------------------------------\n');
fprintf('>> 화물 커질수록 강하↑/회복↑/틸트↑ = 적응으로 줄일 여지(개선 목표).\n\n');
xline(ax1,1,'k--'); legend(ax1,'show','Location','southeast'); ylabel(ax1,'고도[m]');
xline(ax2,1,'k--'); legend(ax2,'show','Location','northeast'); xlabel(ax2,'시간[s]');

%% ===== 보조 =====
function fm = mkctrl(cdes, aAtti, aPos)
    copter = cdes;
    fm = lindiCopterAutoCreate(copter,'AgilityAtti',aAtti,'AgilityPos',aPos, ...
                               'FilterStrength',0,'CntrlEffectScaling',1);
    fm.psc.inditype = 1;
end
function c = mkplant(cdry, pm, poff, withCoG)
    % 중심 하부 점질량 화물: 질량 + 롤/피치 관성(환산질량, 물리정확).
    M = cdry.body.m + pm;  mu = cdry.body.m*pm/M;          % 환산질량
    c = cdry;  c.body.m = M;
    c.body.I = cdry.body.I + mu*poff^2*diag([1 1 0]);
    if withCoG, c.config.CoG_Pos_c = c.config.CoG_Pos_c + [0;0;(pm/M)*poff]; end
end
function [te,tilt] = tiltsig(out)
    te=out.Euler_angles.Time(:); E=sq3(out.Euler_angles.Data,3);   % N×3 [roll pitch yaw] rad
    tilt = acosd( max(-1,min(1, cos(E(:,1)).*cos(E(:,2)) )) );      % 수직 대비 총 기울기[°]
end
function [drop,trec,mxtilt,zerr] = perfmetrics(out)
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    post=ts>=1; upp=up(post); tp=ts(post);
    [mn,idx]=min(upp); drop=-mn;
    after=idx:numel(upp); rec=find(abs(upp(after))<0.5,1);
    if isempty(rec), trec=inf; else, trec=tp(after(rec))-1; end
    zerr = abs(up(end));                          % 최종 고도오차(0 기준)
    [te,tilt]=tiltsig(out); mxtilt = max(tilt(te>=1));
end
function X = sq3(D,nch)
    X=squeeze(D);
    if size(X,2)~=nch && size(X,1)==nch, X=X.'; end
end
