%% diag_thrust_IRIS.m — 추력 포화 진단 (화물 스윕)
% 질문: 500g 추락이 "제어 실패"인가 "추력 부족(포화)"인가?
% 방법: 화물 0~1000g을 키우며, 고장 후 살아있는 3모터의 (1)최대 회전속도,
%        (2)최대 모터명령(input)을 측정. 화물 키워도 이 값이 천장에 붙어
%        더 안 오르면 = 추력 포화(더 못 짜냄) = 제어가 아니라 물리 한계.
% 보너스: 낮은 화물에서 게인 변경이 강하를 바꾸는지도 같이 확인(게인 권한 점검).
%
% 사용법:  diag_thrust_IRIS

if ~exist('COPTER','var') || isempty(COPTER), COPTER='copter_params_IRIS'; end
clc; addPathFtc();
poffset = 0.10;
fprintf('[기체] %s\n', COPTER);

%% 공통 설정
failure_time_mot_1=1000; failure_time_mot_2=1; failure_time_mot_3=1000; failure_time_mot_4=1000;
simin = automatedStickCommands();
copter = copterLoadParams(COPTER);
envir = envirLoadParams('params_envir','envir',0);
IC.omega_Kb=[0;0;0]; IC.q_bg=euler2Quat([0;0;0]); IC.V_Kb=[0;0;0]; IC.s_Kg=[0;0;0];
IC.omega_mot=[1;1;1;1]*500;
grnd=groundLoadParams('params_ground_default');
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=15;
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;

copter_dry = copter;  m_dry = copter_dry.body.m;

m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');

%% A) 화물 스윕 — 추력 포화 진단 (제어기는 각 화물에 질량보정)
payloads = [0 0.2 0.5 0.75 1.0 1.5 2.0];
fprintf('\n=== 추력 포화 진단 (제어기 질량보정, 살아있는 모터 1·3·4) ===\n');
fprintf(' 화물[g] | 총질량[kg] | 호버ω | 고장후 최대ω | 최대명령 | 명령포화%% | 강하[m] | 생존\n');
fprintf('--------------------------------------------------------------------------------\n');
for pm = payloads
    fm_loiter = mkctrl(copter_dry, pm, poffset, 0.5, 0.5);   % 질량보정 제어기
    copter    = mkplant(copter_dry, pm, poffset, true);      % plant=진짜 화물
    out = sim(m,'StopTime','18');
    [drop,~,~,ok] = pmetrics(out);
    [hovW, pkW, pkU, satF] = motormetrics(out);
    fprintf('  %4.0f   |   %.3f    | %5.0f |   %6.0f    |  %.3f  |  %5.1f   | %6.2f | %s\n', ...
            pm*1000, m_dry+pm, hovW, pkW, pkU, satF*100, drop, sv(ok));
end
fprintf('--------------------------------------------------------------------------------\n');
fprintf('>> 화물 키워도 "고장후 최대ω/최대명령"이 더 안 오르고 명령포화%%↑ = 추력 포화(물리한계).\n');
fprintf('   강하만 깊어지고 모터는 천장 = 게인 무력(제어 권한 없음) 설명됨.\n\n');

%% B) 낮은 화물에서 게인 권한 점검 (200g, 게인 0.5 vs 1.2)
fprintf('=== 게인 권한 점검 (화물 200g: 추력 여유 있는 조건) ===\n');
pm = 0.2;
for ag = [0.5 1.2]
    fm_loiter = mkctrl(copter_dry, pm, poffset, ag, ag);
    copter    = mkplant(copter_dry, pm, poffset, true);
    out = sim(m,'StopTime','18');
    [drop,~,zf,ok] = pmetrics(out);
    [~,pkW,pkU,satF] = motormetrics(out);
    fprintf('  Agility %.1f/%.1f: 강하 %.2f m, 최종 %.2f, 최대명령 %.3f, 포화%% %.1f, %s\n', ...
            ag, ag, drop, zf, pkU, satF*100, sv(ok));
end
fprintf('>> 200g에서 게인 올려 강하가 바뀌면 = 게인은 추력여유 있을 때만 효과(500g는 포화라 무력).\n\n');

%% ===== 보조 =====
function fm = mkctrl(cdry, pm, poff, aAtti, aPos)
    copter = mkplant(cdry, pm, poff, false);   % 질량+관성 보정(제어기는 CoG 안 씀)
    fm = lindiCopterAutoCreate(copter,'AgilityAtti',aAtti,'AgilityPos',aPos, ...
                               'FilterStrength',0,'CntrlEffectScaling',1);
    fm.psc.inditype = 1;
end
function c = mkplant(cdry, pm, poff, withCoG)
    c = cdry;  c.body.m = c.body.m + pm;
    c.body.I = c.body.I + pm*poff^2*diag([1 1 0]);
    if withCoG, c.config.CoG_Pos_c = c.config.CoG_Pos_c + [0;0;poff]; end
end
function [hovW,pkW,pkU,satF] = motormetrics(out)
    live=[1 3 4];                                               % 모터2 고장
    % --- 모터 회전속도 (자기 시간축) ---
    t=out.motor_speed.Time(:); W=sq(out.motor_speed.Data,4);   % N x 4
    hovW = mean(max(W(t>=-0.5 & t<=0.8, live),[],2));
    pkW  = max(max(W(t>=1, live),[],2));
    % --- 모터명령 input (자기 시간축, 길이 다를 수 있음) ---
    tu=out.input.Time(:); U=sq(out.input.Data,4);
    if size(U,2)>=4
        Up = U(tu>=1, live);
        pkU  = max(Up(:));
        satF = mean( max(Up,[],2) >= 0.99*max(U(:)) );
    else
        Up = U(tu>=1); pkU = max(Up(:)); satF = NaN;
    end
end
function [drop,trec,zf,ok] = pmetrics(out)
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    post=ts>=1; upp=up(post); tp=ts(post);
    [mn,idx]=min(upp); drop=-mn;
    after=idx:numel(upp); rec=find(abs(upp(after))<0.5,1);
    if isempty(rec), trec=inf; else, trec=tp(after(rec))-1; end
    zf = up(end);
    ss=ts>=(ts(end)-3); ok=isfinite(trec) && max(abs(up(ss)))<2;
end
function X = sq(D,nch)
    X=squeeze(D);
    if size(X,2)~=nch && size(X,1)==nch, X=X.'; end
end
function s=sv(ok), if ok, s='생존 ✓'; else, s='추락 ✗'; end, end
