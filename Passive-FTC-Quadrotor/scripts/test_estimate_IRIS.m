%% test_estimate_IRIS.m — 무게추정 검증 (IRIS, 호버 모터속도 기반)
% 원리: 호버에서 sum(모터속도^2) 는 총무게에 비례.
%       마른 드론으로 1회 보정 → 화물 호버 모터속도로 총무게 역산.
%       m_total = m_dry * sum(w_hover^2) / sum(w_dry_hover^2)
% 추정은 고장(t=1) 전 호버 구간에서 수행 → 시뮬 짧게(StopTime 2)만 돌리면 됨.
%
% 사용법:  test_estimate_IRIS

clear; clc; addPathFtc();

%% 공통 설정
failure_time_mot_1=1000; failure_time_mot_2=1; failure_time_mot_3=1000; failure_time_mot_4=1000;
simin=automatedStickCommands();
copter=copterLoadParams('copter_params_IRIS');
envir=envirLoadParams('params_envir','envir',0);
IC.omega_Kb=[0;0;0]; IC.q_bg=euler2Quat([0;0;0]); IC.V_Kb=[0;0;0]; IC.s_Kg=[0;0;0];
IC.omega_mot=[1;1;1;1]*500;
grnd=groundLoadParams('params_ground_default');
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=15;
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;
fm_loiter=lindiCopterAutoCreate(copter,'AgilityAtti',0.5,'AgilityPos',0.5, ...
                                'FilterStrength',0,'CntrlEffectScaling',1);
fm_loiter.psc.inditype=1;
copter_dry=copter; m_dry=copter_dry.body.m;

m='QuadcopterSimModel_Loiter_FTC'; load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');   % FlightGear 주석처리로 생기는 미연결 포트 경고 끄기

poffset=0.10;

%% 보정: 무화물 호버
out0=runpay(m,copter_dry,0,poffset);
sumsq_dry=hover_sumsq(out0);
fprintf('\n[보정] 무화물 호버 sum(omega^2) = %.4g\n\n', sumsq_dry);

%% 화물별 추정 검증
fprintf(' 참값[g] | 추정 총질량[kg] | 추정 화물[g] | 오차[g]\n');
fprintf('---------------------------------------------------\n');
for pm=[0 0.2 0.5 1.0]
    out=runpay(m,copter_dry,pm,poffset);
    ss=hover_sumsq(out);
    mtot=m_dry*ss/sumsq_dry;  pe=mtot-m_dry;
    fprintf('  %4.0f   |     %.3f      |    %4.0f     | %+5.0f\n', ...
            pm*1000, mtot, pe*1000, (pe-pm)*1000);
end
fprintf('---------------------------------------------------\n');
fprintf('>> 추정 화물이 참값에 가까우면 = 추정기 이식 성공.\n\n');

%% ---- 보조 ----
function out=runpay(m,cdry,pm,poff)
    copter=cdry;
    copter.body.m=copter.body.m+pm;
    copter.body.I=copter.body.I+pm*poff^2*diag([1 1 0]);
    copter.config.CoG_Pos_c=copter.config.CoG_Pos_c+[0;0;poff];
    assignin('base','copter',copter);
    out=sim(m,'StopTime','2');         % 고장(t=1) 전 호버만 필요
end
function s=hover_sumsq(out)
    t=out.motor_speed.Time(:); W=sq(out.motor_speed.Data,4);   % N x 4
    idx = t>=-0.5 & t<=0.8;            % 고장 전 안정 호버 구간
    s = mean(sum(W(idx,:).^2,2));
end
function X=sq(D,nch)
    X=squeeze(D);
    if size(X,2)~=nch && size(X,1)==nch, X=X.'; end
end
