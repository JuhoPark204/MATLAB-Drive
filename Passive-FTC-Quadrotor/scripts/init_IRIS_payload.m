% IRIS + 하단 고정 화물 시뮬 초기화
%  ★ 핵심 설계: plant(진짜 드론)엔 화물 반영, 제어기(fm_loiter)는 마른 IRIS 기준
%               → 제어기는 화물을 모름 = baseline B (적응 안 함)
%  화물 무게만 바꿔가며 "고장 후 강하가 얼마나 심해지나" 확인용.

%% add to path
addPathFtc();
clc_clear;

%% ===== 화물 설정 (여기만 바꿔가며 실험) =====
payload_mass   = 0.2;    % [kg] 화물 무게  (실험: 0 / 0.2 / 0.5 / 1.0)
payload_offset = 0.10;   % [m]  무게중심 아래 오프셋 (하단 고정, 고정값)
% ============================================

%% rotor failure parameters (모터2, t=1s 단일 고장)
failure_time_mot_1 = 1000;
failure_time_mot_2 = 1;
failure_time_mot_3 = 1000;
failure_time_mot_4 = 1000;

%% control inputs (5초 후 5m 사각형)
simin = automatedStickCommands( );

%% 드론 파라미터 (마른 IRIS)
copter     = copterLoadParams( 'copter_params_IRIS' );
copter_dry = copter;                    % 마른 드론 보관 (제어기용)

%% environment
envir = envirLoadParams('params_envir','envir',0);

%% controller  ★ 마른 드론 기준으로 생성 → 화물 모름 (baseline B)
fm_loiter = lindiCopterAutoCreate( copter_dry, 'AgilityAtti', 0.5, ...
    'AgilityPos',0.5,'FilterStrength', 0, 'CntrlEffectScaling', 1 );
fm_loiter.psc.inditype = 1;

%% ★ plant용 copter에 화물 반영 (진짜 무게/관성/무게중심)
copter.body.m = copter.body.m + payload_mass;                                  % 총질량
copter.body.I = copter.body.I + payload_mass*payload_offset^2*diag([1 1 0]);   % 평행축(롤/피치)
copter.config.CoG_Pos_c = copter.config.CoG_Pos_c + [0;0;payload_offset];      % 무게중심 아래로 ↓

%% initial conditions
IC.omega_Kb = [ 0; 0; 0 ];
IC.q_bg     = euler2Quat( [ 0; 0; 0 ] );
IC.V_Kb     = [ 0; 0; 0 ];
IC.s_Kg     = [ 0; 0; 0 ];
IC.omega_mot = [ 1; 1; 1; 1 ] * 500;

%% ground / reference / flightgear
grnd = groundLoadParams( 'params_ground_default' );
pos_ref.lat = 37.6117; pos_ref.lon = -122.37822; pos_ref.alt = 15;
fg.remoteURL = '127.0.0.1'; fg.remotePort = 5502;

%% open model
open_model('QuadcopterSimModel_Loiter_FTC')
m='QuadcopterSimModel_Loiter_FTC';
set_param([m '/To FlightGear'],'Commented','on');           % Coder 의존 → 주석처리
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');                  % 미연결 포트 경고 끄기

fprintf('\n[init_IRIS_payload] 화물 %.0f g 장착 (plant만, 제어기는 모름).\n', payload_mass*1000);
fprintf('  총질량 = %.3f kg (마른 %.3f + 화물 %.3f)\n', copter.body.m, copter_dry.body.m, payload_mass);
