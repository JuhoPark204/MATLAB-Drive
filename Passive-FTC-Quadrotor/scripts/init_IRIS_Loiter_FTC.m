% Initialize simulation of quadcopter IRIS (1.37 kg) with fault-tolerant
% loiter controller — Minnie 대신 더 큰 드론으로 교체한 버전.
% (원본: init_Minnie_Loiter_FTC.m, 바뀐 곳은 copterLoadParams 한 줄뿐)

%% add to path
addPathFtc();
clc_clear;

%% rotor failure parameters  (모터 2번이 1초에 고장 = 단일 고장)
failure_time_mot_1      = 1000;
failure_time_mot_2      = 1;
failure_time_mot_3      = 1000;
failure_time_mot_4      = 1000;

%% control inputs
% fly 5m square after 5s
simin = automatedStickCommands( );

%% load physical copter parameters  ★★ 여기만 바뀜: Minnie -> IRIS ★★
copter = copterLoadParams( 'copter_params_IRIS' );

%% environment parameters
envir = envirLoadParams('params_envir','envir',0);

%% controller parameters
% load parameters
fm_loiter = lindiCopterAutoCreate( copter, 'AgilityAtti', 0.5, ...
    'AgilityPos',0.5,'FilterStrength', 0, 'CntrlEffectScaling', 1 );   % baseline 튜닝값 (강하 최소)
fm_loiter.psc.inditype = 1;

%% initial conditions (IC)
% initial angular velocity omega_Kb, in rad/s
IC.omega_Kb = [ 0; 0; 0 ];
% initial orientation in quaternions q_bg
IC.q_bg = euler2Quat( [ 0; 0; 0 ] );
% initial velocity V_Kb, in m/s
IC.V_Kb = [ 0; 0; 0 ];
% initial position s_Kg, in m
IC.s_Kg = [ 0; 0; 0 ];
% initial motor angular velocity, in rad/s
% (IRIS는 큰 프로펠러라 호버 RPM이 다름. 일단 낮게 시작 → 제어기가 호버로 끌어올림)
IC.omega_mot = [ 1; 1; 1; 1 ] * 500;

%% load ground parameters (grnd)
grnd = groundLoadParams( 'params_ground_default' );

%% reference position lat, lon, alt
pos_ref.lat = 37.6117;
pos_ref.lon = -122.37822;
pos_ref.alt = 15;

%% Flight Gear settings for UDP connection
fg.remoteURL = '127.0.0.1';
fg.remotePort = 5502;

%% Open Simulink model
open_model('QuadcopterSimModel_Loiter_FTC')
m='QuadcopterSimModel_Loiter_FTC';
set_param([m '/To FlightGear'],'Commented','on');           % Coder 의존 → 주석처리
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');                  % 미연결 포트 경고 끄기
