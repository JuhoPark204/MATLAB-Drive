%% Controller related Parameters
% Controls
par.freq = 500; % control frequency

%
%   (1)<-- b -->(2)
%      \       / ^
%       \     /  |
%       /     \  | l
%      /       \ v
%   (4)         (3)

par.fail_id = [3];      % index of the failured propeller
par.DRF_enable = 1;     % failure of two diagonal rotors?
par.fail_time = 5.0;    % moment failiure occurs (0~5s 멀쩡한 호버 → 무게추정 구간)

% drone parameters
par.b = 0.1150;     % [m]
par.l = 0.0875;
par.Ix = 0.0014;    % [kg m^2]
par.Iy = 0.0013;
par.Iz = 0.0025;
par.mass = 0.375;   % [kg]  (드론 자체무게 = 기본값)
par.g = 9.81;

%% === 적응형 FTC 스위치 (3단계) ============================
%  par.adaptive = 0  → 제어기 B : 기존 FTC (질량 0.375 고정, 화물 모름)
%  par.adaptive = 1  → 제어기 C1: 추정 화물무게를 질량·관성에 반영
%  (게인/허용기울기는 아직 안 건드림 → H1 "질량만 맞춰도 사나?" 검증용)
par.adaptive       = 1;       % 0 = 기존(B),  1 = 적응(C1)
par.payload_est    = 0.207;   % [kg] estimate_payload 결과 입력 (호버에서 추정한 값)
par.payload_offset = 0.05;    % [m]  화물 하단 오프셋 (기지 고정값, simParams와 동일)

if par.adaptive
    par.mass = par.mass + par.payload_est;                       % 질량 보정
    par.Ix   = par.Ix   + par.payload_est*par.payload_offset^2;  % 평행축 (롤축)
    par.Iy   = par.Iy   + par.payload_est*par.payload_offset^2;  % 평행축 (피치축)
    % par.Iz : 요축은 z방향 점질량 오프셋의 영향 없음 → 그대로
end
% ==========================================================

par.k0 = 1.9e-6;    % propeller thrust coefficient
par.t0 = 1.9e-8;    % torque coefficient
par.w_max = 1200;   % max / min propeller rotation rates, [rad/s]
par.w_min = 0;

%% INDI reduced att control
par.chi = 105;          % output scheduling parameter, [deg].
par.pos_z_p_gain = 10;   % altitude control pd gains
par.pos_z_d_gain = 6;
par.axis_tilt = 0.0;    % primary axis tilting param, 0 ~ 0.2,  
                        % must be 0 for double rotor failure cases

par.att_p_gain = 200;   % attitude control pd gains 
par.att_d_gain = 30;
par.t_indi = 0.02;      % low-pass filter time constant, [s]

% Yaw control
par.YRC_Kp_r = 5.0;
par.YRC_Kp_psi = 5.0;

% position control
par.position_maxAngle = 10/57.3;    % maximum thrust tilt angle [rad]  
par.position_Kp_pos = [1.0, 1.0, par.pos_z_p_gain];  % position control gains
par.position_maxVel = 10;           % maximum velocity
par.position_intLim = 5.0; 
par.position_Ki_vel = [0.0, 0.0, 0.0];  % velocity gains
par.position_Kp_vel = [2.0, 2.0, par.pos_z_d_gain];
