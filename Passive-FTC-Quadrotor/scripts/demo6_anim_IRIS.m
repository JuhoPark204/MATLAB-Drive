%% demo6_anim_IRIS — 3D 애니메이션 (스핀 안정화 회복, 빨강=고장 모터2)
% 화물 장착 → 추정 → 적응 제어로 고장 후 회복하는 모습을 3D로 재생.
% 상단 PAYLOAD/USE_ADAPT 만 바꿔 실행.
% 사용법:  demo6_anim_IRIS
clear; clc; addPathFtc();
addpath(fileparts(fileparts(mfilename('fullpath'))));   % repo 루트 (animate_drone.m 위치)
poffset = 0.10;
PAYLOAD   = 0.34;          % 화물[kg]
MODE      = 'full';       % 'baseline'=기존(미반영) | 'model'=(가)질량반영+게인고정 | 'full'=(가)+(나)결합
BESTGAIN  = [0.437 0.724]; % 340g 최적게인 — optimize_gain_bo_IRIS(BO) 결과 ('full'에서만 사용)
PLAYSPEED = 2;             % 재생 배속

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

%% 제어기 구성
cpl=mkplant(copter_dry,PAYLOAD,poffset,true);
if strcmp(MODE,'baseline')
    fm_loiter=mkctrl(copter_dry,0.5,0.5); tag='기존(미반영)';
else
    % 'model'·'full' 공통: 호버에서 화물 질량 추정
    fprintf('추정 중...\n');
    copter=copter_dry; fm_loiter=mkctrl(copter_dry,0.5,0.5);
    out0=sim(m,'StopTime','2'); sumsq_dry=hover_sumsq(out0);
    copter=cpl; fm_loiter=mkctrl(copter_dry,0.5,0.5);
    outh=sim(m,'StopTime','2'); m_est=m_dry*hover_sumsq(outh)/sumsq_dry-m_dry;
    fprintf('  추정 화물 = %.0fg\n', m_est*1000);
    cctrl=mkplant(copter_dry,m_est,poffset,false);    % 추정질량·관성 반영
    if strcmp(MODE,'full')
        fm_loiter=mkctrl(cctrl, BESTGAIN(1),BESTGAIN(2)); tag='(가)+(나) 결합';
    else  % 'model' = (가)
        fm_loiter=mkctrl(cctrl, 0.5,0.5); tag='(가) 질량반영(게인고정)';
    end
end

%% 비행 + 재생
fprintf('비행 시뮬 (%s, %dg)...\n', tag, round(PAYLOAD*1000));
copter=cpl; out=sim(m,'StopTime','25');
fprintf('재생 — 빨강 프로펠러 = 고장 모터2\n');
animate_drone(out, PLAYSPEED);

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
function s=hover_sumsq(o)
    t=o.motor_speed.Time(:); W=sq(o.motor_speed.Data,4);
    idx=t>=-0.5 & t<=0.8; s=mean(sum(W(idx,:).^2,2));
end
function X=sq(D,nch), X=squeeze(D); if size(X,2)~=nch && size(X,1)==nch, X=X.'; end, end
