%% sweep_payload_IRIS.m — IRIS baseline(B)에 화물별 영향 (문제 입증)
% 제어기는 마른 IRIS 기준으로 1회 생성(고정) = baseline B (화물 모름).
% 화물(0/200/500/1000g)만 plant에 바꿔가며 단일고장 후 강하/생존 비교.
%
% 사용법:  sweep_payload_IRIS

clear; clc;
addPathFtc();

%% 고정 설정
failure_time_mot_1=1000; failure_time_mot_2=1; failure_time_mot_3=1000; failure_time_mot_4=1000;
simin = automatedStickCommands();
copter = copterLoadParams('copter_params_IRIS');   % ★ 변수명 'copter' 필수 (함수가 evalin으로 읽음)
envir = envirLoadParams('params_envir','envir',0);
IC.omega_Kb=[0;0;0]; IC.q_bg=euler2Quat([0;0;0]); IC.V_Kb=[0;0;0]; IC.s_Kg=[0;0;0];
IC.omega_mot=[1;1;1;1]*500;
grnd=groundLoadParams('params_ground_default');
pos_ref.lat=37.6117; pos_ref.lon=-122.37822; pos_ref.alt=15;
fg.remoteURL='127.0.0.1'; fg.remotePort=5502;

% ★ 제어기: 마른 드론 기준 (baseline B, 모든 화물에 동일하게 고정)
fm_loiter = lindiCopterAutoCreate(copter,'AgilityAtti',0.5,'AgilityPos',0.5, ...
                                  'FilterStrength',0,'CntrlEffectScaling',1);
fm_loiter.psc.inditype=1;
copter_dry = copter;                               % 제어기 생성 후 마른 드론 보관

m='QuadcopterSimModel_Loiter_FTC';
load_system(m);
set_param([m '/To FlightGear'],'Commented','on');
set_param([m '/Environment/From FlightGear'],'Commented','on');
set_param(m,'UnconnectedInputMsg','none');   % FlightGear 주석처리로 생기는 미연결 포트 경고 끄기

%% 화물 스윕
payloadList = [0 0.2 0.5 1.0];
poffset     = 0.10;

figure(40); clf; hold on; grid on; cols=lines(numel(payloadList));
fprintf('\n 화물[g] | 총질량[kg] | 최대강하[m] | 회복시간[s] | 생존\n');
fprintf('-----------------------------------------------------------\n');
for j=1:numel(payloadList)
    pm = payloadList(j);
    copter = copter_dry;                                   % 마른 드론에서 시작
    copter.body.m = copter.body.m + pm;                    % 총질량
    copter.body.I = copter.body.I + pm*poffset^2*diag([1 1 0]);  % 평행축
    copter.config.CoG_Pos_c = copter.config.CoG_Pos_c + [0;0;poffset]; % 무게중심 아래로
    assignin('base','copter',copter);
    try
        out = sim(m,'StopTime','18');
        [drop,trec,ok] = pmetrics(out);
        ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
        plot(ts,up,'Color',cols(j,:),'LineWidth',1.5,'DisplayName',sprintf('%dg',pm*1000));
        v='추락 ✗'; if ok, v='생존 ✓'; end
        fprintf('  %4.0f   |   %.3f    | %9.2f | %9.2f | %s\n', pm*1000, copter.body.m, drop, trec, v);
    catch ME
        fprintf('  %4.0f   |   %.3f    |  에러: %s\n', pm*1000, copter.body.m, ME.message);
    end
end
fprintf('-----------------------------------------------------------\n');
fprintf('>> 화물 클수록 강하 깊어지면 = "문제" 입증. 1000g 추락하면 = 적응 필요성 강력.\n\n');
xline(1,'k--','고장'); xlabel('시간 [s]'); ylabel('고도 [m]');
title('IRIS baseline(B): 화물별 단일고장 후 고도'); legend('show','Location','southeast');

%% ---- 지표 ----
function [drop,trec,ok]=pmetrics(out)
    ts=out.s_g.Time(:); up=-squeeze(out.s_g.Data(3,1,:));
    post=ts>=1; upp=up(post); tp=ts(post);
    [mn,idx]=min(upp); drop=-mn;
    after=idx:numel(upp); rec=find(abs(upp(after))<0.5,1);
    if isempty(rec), trec=inf; else, trec=tp(after(rec))-1; end
    ss=ts>=(ts(end)-3); ok=isfinite(trec) && max(abs(up(ss)))<2;
end
