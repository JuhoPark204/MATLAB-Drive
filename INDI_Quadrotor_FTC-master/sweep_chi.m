%% sweep_chi.m — 단일 로터 고장 안정화 chi 자동 탐색
% 단일고장(DRF_enable=0) + 화물0 에서 par.chi 를 여러 값으로 쓸어가며,
% 고장(5초) 후 드론이 살아남는(자세 발산 안 하는) chi 가 있는지 찾는다.
%
% 사용법:  sweep_chi      % run.m 대신 이거 실행

clear all
addpath(genpath('Bebop2Model')); addpath('functions');
addpath('INDI_FTC'); addpath('sim_modules');
simParams;
controlParams;
createAllBusses;

par.DRF_enable = 0;     % 단일 로터 고장
par.adaptive   = 0;     % 기본 제어기 (화물 없음)
Tsim = 10;              % 0~10초 (고장 5초 → 5초 관찰)

chiList = [20 40 60 75 90 105 120 140 160];

fprintf('\n  chi[deg] |   결과    | 최종z[m] | 최대하강[m] | 기울기[deg]\n');
fprintf('  (생존 기준: 최종 고도가 시작 근처 |z|<3m. 기울기는 정보용)\n');
fprintf('-------------------------------------------------------------------\n');
best = []; bestTilt = inf;
for c = chiList
    par.chi = c;
    assignin('base','par',par);
    ok = true;
    try
        sim('frame.slx', Tsim);
    catch
        ok = false;          % 시뮬 에러(보통 추락→시각화 실패) = 추락
    end
    if ok && exist('att','var') && exist('pos','var')
        t = att.Time;
        A = tsmat(att);                                  % N x 3 (roll,pitch,yaw)
        P = tsmat(pos);                                  % N x 3
        post = t >= 5.5;                                 % 고장 후 구간
        tilt = max( sqrt(A(post,1).^2 + A(post,2).^2) )*57.3;  % 최대 기울기(정보용)
        zf      = P(end,3);                              % 최종 z (Down, +면 떨어짐)
        maxfall = max(P(post,3));                        % 최대 하강량
        % ★ 생존 기준 = 고도 (기울기 아님): 최종고도가 시작 근처면 생존
        surv = isfinite(zf) && abs(zf) < 3;
        if surv
            verdict = '생존 ✓';
            if maxfall < bestTilt; bestTilt = maxfall; best = c; end
        else
            verdict = '추락 ✗';
        end
        fprintf('  %6.0f   |  %-8s | 최종z %7.2f | 최대하강 %7.2f | 기울기 %6.1f\n', ...
                c, verdict, zf, maxfall, tilt);
    else
        fprintf('  %6.0f   |  추락(에러)  |     -        |      -         |    -\n', c);
    end
end
fprintf('---------------------------------------------------------------\n');
if ~isempty(best)
    fprintf('>> 생존하는 chi 발견! chi = %.0f deg (최대하강 %.2f m)\n', best, bestTilt);
    fprintf('   → 옛 코드도 단일고장 가능할 수도. 재검토 필요.\n\n');
else
    fprintf('>> 생존하는 chi 없음 (전부 |z|>=3m 추락) → 옛 코드는 단일고장 진짜 불가 확정.\n\n');
end

%% 로컬 함수
function X = tsmat(ts)
    N = numel(ts.Time); d = squeeze(ts.Data);
    if size(d,1)==N; X = d; elseif size(d,2)==N; X = d.'; else; X = d; end
end
