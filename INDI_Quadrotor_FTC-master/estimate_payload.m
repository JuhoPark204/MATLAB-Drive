%% estimate_payload.m
% 화물 무게 추정 (호버 구간 후처리 방식, 2단계 1차 프로토타입)
%
% 사용법:  run        % 먼저 시뮬 돌려서 accB, wRotorMeas 등을 워크스페이스에 채움
%          estimate_payload
%
% 원리: 수평 호버에서 IMU 비력 accB_z = -T/m 이므로  m = T / |accB_z|.
%       추력 T는 측정 로터속도로 독립 계산 (T = Σ k0·wᵢ²).
%       화물무게 = 추정 총질량 m_est - 드론 자체무게 mDry.

%% ---- 추정 설정 (필요시 조정) ----
hoverStart = 2.0;        % 추정 구간 시작 [s] (초기 과도응답 지난 뒤)
hoverEnd   = 2.5;        % 추정 구간 끝   [s] (가장 안정된 호버 초반만 사용)
k0   = 1.9035e-6;        % 프로펠러 추력계수 [추정기가 아는 값, simVars.aero.k0]
g    = 9.8124;           % 중력 [simVars.g]
mDry = 0.375;            % 드론 자체무게 [kg, 이미 아는 값]

%% ---- 신호 추출 (차원 자동정렬: [시간 x 채널]) ----
ta = accB.Time(:);       A = tsmat(accB);        % A: N x 3
az = A(:,3);                                      % 수직 비력 [m/s^2], 호버 ≈ -g
tw = wRotorMeas.Time(:); W = tsmat(wRotorMeas);  % W: M x 4

T_w = sum(k0 .* W.^2, 2);                  % 총추력 [N] (tw 시간축)
T   = interp1(tw, T_w, ta, 'linear', 'extrap');   % accB 시간축 ta 로 정렬

%% ---- 호버 구간 마스크 (가용 데이터 범위로 클램프) ----
tEnd = min(ta(end), tw(end));
hEnd = min(hoverEnd, tEnd);
idx  = (ta >= hoverStart) & (ta <= hEnd);
if nnz(idx) < 5
    error('호버 구간 데이터 부족. hoverStart/End(%.1f~%.1f) 또는 데이터 끝(%.2fs) 확인.', ...
          hoverStart, hoverEnd, tEnd);
end

%% ---- 질량 역산: m = T / |accB_z| ----
m_inst = T(idx) ./ abs(az(idx));
m_est  = mean(m_inst);
payload_est = m_est - mDry;

%% ---- 결과 출력 ----
fprintf('\n===== 화물 무게 추정 결과 =====\n');
fprintf(' 추정 구간       : %.1f ~ %.1f s\n', hoverStart, hEnd);
fprintf(' 추정 총질량     : %.4f kg\n', m_est);
fprintf(' 드론 자체무게   : %.4f kg\n', mDry);
fprintf(' >> 추정 화물무게: %.4f kg  (= %.0f g)\n', payload_est, payload_est*1000);
fprintf(' 구간 표준편차   : %.4f kg\n', std(m_inst));
if exist('simVars','var') && isfield(simVars,'payload')
    truth = simVars.payload.mass;
    err   = payload_est - truth;
    if truth > 1e-6
        fprintf(' [참값 %.0f g | 오차 %+.0f g | 오차율 %.1f%%]\n', ...
                truth*1000, err*1000, 100*abs(err)/truth);
    else
        fprintf(' [참값 %.0f g | 오차 %+.0f g | (참값 0 → 오차율 생략)]\n', ...
                truth*1000, err*1000);
    end
end
fprintf('================================\n');

%% ---- 시각화 ----
figure(200); clf;
subplot(2,1,1);
plot(ta, T); hold on; grid on;
xline(hoverStart,'g--'); xline(hEnd,'g--');
xline(5,'r--','고장','LabelVerticalAlignment','bottom');
ylabel('총추력 T [N]'); xlabel('t [s]'); title('로터속도로 계산한 추력');
subplot(2,1,2);
plot(ta(idx), m_inst, '.-'); hold on; grid on;
yline(m_est,'r-','m_{est}');
ylabel('순간 추정 총질량 [kg]'); xlabel('t [s]'); title('질량 추정 (호버 구간)');

%% ---- 로컬 함수: timeseries Data를 [N x 채널]로 정렬 ----
function X = tsmat(ts)
    N = numel(ts.Time);
    d = squeeze(ts.Data);
    if size(d,1) == N
        X = d;                 % 이미 [시간 x 채널]
    elseif size(d,2) == N
        X = d.';               % [채널 x 시간] -> 전치
    else
        error('tsmat: 차원 해석 실패 (size=%s, N=%d)', mat2str(size(d)), N);
    end
end
