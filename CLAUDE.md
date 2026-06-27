# 프로젝트 컨텍스트 — 화물 드론 로터 고장 적응 제어 (Claude Code 인수인계)

> 이 파일은 Claude Code가 매 세션 자동으로 읽습니다. **다른 컴퓨터에서 새 채팅으로 이어가도 이 맥락이 유지됩니다.** (작업 폴더: `c:\Users\pc\MATLAB Drive`)

---

## 0. 작업 스타일 (반드시 지킬 것)

- **항상 caveman full 모드로 응답.** 한국어 대화 시 한국어 caveman 사용. (단, **보고서·발표 문서는 깔끔한 정식 한국어** — 교수님께 보여줄 수 있음)
- **성과가 날 때마다 보고서 갱신** (오류·시행착오 말고 성과만). 대상 2개를 **항상 동기화**:
  - `PROGRESS.md` (마크다운), `report.html` (다크테마 HTML) — 둘 다 MATLAB Drive 루트
  - 각 실험은 **가설 / 방법 / 검증결과** 구조로 작성.
- **실험 상태마다 git 커밋** (재현용, main 브랜치). 푸시는 요청 시에만.
- 발표용 문서: `발표_2026-06-16.md` (용어 수준 7 = 전문용어 쓰되 처음 나올 때 괄호로 풀어줌).

---

## 1. 연구 개요

- **과제:** "화물 장착 드론의 로터 고장 대응 적응형 제어기 설계"
- **연구원:** 박주호(호서대 학부, 학번 20210972) | **지도교수:** 임헌국 | **기간:** 2026-04-01 ~ 2026-11-30 (산학)
- **목표(재정립됨):** 무게 모르는 하부 화물이 실렸을 때 → 드론이 **호버 중 화물 질량 추정** → 추정값으로 제어기 적응 → **로터 1개 완전 고장** 시 고장 후 **회복 성능 개선**.
  - ⚠️ 원래 "추락을 생존으로 바꾼다"였으나 물리적으로 불가 판명(아래 3번) → **"생존 범위 내 성능 개선"**으로 재정의.
- **환경:** MATLAB/Simulink (R2026a) 전용, 하드웨어 없음.

---

## 2. 코드·실행 방법

- **기반 코드:** `Passive-FTC-Quadrotor/` (Beyer et al. 2023, iff-gsc, INDI passive FTC, 단일 완전 로터 고장용).
- **드론:** IRIS (1.37 kg) — `copterLoadParams('copter_params_IRIS')`.
- **모델:** `QuadcopterSimModel_Loiter_FTC`. 모터 2번이 t=1s에 영구 완전 고장 → 스핀 안정화로 회복.
- **실행 시 필수 처리** (스크립트에 이미 포함):
  ```matlab
  set_param([m '/To FlightGear'],'Commented','on');           % FlightGear는 MATLAB Coder 필요(없음)
  set_param([m '/Environment/From FlightGear'],'Commented','on');
  set_param(m,'UnconnectedInputMsg','none');                   % 미연결 포트 경고 끄기
  ```
- **제어기 생성 패턴:** `fm_loiter=lindiCopterAutoCreate(copter,'AgilityAtti',a,'AgilityPos',p,'FilterStrength',0,'CntrlEffectScaling',1); fm_loiter.psc.inditype=1;`
  - ★ 이 함수는 `evalin('caller','copter')`로 읽으므로 **변수명이 반드시 `copter`** 여야 함.
- **시뮬:** base 워크스페이스에 simin/envir/IC/grnd/pos_ref/fg/failure_time_mot_* + fm_loiter + copter 있어야 `sim(m)` 동작. (우리 스크립트는 "스크립트 + 끝에 로컬함수" 패턴 — 최상위 코드가 base에서 돌아야 sim이 읽음.)
- **시각화:** `Passive-FTC-Quadrotor/animate_drone.m` (FlightGear 불필요).
- **git 주의:** `Passive-FTC-Quadrotor/`는 .gitignore 대상이지만 우리 스크립트는 `git add -f`로 추적 중(중첩 .git 제거함).

---

## 3. 지금까지의 핵심 결과 (= 실제 연구 성과)

1. **화물 질량 추정 (성공, 오차 ≤2%):** `총질량 = 무화물질량 × Σω²_호버 / Σω²_무화물`. 무화물로 1회 보정 후, 고장 직전 호버(t=−0.5~0.8s) 모터 회전수로 역산. → `test_estimate_IRIS.m`. 결과 0/200/500/1000g 전부 오차 2% 이내.
2. **추력 물리한계 (원래 목표 불가 판명):** IRIS는 로터 1개 잃으면 0g에서도 추력 ~97% 사용. 500g → 모터 포화(천장 ~775 rad/s, 명령 74% 포화) → **영구 추락**(어느 고도서 시작해도 강하량=시작고도). 생존 한계 ~350g. **적응(질량/게인/고도)으로 못 살림 = 물리 한계.** 실제 배송드론은 자체무게 20~30% 화물·TWR~2, 내결함성은 헥사/옥토 사용.
3. **방향 재정립:** 스톡 IRIS + 화물 **≤25%(≤340g)** = 현실적 + 생존범위. 목표 = 그 안에서 성능 개선.
4. **화물 물리모델 검증 (엄밀):** 하부 화물 = **질량 + 롤/피치 관성**만. 평행축 관성은 **환산질량 μ=m_dry·pm/M** 사용(전체 pm 아님). **z-CoG 이동은 자유비행 회전동역학에 무영향**(추력∥z축이라 모멘트 0, 중력은 CoG에 작용, 피벗 없음). plant은 어차피 `CoG_Pos_c` 안 씀(제어기만 씀). → 모델 물리적으로 타당.
5. **적응 비교 (가/나/다 프레임 확정):** 제어기 3종 — 가(기존,미반영) / 나(추정질량·관성 반영,게인0.5/0.5) / 다(추정반영+화물별 최적게인).
   - **비용 J 재정의(강하 포함):** 4지표 [최대강하·회복시간·말기틸트·최종오차]를 **(가) 기준 정규화 가중평균** (가=1.0, 낮을수록 우수). 옛 J(강하 미포함)는 과공격 게인을 못 걸러냄 → 강하 추가가 핵심.
   - **헤드라인 비교 = (가) vs (다)** 아니라 실제론 가/나/다 3단. "실제 드론도 무게 앎(호버추력 학습)" → 나=현실 기준선. 기여 = 무게추정 아니라 **게인 적응**.
   - **게인 탐색 = 베이지안 최적화(BO, `bayesopt`)**. 격자탐색(`optimize_gain_IRIS`) 대체. sim 결정론적(노이즈 off) → `IsObjectiveDeterministic=true`, eval당 1회.
   - **340g BO 결과 (확정):** 최적게인 **[aA 0.437, aP 0.724]**. 다 J=0.51 (가 대비 49%↓, 나 0.78도 이김). **다가 4지표 전부 최우수** (강하 6.25<6.37, 회복 7.2, 틸트 4.8, 오차 0.042). aA를 낮추고(0.44) aP 높이는(0.72) 게 정답 — 자세게인 과하면 강하 폭증.
   - ★ 옛 결론("결합 J 13~20, 72%개선")은 옛 J(강하 미포함) 기준 — 폐기. 현 기준 = 위 BO 결과.
   - ⚠️ sim 결정론 확인됨(같은 입력=같은 출력). "편차"는 노이즈 아닌 게인·화물 민감도. 노이즈 켜면(미구현, 우리가 추가) BO `false`+eval당 K회 평균 필요.

---

## 4. 주요 스크립트 (`Passive-FTC-Quadrotor/scripts/`)

| 스크립트 | 역할 |
|------|------|
| `test_estimate_IRIS.m` | 화물 질량 추정 검증 |
| `diag_thrust_IRIS.m` | 추력 포화 진단 (화물 스윕) |
| `diag_altitude_IRIS.m` | 영구추락 vs 과도 판별 (시작고도 스윕) |
| `bench_perf_IRIS.m` | baseline 성능 벤치마크 (화물 0~340g) |
| `compare_perf_IRIS.m` | 적응 전략 비교 (B/C1/게인변형) |
| `check_payload_phys_IRIS.m` | 화물 물리모델 검증 (질량/관성/CoG 분리) |
| `optimize_gain_IRIS.m` | (구) 격자탐색 게인. BO로 대체됨 — 비교 baseline용 |
| `optimize_gain_bo_IRIS.m` | ★ **베이지안 최적화 게인 탐색** (강하 포함 J, 가/나/다). 340g 결과 [0.437,0.724]. `PAYLOADS` 벡터로 주면 스케줄용 |
| `demo_compare3_IRIS.m` | 가/나/다 3제어기 비교 (4지표 막대+궤적+종합J). `BESTGAIN`=BO결과 |
| `final_BC_IRIS.m` | 헤드라인 B vs C (추정→적응 end-to-end) |
| `demo1~6_*_IRIS.m` | **교수님 시연용 개별 데모** (1:추정막대 2:추력한계궤적 3:baseline 4:적응J막대+2×2 5:기존vs적응궤적@340g 6:3D애니). demo5·6 상단 `BESTGAIN` = optimize_gain 최적값으로 갱신 필요 |

- 각 스크립트에 `mkplant`(plant 화물 주입)·`mkctrl`(제어기 생성) 로컬함수 반복. **`mkplant`는 환산질량 버전이 정확**: `M=m_dry+pm; mu=m_dry*pm/M; I += mu*poff^2*diag([1 1 0])`.

---

## 5. 현재 상태 & 다음 할 일

**완료:** 무게추정 / 추력한계 규명 / 방향 재정립 / 물리모델 검증 / 적응 비교(200·300·340g) / 발표자료.

**다음 (TODO):**
- [x] ~~반복 평균~~ → sim 결정론 확인됨, 불필요 (노이즈 켜면 다시 필요).
- [x] 강하 포함 J 재정의 + BO로 340g 최적게인 확정 [0.437,0.724].
- [ ] **게인 스케줄** — `optimize_gain_bo_IRIS`에 `PAYLOADS=[0.2 0.3 0.34]` 줘서 화물별 BO → payload→gain 보간표.
- [ ] **센서 노이즈 추가** (오픈소스 미구현 = 우리 기여). (A)추정 RPM노이즈 후처리 쉬움 / (B)제어루프 블록추가 / (C)`imuSensor`. 계획서 요구사항.
- [ ] **발표자료·report.html J 수치 갱신** (옛 J 13.1 등 → 새 정규화 J 0.51, BO 방법).
- [ ] PID(A) 비교 추가 시 계획서 4단계 완전 충족.

**산출 문서:** `PROGRESS.md`, `report.html`, `발표_2026-06-16.md` (모두 루트). 최신 결론 반영됨.

---

## 6. 기타 환경 메모

- 설치 안 된 툴박스 회피: 일부 함수는 toolbox-free 자체 구현 사용(예전 `INDI_Quadrotor_FTC-master/functions/eul2quat.m` 등). 새 코드는 자체 의존성으로 동작.
- 옛 베이스 `INDI_Quadrotor_FTC-master`는 이중-대향로터 전용이라 단일고장 불가 → 폐기. (참고용으로 폴더만 남아있음)
- 마크다운 린트 경고(MD060/MD032 등)·맞춤법 경고는 미관이라 무시.
