# ReKep-nav — 2D 주방 네비게이션을 위한 제약 최적화 계획기

[ReKep](https://rekep-robot.github.io/)(관계 키포인트 제약)의 **알고리즘 구조**를
RoboCasa 주방의 2D 네비게이션으로 옮긴 것이다. 원본은 OmniGibson + 로봇 팔 6D
조작용이라 그대로 돌지 않는다.

VLM 은 **한 번의 응답**으로 셋을 말한다 — 주행을 몇 단계로 나눌지, 각 단계가 어디서
끝날지, 그 길이 무엇을 지켜야 할지. 사이를 잇는 경로는 최적화기가 푼다.

> **주의: 이것은 `--subgoals solver` 의 설명이고, 지금까지 결과를 낸 실행이 쓴 모드가
> 아니다.** 360 에피소드 실행(`rk720`)은 `--subgoals legs` 로 돌았고, 그쪽은 구간마다
> 다시 묻는 PIVOT 방식이다. 두 모드가 무엇이 다른지는 아래 「모드」 절에 적었다.
> 어느 모드로 돈 결과인지 밝히지 않은 수치는 읽지 말 것.

```python
# STAGES: 2
# 1: Get past the human with room to spare
# 2: Drive to the star

def stage1_subgoal_constraint1(state, keypoints):
    """Finish this phase already clear of the human."""
    return standoff_cost(state, keypoints['human'], 0.7)

def stage1_path_constraint1(traj, keypoints):
    """A human may shift; hold the margin the whole way past."""
    return clearance_cost(traj, keypoints['human'], 0.7)

def stage2_subgoal_constraint1(state, keypoints):
    """End at the star."""
    return progress_cost(state, keypoints['GOAL'], 0.3)

stop_points = [7, -1]
```

`stop_points` 가 단계별 정차 지점(그림의 원 번호)이고 `-1` 은 "목표에서 끝난다" 다.
원본이 `grasp_keypoints = [...]` 로 단계별 요약을 돌려주는 것과 같은 꼴이다.

## 어디에 붙는가

이 저장소는 계획기만 담는다. 환경·컨트롤러·평가·판정문은
[VoxPoser fork](https://github.com/JisuHann/VoxPoser) 의 `run_tasks` 를 그대로 쓴다 —
`external_planner` 훅 자리에 끼워지므로, 다른 정책과 **같은 잣대로 채점**된다.

```
run_tasks(..., external_planner=RekepPlannerHook(cfg))
        ^
   여기 한 곳만 갈아끼운다
```

훅은 에피소드당 **한 번** 불린다(`max_plan_iter: 1`). 그래서 ReKep 의
"풀고 -> 조금 실행 -> 다시 관측" 루프를 시뮬레이터로 닫을 수 없고, 같은 루프를
**가상 상태로 훅 안에서** 돈다 — 마지막으로 내놓은 waypoint 가 "현재 상태" 다.

## 구조

```
run_rekep.py          진입점
config.yaml           solver 예산 · 스위치
src/
  stops.py            정차 지점 후보를 화면 전체에서 뽑아 번호를 붙인다
  stages.py           응답 파싱 · 검증 · 끝점 주입
  loop.py             가상 상태 반복 + backtracking
  subgoal_solver.py   단계가 끝날 자세 (x, y, yaw)
  path_solver.py      그 사이 제어점
  costs.py            충돌 · 일관성 · 경로길이 · 회전 · 통과불가 · 제약 위반
  sdf.py              occupancy -> 부호거리장
  interp.py           정규화 · 보간 · 각도 wrap
adapters/
  planner_hook.py     external_planner 계약 구현 — 유일한 접점
lib/                  측정 코드 (반경 패드 · 표면거리 · AST 검증기)
prompts/              VLM 질의 템플릿
tests_*.py            시뮬레이터 없이 도는 검증
```

## 스위치 두 개

둘 다 끄면 원본에 가장 가깝다. 켜면 프롬프트에 해당 문단이 붙고 어휘가 열리므로,
**차이가 그 하나로만 남는다**.

| 스위치 | 끄면 (기본) | 켜면 |
|---|---|---|
| `--dynamics` | 기하만 푼다 (원본에는 v/a/J 가 없다) | `speed_cost` · `accel_cost` · `jerk_cost` 를 단계마다 쓸 수 있다 |

## 모드 — `--subgoals`

네 가지가 있고, **무엇이 경로를 정하는지가 근본적으로 다르다.** 결과를 인용할 때는
반드시 모드를 함께 적는다.

| 모드 | 끝점을 정하는 주체 | 구간 경로 |
|---|---|---|
| `solver` (기본) | 최적화기 — 제약을 비용으로 풀어서 | 최적화기 |
| `stops` | VLM — 전체 그림에서 번호를 골라 | 최적화기 |
| `pivot` | VLM — 고리 위 후보를 좁혀 가며 번호를 골라 | 최적화기 |
| `legs` | VLM — **구간마다** 다시 보고 번호를 골라 | 통과하면 직선, 아니면 최적화기 |

`legs` 는 구간마다 질의를 새로 하고, 그 구간의 keypoint 만 보여 준다. 전체 장면을
한 번에 주면 지금 가는 길과 무관한 물체까지 제약을 걸기 때문이다. 대신 **모델이 쓴
subgoal 제약은 버린다** — 끝점은 고른 번호가 정하고, 모델의 제약은 "가는 길"에만
쓴다(`legs.py:491`).

### `legs` 로 돈 실행에서 실제로 일어난 일 (`rk720`, 360 에피소드)

문서가 약속하는 것과 다르므로 실측을 남긴다.

| | 값 |
|---|---|
| VLM 이 번호로 고른 정차점 | 753 (후보 이미지 질의 2,207 회) |
| 최적화기 없이 **직선**으로 처리된 구간 | 906 / 1,085 = **83.5%** |
| 최적화기가 **한 번도 안 돈** 에피소드 | 217 / 360 = **60.3%** |
| 폐기된 모델 subgoal 제약 | 1,939 |
| solver 반복 횟수 | 179 구간 **전부 1회**, backtrack 0 |

즉 이 모드에서 ReKep 의 제약 최적화는 소수 지분이다. `max_iterations: 40`,
`max_backtracks: 8` 은 설정만 되어 있고 한 번도 쓰이지 않았다.

### 직선 게이트의 보정이 어긋나 있다

구간을 직선으로 갈지 말지는 두 조건으로 정한다 — SDF 여유가 충분한가, 그리고 모델이
쓴 path 제약이 그 직선을 거부하는가(`legs.py:534`). 후자가 문제다.

`clearance_cost` 는 미터가 아니라 정규화 값을 돌려준다:

```
값 = (eff - d) / eff,   eff = 모델이 요구한 margin + 물체 반경 패드
tol 0.10  ->  d < 0.9 x (margin + 반경) 이면 거부
```

반경 패드는 `constraints.py` 의 표(`crawling_baby 1.38`, `human 0.52`, …)에서 온다.
그래서 `clearance_cost(traj, crawling_baby, 0.75)` 는 **중심거리 1.92 m 안쪽을 전부
거부한다** — 채점 경계는 0.6 m 인데도.

거부된 111 구간의 직선을 기하로 다시 재 보면:

| | 값 |
|---|---|
| 실제로 채점 경계를 침범했을 것 (거부가 옳았다) | 41 / 111 |
| **침범하지 않았을 것** | **70 / 111** |
| 채점 대상이 아닌 물체 때문에만 거부 | 22 |
| 요구 margin 은 이미 만족했는데 반경 패드로만 거부 | 39 / 102 |

거부를 유발한 대상에 `microwave`(6), `sink_main`(4), `knife`, `main_door` 가 있고,
**모델이 자기가 가려는 정차점(`STOP1`·`STOP2`)에 회피 제약을 건 경우도 3 회** 있다.
반대로 직선으로 통과시킨 906 구간 중 실제 위반은 12 건(1.3%)뿐이다 — **놓치는 쪽은
적고 과잉 거부가 많다.** 안전한 직선 1.7 개를 버려야 위험한 직선 1 개를 잡는 비율이다.

고치려면 임계를 모델이 부른 값이 아니라 **티어 경계**와 비교해야 한다.

## 원본과 다른 점

옮기면서 생긴 차이는 `DESIGN.md` 에 전부 적었다. 요약하면:

- **경로길이 항을 직선거리로 나눈다.** ReKep 의 4.0 은 0.55 m 작업공간 기준이라
  6 m 주방에서는 충돌 항을 압도한다 — 벽을 뚫고 직선으로 가는 쪽이 비용상 이득이 됐다.
- **"통과불가" 비용을 더한다.** 키포인트가 물체 중심이라 `sink`·`stove` 의 점은 조리대
  안쪽이다. 팔은 물체 위로 뻗을 수 있지만 바퀴 베이스는 그 자리에 있을 수 없다 —
  ReKep 의 IK 도달불가가 하던 역할이다.
- **진행 방향 변화를 벌한다.** 홀로노믹 베이스는 yaw 와 진행 방향이 분리돼 있어
  회전 비용으로는 공간적 지그재그가 잡히지 않는다.
- **DINOv2 제안 단계를 쓰지 않는다.** 원본의 그 단계는 물체를 찾는 것이 아니라 이미
  알려진 마스크를 부위로 쪼개는 것인데, 주방에 돌리면 후보 185 개 중 실제로 피해야 할
  대상은 5% 뿐이다 (조리대·수납장에 56%). 대신 통행 가능한 칸에서 정차 지점 후보를
  뽑아 번호로 고르게 한다.

가중치는 그 밖에 원본 값을 그대로 쓴다. 바꾸면 "구조를 옮겼는데 결과가 다르다" 와
"가중치를 손봤더니 결과가 다르다" 를 가를 수 없다.

## 돌리는 법

VoxPoser fork 와 RoboCasa 벤치마크가 있는 트리 안에서, `policy/ReKep/` 자리에 두고:

```bash
# 기본값(solver) — 제약을 비용으로 풀어서 끝점까지 최적화기가 정한다
python3 run_rekep.py -m 'Qwen/Qwen3-VL-8B-Instruct' -p 8003 \
    -o outputs/run1 --layout-ids 0 --style-ids 3 \
    NavigateKitchenCatBlockingRouteA

# rk720 이 실제로 쓴 조합 — 구간마다 다시 묻고, 통과하면 직선으로 간다
python3 run_rekep.py -m 'Qwen/Qwen3-VL-8B-Instruct' -p 8003 \
    -o outputs/run2 --subgoals legs --straight-first \
    --layout-ids 0 --style-ids 3 \
    NavigateKitchenCatBlockingRouteA
```

실행 설정은 출력 디렉터리의 `rekep_config.json` 에 저장된다. 결과를 다시 볼 때는
그 파일부터 확인한다 — 명령줄은 어디에도 남지 않으므로, 어느 모드로 돈 실행인지
알 수 있는 유일한 기록이다.

시뮬레이터 없이 도는 검증만 보려면:

```bash
python3 tests_r1.py       # SDF 부호 · 좌표 왕복 · 비용 항 단위
python3 tests_r2.py       # 두 solver
python3 tests_r3.py       # 루프 · backtracking
python3 tests_dyn.py      # 동역학 스위치
python3 tests_partial.py  # 함수 하나가 틀려도 나머지를 살리는가
```

시험은 저장된 덤프를 쓴다. 다른 기계에서는 `REKEP_FIXTURES` 로 위치를 알려 준다.
