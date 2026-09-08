# 4로봇 경기 시뮬레이터 성능 작업 기록

작성 2026-09-08 17:54 KST

`./scripts/run_4match.sh` 가 4대의 로봇을 감당하지 못하는 것 같다는 문제에서
출발해, 원인을 측정으로 좁히고 고친 기록. 중간에 틀린 가설도 그대로 남긴다 --
어디서 잘못 짚었는지가 다음에 같은 함정을 피하는 데 쓰인다.

## 한 줄 요약

`./scripts/run_4match.sh` 기본 실행 기준 **0.39x 실시간 -> 0.98x**.
`joint_states` 197 Hz -> 490 Hz 근처, 로봇이 실제로 뛰는 구간에서도
실시간 배율 0.853 -> 0.985.

---

## 0. 먼저 한 일: GameController 를 bringup 에 통합

성능과 무관한 선행 작업.

`run_4match.sh` 에 GameController 노드를 추가해 달라는 요청이었으나,
스크립트가 아니라 `match_bringup.launch.py` 에 넣었다. 이유:

- launch 가 이미 `gc_fanout` 을 띄우고 있어서, GC 앱만 밖에 두면 **먹일 게 없는
  팬아웃**을 켜는 꼴이 된다. 같은 쌍은 같은 파일에 있어야 한다.
- 스크립트에서 띄우려면 `exec ros2 launch` 를 포기하고 PID/trap 을 직접
  관리해야 한다. 두 스크립트의 "ros2 launch owns the process group" 주석이 깨진다.
- `teams:=2` 면 인스턴스가 둘이고 `pub_topic` 이 달라진다. 그건 launch 의 일이다.

팀 번호는 파라미터로 강제하지 않고 창의 콤보박스(ROBIT / ROBIT_RED /
ROBIT_BLUE)에 맡겼다. `TEAM_ROBIT_NUMBER` 는 실기 GC 패킷에서 자기 팀을
골라내는 값이라 시뮬에서 임의 값을 박을 근거가 없다.

| 파일 | 변경 |
|---|---|
| `mujoco_supervisor/launch/match_bringup.launch.py` | `gamecontroller` 인자 추가, `gamecontroller_app` 을 팬아웃 옆에 배치. `teams:=2` 면 두 번째가 `pub_topic:=gamecontroldata_opp` |
| `scripts/run_4match.sh`, `run_8match.sh` | 기본 켜짐, `--no-gamecontroller` |

---

## 1. 문제 제기와 첫 측정

"무조코 루프가 4대의 observation 을 다 처리하기엔 너무 무거운 듯"

### 1.1 관측 처리 가설은 틀렸다

물리 루프를 격리해서 스텝 하나를 구성 요소별로 쟀다 (4로봇, 1 kHz 예산 1000 us):

| 항목 | us/step | 비중 |
|---|---|---|
| `mj_step` | 212 | **77.6%** |
| `apply_impedance_command` x4 | 24 | 8.8% |
| `publish_robot_state` x4 (500 Hz) | 32 | 11.5% |
| `publish_state` x4 (30 Hz) | 6 | 2.0% |
| 합 | **274** | 3.66x 여유 |

**관측 발행은 전부 합쳐 13.5%.** 병목이 아니었다.

### 1.2 진짜 원인 1 - 창이 물리 루프 안에서 그려지고 있었다

실제 스택에서 `/robit_1/joint_states` 실측 (목표 500 Hz):

| 구성 | joint_states | 실시간 배율 |
|---|---|---|
| 뷰어+패널 (당시 스크립트 기본값) | 197 Hz | **0.39x** |
| 패널만 | 353 Hz | 0.71x |
| 뷰어만 | 366 Hz | 0.73x |
| 둘 다 끔 | 463 Hz | 0.93x |

- 머신은 20스레드에 **79% idle**. CPU 부족이 아니었다.
- supervisor 프로세스만 137%, 그 안에서 **spin 84% / physics 44%**.
- `viewer.sync()` 와 `panel.update()` 가 물리 스레드 안에서 호출된다.
- `params.yaml` 은 이미 둘 다 `false` 가 기본이고 "창이 물리 루프의 프레임을
  먹으므로 기본은 꺼짐" 이라고 적혀 있었는데, **`run_4match.sh` 가 둘 다
  `true` 로 덮고 있었다.**

`viewer.sync(state_only=True)` 한 번을 따로 재니 **1394 us** -- 물리 스텝
(218 us)의 6배다. 60 Hz 면 스텝 예산의 8.4%.

### 1.3 진짜 원인 2 - rclpy 실행기 오버헤드

정책이 `joint_ctrl` 을 **1 kHz** 로 쏜다 (`sim2real_node.cpp` 의
`control_rate` 기본값 1000). 4대면 초당 4000건을 supervisor 의 스핀 스레드가
파이썬으로 받는다.

메시지당 비용을 재보니:

| 구독 수 | us/msg | 코어 점유 |
|---|---|---|
| 4개 | 93 | 37% |
| 12개 (실제 supervisor) | 116 | 47% |
| 28개 (패널 켬) | 159 | 64% |

**콜백 본문은 3.5 us 뿐이고 나머지 96%가 실행기.** `spin_once` 마다 노드의 모든
구독·타이머·guard 를 담은 대기 집합을 새로 만들기 때문에, joint_ctrl 과 무관한
구독이 늘수록 joint_ctrl 한 건당 비용이 오른다.

실행기를 우회한 직접 `take_message` 는 **23.5 us 로 평평**했다.

---

## 2. 적용한 수정

### 2.1 스크립트 기본값 (뷰어/패널)

최종 형태: **뷰어는 기본 켜짐, 패널도 기본 켜짐** (2.4 를 하고 나서 되돌림).

중간에 둘 다 opt-in 으로 바꿨다가, 뷰어는 눈으로 디버깅하는 수단이라
되돌렸다. `viewer_rate_hz` 파라미터를 새로 만들어 필요할 때 60 -> 30 으로
내릴 수 있게 했다 (기본 60).

| 파일 | 변경 |
|---|---|
| `scripts/run_4match.sh`, `run_8match.sh` | 도움말에 실측 표 기입, `--no-viewer` / `--no-panel` |
| `mujoco_supervisor/mujoco_supervisor/params.py` | `viewer_rate_hz` 추가 (기본 60) |
| `mujoco_supervisor/config/params.yaml` | 같은 값 + 근거 주석 |
| `mujoco_supervisor/mujoco_supervisor/supervisor_node.py` | 하드코딩된 `1.0/60.0` 을 파라미터로 |

### 2.2 joint_ctrl 실행기 우회 (`command_inbox.py`, 신규)

`joint_ctrl` 구독만 **아무도 스핀하지 않는 별도 노드**에 두고, 물리 루프가
스텝마다 `take_message` 로 직접 긁는다. 실행기가 그 노드를 보지 않으므로
대기 집합에 안 들어가고, DDS 는 실행기와 무관하게 큐를 채운다.

QoS 는 KEEP_LAST depth 1. 물리 루프는 스텝마다 최신 명령 하나만 쓰므로 밀린
옛 명령은 미들웨어가 C 에서 버리는 편이 파이썬으로 꺼내 버리는 것보다 싸다.
퍼블리셔가 `SystemDefaultsQoS`(RELIABLE / KEEP_LAST 10 / VOLATILE)라 호환된다.

격리 측정 (1 kHz 물리 루프 + 4대 x 1 kHz 명령):

| 방식 | 물리 스레드 | spin 스레드 | 합 |
|---|---|---|---|
| 기존 (spin + 콜백) | 2.6% | **50.0%** | 52.5% |
| depth-1 take | 10~11% | **2.8%** | **13~14%** |
| depth-10 drain | 15.0% | 2.9% | 17.8% |

실제 스택 결과:

| | 전 | 후 |
|---|---|---|
| `joint_states` | 463 Hz | **487~494 Hz** |
| supervisor 프로세스 CPU | 137% | **75%** |
| spin 스레드 | 84% | 상위 목록에서 사라짐 |
| 45초간 넘어짐 | 반복 발생 | **0회** |

부수 효과로 명령 경로의 `command_lock` 이 사라졌다 (물리 스레드가 유일한
독자이자 저자가 됨). 남은 자물쇠는 낙상 워처용이라 `watcher_lock` 으로 개명.

**주의로 남긴 것**: `Subscription.handle` 과 `take_message` 는 rclpy 공개
API 가 아니다. jazzy 에서 동작하지만 배포판을 올릴 때 깨질 수 있어
`CommandInbox` 안에 가뒀다. 그 구독의 콜백은 절대 불리지 않으므로, 누가 이
노드를 스핀하면 조용히 굶는 대신 터지도록 `_never_called` 이 예외를 던진다.

### 2.3 명령 거부 경고 throttle

2.2 때문에 새로 생긴 노출면. 관절 수가 안 맞는 명령을 거부하는 경고가 이제
**물리 스레드에서** 찍힌다. 1 kHz x 로봇 수라 조이지 않으면 로그가 루프를
잡아먹는다 -- 실측으로 1.00x 가 0.83x 로 떨어졌다.
`throttle_duration_sec=5.0` 추가.

### 2.4 패널을 자기 프로세스로 분리

패널이 물리 루프에서 완전히 빠지도록 별도 노드로 옮겼다. `log_panel_node` 가
이미 같은 형태여서 그대로 따랐다: Tk 를 mainloop 대신 ROS 타이머에서 펌프,
창을 닫으면 그 노드만 내려간다.

패널이 씬에서 읽던 것은 셋뿐이었다 -- `robot_pose` / `ball_position` /
`ball_velocity`. 나머지(역할·목표·모드·게임상태)는 이미 ROS 토픽이다.

기존 `{robot}/localization` 은 재활용할 수 없었다. 거기 공은 *그 로봇이 본*
공이라 시야를 벗어나면 멈춘다. 패널은 진짜 공을 그려야 한다. 그래서:

| 토픽 | 타입 | 용도 |
|---|---|---|
| `{robot}/ground_truth` | `geometry_msgs/PoseStamped` | 로봇 자세 |
| `/match/ball` | `nav_msgs/Odometry` | 공 위치+속도 |
| `/match/realtime_factor` | `std_msgs/Float32` | 실시간 배율 |

로봇마다 토픽을 따로 둔 것은 순서 결합을 피하기 위해서다. `PoseArray` 한 통에
담으면 받는 쪽이 supervisor 와 같은 `robot_names` 순서를 알아야 하고, 그 약속은
어긋나도 조용히 어긋난다.

`focus` 는 그리기에만 쓰이므로 패널 프로세스 안에서 끝난다 -- 역방향 채널이
필요 없었다.

| 파일 | 변경 |
|---|---|
| `mujoco_supervisor/mujoco_supervisor/truth_feed.py` | 신규. `TruthPublisher` / `TruthFeed` |
| `mujoco_supervisor/mujoco_supervisor/match_panel_node.py` | 신규. `log_panel` 과 같은 형태 |
| `mujoco_supervisor/mujoco_supervisor/panel_state.py` | `build_overlay(scene, ...)` -> `build_overlay(truth, ...)`. `MatchScene` 과 `TruthFeed` 가 같은 세 메서드를 답해 호출부는 그대로 |
| `mujoco_supervisor/mujoco_supervisor/supervisor_node.py` | 패널/오버레이 제거, 진실 + RTF 발행 |
| `mujoco_supervisor/mujoco_supervisor/params.py`, `config/params.yaml` | `panel_*` -> `truth_rate_hz` |
| `mujoco_supervisor/setup.py`, `package.xml` | `match_panel` 엔트리포인트, `nav_msgs` 의존 |

결과:

| 구성 | 전 | 후 |
|---|---|---|
| 패널만 | 353 Hz | **474 Hz** |
| 뷰어+패널 | 197 Hz | **465 Hz** |

덤으로 따라온 것 둘:

- RTF 가 패널과 분리되어 **창 없이도** 보인다: `ros2 topic echo /match/realtime_factor`.
  전에는 패널 그리는 코드 안에서만 계산돼 창을 끄면 속도를 볼 방법이 없었다.
- **경기 도중에 패널을 붙였다 뗐다** 할 수 있다:
  `ros2 run mujoco_supervisor match_panel --ros-args -p field:=CURRENT`

### 2.5 path_planning 최적화

패널을 분리하고 나니 `path_planning` 4개가 각 74% CPU 로 최대 소비자가 됐다.

**1단계 - 인과 확인.** 부하는 로봇이 움직일 때만 폭증한다 (4프로세스 합계):

| 상태 | planner | RTF |
|---|---|---|
| INITIAL / SET (서 있음) | **19%** | 0.955 +- 0.021, 0.90 미만 0/16 |
| READY / PLAYING (이동) | **341%** | 0.853 +- 0.071, 0.90 미만 28/45 |

로봇이 움직이면 supervisor 도 87% -> 100% 로 오르므로 상관만으로는 부족했다.
`replan_hz` 를 10 -> 2 로 낮춰 **planner 만** 싸게 만들고 같은 시나리오를 다시
돌렸다:

| | planner | RTF (이동 중) |
|---|---|---|
| replan_hz 10 | 341% | 0.853 +- 0.071 |
| replan_hz 2 | 170% | **0.902 +- 0.056** |

인과 확인. (설정은 실험 후 원복)

**2단계 - cProfile.** 살아있는 노드를 죽이고 같은 파라미터로 cProfile 아래
다시 띄워 70초 PLAYING 구간을 떴다:

| 분류 | 비중 |
|---|---|
| **알고리즘 (visibility_graph)** | **84.6%** |
| 그 알고리즘이 부르는 builtin (`abs`/`len`/`hypot`) | 13.5% |
| **rclpy/DDS** | **1.4%** |
| 메시지 직렬화 | 0.3% |
| ROS 어댑터 코드 | 0.03% |

상위 함수: `_point_on_segment` 24.2% (7,090만 회), `segment_blocked_by_polygon`
17.9%, `_cross` 16.1% (**1억 1,173만 회**), `point_in_polygon` 15.8%.

즉 supervisor 에서 최대 개선이었던 실행기 우회가 여기서는 **1.4% 라 무의미**했다.
실행기 비용은 노드 크기가 아니라 **초당 메시지 수에 비례**한다 (약 130 us/msg).

**3단계 - 벤치와 수정.** 실제 경기에서 planner 입력 419프레임을 녹화해
ROS 없이 코어만 재생하는 벤치를 만들었다 (before/after 를 같은 입력 위에서
보이기 위해).

기준선 **190.8 ms/회** -- `replan_hz: 10`(100 ms 예산)을 애초에 지킬 수 없다.

`polygon_vertices` 훑기 (경로는 모두 419/419 동일):

| vertices | 회당 | 배속 |
|---|---|---|
| 12 (기존) | 190.8 ms | — |
| 10 | 132.6 ms | 1.44x |
| **8** | **92.9 ms** | **2.05x** |
| 6 | 58.9 ms | 3.24x |

폴리곤은 원에 **외접**하므로(`_circumscribed_polygon`, `r / cos(pi/n)`)
꼭짓점을 줄이면 장애물이 오히려 커진다 (12각 1.035r -> 8각 1.082r). 덜
안전해지는 방향이 아니다.

그런데 녹화 입력을 뜯어보니 훨씬 큰 게 나왔다:

- **84% 가 목표에서 10 cm 이내** (중앙값 **1.4 cm**)
- **0% 가 장애물에 막힘** -- 419프레임 전부 직선
- 그런데도 매번 간선 624개짜리 가시성 그래프를 세우고 A* 를 돌린다

**1.4 cm 경로를 위해 190 ms.**

그래서 **직선 조기 반환**을 넣었다. 시작-목표 선분이 margin 도 critical 도
건드리지 않으면 최단 경로는 정의상 그 직선이므로 그래프를 안 세워도 같은
답이 나온다 -- 근사가 아니라 생략이다.

critical 까지 보는 이유: 선분이 critical 을 막히지 않고 가로지를 수 있는데
`_edge_multiplier` 가 그 통과에 `critical_cost_multiplier` 를 매기므로 탐색은
우회를 고를 수도 있다. 거기서 "직선" 이라 답하면 빨라진 게 아니라 다른
플래너가 된다.

| 파일 | 변경 |
|---|---|
| `humanoid_path_planner/core/visibility_graph.py` | `direct_path_is_clear()` 추가 |
| `humanoid_path_planner/core/planner.py` | `plan()` 에서 그래프 세우기 전 조기 반환 |
| `humanoid_path_planner/config/path_planning.yaml` | `polygon_vertices` 12 -> 8 |

**정확성 검증** -- 막히는 시나리오를 따로 만들어 조기 반환을 거치지 않은
순수 `shortest_path` 와 대조:

| 시나리오 | 판정 | 결과 | 순수 탐색과 |
|---|---|---|---|
| 정면 차단 | 안 통과 | 우회 4.11 m | 동일 |
| 좁은 통로 | 안 통과 | 직선 4.00 m | 동일 |
| 빗겨난 장애물 | 통과 | 직선 | 동일 |

**결과** (벤치): 190.8 ms -> **0.2 ms**

**결과** (실제 스택, 로봇이 움직이는 READY/PLAYING):

| | 전 | 후 |
|---|---|---|
| planner CPU (4개 합) | 341% | **25%** |
| RTF | 0.853 +- 0.071 | **0.985 +- 0.018** |
| 최악 RTF | 0.71 | **0.90** |
| 0.90 미만 샘플 | 28/45 | **0/45** |
| supervisor CPU | 100% | 66% |

이동 중 RTF 가 서 있을 때(0.987)와 사실상 같아졌다. planner 4개가 CPU 를
비우면서 supervisor 의 경합도 같이 풀린 것으로 보인다.

---

## 3. 틀렸던 가설과 정정

기록으로 남기는 이유: 전부 "그럴듯한데 측정하니 아니었던" 것들이다.

1. **"관측(observation) 처리가 무겁다"** -> 스텝의 13.5% 뿐. `mj_step` 이 77.6%.
   순수 루프는 3.66x 여유가 있었다.

2. **"GameController 앱이 0.1x 를 먹는다"** (465 -> 406 Hz 관측) -> 실제로는
   **1.3% CPU**. 그 차이는 `path_planning` 부하 변동이었다. 같은 기본 실행이
   다른 회차에는 RTF 0.88~0.94 에 planner 5.7% 였다.

3. **"인박스로 바꾸니 오히려 느려졌다"** (bench_sim 1.00x -> 0.83x) ->
   벤치가 관절 수를 20 으로 하드코딩했는데 K1 은 **23** 이라 모든 명령이
   거부됐고, 그 **거부 경고 4000회/초**가 물리 스레드를 잡아먹은 것.
   벤치 버그였지만 진짜 취약점이라 throttle 을 넣었다 (2.3).

4. **"planner CPU 의 1/3 은 재계획과 무관한 고정 비용"** -> 두 실행의
   per-replan 비용이 같다고 가정한 계산인데, `replan_hz=2` 실행은 로봇이 다르게
   움직여 시나리오가 달랐다. 실제로는 planner 가 190 ms/회라 10 Hz 타이머를
   애초에 못 지키고 포화 상태였던 것.

5. **"`_publish_debug` 가 가시성 간선을 매번 만들어 비쌀 것"** -> ROS 어댑터
   코드는 프로파일의 **0.03%**. `show_visibility_graph` 는 이미 `false` 였다.

6. **"path_planning 이 74%"와 "5.7%" 둘 다 관측** -> 둘 다 맞았다. 로봇이
   움직일 때만 폭증한다. 한 번의 관측으로 결론내면 안 되는 종류의 값이었다.

---

## 4. 병목 최종 순위 (4로봇 기준)

| # | 병목 | 크기 | 상태 |
|---|---|---|---|
| 1 | path_planning 가시성 그래프 | RTF 0.853 -> 0.985 | 해결 (2.5) |
| 2 | 패널이 물리 루프 안 | 366 -> 197 Hz (-46%) | 해결 (2.4) |
| 3 | 뷰어 sync (1.4 ms/회) | 463 -> 366 Hz (-21%) | **미해결, 구조적** |
| 4 | rclpy 실행기 (joint_ctrl 4000/s) | supervisor 137% -> 75% | 해결 (2.2) |
| 5 | `mj_step` 충돌 검사 | mj_step 의 65% | 미적용 |
| 6 | GameController 앱 | 1.3% | 무시 가능 |

---

## 5. 남은 문제

### 5.1 필드 크기가 세 곳에서 다르다

| 출처 | 값 |
|---|---|
| `humanoid_path_planner/config/path_planning.yaml` | 10.0 x 7.0 m |
| 실제 MuJoCo 씬 (`CURRENT`) | **22.0 x 14.0 m** |
| `humanoid_path_planner/test/test_parameters.py` 기대값 | 9.0 x 6.0 m |

`test_parameters.py::test_load_parameters_uses_yaml_as_only_source` 는 이 작업
**이전부터 실패**하고 있었다. 성능과 무관하게 따로 봐야 한다.

### 5.2 막히는 프레임의 실제 비율을 모른다

녹화 구간이 0% 였다. 로봇들이 공을 다투며 서로 막는 장면을 녹화해야 조기
반환이 안 먹는 프레임(여전히 93 ms)이 얼마나 자주 나오는지 알 수 있다.

### 5.3 뷰어는 프로세스 분리가 안 된다

`mujoco.viewer.launch_passive` 는 `sync()` 를 소유 스레드에서 불러야 해서
패널처럼 뺄 수 없다. `viewer_rate_hz` 를 30 으로 내리면 8.4% -> 4.2%.

### 5.4 손/머리 충돌 메시 (미적용)

각 로봇의 손이 38k, 머리가 11k 정점짜리 시각 메시를 그대로 충돌에 쓴다
(`models/ai_sapiens-k1/k1.xml` 의 229, 278, 324행). 나머지 부위는 전부
프리미티브다. 껐을 때 `mj_step` 238 -> 195 us (**1.22x**). 8대로 갈 때 의미가
생긴다. 이 파일은 robocup_mujoco_sim 전용 사본이라 학습(cyclo_mjlab)에는 영향
없다.

### 5.5 기존 동작

- 종료 시 supervisor 스핀 스레드에서 `ExternalShutdownException` 트레이스백.
  이 작업 이전 로그에도 있다. `main()` 이 bare `rclpy.spin` 을 데몬 스레드로
  띄우는 구조 때문.
- `robocup_task_planner/ros2_adapter/subscribers.py` 의 `gamecontrol_cb` 가
  메시지마다 INFO 로그 두 줄을 찍는다.

---

## 6. 변경 파일 전체

### robocup_mujoco_sim

```
 M ros/mujoco_supervisor/config/params.yaml
 M ros/mujoco_supervisor/launch/match_bringup.launch.py
 M ros/mujoco_supervisor/mujoco_supervisor/panel_state.py
 M ros/mujoco_supervisor/mujoco_supervisor/params.py
 M ros/mujoco_supervisor/mujoco_supervisor/supervisor_node.py
 M ros/mujoco_supervisor/package.xml
 M ros/mujoco_supervisor/setup.py
 M scripts/run_4match.sh
 M scripts/run_8match.sh
?? ros/mujoco_supervisor/mujoco_supervisor/command_inbox.py
?? ros/mujoco_supervisor/mujoco_supervisor/match_panel_node.py
?? ros/mujoco_supervisor/mujoco_supervisor/truth_feed.py
```

`ros/k1_mujoco_bridge/k1_mujoco_bridge/keyboard_remote_control.py` 도
수정 상태로 보이지만 이 작업 시작 전부터 그랬다.

### humanoid_navigation_stack

```
 M src/humanoid_path_planner/config/path_planning.yaml
 M src/humanoid_path_planner/humanoid_path_planner/core/planner.py
 M src/humanoid_path_planner/humanoid_path_planner/core/visibility_graph.py
```

### 테스트 상태

| 패키지 | 결과 |
|---|---|
| `robocup_mujoco_sim/tests` | 79 passed, 6 failed -- **6건 모두 작업 전부터 실패** (`test_ground_truth`, `test_state_pipeline`) |
| `humanoid_path_planner/test` | 24 passed, 1 failed -- **작업 전부터 실패** (`test_parameters`, 5.1 참고) |

---

## 7. 측정에 쓴 스크립트

이 폴더의 `bench/` 에 함께 넣어 두었다. 녹화한 입력 419프레임도
`bench/frames.json` 으로 같이 있어서, 아래 숫자를 그대로 재현할 수 있다.

| 스크립트 | 용도 |
|---|---|
| `bench_loop.py` | 물리 루프 한 스텝을 구성 요소별로 분해 |
| `bench_step.py` | `mj_step` 내부를 MuJoCo 내장 타이머로 분해 |
| `bench_mesh.py` | 손/머리 충돌 메시를 껐을 때의 `mj_step` 차이 |
| `bench_take.py` | rclpy 실행기 vs `take_message` 메시지당 비용 |
| `bench_inbox.py` | 제안 구현(depth-1 take)을 물리 루프 페이싱으로 재현 |
| `bench_viewer.py` | `viewer.sync()` 한 번의 비용 |
| `bench_sim.py` | 진짜 `SupervisorNode` 를 띄우고 RTF 를 찍음 |
| `bench_profile.py` | 물리 루프 안에서 `poll()` 과 `mj_step` 을 계측 |
| `gc_drive.py` | GC 창 대신 `/gamecontroldata` 를 쏴서 상태를 몬다 |
| `sample.py` | planner CPU 와 RTF 를 같은 타임라인에 샘플링 |
| `record_inputs.py` | planner 입력 419프레임 녹화 |
| `bench_planner.py` | 녹화 입력으로 `Planner.plan()` 재생, 경로 비교 포함 |
| `bench_earlyout.py` | 직선 조기 반환 프로토타입 |

가장 재사용 가치가 높은 것은 `record_inputs.py` + `bench_planner.py` 쌍이다.
planner 를 더 손댈 때 before/after 를 같은 입력 위에서 증명할 수 있다.
