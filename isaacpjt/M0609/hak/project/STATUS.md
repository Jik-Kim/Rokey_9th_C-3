# 시뮬 환경 구성 및 랜덤 스폰

기준 씬: `project_1/test1.usd`

동작 영상: [`hak_test_1.mp4`](~/Videos/hak_test_1.mp4)

---

## 1. 작동 오류

### 1-1. 세 번째 A43에서 박스가 멈춤

`SorterRed`를 프림 복사로 만들 때 그래프 안의 경로가 원본(`SorterGreen`)을 그대로 가리켰습니다. **10건**이었습니다.

```
SorterRed/ConveyorBeltGraph/ConveyorNode
    inputs:conveyorPrim  → /World/SorterGreen/Rollers            (relationship)
    inputs:velocity      → /World/SorterGreen/.../read_speed     (connection)
    inputs:onStep        → /World/SorterGreen/.../OnTick         (connection)
    inputs:delta         → /World/SorterGreen/.../OnTick         (connection)
    inputs:graph         → /World/SorterGreen/ConveyorBeltGraph  (relationship)
    ... ConveyorBeltGraph_01 도 동일하게 5건
```

그래프 간 연결은 평가되지 않습니다. velocity가 비면 컨베이어 노드가 `physxSurfaceVelocity`를 **아예 authoring하지 않습니다.** Play 중 실측에서 `SorterRed/Rollers`에 그 속성 자체가 없었습니다.

박스는 정확히 `y = 11.33`에서 섰습니다. `SorterRed/Rollers`의 시작점이 `y = 11.240`입니다.

같은 원인으로 `ConveyorTrack_01`의 `graph:variable:Velocity`도 비어 있어 라인 끝이 통째로 정지 상태였습니다.

### 1-2. 소터에서 박스가 동쪽으로 밀림

분기 롤러 `Rollers_01`의 로컬 프레임이 월드 기준 +45°인데 `inputs:direction`이 `(1,0,0)`이었습니다. 팝업 휠이 꺼져 있어도 **항상 북동쪽으로** 밀고 있었습니다.

### 1-3. 소터마다 8.8mm 턱

```
Rollers                  상면 0.8997
Sorter/Conveyor_physics  상면 0.9085     실린더 24개, purpose=proxy
```

박스가 소터마다 이 턱을 기어올라야 했습니다. `SorterSpeed`를 0.5로 두면 못 넘어 입구에 쌓이고, 1.0으로 올리면 넘기는 하지만 데크(1.0)와 본선 롤러(0.5)의 속도차로 요 토크가 생겨 박스가 옆으로 걸어나갔습니다. 어느 쪽도 답이 아니었습니다.

### 1-4. `SorterSpeed` 에셋 기본값이 −3.0

오버라이드가 없으면 데크가 **역방향 3 m/s**로 돕니다. 라인 속도의 6배로 뒤로 밀어내서 박스가 소터 입구에 쌓입니다.

`SorterRed`는 나중에 추가돼서 이 오버라이드가 없었고, 그래서 세 번째 소터에서만 증상이 달랐습니다.

### 1-5. GUI에서만 튕김 — 시험 조건이 실제와 달랐음

`open_test1.py`가 항상 `sorter.start()`를 불렀습니다. 즉 **GUI에서는 색상 분류가 켜진 채로** 돌고 있었습니다. 팝업 휠이 올라와 박스를 분기로 밀어내는데, 분기 롤러는 아무 벨트에도 안 닿아 있어서 허공으로 밀어냅니다. 그래서 튕기고 막혔습니다.

헤드리스 검증 스크립트는 소터를 강제로 껐기 때문에 이 문제가 안 보였습니다. **시험이 통과하는데 화면에서는 안 되던 이유가 이것입니다.** 시험 조건을 실제 실행 경로와 맞추지 않은 게 잘못이었습니다.

### 1-6. 스폰이 안 된다는 증상

`open_test1.py`가 스테이지만 열고 피더를 로드하지 않던 시기가 있었습니다. Play를 눌러도 박스가 안 나옵니다. 그리고 Stop을 누르면 `spawn_feeder._on_timeline`이 박스를 전부 지웁니다 (의도된 동작입니다. Pause는 남습니다).

---

## 2. 해결 방법

### 적용한 것

| 오류 | 스크립트 | 내용 |
| --- | --- | --- |
| 1-1 | `run_fix_graph_refs.sh` | 그래프 안의 `/World/<다른조각>/...` 경로를 자기 자신으로 재작성. **relationship과 connection 둘 다**. 빈 `Velocity`도 0.5로 채움 |
| 1-2 | `run_fix_sorter_wiring.sh` | 분기 롤러 `direction`을 로컬 `(0.7071, 0.7071, 0)` = 월드 +Y 로 |
| 1-3 | `run_level_sorters.sh` | 세 접촉면을 0.9000으로 정렬 |
| 1-4 | `PL_SPEED=0.5 PL_DIR=45 ./run_tune_sorters.sh` | `SorterSpeed` 오버라이드 생성 |
| 1-5 | `open_test1.py` 기본값 변경 + `run_set_sorters_off.sh` | 분류를 기본 OFF로. 켜려면 `PL_SORT=1` |
| 1-6 | `open_test1.py`, `attach.py` | 스테이지 열 때 피더를 같이 붙임 |

1-3의 정렬 수치:

```
조각 원점    z 0.1300 → 0.1215   (−8.5mm)   ⇒ 데크 0.9085 → 0.9000
Rollers      로컬 z +0.0088                  ⇒ 0.8912 → 0.9000
Rollers_01   로컬 z +0.0092                  ⇒ 0.8908 → 0.9000
```

> **현재 소터는 OFF 상태입니다. 카메라 인식을 붙인 뒤 소터를 다시 구성합니다.**

### 틀렸던 시도 — 다시 하지 말 것

**데크만 내리기.** 8.8mm 내렸더니 9/10 → 1/10으로 나빠졌습니다. 롤러 배치를 재고 이유를 알았습니다:

```
SorterBlue/Rollers   46개, 평균 간격 87mm, 그런데 최대 간격 892.9mm
데크 구간(y 4.65~5.69) 안의 롤러는 3개뿐
데크 실린더가 y 4.933~5.629 를 채운다
```

데크가 그 구간의 **유일한 지지면**입니다. 내리면 턱이 없어지는 게 아니라 893mm짜리 구덩이가 생깁니다. 그래서 조각 전체를 내리고 롤러를 올리는 방식으로 바꿨습니다.

**분기 롤러 정지.** 속도를 0으로 세웠더니 죽은 마찰면이 되어 10개 전부 그 자리에 섰습니다. 박스는 이 롤러 위를 지날 수밖에 없는 구조라, 세우면 안 되고 방향만 돌려야 합니다.

**가이드 레일 전면 차단.** 분기 창까지 다 막았더니 박스 두 개가 나란히 끼었습니다.

```
레일 사이 유효폭 945mm  >  480(5호) + 250(3호)
```

레일이 없을 땐 하나가 옆으로 밀려나며 풀렸는데, 막으니 끼어버립니다. 폭을 박스 하나만 지나가게 좁혀야 의미가 있습니다.

**높이 계열 교체 + 분기 재배치를 한꺼번에.** 높이는 잘 맞았지만 (전 조각 900.0mm, 지면 위 121~131mm) 그 김에 분기 레이아웃까지 건드리다 씬이 엉켜 되돌렸습니다. 다시 한다면 **높이 교체만** 하고 레이아웃은 손대지 마세요.

### 검증 결과

단독 박스 추적 (0.5초 간격, 물리 실측):

```
y   3.18 → 15.95     SorterBlue(3.5~7.5) → SorterGreen(7.5~11.5) → SorterRed(11.2~15.2) → _01
x   −9.419 고정       소수점 셋째 자리까지 변화 없음
z   1.040 고정       속도 0.50 m/s 일정, 소터에서 튐 없음
```

12개를 3초 간격으로 흘린 결과: 전부 본선 위(x −9.25~−9.74), y 13.8~16.9에서 엔드스톱에 줄을 섰습니다. 기울기 최대 3.1도, 낙하 0개, 분기 이탈 0개.

---

## 3. 아직 미진한 것

> **카메라 인식을 붙인 뒤 해결할 문제들입니다.**

### 3-1. 분기 3개가 갈 곳이 없음 — 제일 큰 문제

```
[본선]  ConveyorTrack → _17(커브) → _03 → SorterBlue → SorterGreen → SorterRed → _01
        7개 면 전부 맞닿음. 완성.

[섬 1]  ConveyorTrack_10 ← SorterGreen/Rollers_01, SorterRed/Rollers_01 이 붙어 있음
[섬 2]  ConveyorTrack_04   단독. _10 과 1.239m 떨어짐
[섬 3]  SorterBlue/Rollers_01   단독. 받는 벨트 없음
```

**이것 때문에 분류를 못 켭니다.** 켜면 박스가 허공으로 밀려납니다.

배정 계획 (먼 곳부터 RGB):

| 색 | 규격 | 목적지 | 상태 |
| --- | --- | --- | --- |
| 빨강 3호 | 340×250×210, 3kg | 본선 끝 `_01` | 라인 완성 |
| 초록 4호 | 410×310×280, 5kg | `SorterGreen` 분기 → `_10` → `_04` | `_10`↔`_04` 1.239m 끊김 |
| 파랑 5호 | 480×360×340, 9kg | `SorterBlue` 분기 → ? | 받는 벨트 자체가 없음 |

`SorterRed/Rollers_01`은 빨강이 직진하므로 쓰이지 않습니다.

참고로 `project_1/test2.usd`(소터 도입 전 원래 라인)에는 목적지 3개가 살아 있습니다. A24 T_MERGE에서 갈라져 A37 램프로 내려가 A05 스퍼에 도착하는 구조입니다.

```
스퍼 좌표   (-3.244, 11.413)   (-3.244, 15.413)   (-3.993, 18.991)
```

y가 지금 소터 위치와 안 맞아 그대로 못 쓰지만, 배치 의도는 여기서 읽을 수 있습니다.

### 3-2. 색 인식이 가짜

`sorter.py`의 `_color_of()`가 프림 이름의 `No3/No4/No5` 접미사를 읽습니다. 정답지를 보는 것이라 인식이 아닙니다.

### 3-3. 박스끼리 부딪히면 옆으로 밀림

단독 박스는 x 편차 0으로 직진하지만, 여러 개를 흘리면 뒤차가 앞차를 밀어 튕겨나갑니다. 지금은 분기가 막혀 있어 티가 안 나지만, 분류를 켜면 엉뚱한 박스가 분기로 빠집니다.

---

## 4. 앞으로의 방향

**① 분기 레이아웃 결정** — 파랑/초록 박스를 각각 어디에 모을지. 받는 라인이 `_10` 하나뿐이라 세 목적지를 만들려면 벨트를 더 놓아야 합니다. GUI에서 직접 배치하고 연결·구동만 스크립트로 맞추는 게 빠를 수 있습니다.

**② 받는 벨트 연결 + 엔드스톱** — 결정된 레이아웃대로 벨트를 놓고, 각 목적지에 엔드스톱을 세워 박스가 로봇 픽업 위치에 줄 서게 합니다. 지금 엔드스톱은 본선 끝(`y 17.13~17.24`)에만 있습니다.

**③ 가이드 레일 폭 조정** — 박스 하나만 지나가게 좁힙니다. 5호 480mm 기준으로 유효폭 560mm 정도. 분기 창은 열어둡니다.

**④ 분류 켜기** — `PL_SORT=1`. 여기서 처음으로 팝업 휠이 실제로 동작합니다.

**⑤ 카메라 색 인식** — 첫 분기 상류에 하향 카메라를 달고 RGB→HSV 판정으로 `_color_of()`만 교체합니다. 그 함수 하나만 갈아끼우면 되도록 만들어뒀습니다.

**⑥ 트럭 · 팔레트 · 로봇** — 트럭 적재 높이 900mm에 벨트면을 맞춰놨으므로 트럭을 라인 끝에 대면 됩니다. 로봇 프림을 다시 넣고 픽앤플레이스를 붙입니다.

**⑦ 높이 계열 정리** (선택) — 겉보기 문제라 급하지 않습니다. 하려면 `run_swap_to_low.sh`만 돌리고 레이아웃은 건드리지 마세요.

---

## 실행

```bash
cd ~/isaacsim && ./isaac-sim.sh --exec ~/cobot3_ws/isaacpjt/M0609/hak/project/open_test1.py
```

기본은 **분류 끔**. Play만 누르면 3초마다 1개씩 10개가 투입되고 본선 끝까지 흘러갑니다.

| 환경변수 | 효과 |
| --- | --- |
| `PL_SORT=1` | 색상 분류 켜기 (분기에 받는 벨트를 놓은 뒤에) |
| `PL_NO_FEEDER=1` | 피더 없이 씬만 보기 |
| `PL_N=20` | 투입 개수 |
| `PL_INTERVAL=5` | 투입 간격 [초] |

이미 열려 있는 창에 붙이려면 Script Editor에서:

```python
import sys; sys.path.insert(0, "/home/rokey/cobot3_ws/isaacpjt/M0609/hak/project")
import attach; attach.go(reopen=True)
```

왜 스폰이 안 되는지 짚어보려면 `attach.check()`.

---

## 스크립트

| 파일 | 역할 |
| --- | --- |
| `open_test1.py` | `test1.usd` 열기 + 피더 부착. 분류는 기본 꺼짐 |
| `open_test2.py` | `test2.usd` 열기. 벨트면이 두 층이라 피더는 기본 안 붙음 (`PL_FEEDER=1`) |
| `attach.py` | 이미 열린 세션에 붙이기. `go(reopen=True)`, `check()` |
| `spawn_feeder.py` | `PL_INTERVAL`초마다 박스 1개. 3:4:5호 = 50:30:20 |
| `sorter.py` | 색상별 분기. `_color_of()`가 색 판정 진입점 |
| `run_fix_graph_refs.sh` | 그래프 교차 참조 교정 + 빈 Velocity 채우기 |
| `run_fix_sorter_wiring.sh` | 분기 롤러 방향을 직진으로 |
| `run_level_sorters.sh` | 소터 세 접촉면을 0.9000으로 정렬 |
| `run_tune_sorters.sh` | `Direction` / `SorterSpeed`. `PL_SPEED`, `PL_DIR` |
| `run_set_sorters_off.sh` | `binary_switch = False`를 씬에 명시 |
| `run_add_guiderails.sh` | 본선 가이드 레일. `PL_RAIL_FULL=1`이면 분기 창까지 차단 |
| `run_swap_to_low.sh` | A06→A05, A03→A02 교체 (높이 정리용) |
| `run_connect_branches.sh` | 분기 연결 + 파랑 스퍼 신설 |
| `run_install_sorters.sh` | A43 소터 배치 |

지금 안 쓰는 스크립트는 `archive/`에 있습니다. 지우지 마세요 — 씬을 특정 시점으로 되돌릴 때 그 상태를 재현하는 유일한 기록입니다. 사유는 `archive/README.md`에 적어뒀습니다.

모든 씬 편집 스크립트는 실행 전 `project_1/test1.usd.bak.<타임스탬프>`로 백업합니다.
