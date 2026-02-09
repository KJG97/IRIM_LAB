# IRIM_LAB 리팩터링 계획 (02)

**범위**: `extsUser/IRIM_LAB` 이하만 고려한다.  
**원칙**: 한 번에 전체 리팩터링하지 않는다. 아래 **태스크를 하나씩 에이전트에게 지시**하고, 사용자가 만족할 때까지 반복한다.

---

## 1) 현재 문제

- `irim_lab_python` 아래에 **폴더·모듈이 과하게 쪼개져** 있어 가동성(찾기, 수정, 추적)이 매우 떨어진다.
- Sim2Sim / Sensor Lab **두 기능만** 유지하면 되는데, 그와 **무관한 코드**가 섞여 있다.
- 목표: **Sim2Sim + Sensor Lab 관련만 남기고**, 구조를 단순화한다.

---

## 2) 범위

- **고려 경로**: `/home/jkkim/isaac-sim/extsUser/IRIM_LAB` (또는 프로젝트 기준 `IRIM_LAB/`) 이하만.
- **유지할 기능**: Sim2Sim(로봇 로드, LOAD/RESET/RUN, 관절 제어, ROS2, 오버레이), Sensor Lab(센서 에셋 로드, 토크 테스트 UI).
- **제거 대상**: 위 두 기능과 **관련 없는** 코드·폴더·파일(사용자 확인 후 제거).

---

## 3) 사용 방법 (에이전트에게 시키기)

- 이 문서의 **태스크 번호**를 지정해서 요청한다.  
  예: *"02_Refactoring_Plan.md의 Task 1 실행해줘"*, *"Task 2만 진행해줘"*
- 한 태스크가 끝나면 **결과 요약**을 받고, 필요하면 다음 태스크를 지시한다.
- **한 번에 전부 하지 말고**, 태스크 단위로 진행한 뒤 사용자가 만족할 때까지 반복한다.

---

## 4) 태스크 목록

### Task 1 — Sim2Sim/Sensor Lab 비관련 코드 목록 작성

**지시문**:  
"`DDVC/02_Refactoring_Plan.md`의 **Task 1** 실행해줘.  
`irim_lab_python` 아래 모든 폴더·파일을 훑고, **Sim2Sim 또는 Sensor Lab과 직접 관련 없는** 후보 목록을 정리해서 보여줘."

**에이전트가 할 일**  
- `irim_lab_python` 내 디렉터리·파일 목록을 기준으로, 각 항목이 **Sim2Sim용 / Sensor Lab용 / 비관련(제거 후보)** 인지 분류한다.
- 비관련 후보에 대해 **이유(한 줄)** 를 붙여서 마크다운 테이블 또는 리스트로 정리한다.
- 제거 시 **import/참조 끊김**이 생길 수 있는 항목은 "의존성 확인 필요"로 표시한다.

**산출물**: `irim_lab_python` 기준 비관련 후보 목록 (파일/폴더 + 분류 이유).

---

### Task 2 — 비관련 코드 제거 (사용자 확인 후)

**지시문**:  
"`DDVC/02_Refactoring_Plan.md`의 **Task 2** 실행해줘.  
Task 1에서 정리한 **비관련 후보** 중에서 [사용자가 지정한 항목 또는 전체] 제거해줘.  
제거 전에 참조하는 곳이 있으면 알려줘."

**에이전트가 할 일**  
- 제거 대상 목록을 사용자가 지정했으면 그에 맞춘다. 지정이 없으면 Task 1 산출물 전체를 기본으로 하되, **의존성 확인 필요** 항목은 제거 전에 보고한다.
- 제거 전: 해당 코드를 **참조하는 위치**(다른 .py, 설정 등)를 grep 등으로 찾아서 정리해 보여준다.
- 사용자 확인 또는 "진행해" 지시 후: 파일/폴더 삭제, 해당 코드를 참조하던 곳에서 import·호출 제거한다.
- 제거 후: extension 실행 경로(Sim2Sim 창, Sensor Lab 창)가 **정상 동작하는지** 확인할 수 있는 체크리스트를 짧게 제시한다.

**산출물**: 제거된 항목 목록, 수정된 파일 목록, 동작 확인 체크리스트.

---

### Task 3 — 폴더 구조 단순화 제안

**지시문**:  
"`DDVC/02_Refactoring_Plan.md`의 **Task 3** 실행해줘.  
Sim2Sim과 Sensor Lab만 남긴 상태에서, **폴더를 줄이거나 합치는** 방안을 제안해줘.  
(예: config/core/event_handlers/ui/ros2 등 개수·깊이 감소)"

**에이전트가 할 일**  
- 현재 `irim_lab_python` 폴더 구조를 나열하고, Sim2Sim·Sensor Lab 관점에서 **꼭 나눌 필요가 없는** 폴더를 식별한다.
- "A 폴더와 B 폴더를 C 하나로 합치기", "D 폴더는 루트로 올리기" 등 **구체적 제안**을 3~5개 정도 작성한다.
- 각 제안에 대해 **이동/병합 시 바꿔야 할 import 경로**를 짧게 적는다.
- 실제 이동·수정은 하지 않고 **제안만** 한다.

**산출물**: 폴더 단순화 제안서 (현 구조 → 제안 구조, import 변경 요약).

---

### Task 4 — 폴더 단순화 1건 실행 (사용자 선택)

**지시문**:  
"`DDVC/02_Refactoring_Plan.md`의 **Task 4** 실행해줘.  
Task 3 제안 중 **[사용자가 선택한 1건]** 적용해줘.  
import·테스트 경로 다 수정해줘."

**에이전트가 할 일**  
- 사용자가 고른 1건(폴더 병합/이동)만 수행한다.
- 파일 이동, `import` 경로 수정, `__init__.py` 등 참조 업데이트를 모두 적용한다.
- 적용 후 Sim2Sim·Sensor Lab이 **로드·실행되는지** 확인할 수 있는 짧은 체크리스트를 제시한다.

**산출물**: 변경된 디렉터리/파일 목록, 수정된 import 목록, 동작 확인 체크리스트.

---

### Task 5 — scenario.py / ui_builder 역할 나누기 (선택)

**지시문**:  
"`DDVC/02_Refactoring_Plan.md`의 **Task 5** 실행해줘.  
`scenario.py`(ALLEXDigitalTwin)와 `ui_builder.py`에 몰린 역할을 **한 단계만** 나눠줘.  
(예: 오버레이만 별도 모듈로 분리, 또는 이벤트 핸들러 정리 1건)"

**에이전트가 할 일**  
- PRD·기존 REFACTORING_ROADMAP 방향에 맞춰, **한 번에 한 블록만** 분리한다(예: 오버레이, 또는 Follow Cube, 또는 이벤트 핸들러 1종).
- 분리 후 import·호출 관계를 정리하고, 동작 확인 방법을 짧게 적는다.
- 사용자가 "더 나눠줘"라고 하면, 같은 태스크를 반복해 다음 블록을 나눌 수 있다.

**산출물**: 새로 만든/수정한 파일 목록, 변경된 호출 관계 요약.

---

### Task 6 — 중복· dead code 정리

**지시문**:  
"`DDVC/02_Refactoring_Plan.md`의 **Task 6** 실행해줘.  
`irim_lab_python` 안에서 **같은 이름/역할의 중복 파일**, **어디서도 쓰이지 않는 dead code** 후보를 찾아서 정리해줘."

**에이전트가 할 일**  
- 동일·유사 파일명(예: `sim2real_debugger_gui.py`가 두 곳에 있는지) 검사.
- 전역 검색으로 **import되지 않는** .py 모듈·함수 후보를 나열한다.
- 제거 시 영향이 있을 수 있는 항목은 "확인 필요"로 표시하고, 나머지는 제거 또는 통합 제안을 한다.
- 사용자 확인 후 실제 삭제·통합을 수행할지 정한다.

**산출물**: 중복/ dead code 후보 목록, 제거/통합 제안.

---

## 5) 진행 순서 권장

1. **Task 1** → 비관련 목록 확인  
2. **Task 2** → 비관련 코드 제거 (원하는 것만 골라서)  
3. **Task 3** → 폴더 단순화 제안 검토  
4. **Task 4** → 제안 중 1건씩 적용 (만족할 때까지 반복 가능)  
5. **Task 5** — 필요 시 scenario/ui_builder 역할 나누기  
6. **Task 6** — 중복· dead code 정리  

필수는 1→2→3→4 순서이고, 5·6은 필요할 때만 시킨다.

---

## 6) 태스크 추가·수정

- 사용자가 "Task 7 추가해줘: …"처럼 요청하면, 이 문서에 **Task 7**을 새 섹션으로 넣고, 지시문·에이전트가 할 일·산출물을 같은 형식으로 작성한다.
- 기존 태스크 내용을 바꾸고 싶으면 "Task N을 …로 수정해줘"라고 하면 된다.

---

## 7) 현재 irim_lab_python 구조 (참고)

```
irim_lab_python/
├── config/          # joint, ros2, ui, visibility 설정
├── controllers/     # (많은 .obj/.mtl - 에셋?)
├── core/            # asset_manager, initialization, joint_controller, sensor_manager, simulation_loop, via_point_manager, visualization
├── event_handlers/  # joint, ros2, scenario, visibility
├── rmpflow/         # yaml
├── ros2/            # callbacks, manager, node
├── sim2sim_console/ # hydra, importer, parse_cfg, sim2real_debugger (policy .pt, ui 등)
├── ui/              # p2p_plot_window, ui_builders, ui_components, ui_styles
├── utils/           # constants, sim2real_debugger_gui (중복 가능성)
├── extension.py
├── scenario.py      # ALLEXDigitalTwin (대형)
├── ui_builder.py
├── sensor_lab_ui_builder.py
├── sensor_lab_joint_torque_test.py
├── global_variables.py
├── joint_config.json
└── DDVC/            # 문서
```

Task 1에서 위 구조를 기준으로 Sim2Sim / Sensor Lab / 비관련을 분류한다.

---

## 8) Task 1 산출물 — 비관련 후보 목록

(에이전트가 Task 1 실행 후 아래를 채움. 사용자는 이 목록을 기준으로 Task 2에서 제거 대상을 지정한다.)

### 전체 분류 요약

| 경로 | 분류 | 비고 |
|------|------|------|
| `extension.py` | Sim2Sim + Sensor Lab | 진입점 |
| `ui_builder.py` | Sim2Sim | 메인 Sim2Sim UI |
| `sensor_lab_ui_builder.py` | Sensor Lab | Sensor Lab UI |
| `sensor_lab_joint_torque_test.py` | Sensor Lab | 토크 테스트 로직 |
| `scenario.py` | Sim2Sim | ALLEXDigitalTwin |
| `global_variables.py` | 공통 | |
| `joint_config.json` | Sim2Sim | |
| `config/` | 공통 | joint, ros2, ui, visibility — Sim2Sim·Sensor Lab 모두 사용 |
| `core/` (via_point_manager 제외) | Sim2Sim + Sensor Lab | asset_manager, joint_controller 등 |
| `core/via_point_manager.py` | **비관련** | 어디서도 import 안 함. core/__init__에도 없음. |
| `event_handlers/` | Sim2Sim | scenario, joint, ros2, visibility |
| `ros2/` | Sim2Sim | ROS2 연동 |
| `rmpflow/` | Sim2Sim | IK 컨트롤러용 yaml (controllers에서 참조) |
| `controllers/` | Sim2Sim | Follow Cube용 left/right_ik_solver 등 (scenario에서 import) |
| `ui/ui_builders.py`, `ui_components.py`, `ui_styles.py` | Sim2Sim + Sensor Lab | 사용 중 |
| `ui/p2p_plot_window.py` | **비관련** | ui/__init__에 없음, 어디서도 import 안 함. |
| `utils/constants.py` | 공통 | scenario, sensor_lab_ui_builder에서 사용 |
| `utils/sim2real_debugger_gui.py` | **비관련** | PyQt 독립 앱. extension/ui_builder/scenario에서 import 없음. |

---

### 비관련(제거 후보) 상세

| 항목 | 이유 | 의존성 확인 |
|------|------|-------------|
| **utils/sim2real_debugger_gui.py** | Sim2Real Debugger PyQt 창 단일 파일. extension/ui_builder/scenario 어디서도 import 안 함. | 없음. 제거 시 import 끊김 없음. |
| **core/via_point_manager.py** | core/__init__에 없고, 다른 .py에서 import 안 함. | 없음. 제거 시 core/__init__ 수정 불필요. |
| **ui/p2p_plot_window.py** | ui/__init__에 없고, 어디서도 import 안 함. | 없음. |

---

### Sim2Sim / Sensor Lab 유지 항목 (참고)

- **Sim2Sim 전용**: `extension.py`(Sim2Sim 창), `ui_builder.py`, `scenario.py`, `config/`, `core/`(via_point_manager 제외), `event_handlers/`, `ros2/`, `rmpflow/`, `controllers/`, `ui/`(p2p_plot_window 제외), `utils/constants.py`, `global_variables.py`, `joint_config.json`
- **Sensor Lab 전용**: `sensor_lab_ui_builder.py`, `sensor_lab_joint_torque_test.py`, `core/asset_manager`(load_sensor_test_asset), `utils/constants.py`, `ui/`(SensorLabWorldControlsBuilder 등)
- **공통**: `config/`, `utils/constants.py`, `global_variables.py`, `extension.py`

---

## 9) Task 2 산출물 — 제거 실행 결과

(사용자 지시: 비관련 코드 중 **sim2sim_console**, **DDVC_참고용** 제외하고 모두 제거)

### 제거된 항목

| 항목 | 비고 |
|------|------|
| `utils/sim2real_debugger_gui.py` | 삭제 완료. 참조 없음. |
| `core/via_point_manager.py` | 삭제 완료. core/__init__에 없었음. |
| `ui/p2p_plot_window.py` | 삭제 완료. ui/__init__에 없었음. |

### 유지한 항목 (사용자 요청)

- `sim2sim_console/` 전체 — 1차 목표로 IRIM_LAB과 통합 예정.
- `DDVC_참고용/` — 참고 문서로 유지.

### 수정된 파일

- 없음 (삭제한 파일을 import하던 코드 없음).

### 동작 확인 체크리스트

- [ ] Isaac Sim에서 IRIM_LAB 확장 로드
- [ ] 메뉴에서 **Sim2Sim** 창 열기 → LOAD → RUN 동작
- [ ] 메뉴에서 **Sensor Lab** 창 열기 → LOAD 동작
- [ ] 확장 리로드 후 창 토글 시 크래시 없음
