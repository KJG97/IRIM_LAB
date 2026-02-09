# Code Refactoring & Optimization Guide

DDVC 및 PRD(`100_PRD.md`) 요구사항을 반영한 **IRIM_LAB** 코드 리팩토링·구조 최적화 지침서.

**범위**: `extsUser/IRIM_LAB` 이하.  
**원칙**: 한 번에 전체 리팩토링하지 않고, 단계별로 진행한 뒤 검증·만족 시 다음 단계로 진행한다.

---

## 1. 리팩토링 핵심 원칙 (Refactoring Principles)

### 코드 가독성 및 구조화

- **의미 있는 명명 (Meaningful Naming):** 변수·함수·모듈 이름만으로 역할을 파악할 수 있도록 명명.
- **단일 책임 (SRP):** 한 함수·한 모듈은 하나의 역할만 수행. 대형 파일(`scenario.py`, `ui_builder.py`)은 역할별로 분리 검토.
- **추상화 수준 통일:** 같은 계층에서는 비슷한 수준의 추상화만 유지.
- **매직 넘버 제거:** 숫자·경로는 `config/`, `global_variables.py` 또는 `constants`로 분리.

### 의존성 관리 및 결합도 감소

- **모듈화:** Sim2Sim 전용 / Sensor Lab 전용 / 공통(core, config, ui)을 명확히 구분.
- **중복 제거 (DRY):** 반복 로직은 공통 유틸 또는 상위 모듈로 추상화.
- **불필요 코드 제거:** Sim2Sim·Sensor Lab과 무관한 코드는 식별 후 사용자 확인 하에 제거.

---

## 2. 구조 최적화 전략 (Structure Optimization)

### 폴더·모듈 구조

- **기능 단위 분리:** `sim2sim_test/`(Sim2Sim), `sensor_test/`(Sensor Lab)로 진입점·시나리오·핸들러 분리. 공통은 루트 `config/`, `core/`, `ui/`, `utils/` 유지.
- **폴더 깊이·개수 절감:** 꼭 나눌 필요 없는 폴더는 병합(예: `utils/constants` → `config/constants`, `rmpflow/` → `config/rmpflow` 등) 검토.
- **import 경로 정리:** 이동·병합 시 상대 import 일괄 수정, `__init__.py` 및 확장 진입점(`extension.py`) 참조 반영.

### 역할 분리 (scenario / ui_builder)

- **한 번에 한 블록만 분리:** 오버레이, Follow Cube, 이벤트 핸들러 등 하나씩 별도 모듈로 분리 후 import·호출 관계 정리.
- **동작 검증:** 분리 후 Sim2Sim LOAD → RUN, Sensor Lab LOAD 동작 확인.

### 품질·성능

- **Dead code·중복 제거:** import 되지 않는 모듈·함수, 동일·유사 역할 파일 정리.
- **리소스 관리:** 대형 에셋·모델은 필요 시점 로드, 사용 후 해제로 메모리·로드 시간 관리.

---

## 3. DDVC & PRD 연계 워크플로우

### PRD 1차 목표 반영

| PRD 목표 | 리팩토링 시 점검 사항 |
|----------|------------------------|
| **통합** (sim2sim_test ↔ sim2sim_console) | 메뉴·데이터·실행 경로가 하나의 확장 플로우로 동작하는지 확인. `sim2sim_console` 제거하지 않고 통합 대상으로 유지. |
| **구조 최적화** (sim2sim_test) | 모듈화, 설정 분리, 역할 분리로 가동성(찾기, 수정, 추적)이 향상되었는지 확인. |

### DDVC 문서 동기화

- **코드와 문서 일치:** 폴더·모듈 변경 시 이 문서(§4 체크리스트, §5 진행 단계) 및 `102_*` 기술 스펙에 반영.
- **재현성:** 리팩토링 후에도 PRD 성공 기준(LOAD → ROS2 → RUN, Sensor Lab LOAD, 확장 리로드 시 크래시 없음)이 유지되는지 검증.

---

## 4. 리팩토링 체크리스트 (Review Checklist)

| 구분 | 체크 항목 | 확인 |
|------|-----------|------|
| **품질** | Sim2Sim·Sensor Lab과 무관한 코드가 제거되었는가? | □ |
| **품질** | 중복·dead code가 정리되었는가? | □ |
| **구조** | sim2sim_test / sensor_test / 공통 역할이 명확한가? | □ |
| **구조** | import 경로가 일관되고 끊김 없이 동작하는가? | □ |
| **안정성** | 확장 로드 → Sim2Sim 창 LOAD/RUN, Sensor Lab 창 LOAD가 정상인가? | □ |
| **안정성** | 확장 리로드·창 토글 시 크래시가 없는가? | □ |
| **규격** | config·상수 분리, 프로젝트 스타일(Lint)을 준수했는가? | □ |

---

## 5. 실행 및 진행 (Execution)

1. **단계별 진행:** 한 번에 전부 하지 않고, 아래 진행 단계에서 한 단계씩 수행 후 체크리스트로 검증.
2. **에이전트 활용:** "`DDVC/101_Refactoring.md`의 [해당 단계] 실행해줘" 형태로 지시. 결과 요약 확인 후 다음 단계 지시.
3. **Staging 검증:** Isaac Sim에서 확장 로드 → Sim2Sim·Sensor Lab 창 동작 확인.
4. **커밋:** `refactor:`, `perf:` 접두사로 변경 사항 명시.

---

## 6. 진행 단계 참고 (Recommended Phases)

PRD 1차 목표(통합 + sim2sim_test 구조 최적화)에 맞춘 권장 순서.

| 순서 | 단계 | 요약 |
|------|------|------|
| 1 | **비관련 코드 식별** | `irim_lab_python` 하위를 Sim2Sim용 / Sensor Lab용 / 비관련으로 분류, 제거 후보 목록 작성. |
| 2 | **비관련 코드 제거** | 사용자 확인 후 제거, 참조 정리, 동작 체크리스트 제시. |
| 3 | **폴더 단순화 제안** | 폴더 병합·이동 방안 제안(import 변경 요약 포함). 실제 수정은 다음 단계에서 1건씩. |
| 4 | **폴더 단순화 적용** | 제안 중 선택한 1건만 적용, import·경로 수정, 동작 확인. |
| 5 | **역할 나누기** (선택) | `scenario.py` / `ui_builder.py`에서 한 블록씩 분리(오버레이, 이벤트 핸들러 등). |
| 6 | **중복·dead code 정리** | 미사용 모듈·함수, 중복 파일 정리. |
| 7 | **sim2sim_test / sensor_test 분리** | 이미 적용된 구조 유지. 통합(sim2sim_console) 시 진입점·메뉴 통합 검토. |

**현재 구조 (적용 완료 반영):**  
`extension.py`(루트) → `sim2sim_test/`(ui_builder, scenario, ros2, controllers, rmpflow), `sensor_test/`(sensor_lab_ui_builder, sensor_lab_joint_torque_test). 공통: `config/`, `core/`, `ui/`, `utils/` 루트 유지. `sim2sim_console/`는 1차 통합 대상으로 유지. (이벤트 핸들러는 ui_builder 내부로 통합됨.)

---

## 7. 문서 참조

| 문서 | 용도 |
|------|------|
| `100_PRD.md` | 1차 목표(통합·구조 최적화), 기능·성공 기준 |
| `101_Refactoring.md` | 본 문서 — 리팩토링 원칙·전략·체크리스트·진행 단계 |
| `102_*` | 구조·모듈·기술 스펙 (추후) |
