# IRIM_LAB PRD (Product Requirement Document)

Isaac Sim 확장으로 ALLEX 로봇의 디지털 트윈(Sim2Sim) 및 센서/제어 실험(Sensor Lab) 환경을 제공하기 위한 요구사항 문서입니다.

---

## 1) 프로젝트 목적

- **\[1차 목표] sim2sim_console과 IRIM_LAB 통합**: `irim_lab_python/sim2sim_console`(Sim2Real Debugger, policy·Hydra 연동 등)과 `IRIM_LAB` 확장(Sim2Sim 창, Sensor Lab, scenario·ROS2)을 **하나의 확장 플로우로 통합**하는 것이 이 프로젝트의 첫 번째 목표이다. 현재는 별도 모듈로 존재하므로, 메뉴·데이터·실행 경로를 통합해 사용성이 올라가도록 한다.
- **ALLEX 디지털 트윈**: Isaac Sim 상에서 ALLEX 휴머노이드 모델을 로드하고, ROS2·외부 제어와 연동하여 Sim2Sim(시뮬레이션 간/시뮬레이션–실기 연동) 검증
- **센서/제어 실험**: 토크 센서 등 센서 테스트 에셋 로드, 조인트 토크·외력 시각화, 디버깅용 UI 제공
- **재사용·확장**: Robotics_Study 등 참고 구조를 활용한 모듈화, 설정(config) 분리, 유지보수 용이

---

## 2) 기능 컨셉 요약

| 영역 | 설명 |
|------|------|
| **Sim2Sim** | 로봇 에셋 로드, LOAD/RESET/RUN 제어, 관절 제어(슬라이더/ROS2), 관절 그룹별 표시 on/off, 오버레이(관절/손 데이터), Follow Cube·IK 등 |
| **Sensor Lab** | 센서 테스트 전용 에셋 로드, 카메라 뷰, 구 등 Rigid Body 생성, 토크 테스트 UI |
| **ROS2** | Publisher/Subscriber 독립 제어, 통합/개별 토픽 모드, 관절 위치·토크·정책 액션 등 송수신 |
| **시각화** | Force/Torque 시각화, 관절 위치·손 관절 텍스트 오버레이 |

---

## 3) 구현 방향

- **플랫폼**: Isaac Sim Extension (Python), Omniverse Kit 메뉴·UI
- **구조**: `extension.py` → UIBuilder / SensorLabUIBuilder, `scenario.py`(ALLEXDigitalTwin) 및 core(config, core, ros2, event_handlers, ui) 모듈 분리
- **설정**: `config/`(joint, ros2, ui, visibility 등), `global_variables.py`로 제목·상수 관리
- **참고**: Robotics_Study의 기능별 scenario+ui 분리, ScrollingWindow·이벤트 구독 패턴

---

## 4) 산출물·주요 기능

- **\[1차 목표] sim2sim_console–IRIM_LAB 통합**: Sim2Real Debugger·policy 등이 포함된 `sim2sim_console`과 확장 메인 플로우(Sim2Sim/Sensor Lab)를 통합한 단일 사용 경로 제공.
- **Extension 메뉴**: Sim2Sim 창, Sensor Lab 창 (각각 토글)
- **Sim2Sim**: World Controls(LOAD/RESET), Run Scenario(RUN/STOP), Joint Controls, ROS2 Manager, 관절 그룹 체크박스, 오버레이
- **Sensor Lab**: World Controls(LOAD 등), 센서 테스트용 에셋·카메라·구 생성
- **문서**: DDVC 폴더 내 PRD(본 문서), 이후 System Architecture & Tech Spec, Rules

---

## 5) 문서 구성 (DDVC)

- `DDVC/01_PRD.md` — 본 문서 (목적·기능·방향·산출물)
- `DDVC/02_Refactoring_Plan.md` — 리팩터링 계획 (태스크 단위, 에이전트 지시용)
- `DDVC/02_System_Architecture_And_Tech_Spec.md` — 구조·모듈·기술 스펙 (추후 작성)
- `DDVC/03_Commit_Convention.md` — 커밋 메시지 규칙 (Conventional Commits, 7대 원칙)
- `DDVC/03_Rules.md` — 코딩 규칙·네이밍·컨벤션 (추후 작성)
- `DDVC/.cursorrules` — DDVC 폴더 참고 지침

---

## 6) 성공 기준 (예시)

- Sim2Sim: LOAD → ROS2 연동 → RUN 시 관절 제어 및 오버레이 정상 동작
- Sensor Lab: 전용 에셋 로드 및 토크 테스트 UI 동작
- 확장 리로드·창 토글 시 크래시 없음 (extension 방어 코드 유지)
