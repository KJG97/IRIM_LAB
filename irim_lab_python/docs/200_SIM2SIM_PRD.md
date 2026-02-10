# Sim2Sim 프로젝트 문서 (PRD)

이 문서는 Isaac Lab에서 학습한 정책(.pt)을 Isaac Sim에서 배포(Deploy)하는 **Sim2Sim** 프로젝트의 제품 요구사항을 정의합니다. Sim2Real용 디버거 GUI를 기반으로, ROS2 없이 Isaac Sim 내부 API와 연동하는 Sim2Sim 전용 흐름을 구축하는 것을 목표로 합니다.

---

## 1) 프로젝트 목적

- Isaac Lab에서 학습한 .pt 파일을 Isaac Sim 환경에서 그대로 배포·추론하는 Sim2Sim 파이프라인 구축
- Sim2Real(ROS2)과 Sim2Sim(Isaac Sim 내부 API) **모드 분리**로 용도별 실행 경로 명확화
- 학습 시 사용한 env.yaml, agent.yaml과 Isaac Sim 환경이 일치하는지 **디버깅 가능한 기능** 제공

---

## 2) 기능 컨셉 요약

- **Sim2Sim 전용 실행**: ROS2 미사용, Isaac Sim 내부 API만으로 정책 추론 실행
- **sim2real_debugger_gui 연동**: sim2sim_test UI에 디버거 GUI 실행 버튼 추가 → Isaac Sim과 한 흐름으로 연동
- **환경 일치 검증**: env.yaml, agent.yaml 기반으로 학습 환경과 Sim 배포 환경 일치 여부 확인·디버깅
- **모드 분리**: Sim2Real = ROS2, Sim2Sim = Isaac Sim API 각각 독립 모드로 동작

---

## 3) 구현 방향

- **UI**: sim2sim_test UI에 sim2real_debugger_gui를 실행하는 버튼 추가, Isaac Sim 내부에서 동일 흐름으로 접근
- **코드**: 기존 sim2real_debugger_gui 기능을 토대로, ROS2를 쓰지 않는 **Sim2Sim 전용 코드** 신규 개발
- **추론**: Sim2Sim 모드에서는 Isaac Sim 내부 API와만 연동하여 정책 추론 실행
- **설정 검증**: env.yaml, agent.yaml을 읽어 학습 환경과 Isaac Sim 환경이 일치하는지 디버깅할 수 있는 기능 추가

---

## 4) 핵심 요구사항

- sim2sim_test → sim2real_debugger_gui 실행 → Isaac Sim 연동이 하나의 사용자 플로우로 동작
- Sim2Real(ROS2) / Sim2Sim(Isaac Sim API) 모드가 명확히 분리되어 선택·실행 가능
- 학습 설정(env.yaml, agent.yaml)과 Sim 환경 불일치 시 확인·보고 가능한 디버깅 기능
- Isaac Sim과 sim2real_debugger_gui 연동이 비효율적이라고 판단될 경우 **사용자에게 즉시 보고**

---

## 5) Sim2Real Debugger GUI — 의존성 설치 (Isaac Sim)

Sim2Real Debugger GUI는 **PySide6**와 **pyqtgraph**를 사용합니다. Isaac Sim 번들 Python에는 기본적으로 없으므로, **Isaac Sim이 사용하는 Python**으로 아래처럼 한 번 설치합니다.

**Isaac Sim이 설치된 경로의 `kit/python/bin`** 아래 Python으로 pip를 실행하면 됩니다.

```bash
# 이 프로젝트(workspace) 기준 — kit/python/bin 에 python3, python3.11 있음
/home/jkkim/isaac-sim/kit/python/bin/python3 -m pip install PySide6 pyqtgraph
```

다른 설치 형태 예시:

```bash
# 일반적인 경우: 경로만 본인 Isaac Sim 설치 경로로 바꿔서 사용
<ISAAC_SIM_ROOT>/kit/python/bin/python3 -m pip install PySide6 pyqtgraph
```

```bash
# NVIDIA 런처로 설치한 경우 (python.sh 사용)
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh -m pip install PySide6 pyqtgraph
```

설치 후 Isaac Sim을 다시 띄우고, sim2sim_test UI의 **Sim2sim Deploy → Open Sim2Real Debugger** 버튼으로 디버거 창을 열 수 있습니다.

---

## 6) 참고 자료

- **문서**: [Isaac Sim Policy Deployment (5.1.0)](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/isaac_lab_tutorials/tutorial_policy_deployment.html)
- **참고 코드**  
  - `exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/franka.py`  
  - `exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/utils/actuator_network.py`
