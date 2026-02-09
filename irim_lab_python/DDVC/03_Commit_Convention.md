# 03 — 커밋 메시지 규칙 (Conventional Commits)

전문가들과 잘 관리된 오픈소스 프로젝트에서 가장 범용적으로 사용하는 **Conventional Commits** 규격을 따른다. 메시지만 보고도 **어떤 종류의 변경인지**, **어느 범위(Scope)를 건드렸는지** 직관적으로 알 수 있게 작성하는 것이 핵심이다.

---

## 1. 기본 구조

커밋 메시지는 아래 3단계 구조를 따른다. (본문·바닥글은 선택.)

```text
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

---

## 2. 커밋 타입 (Type)

변경의 성격을 규정한다.

| 타입 | 의미 | 예시 |
|------|------|------|
| **`feat`** | 새로운 기능 추가 | `feat: add habilis brain inference logic` |
| **`fix`** | 버그 수정 | `fix: resolve ik solver convergence issue` |
| **`docs`** | 문서 수정 (Markdown 등) | `docs: update prd for gripper control` |
| **`style`** | 코드 포맷팅 (세미콜론, 공백 등 로직 변경 X) | `style: linting via black` |
| **`refactor`** | 코드 리팩토링 (기능은 그대로, 구조 개선) | `refactor: optimize point cloud processing` |
| **`test`** | 테스트 코드 추가 및 수정 | `test: add unit test for quaternion utility` |
| **`chore`** | 빌드 업무, 패키지 매니저 설정 등 | `chore: update torch version in requirements` |

---

## 3. 작성 규칙 (7대 원칙)

1. **제목과 본문 사이는 한 줄을 띄운다** (가독성 확보).
2. **제목은 50자 이내**로 요약한다.
3. **제목 첫 글자는 대문자**로 시작한다 (영문 기준).
4. **제목 끝에 마침표(.)를 찍지 않는다**.
5. **제목은 명령문 형태**로 작성한다 (예: Fixed가 아닌 **Fix**, Added가 아닌 **Add**).
6. **본문(Body)은 '무엇을', '왜' 했는지**에 집중한다 (어떻게는 코드에 드러남).
7. **제목에 변경된 위치(Scope)를 명시**하면 더 좋다 (예: `feat(isaac-sim): ...`).

---

## 4. 실전 예시 (로보틱스·IRIM_LAB 맥락)

**[사례 1: 새로운 기능 추가]**

```text
feat(control): implement system-2 reasoning for habilis

- CLIP-RT 기반의 대조 학습 로직 추가
- 실시간 제어를 위한 163Hz 추론 파이프라인 최적화
```

**[사례 2: 설계 문서 기반 수정 (DDVC)]**

```text
docs(spec): sync hand-arm kinematics with latest design

- kinematics_model.md의 관절 한계값 수정에 따른 업데이트
- 실제 로봇의 20자유도 사양 반영
```

**[사례 3: 버그 수정]**

```text
fix(env): solve contact reporting error in Isaac Lab

- PhysX 엔진의 컨택 포인트 계산 누락 현상 수정
```

**[사례 4: IRIM_LAB 확장]**

```text
feat(irim_lab): add Sim2Sim window and Sensor Lab menu

- extension.py에 Sim2Sim/Sensor Lab 창 토글 등록
- PRD 1차 목표(sim2sim_console 통합) 문서 반영
```

---

## 5. Cursor AI로 커밋 메시지 자동 작성

Cursor를 사용 중이라면 커밋 메시지를 직접 쓰지 말고, 아래처럼 요청한다.

- **명령 예시:**  
  "현재 변경 사항을 **Conventional Commits** 양식에 맞춰서 커밋 메시지로 작성해 줘. `DDVC/03_Commit_Convention.md` 규칙을 따르고, 수정된 PRD·문서를 참조했으면 `docs` 타입을 써 줘."
- **이점:** AI가 변경 이력을 보고 `feat`, `fix`, `docs` 등을 구분해 준다.

---

## 6. 에이전트가 커밋까지 직접 수행하도록 하는 지침

사용자가 **"커밋 메시지 작성 + 커밋까지 실행해 줘"**라고 요청하면, 에이전트는 아래 순서로 수행한다.

1. **변경 사항 확인**  
   - `git status`, `git diff --stat` 등으로 변경·삭제·추가된 파일을 파악한다.

2. **커밋 메시지 작성**  
   - 본 문서(§1~§3)의 Conventional Commits 규격과 7대 원칙에 맞춰 **제목 + 본문**을 작성한다.  
   - 변경 성격에 맞는 타입(`feat`, `fix`, `docs`, `refactor`, `style`, `test`, `chore`)과 필요 시 scope를 붙인다.

3. **스테이징**  
   - 커밋에 포함할 경로를 `git add <path>` 또는 `git add -A`(전체)로 스테이징한다.  
   - 사용자가 특정 파일만 커밋하라고 하지 않은 한, **이번 변경과 관련된 파일만** 넣는다. (불필요한 파일·폴더는 제외.)

4. **커밋 실행**  
   - `git commit -m "<제목>" -m "<본문>"` 형태로, 작성한 메시지를 그대로 넣어 커밋한다.  
   - 본문이 여러 줄이면 `-m "첫 줄" -m "둘째 줄"` 처럼 `-m`을 반복하거나, 임시 파일에 메시지를 쓴 뒤 `git commit -F <파일>`을 쓴다.

5. **결과 확인**  
   - `git log -1 --oneline` 또는 `git show --stat`으로 방금 만든 커밋이 의도대로 들어갔는지 한 줄 요약해 사용자에게 알린다.

**사용자 요청 예시**  
- "지금 변경 사항을 DDVC/03_Commit_Convention.md 규칙으로 커밋 메시지 작성해 줘. **그 메시지로 커밋까지 실행해 줘.**"  
- "제안한 커밋 메시지로 **자동으로 커밋까지** 에이전트가 직접 하도록 … **너도 직접 커밋까지 마무리해 줘.**"

---

## 7. 팀 협업 시

연구실(Lab) 단위 협업이라면 **커밋 타입 목록만이라도 팀 내에서 통일**하면, 나중에 특정 실험·버그 수정 이력을 추적할 때 속도가 빨라진다.
