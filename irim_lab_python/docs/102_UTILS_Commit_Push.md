# 03 — 커밋 메시지 규칙 (Conventional Commits)

**Conventional Commits**를 따른다. 메시지만 보고 **변경 종류**와 **범위(Scope)**를 바로 알 수 있게 쓴다.

---

## 1. 구조

```
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

---

## 2. 타입 (Type)

| 타입 | 의미 | 예시 |
|------|------|------|
| `feat` | 기능 추가 | `feat: add habilis brain inference logic` |
| `fix` | 버그 수정 | `fix: resolve ik solver convergence issue` |
| `docs` | 문서 수정 | `docs: update prd for gripper control` |
| `style` | 포맷팅만 (로직 변경 없음) | `style: linting via black` |
| `refactor` | 리팩터링 (기능 동일) | `refactor: optimize point cloud processing` |
| `test` | 테스트 추가/수정 | `test: add unit test for quaternion utility` |
| `chore` | 빌드·패키지 등 | `chore: update torch version in requirements` |

---

## 3. 작성 규칙

- 제목과 본문 사이 **한 줄 띄우기**
- 제목 **50자 이내**, **첫 글자 대문자**, **끝에 마침표 없음**
- 제목은 **명령문** (Fix, Add — Fixed, Added 아님)
- 본문은 **무엇을·왜** 위주 (어떻게는 코드에)
- 가능하면 **scope 명시** (예: `feat(isaac-sim): ...`)

---

## 4. 에이전트: 커밋 + push 절차

사용자가 **"커밋해 줘"** / **"커밋 + push 해 줘"**라고 하면 아래 순서로 끝까지 수행한다.

1. **변경 확인** — `git status`, `git diff --stat`으로 변경 파일 파악
2. **메시지 작성** — 위 §1~§3 규격·규칙에 맞춰 제목(+ 본문), 타입·scope 적용
3. **스테이징** — 관련 파일만 `git add` (지정 없으면 변경분만; 불필요 파일 제외)
4. **커밋** — `git commit -m "<제목>" -m "<본문>"` 또는 본문 여러 줄이면 `-m` 반복 / `git commit -F <파일>`
5. **확인** — `git log -1 --oneline` 또는 `git show --stat`으로 커밋 검증
6. **push**
   - `git remote -v`로 원격 확인 후, 추적 중이면 `git push`, 아니면 `git push -u origin <현재브랜치>`
   - **실패 시**: 원격이 앞서 있으면 에러 전달 + `git pull --rebase` / `git pull` 후 재시도 안내; 인증 실패는 사용자에게 맡김
   - **성공 시**: `git log -1 --oneline`과 함께 "커밋 및 push 완료" 요약

**요청 예시**  
- "지금 변경 사항 DDVC 규칙으로 커밋 메시지 작성해 줘. 그걸로 **커밋하고 push까지** 해 줘."  
- "**커밋 + push**까지 에이전트가 다 해 줘."

---

## 5. 팀 협업

연구실 단위에서는 **타입 목록만이라도 통일**하면 실험·버그 수정 이력 추적이 빨라진다.
