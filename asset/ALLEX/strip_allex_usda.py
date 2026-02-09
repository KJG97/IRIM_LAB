#!/usr/bin/env python3
"""
ALLEX_Param_text_sim_only.txt에서 Mesh·Looks(머티리얼) 블록을 제거합니다.
동역학·기구학·조인트·드라이브 파라미터만 남깁니다.

실행: python3 strip_allex_usda.py
"""

import sys
import time
from pathlib import Path

# 제거할 블록 시작 패턴
SKIP_PATTERNS = [b'def Mesh ', b'def Scope "Looks"']


def should_skip(line_stripped):
    return any(p in line_stripped for p in SKIP_PATTERNS)


def run():
    asset_dir = Path(__file__).resolve().parent
    src = asset_dir / "ALLEX_Param_text_sim_only.txt"
    out = asset_dir / "ALLEX_Param_sim_params.txt"

    if not src.exists():
        print(f"파일 없음: {src}")
        return 1

    total = src.stat().st_size
    print(f"입력: {src.name} ({total / 1024**2:.1f} MB)")
    print("Mesh·Looks 블록 제거 중...")

    start = time.time()
    read_bytes = 0
    skip_depth = 0          # > 0 이면 블록 본체 스킵 중
    waiting_for_brace = False  # 패턴 매칭 후 { 를 아직 못 찾은 상태

    with open(src, "rb") as fin, open(out, "wb") as fout:
        for i, line in enumerate(fin, 1):
            read_bytes += len(line)

            if i % 500 == 0:
                pct = read_bytes / total * 100
                elapsed = time.time() - start
                print(f"\r  진행: {pct:.1f}% ({i:,}줄, 경과 {elapsed:.0f}초)", end="", flush=True)

            stripped = line.lstrip()

            # ① 블록 본체 스킵 중 → { } 카운트
            if skip_depth > 0:
                skip_depth += stripped.count(b"{") - stripped.count(b"}")
                continue

            # ② 패턴 매칭 후 아직 { 를 찾는 중 (def Mesh "..." ( ... ) 부분)
            if waiting_for_brace:
                if b"{" in stripped:
                    skip_depth = stripped.count(b"{") - stripped.count(b"}")
                    waiting_for_brace = False
                continue  # ( ... ) 헤더도 스킵

            # ③ 새 스킵 패턴 감지
            if should_skip(stripped):
                if b"{" in line:
                    # 같은 줄에 { 있음 → 바로 본체 진입
                    skip_depth = line.count(b"{") - line.count(b"}")
                else:
                    # { 는 아래 줄에 있음 (def Mesh "..." ( 형태)
                    waiting_for_brace = True
                continue

            fout.write(line)

    elapsed = time.time() - start
    print(f"\r  완료: {i:,}줄 처리 (경과 {elapsed:.1f}초)                    ")

    if out.exists():
        sz = out.stat().st_size
        if sz > 1024 * 1024:
            print(f"출력: {out.name} ({sz / 1024**2:.2f} MB)")
        else:
            print(f"출력: {out.name} ({sz / 1024:.1f} KB)")
    print("완료.")
    return 0


if __name__ == "__main__":
    sys.exit(run())
