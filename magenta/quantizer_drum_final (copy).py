#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
drum_quantizer_minimal.py
- 함수: quantize_drum_midi(input_path) -> str
  · 전 구간을 0.125초(=120BPM 16분) 그리드로 스냅
  · 시작: 최근접 0.125s 배수
  · 끝: 다음 0.125s 경계(최소 1ms 보장, 곡 끝 초과 금지)
  · 같은 (pitch, grid_idx) 중복은 velocity 최댓값 1개만 유지
  · 드럼 트랙만 교체, 비드럼은 그대로 보존
  · 출력 파일명: 입력 + "_양자화.mid"
  · 반환값: 생성된 출력 파일 경로 (문자열)

- 메인:
  · 경로 입력 → 함수 호출 → 반환된 경로만 출력
"""

import os
import sys
from collections import defaultdict
from typing import Dict, List, Tuple
import pretty_midi


def quantize_drum_midi(input_path: str) -> str:
    """입력 MIDI의 드럼 노트를 0.125초 그리드로 양자화하고, 생성된 파일 경로를 반환한다."""
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"Input MIDI not found: {input_path}")

    # 내부 파라미터
    GRID_SEC = 0.125      # 120 BPM 기준 16분음표 길이(초)
    MIN_DUR_SEC = 0.001   # 최소 길이 1ms

    def to_idx_nearest(t: float) -> int:
        return max(0, int(round(t / GRID_SEC)))

    def idx_time(i: int) -> float:
        return i * GRID_SEC

    pm = pretty_midi.PrettyMIDI(input_path)
    song_end = pm.get_end_time()

    # (pitch, grid_idx) -> 후보 노트들
    bucket: Dict[Tuple[int, int], List[pretty_midi.Note]] = defaultdict(list)

    # 드럼 노트만 수집/스냅
    for inst in pm.instruments:
        if not inst.is_drum:
            continue
        for note in inst.notes:
            idx = to_idx_nearest(max(0.0, note.start))
            snapped_start = idx_time(idx)
            next_boundary = idx_time(idx + 1)
            snapped_end = max(snapped_start + MIN_DUR_SEC, min(next_boundary, song_end))

            bucket[(note.pitch, idx)].append(pretty_midi.Note(
                velocity=note.velocity,
                pitch=note.pitch,
                start=snapped_start,
                end=snapped_end
            ))

    # 중복 해소: 동일 (pitch, grid_idx)에서 velocity 최댓값 1개만 유지
    snapped_notes: List[pretty_midi.Note] = []
    for (pitch, idx), cands in bucket.items():
        best = max(cands, key=lambda n: n.velocity)
        snapped_notes.append(best)
    snapped_notes.sort(key=lambda n: n.start)

    # 드럼 트랙 재구성(단일 트랙으로 교체), 비드럼 트랙 보존
    non_drums = [i for i in pm.instruments if not i.is_drum]
    drum_out = pretty_midi.Instrument(program=0, is_drum=True, name="Drums(quantized-0.125s)")
    drum_out.notes = snapped_notes
    pm.instruments = non_drums + [drum_out]

    # 출력 경로 생성 및 저장
    base, _ = os.path.splitext(input_path)
    output_path = f"{base}_quantizer.mid"
    pm.write(output_path)
    return output_path


# --- 메인: 경로 입력 -> 함수 호출 -> 반환 경로만 출력 ---
if __name__ == "__main__":
    in_path = input("입력 MIDI 파일 경로: ").strip()
    try:
        out_path = quantize_drum_midi(in_path)
        # 요구사항: 반환 경로만 출력
        print(out_path)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
