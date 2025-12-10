# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# """
# drum_quantizer_minimal.py
# - 함수: quantize_drum_midi(input_path) -> str
#   · 전 구간을 0.125초(=120BPM 16분) 그리드로 스냅
#   · 시작: 최근접 0.125s 배수
#   · 끝: 다음 0.125s 경계(최소 1ms 보장, 곡 끝 초과 금지)
#   · 같은 (pitch, grid_idx) 중복은 velocity 최댓값 1개만 유지
#   · 드럼 트랙만 교체, 비드럼은 그대로 보존
#   · 출력 파일명: 입력 + "_양자화.mid"
#   · 반환값: 생성된 출력 파일 경로 (문자열)

# - 메인:
#   · 경로 입력 → 함수 호출 → 반환된 경로만 출력
# """

# import os
# import sys
# from collections import defaultdict
# from typing import Dict, List, Tuple
# import pretty_midi


# def quantize_drum_midi(input_path: str) -> str:
#     """입력 MIDI의 드럼 노트를 0.125초 그리드로 양자화하고, 생성된 파일 경로를 반환한다."""
#     if not os.path.isfile(input_path):
#         raise FileNotFoundError(f"Input MIDI not found: {input_path}")

#     # 내부 파라미터
#     GRID_SEC = 0.05      # 120 BPM 기준 16분음표 길이(초)
#     MIN_DUR_SEC = 0.001   # 최소 길이 1ms

#     def to_idx_nearest(t: float) -> int:
#         return max(0, int(round(t / GRID_SEC)))

#     def idx_time(i: int) -> float:
#         return i * GRID_SEC

#     pm = pretty_midi.PrettyMIDI(input_path)
#     song_end = pm.get_end_time()

#     # (pitch, grid_idx) -> 후보 노트들
#     bucket: Dict[Tuple[int, int], List[pretty_midi.Note]] = defaultdict(list)

#     # 드럼 노트만 수집/스냅
#     for inst in pm.instruments:
#         if not inst.is_drum:
#             continue
#         for note in inst.notes:
#             idx = to_idx_nearest(max(0.0, note.start))
#             snapped_start = idx_time(idx)
#             next_boundary = idx_time(idx + 1)
#             snapped_end = max(snapped_start + MIN_DUR_SEC, min(next_boundary, song_end))

#             bucket[(note.pitch, idx)].append(pretty_midi.Note(
#                 velocity=note.velocity,
#                 pitch=note.pitch,
#                 start=snapped_start,
#                 end=snapped_end
#             ))

#     # 중복 해소: 동일 (pitch, grid_idx)에서 velocity 최댓값 1개만 유지
#     snapped_notes: List[pretty_midi.Note] = []
#     for (pitch, idx), cands in bucket.items():
#         best = max(cands, key=lambda n: n.velocity)
#         snapped_notes.append(best)
#     snapped_notes.sort(key=lambda n: n.start)

#     # 드럼 트랙 재구성(단일 트랙으로 교체), 비드럼 트랙 보존
#     non_drums = [i for i in pm.instruments if not i.is_drum]
#     drum_out = pretty_midi.Instrument(program=0, is_drum=True, name="Drums(quantized-0.125s)")
#     drum_out.notes = snapped_notes
#     pm.instruments = non_drums + [drum_out]

#     # 출력 경로 생성 및 저장
#     base, _ = os.path.splitext(input_path)
#     output_path = f"{base}_quantizer.mid"
#     pm.write(output_path)
#     return output_path


# # --- 메인: 경로 입력 -> 함수 호출 -> 반환 경로만 출력 ---
# if __name__ == "__main__":
#     in_path = input("입력 MIDI 파일 경로: ").strip()
#     try:
#         out_path = quantize_drum_midi(in_path)
#         # 요구사항: 반환 경로만 출력
#         print(out_path)
#     except Exception as e:
#         print(f"ERROR: {e}", file=sys.stderr)
#         sys.exit(1)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
drum_quantizer_csv.py
- 기능 1: MIDI 양자화 (기존 기능)
- 기능 2: 양자화된 데이터를 기반으로 로봇 제어용 CSV 파일 생성 (신규 기능)
  · CSV 형식: Time(sec), MotorID, Velocity
  · 파일명 규칙: drum_recording_X_Y.mid -> drum_events_X_Y.csv
"""

import os
import sys
import csv
from collections import defaultdict
from typing import Dict, List, Tuple
import pretty_midi

# 1. 로봇 모터 매핑 정의
ROBOT_MAP = {
    38: 1, 41: 2, 45: 3, 47: 4, 48: 4, 50: 4, 
    42: 5, 51: 6, 49: 7, 57: 8, 36: 10, 46: 11
}

def quantize_drum_midi(input_path: str) -> str:
    """
    입력 MIDI를 양자화하여 .mid로 저장하고, 
    동시에 로봇 제어용 .csv 파일을 생성합니다.
    """
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"Input MIDI not found: {input_path}")

    # --- [Step 1] MIDI 양자화 로직 ---
    GRID_SEC = 0.05
    MIN_DUR_SEC = 0.001
    
    # 그리드 계산 함수
    def to_idx_nearest(t: float) -> int:
        return max(0, int(round(t / GRID_SEC)))
    def idx_time(i: int) -> float:
        return i * GRID_SEC

    pm = pretty_midi.PrettyMIDI(input_path)
    song_end = pm.get_end_time()

    bucket: Dict[Tuple[int, int], List[pretty_midi.Note]] = defaultdict(list)

    # 3. 드럼 트랙만 추출
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

    # 중복 해소 및 정렬
    snapped_notes: List[pretty_midi.Note] = []
    for (pitch, idx), cands in bucket.items():
        best = max(cands, key=lambda n: n.velocity)
        snapped_notes.append(best)
    
    # 시간순 정렬 (CSV 저장을 위해 필수)
    snapped_notes.sort(key=lambda n: n.start)

    # MIDI 구조 재구성
    non_drums = [i for i in pm.instruments if not i.is_drum]
    drum_out = pretty_midi.Instrument(program=0, is_drum=True, name="Drums(quantized)")
    drum_out.notes = snapped_notes
    pm.instruments = non_drums + [drum_out]

    # --- [Step 2] 파일 경로 생성 ---
    dir_name, filename = os.path.split(input_path)
    base_name, _ = os.path.splitext(filename)

    # MIDI 출력 경로 (기존 위치에 저장)
    midi_output_path = os.path.join(dir_name, f"{base_name}_quantized.mid")
    
    # ---------------------------------------------------------
    # [수정됨] CSV 저장 경로 로직
    # 1. 현재 폴더(dir_name)의 '부모 폴더'를 찾습니다.
    parent_dir = os.path.dirname(dir_name)
    
    # 2. 부모 폴더 아래에 'velocity' 폴더 경로를 설정합니다.
    velocity_dir = os.path.join(parent_dir, "velocity")
    
    # 3. 해당 폴더가 없으면 자동으로 생성합니다. (에러 방지)
    os.makedirs(velocity_dir, exist_ok=True)

    # 4. 파일명 결정
    if "drum_recording" in base_name:
        csv_filename = base_name.replace("drum_recording", "drum_events") + ".csv"
    else:
        csv_filename = base_name + "_events.csv"
    
    # 5. 최종 CSV 경로 결합
    csv_output_path = os.path.join(velocity_dir, csv_filename)
    # ---------------------------------------------------------

    # --- [Step 3] CSV 저장 로직 (신규 추가) ---
    try:
        with open(csv_output_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # 헤더 작성 (Time, ID, Velocity)
            writer.writerow(['Time', 'ID', 'Velocity'])
            
            for note in snapped_notes:
                # 1. 매핑에 존재하는 피치만 저장
                if note.pitch in ROBOT_MAP:
                    inst_id = ROBOT_MAP[note.pitch]
                    
                    # 2. 초(sec) 단위 시간 (소수점 4자리까지 표기 권장)
                    # 4. 동시 입력은 루프가 돌면서 같은 시간 값으로 자연스럽게 여러 줄 생성됨
                    writer.writerow([f"{note.start:.4f}", inst_id, note.velocity])
                    
        print(f"CSV Saved: {csv_output_path}")
        
    except IOError as e:
        print(f"Error saving CSV: {e}", file=sys.stderr)

    # MIDI 파일 저장
    pm.write(midi_output_path)
    
    return midi_output_path

# --- 메인 실행 ---
if __name__ == "__main__":
    in_path = input("입력 MIDI 파일 경로: ").strip()
    try:
        # 경로의 따옴표 제거 (터미널 입력 시 발생 가능성)
        in_path = in_path.strip("'").strip('"')
        
        out_mid = quantize_drum_midi(in_path)
        print(f"MIDI Saved: {out_mid}")
        
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)