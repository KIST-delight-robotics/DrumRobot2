"""
drum_quantizer.py
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
    38: 1, 
    43: 2, 
    45: 3, 
    47: 4, 48: 4, 50: 4, 
    42: 5, 22: 5, 46: 5, 26: 5, 
    51: 6, 
    57: 7, 
    49: 8
    # 36: 10
}

def quantize_drum_midi(input_path: str, bpm) -> str:    #input_path = 'record/codeName.mid'
    """
    입력 MIDI를 양자화하여 .mid로 저장하고, 
    동시에 로봇 제어용 .csv 파일을 생성합니다.
    """
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"Input MIDI not found: {input_path}")

    # --- [Step 1] MIDI 양자화 로직 ---
    GRID_SEC = 0.6 * (100 / bpm) / 4 # 1/4박자 길이
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
        csv_filename = base_name + ".csv"
    
    # 5. 최종 CSV 경로 결합
    csv_output_path = os.path.join(velocity_dir, csv_filename)
    # ---------------------------------------------------------

    # --- [Step 3] CSV 저장 로직 (신규 추가) ---
    try:
        with open(csv_output_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # 헤더 작성 (Time, ID, Velocity)
            writer.writerow(['Time', 'ID', 'Velocity'])

            time_ex_note = 0
            ex_note = 0
            for note in snapped_notes:
            # for note in sorted(snapped_notes, key=lambda x: x.start):
                if note.pitch in ROBOT_MAP:
                    inst_id = ROBOT_MAP[note.pitch]
                    time_cur_note = note.start * bpm /100     # note 시작 시간을 100bpm 기준 시간으로 변환, 이후 모든 계산은 100bpm 기준으로 진행
                    duration = time_cur_note - time_ex_note           # 이전 노트와의 시간 간격
                    note_duration = note.end - note.start
                    if abs(duration) < 1e-3 and ex_note == inst_id:
                        continue
                    writer.writerow([round(time_cur_note, 4), inst_id, note.velocity, note_duration])
                    time_ex_note = time_cur_note
                    ex_note = inst_id
                    
        print(f"CSV Saved: {csv_output_path}")
        
    except IOError as e:
        print(f"Error saving CSV: {e}", file=sys.stderr)

    # MIDI 파일 저장
    pm.write(midi_output_path)
    
    return midi_output_path

# velocity 데이터를 기반으로 코드 파일의 offset 수정
def modify_intensity(input_path: str) -> None:      # input_path = codeName 예) velocity/solo2.csv 면 input_path = "solo2"
    # code 파일을 열고 drum_velocity 파일과 비교하여 intensity를 수정하는 함수
    code_path = os.path.join("../include/codes", input_path + "0.txt" )    # 악보 번호가 "1.txt"인 경우는 아직 구현 안함
    velo_path = os.path.join("../magenta/velocity", input_path + ".csv" )

    code = []
    velo = []
    min_vel = 70.0

    # print({code_path})
    # print({velo_path})

    try:
        with open(code_path, 'r', encoding='utf-8') as f:
            for line in f:
                row = line.strip().split('\t')
                for i in range(len(row)):
                    row[i] = trim_whitespace(row[i])
                row[1] = float(row[1])

                if len(row) == 8:
                    row.extend([0, 0])
                elif len(row) == 9:
                    row[8] = 0
                    row.append(0)
                elif len(row) == 10:
                    row[8] = int(row[8])
                    row[9] = int(row[9])
                    pass
                elif len(row) < 8:
                    pass
                else:
                    raise ValueError(f"Unexpected row length: {len(row)}")
                code.append(row)
        # print(code)
    except FileNotFoundError:
        print("파일을 찾을 수 없음")
    except Exception as e:
        print(f"에러 발생1: {e}")

    try:
        with open(velo_path, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            next(reader)  # 헤더 건너뛰기
            for row in reader:
                velo.append([float(row[0]),row[1],float(row[2]), 0])    # row[0]: time, row[1]: inst_id, row[2]: velocity, row[3]: used 여부 (0: 미사용, 1: 사용)
        # print(velo)
    except FileNotFoundError:
        print("파일을 찾을 수 없음")
    except Exception as e:
        print(f"에러 발생2: {e}")

    time_sum = 0
    first_note = False
    idx_first_note = 0
    elapsed_time = 0
    note_cnt = 0
    offset_cnt = 0

    # ver1
    for i in range(len(code)):

        if code[i][0] == 'bpm' or code[i][0] == 'end':
            continue

        if not first_note and (code[i][2] != '0' or code[i][3] != '0'):
            first_note = True
            idx_first_note = i

        if first_note:
            if code[i][2] != '0' or code[i][3] != '0':
                # print(f"i: {i}, code[i]: {code[i][2]}, {code[i][3]}")
                note_cnt += 1
                if code[i][2] != '0' and code[i][3] != '0':
                    note_cnt += 1
            time_sum += code[i][1]
            elapsed_time = round((time_sum - code[idx_first_note][1]), 3)

            right_inst = code[i][2]
            left_inst = code[i][3]

            # print(f"elapsed_time: {elapsed_time}, i: {i}, right_inst: {right_inst}, left_inst: {left_inst}")

            if right_inst != '0':
                for j in range(len(velo) - 1):
                    if abs(velo[j][0] - elapsed_time) > 0.2:
                        continue
                    elif abs(velo[j][0] - elapsed_time) <= 0.2:
                        if velo[j][2] >= min_vel:
                            continue
                        if velo[j][3] == 1:
                            continue
                        if velo[j][1] == code[i][2]:
                            print(f"elapsed time : {elapsed_time} / right inst : {code[i][2]} / velocity : {velo[j][2]} / offset : {code[i][8]}")
                            code[i][8] += 1
                            offset_cnt += 1
                            velo[j][3] = 1
                            print(f"------------------------------>> changed offset {code[i][8]}")
                            break
                       
            if left_inst != '0':
                for j in range(len(velo) - 1):
                    if abs(velo[j][0] - elapsed_time) > 0.2:
                        continue
                    elif abs(velo[j][0] - elapsed_time) <= 0.2:
                        if velo[j][2] >= min_vel:
                            continue
                        if velo[j][3] == 1:
                            continue
                        if velo[j][1] == code[i][3]:
                            print(f"elapsed time : {elapsed_time} / left inst : {code[i][3]} / velocity : {velo[j][2]} / offset : {code[i][9]}")
                            code[i][9] += 1
                            offset_cnt += 1
                            velo[j][3] = 1
                            print(f"------------------------------>> changed offset {code[i][9]}")
                            break

    min_vel_cnt = 0
    for j in range(len(velo)):
        if velo[j][2] < min_vel:
            min_vel_cnt += 1
    print(f"number of velocity is under {min_vel} : {min_vel_cnt}")
    print(f"number of offset changed : {offset_cnt}")
    output_path = os.path.join("../include/codes/", input_path + "0.txt")

    print({output_path})

    with open(output_path, "w", encoding="utf-8") as f:

        for row in code:
            line = '\t'.join(map(str, row))
            f.write(line + '\n')

# tap 기준으로 분리한 데이터에서 공백을 제거
def trim_whitespace(s: str) -> str:
    # 앞쪽 공백(스페이스, 탭) 아닌 첫 위치 찾기
    first = 0
    while first < len(s) and s[first] in (' ', '\t'):
        first += 1

    # 전부 공백이면 그대로 반환
    if first == len(s):
        return s

    # 뒤쪽 공백(스페이스, 탭) 아닌 마지막 위치 찾기
    last = len(s) - 1
    while last >= 0 and s[last] in (' ', '\t'):
        last -= 1

    # substring 반환
    return s[first:last + 1]

# 추가된 offset을 초기화
def reset_codes(code_name: str) -> None:

    code = []
    code_path = os.path.join("../include/codes", code_name + "0.txt" )

    try:
        with open(code_path, 'r', encoding='utf-8') as f:
            for line in f:
                row = line.strip().split('\t')
                for i in range(len(row)):
                    row[i] = trim_whitespace(row[i])
                row[1] = float(row[1])

                if len(row) == 8:
                    row.extend([0, 0])
                elif len(row) == 9:
                    row[8] = 0
                    row.append(0)
                elif len(row) == 10:
                    row[8] = 0
                    row[9] = 0
                    pass
                elif len(row) < 8:
                    pass
                else:
                    raise ValueError(f"Unexpected row length: {len(row)}")
                code.append(row)
            # print(code)   

         # 2. 다시 쓰기
        with open(code_path, 'w', encoding='utf-8') as f:
            for row in code:
                f.write('\t'.join(map(str, row)) + '\n')

    except FileNotFoundError:
        print("파일을 찾을 수 없음")
    except Exception as e:
        print(f"에러 발생: {e}")        

    print(f"Code reset completed for {code_name}0.txt")

# --- 메인 실행 ---
if __name__ == "__main__":
    input_path = input("입력 MIDI 파일 경로: ").strip()
    try:
        # 경로의 따옴표 제거 (터미널 입력 시 발생 가능성)
        input_path = input_path.strip("'").strip('"')
        # out_mid = quantize_drum_midi(input_path, 107) # ex) record/solo2.mid
        # print(f"MIDI Saved: {out_mid}") 
        # modify_intensity(input_path)  # ex) solo2
        reset_codes(input_path)  # ex) DrumSolo_


    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        print(type(e))
        print(repr(e))
        print(str(e))
        sys.exit(1)