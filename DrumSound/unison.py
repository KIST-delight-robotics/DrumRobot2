# import mido
# from mido import MidiFile, MidiTrack
# import time
# import sys
# import datetime

# # MIDI 입력 필터링 함수
# def is_saveable_message(msg):
#     return (
#         not msg.is_meta and
#         msg.type in {
#             'note_on', 'note_off',
#             'control_change', 'program_change',
#             'pitchwheel', 'aftertouch', 'polytouch'
#         } and msg.channel == 9
#     )

# def map_drum_note(note):
#     mapping = {
#         38: 1, 41: 2, 45: 3,
#         47: 4, 48: 4, 50: 4,
#         42: 5, 51: 6, 49: 7,
#         57: 8, 36: 10, 46: 11
#     }
#     return mapping.get(note, 0)

# # MIDI 포트 연결
# input_ports = mido.get_input_names()
# if not input_ports:
#     print("MIDI 입력 장치가 감지되지 않았습니다.")
#     sys.exit(1)

# port_index = 1  # 사용자가 원하는 포트 인덱스
# port_name = input_ports[port_index]
# print(f"\n✅ MIDI 장치 연결 확인됨: {port_name}", flush=True)

# def flush_during_recording(inport):
#     for _ in range(5):
#         for _ in inport.iter_pending():
#             pass
#         time.sleep(0.01)

# def round_sec(time):
#     sec_int = int(time * 1000)

#     if (sec_int % 100) < 25:
#         return (int(sec_int / 100) / 10) + 0.00
#     elif (sec_int % 100) >= 25 and (sec_int % 100) < 75:
#         return (int(sec_int / 100) / 10) + 0.05
#     else:
#         return (int(sec_int / 100) / 10) + 0.10

import mido
import time
import sys
import datetime

# --- 상수 정의 ---
SYNC_FILE_PATH = "/home/shy/DrumRobot/include/sync/sync.txt"
LOG_FILE_PATH = "/home/shy/DrumRobot/DrumSound/unison_data.txt"

# (중요) 잔타격을 무시하기 위한 악기별 최소 시간 간격 (초 단위)
# 이 값을 조절하여 필터링 감도를 조절할 수 있습니다. (예: 0.05 = 50ms)
INSTRUMENT_COOLDOWN_S = 0.05

# --- 함수 정의 ---

def is_saveable_message(msg):
    """기록할 MIDI 메시지인지 필터링하는 함수"""
    return (
        not msg.is_meta and
        msg.type == 'note_on' and
        msg.channel == 9 and
        msg.velocity > 0
    )

def map_drum_note(note):
    """MIDI 드럼 노트를 내부 악기 번호로 매핑하는 함수"""
    mapping = {
        38: 1, 41: 2, 45: 3, # 스네어, 로우 플로어 탐, 로우 탐
        47: 4, 48: 4, 50: 4, # 로우 미드 탐, 하이 미드 탐, 하이 탐
        42: 5, 51: 6, 49: 7, # 클로즈드 하이햇, 라이드 심벌, 크래시 심벌
        57: 8, 36: 10, 46: 11 # 크래시 심벌 2, 베이스 드럼, 오픈 하이햇
    }
    return mapping.get(note, 0)

def custom_round_time(t):
    """시간 값을 0.05 단위로 반올림하는 함수"""
    return round(t * 20) / 20.0

def select_midi_port():
    """사용자에게 MIDI 입력 포트를 선택하게 하는 함수"""
    input_ports = mido.get_input_names()
    if not input_ports:
        print("오류: MIDI 입력 장치가 감지되지 않았습니다.")
        sys.exit(1)

    print("\n사용 가능한 MIDI 입력 장치:")
    for i, port in enumerate(input_ports):
        print(f"  {i}: {port}")

    while True:
        try:
            port_index = int(input(f"사용할 장치의 번호를 입력하세요 (0-{len(input_ports) - 1}): "))
            if 0 <= port_index < len(input_ports):
                return input_ports[port_index]
            else:
                print("올바른 번호를 입력해주세요.")
        except ValueError:
            print("숫자를 입력해야 합니다.")

def record_drums(recording_duration):
    """지정된 시간 동안 드럼 연주를 녹음하고 로그 파일로 저장하는 함수"""
    port_name = select_midi_port()
    print(f"\n✅ MIDI 장치 연결 확인됨: {port_name}", flush=True)

    # 카운트다운
    print("\n3초 후 녹음이 시작됩니다...")
    for i in range(3, 0, -1):
        print(i)
        time.sleep(1)
    print("녹음 시작...")

    # 전체 노트 간의 시간차 계산을 위한 변수
    prev_time = None
    # 악기별 마지막 타격 시간을 저장하여 잔타격을 필터링하기 위한 딕셔너리
    last_hit_times = {}

    try:
        with mido.open_input(port_name) as inport:
            recording_start_time = time.time()

            while True:
                if (time.time() - recording_start_time) >= recording_duration:
                    print(f"\n{recording_duration}초 녹음이 완료되었습니다.")
                    break

                msg = inport.poll()
                if not msg or not is_saveable_message(msg):
                    continue

                current_time = time.time()
                instrument = map_drum_note(msg.note)
                velocity = msg.velocity

                # --- 잔타격 필터링 로직 ---
                last_hit_for_this_instrument = last_hit_times.get(instrument, 0)
                if (current_time - last_hit_for_this_instrument) < INSTRUMENT_COOLDOWN_S:
                    # 쿨다운 시간 내에 들어온 신호는 잔타격으로 간주하고 무시
                    continue

                # 유효한 타격이므로, 해당 악기의 마지막 타격 시간을 현재 시간으로 갱신
                last_hit_times[instrument] = current_time

                # --- 첫 타격 처리 ---
                if prev_time is None:
                    elapsed_time = 0.0
                    prev_time = current_time

                    start_datetime_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    with open(SYNC_FILE_PATH, "w") as f_sync:
                        f_sync.write(start_datetime_str)
                    
                    with open(LOG_FILE_PATH, "w") as f_log:
                        f_log.write(f"0.60\t{instrument}\n")
                        # f_log.write(f"{elapsed_time:.2f}\t{instrument}\n")

                    print(f"✅ [{start_datetime_str}] 첫 입력 감지됨 -> 악기: {instrument} | 세기: {velocity}")

                # --- 두 번째 이후 타격 처리 ---
                else:
                    elapsed_time = current_time - prev_time
                    rounded_elapsed_time = custom_round_time(elapsed_time)
                    
                    with open(LOG_FILE_PATH, "a") as f_log:
                        f_log.write(f"{rounded_elapsed_time:.2f}\t{instrument}\n")
                    
                    print(f"✅ 경과 시간: {rounded_elapsed_time:6.2f}초 | 악기: {instrument:2d} | 세기: {velocity:3d}")
                    prev_time = current_time
    
    except KeyboardInterrupt:
        print("\n\n녹음이 중지되었습니다. 프로그램을 종료합니다.")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        sys.exit(0)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("사용법: python your_script_name.py [녹음할 시간(초)]")
        sys.exit(1)

    try:
        duration = float(sys.argv[1])
        if duration <= 0:
            print("오류: 녹음 시간은 0보다 큰 숫자여야 합니다.")
            sys.exit(1)
        record_drums(duration)
    except ValueError:
        print("오류: 녹음 시간으로 올바른 숫자를 입력해야 합니다.")
        sys.exit(1)
