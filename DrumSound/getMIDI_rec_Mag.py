import mido
from mido import MidiFile, MidiTrack, MetaMessage
import time
import sys
import threading
import csv
import os
import argparse
import datetime

import tensorflow.compat.v1 as tf
from magenta.models.drums_rnn import drums_rnn_sequence_generator
from magenta.music import (
    sequence_generator_bundle,
    midi_file_to_sequence_proto,
    sequence_proto_to_midi_file
)
from magenta.music.sequences_lib import extract_subsequence
from magenta.protobuf import generator_pb2

tf.disable_v2_behavior()

parser = argparse.ArgumentParser()
parser.add_argument("--rec_times", nargs='+', type=float, required=True, help="[대기1, 녹음1, 생성1, 대기2, 녹음2, 생성2 ...] 순서의 시간들 (초 단위)")
args = parser.parse_args()

rec_seq = args.rec_times
if len(rec_seq) % 3 != 0:
    print("❌ --rec_times 길이는 3의 배수여야 합니다.")
    sys.exit(1)

num_sessions = len(rec_seq) // 3

# --- 유틸 함수들 ---
def is_saveable_message(msg):
    return (
        not msg.is_meta and
        msg.type in {
            'note_on', 'note_off',
            'control_change', 'program_change',
            'pitchwheel', 'aftertouch', 'polytouch'
        } and getattr(msg, 'channel', 9) == 9
    )

def map_drum_note(note):
    mapping = {
        38: 1, 41: 2, 45: 3,
        47: 4, 48: 4, 50: 4,
        42: 5, 51: 6, 49: 7,
        57: 8, 36: 10, 46: 11
    }
    return mapping.get(note, 0)

def set_tempo_in_sequence(sequence, bpm):
    # 기존 템포를 지우지 않고 맨 앞에 템포를 추가
    t = sequence.tempos.add(qpm=bpm)
    t.time = 0.0

# 경로 설정 - 현재 스크립트 기준 base 디렉토리
base_dir = os.path.dirname(os.path.abspath(__file__))

velo_dir = os.path.join(base_dir, "record_velocity")
os.makedirs(velo_dir, exist_ok=True)

input_dir = os.path.join(base_dir, "record_input")
os.makedirs(input_dir, exist_ok=True)

output_dir = os.path.join(base_dir, "record_output")
os.makedirs(output_dir, exist_ok=True)

sync_dir = os.path.abspath(os.path.join(base_dir, "..", "include", "sync"))
os.makedirs(sync_dir, exist_ok=True)
sync_file = os.path.join(sync_dir, "sync.txt")

def generate_with_magenta(session_idx, rec_number, generate_duration):
    print(f"🔄 [Magenta] Session {session_idx}-{rec_number-1} 생성 시작 (분량 : {generate_duration}s) 🔄")
    #c_t = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3] #마젠타 사용 시간 첫 로딩 시 0.23초 이후 사용시 0.18초
    #print(f"   ⭐⭐⭐마젠타 시작 시간 : {c_t}⭐⭐⭐   ")

    # Magenta 모델 로딩
    bundle_file = os.path.join(base_dir, "drum_kit_rnn.mag")
    bundle = sequence_generator_bundle.read_bundle_file(bundle_file)
    generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](
        checkpoint=None, bundle=bundle)
    generator.initialize()

    # 입출력 파일 경로
    input_path  = os.path.join(input_dir, f"input_{session_idx}{rec_number-1}.mid")
    output_path = os.path.join(output_dir, f"output_{session_idx}{rec_number-1}.mid")

    # NoteSequence 로딩
    primer_sequence = midi_file_to_sequence_proto(input_path)
    start_gen = primer_sequence.total_time
    end_gen = start_gen + generate_duration

    # 생성 설정
    generator_options = generator_pb2.GeneratorOptions()
    section = generator_options.generate_sections.add()
    section.start_time = start_gen
    section.end_time = end_gen
    generator.temperature = 0.3
    generator.steps_per_quarter = 4

    # output.mid 저장
    musicBPM = 100
    generated_full = generator.generate(primer_sequence, generator_options)
    generated_only = extract_subsequence(generated_full, start_gen, end_gen)
    set_tempo_in_sequence(generated_only, bpm=musicBPM)
    sequence_proto_to_midi_file(generated_only, output_path)
    print(f"⭐ [Magenta] Session {session_idx}-{rec_number-1} 완료: {output_path} ⭐")
    
    #c_t = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    #print(f"   ⭐⭐⭐마젠타 종료 시간 : {c_t}⭐⭐⭐   ")

# 녹음 전 미디 입력 무시 (버퍼 비우기)
def flush_for_nsec(inport, duration_sec):
    flushed = 0
    start = time.time()
    while (time.time() - start) < duration_sec:
        for _ in inport.iter_pending():
            flushed += 1
        time.sleep(0.001)
    if flushed:
        print(f"(버퍼 플러시: MIDI 이벤트 {flushed}개 무시)")

# --- 녹음 함수 ---
def record_session(inport, session_idx, rec_duration, rec_number):
    global is_sync_made
    #global is_twosec_waiting       # 나중에 동시 합주할 때 필요함

    # 1. 함수 시작 시간(무한 루프 탈출에 사용)과 첫 녹음 여부와 녹음 시작 여부(elapsed time을 녹음 시작 시간으로부터 계산함)를 확인합니다.
    function_start_time = time.time()
    is_first_recording = not is_sync_made
    record_start = False

    # 2. 녹음이 실제로 끝나는 절대 시간을 저장할 변수를 초기화합니다.
    # 첫 녹음이 아니라면, 함수 시작 시간에 녹음 길이를 더해 종료 시간을 미리 계산합니다.
    # if not is_first_recording:
        # recording_end_time = recording_start_time + rec_duration
    recording_end_time = None

    # 3. 경과 시간(elapsed) 계산의 기준이 될 시간을 설정합니다.
    # 첫 녹음이 아니라면 함수 시작 시간으로, 첫 녹음이라면 None으로 시작합니다.
    recording_start_time = function_start_time if not is_first_recording else None

    recorded_msgs = []      # MIDI용
    events = []             # velocity제작용

    ticks_per_beat = 960
    tempo_us_per_beat = mido.bpm2tempo(100)
    
    print(f"🎙️ Session {session_idx}-{rec_number-1} 녹음 시작 (길이 {rec_duration}s)...")
    if is_first_recording:
        print("   (시작 신호가 될 첫 타격을 기다리는 중...)")

    # --- 녹음 루프 ---
    while True:
        # 4. 녹음 종료 조건
        if recording_end_time is not None and time.time() >= recording_end_time:
            print(f"🛑 Session {session_idx}-{rec_number-1} 녹음 종료 🛑")
            break
        
        # 첫 타격 대기 타임아웃 (3분)
        if (time.time() - function_start_time > 90):
            print("⌛️ 첫 타격 대기 시간 초과.")
            return # 함수를 종료하여 무한 루프 방지

        for msg in inport.iter_pending():
            if is_saveable_message(msg) and msg.type == 'note_on' and msg.velocity > 0:
                if not record_start and is_first_recording:
                    recording_start_time = time.time()
                    record_start = True

                # 5. 프로그램 전체의 첫 sync 타격(시작 신호)을 처리하는 로직
                if not is_sync_made:
                    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    with open(sync_file, "w") as f: f.write(ts)
                    print(f"▶️ 첫 노트 감지 및 sync.txt 생성 ({ts}).")
                    is_sync_made = True
                    
                    # if not is_twosec_waiting:         # 이것도 동시 합주 진행 시 필요함
                    #     print(f"2초 후 녹음을 시작합니다.")
                    #     flush_for_nsec(inport, duration_sec=2.0)
                    #     is_twosec_waiting = True
                    
                    # 6. 2초 대기 후, 실제 녹음 시작 시간과 종료 시간을 설정합니다.
                    recording_start_time = time.time()
                    print(f"녹음 시작")

                recording_end_time = recording_start_time + rec_duration
                    
                    # # 시작 신호 노트는 저장하지 않고 건너뜁니다.
                    # if session_idx == 0:
                    #     continue
                now = time.time()

                # --- 저장 로직 ---
                # 모든 녹음은 각자의 recording_start_time을 기준으로 경과 시간을 계산합니다.
                elapsed = round(now - recording_start_time, 4)
                
                mapped_note = map_drum_note(getattr(msg, 'note', 0))
                events.append([elapsed, mapped_note, getattr(msg, 'velocity', 0)])
                # MIDI 파일 저장을 위해 절대 시간(elapsed)을 time 속성에 기록
                recorded_msgs.append(msg.copy(time=elapsed))
                print(f"✅ 저장됨: {msg.copy(time=elapsed)}")

        time.sleep(0.001)
    
    # 매 세션 두 번째 녹음까지 끝나면 sync.txt 새로 생성
    if rec_number == 2:
        is_sync_made = False

    # --- 파일 저장 (루프 밖) ---
    # C++에서 파일의 끝을 알기 위한 마커 추가
    events.append([-1, 0, 0])
    
    csv_out = os.path.join(velo_dir, f"drum_events_{session_idx}{rec_number-1}.csv")
    with open(csv_out, "w", newline='') as f:
        csv.writer(f, delimiter='\t').writerows(events)
    print(f"💾 CSV 저장: {csv_out} 💾")

    mid = MidiFile(ticks_per_beat=ticks_per_beat)
    track = MidiTrack(); mid.tracks.append(track)
    track.append(MetaMessage('set_tempo', tempo=tempo_us_per_beat, time=0))
    
    # delta_time(전 음표와의 간격) 구하는 코드
    last_event_abs_time = 0
    if recorded_msgs:
        # 시간순으로 정렬하여 MIDI 델타 타임 계산 오류 방지
        recorded_msgs.sort(key=lambda x: x.time)
        first_msg_time = recorded_msgs[0].time
        
        # 첫 메시지의 델타 타임은 녹음 시작부터의 시간
        delta_ticks = mido.second2tick(first_msg_time, ticks_per_beat, tempo_us_per_beat)
        track.append(recorded_msgs[0].copy(time=max(0, int(round(delta_ticks)))))
        last_event_abs_time = first_msg_time
        
        # 나머지 메시지들의 델타 타임 계산
        for i in range(1, len(recorded_msgs)):
            msg = recorded_msgs[i]
            delta_time = msg.time - last_event_abs_time
            last_event_abs_time = msg.time
            ticks = mido.second2tick(delta_time, ticks_per_beat, tempo_us_per_beat)
            track.append(msg.copy(time=max(0, int(round(ticks)))))

    midi_out = os.path.join(input_dir, f"input_{session_idx}{rec_number-1}.mid")
    mid.save(midi_out)
    print(f"💾 MIDI 저장: {midi_out}")

# --- 메인 루프 ---
input_ports = mido.get_input_names()
if not input_ports:
    print("❌ MIDI 입력 장치 미발견")
    sys.exit(1)

# 기본은 index 1을 시도하되, 없으면 0으로 폴백
port_index = 1 if len(input_ports) > 1 else 0
port_name  = input_ports[port_index]
print(f"✅ MIDI 장치: {port_name}")

# is_twosec_waiting = False
is_sync_made = False

with mido.open_input(port_name) as inport:
    for session_idx in range(num_sessions):
        delay_time   = float(rec_seq[session_idx*3 + 0])
        total_record = float(rec_seq[session_idx*3 + 1])
        make_time    = float(rec_seq[session_idx*3 + 2])

        print("-" * 50)
        print(f"➡️ Session {session_idx} 시작")
        print(f"   (Delay: {delay_time}s, Record(total): {total_record}s, Generate: {make_time}s)")
        print("-" * 50)

        # 첫 세션 첫 녹음 전 카운트다운
        if session_idx == 0:
            print("\n⏳ 3초 카운트다운")
            for i in (3, 2, 1):
                print(f"{i}...")
                time.sleep(1)
            print("------------- 첫 타격을 기다립니다 -------------")

        half_rec = total_record / 2.0
        half_make = make_time / 2.0

        # 1) 첫 번째 녹음
        record_session(inport, session_idx, half_rec, rec_number=1)

        # 2) Magenta 1st (Thread) — 두 번째 녹음 중 병행 가능
        t1 = threading.Thread(target=generate_with_magenta, args=(session_idx, 1, half_make))
        t1.start()

        # 3) 두 번째 녹음 (첫 녹음 직후 즉시 시작)
        record_session(inport, session_idx, half_rec, rec_number=2)

        # 4) Magenta 2nd (Thread) — TF 안전을 위해 내부 락으로 직렬화됨
        t2 = threading.Thread(target=generate_with_magenta, args=(session_idx, 2, half_make))
        t2.start()

        # 5) 두 생성 스레드 완료 대기
        t1.join()
        t2.join()

        print("=" *20 + f"Session {session_idx} 모든 작업 완료" + "=" * 20 + "\n")

        # ✅ 세션 간 대기
        if session_idx < num_sessions - 1:
            print(f"⏸ 다음 세션까지 {delay_time}s 대기합니다...")
            flush_for_nsec(inport, delay_time)

print("\n🎉 모든 세션이 완료되었습니다! 🎉")