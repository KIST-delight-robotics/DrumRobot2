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
    t = sequence.tempos.add(qpm=bpm)
    t.time = 0.0

# [추가] 양자화 로직을 수행하는 함수
def quantize_midi_messages(messages, bpm, subdivisions_per_beat=2):
    """
    MIDI 메시지 리스트를 양자화합니다.
    - messages: mido 메시지 객체 리스트. 각 메시지는 .time 속성에 절대 시간(초)을 가집니다.
    - bpm: 분당 비트 수 (템포)
    - subdivisions_per_beat: 한 비트를 몇 개로 쪼갤지 (4 = 16분음표, 2 = 8분음표)
    """
    if not messages:
        return []

    # 1. 양자화의 기준이 될 시간 단위(초)를 계산합니다.
    # 예: 120BPM, 16분음표(subdivisions=4) -> 1비트는 0.5초, 16분음표 간격은 0.125초
    seconds_per_beat = 60.0 / bpm
    quantize_step_duration = seconds_per_beat / subdivisions_per_beat

    quantized_messages = []
    for msg in messages:
        original_time = msg.time  # 녹음된 노트의 절대 시간

        # 2. 원래 시간이 양자화 단위 시간의 몇 배수 위치에 가장 가까운지 계산합니다.
        # 예: 0.26초 노트 -> 0.26 / 0.125 = 2.08 -> 반올림하면 2
        quantized_steps = round(original_time / quantize_step_duration)

        # 3. 계산된 배수를 다시 시간 단위로 변환하여 새로운 시간을 구합니다.
        # 예: 2 * 0.125 = 0.25초. 0.26초에 연주된 노트가 0.25초 위치로 이동합니다.
        quantized_time = quantized_steps * quantize_step_duration

        # 4. 시간이 수정된 메시지를 새로 복사하여 리스트에 추가합니다.
        quantized_msg = msg.copy(time=quantized_time)
        quantized_messages.append(quantized_msg)

    return quantized_messages


# 경로 설정
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
    bundle_file = os.path.join(base_dir, "drum_kit_rnn.mag")
    bundle = sequence_generator_bundle.read_bundle_file(bundle_file)
    generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](
        checkpoint=None, bundle=bundle)
    generator.initialize()
    input_path  = os.path.join(input_dir, f"input_{session_idx}{rec_number-1}.mid")
    output_path = os.path.join(output_dir, f"output_{session_idx}{rec_number-1}.mid")
    output_save = os.path.join("/home/shy/DrumRobot/DrumSound/midi_save_for_duet/midi_output", f"output__{session_idx}{rec_number-1}.mid")
    primer_sequence = midi_file_to_sequence_proto(input_path)
    start_gen = primer_sequence.total_time
    end_gen = start_gen + generate_duration
    generator_options = generator_pb2.GeneratorOptions()
    section = generator_options.generate_sections.add()
    section.start_time = start_gen
    section.end_time = end_gen
    generator.temperature = 0.8
    generator.steps_per_quarter = 4
    musicBPM = 100
    generated_full = generator.generate(primer_sequence, generator_options)
    generated_only = extract_subsequence(generated_full, start_gen, end_gen)
    set_tempo_in_sequence(generated_only, bpm=musicBPM)
    sequence_proto_to_midi_file(generated_only, output_path)
    sequence_proto_to_midi_file(generated_only, os.path.join(output_save))
    print(f"⭐ [Magenta] Session {session_idx}-{rec_number-1} 완료: {output_path} ⭐")

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
    function_start_time = time.time()
    is_first_recording = not is_sync_made
    record_start = False
    recording_end_time = None
    recording_start_time = function_start_time if not is_first_recording else None
    recorded_msgs = []
    events = []
    
    # [수정] BPM과 Ticks Per Beat는 여러 곳에서 사용되므로 상단으로 이동
    BPM = 100
    TICKS_PER_BEAT = 960
    tempo_us_per_beat = mido.bpm2tempo(BPM)
    
    print(f"🎙️ Session {session_idx}-{rec_number-1} 녹음 시작 (길이 {rec_duration}s)...")
    if is_first_recording:
        print("   (시작 신호가 될 첫 타격을 기다리는 중...)")

    while True:
        if recording_end_time is not None and time.time() >= recording_end_time:
            print(f"🛑 Session {session_idx}-{rec_number-1} 녹음 종료 🛑")
            break
        
        if (time.time() - function_start_time > 90):
            print("⌛️ 첫 타격 대기 시간 초과.")
            return

        for msg in inport.iter_pending():
            if is_saveable_message(msg) and msg.type == 'note_on' and msg.velocity > 0:
                if not record_start and is_first_recording:
                    recording_start_time = time.time()
                    record_start = True
                
                if not is_sync_made:
                    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    with open(sync_file, "w") as f: f.write(ts)
                    print(f"▶️ 첫 노트 감지 및 sync.txt 생성 ({ts}).")
                    is_sync_made = True
                    recording_start_time = time.time()
                    print(f"녹음 시작")

                recording_end_time = recording_start_time + rec_duration
                now = time.time()
                elapsed = round(now - recording_start_time, 4)
                mapped_note = map_drum_note(getattr(msg, 'note', 0))
                events.append([elapsed, mapped_note, getattr(msg, 'velocity', 0)])
                recorded_msgs.append(msg.copy(time=elapsed))
                print(f"✅ 저장됨: {msg.copy(time=elapsed)}")

        time.sleep(0.001)
    
    if rec_number == 2:
        is_sync_made = False

    events.append([-1, 0, 0])
    csv_out = os.path.join(velo_dir, f"drum_events_{session_idx}{rec_number-1}.csv")
    with open(csv_out, "w", newline='') as f:
        csv.writer(f, delimiter='\t').writerows(events)
    print(f"💾 CSV 저장: {csv_out} 💾")

    # ------------------- [추가] 양자화 적용 단계 -------------------
    # 1. 양자화 단위를 설정합니다. (4=16분음표, 8=32분음표)
    #    사람의 드럼 연주는 보통 16분음표 단위로 양자화하는 것이 일반적입니다.
    QUANTIZE_SUBDIVISIONS = 4
    print(f"🔄 1/{QUANTIZE_SUBDIVISIONS*4} 음표 단위로 양자화 진행...")

    # 2. 위에서 정의한 양자화 함수를 호출하여 시간이 보정된 새 메시지 리스트를 받습니다.
    quantized_msgs = quantize_midi_messages(
        recorded_msgs,
        bpm=BPM,
        subdivisions_per_beat=QUANTIZE_SUBDIVISIONS
    )
    print(f"✅ 양자화 완료: {len(quantized_msgs)}개 노트 타이밍 보정")
    # ----------------------------------------------------------------

    mid = MidiFile(ticks_per_beat=TICKS_PER_BEAT)
    track = MidiTrack(); mid.tracks.append(track)
    track.append(MetaMessage('set_tempo', tempo=tempo_us_per_beat, time=0))
    
    last_event_abs_time = 0
    # [수정] 원본(recorded_msgs) 대신 양자화된(quantized_msgs) 리스트를 사용합니다.
    if quantized_msgs:
        quantized_msgs.sort(key=lambda x: x.time)
        first_msg_time = quantized_msgs[0].time
        
        delta_ticks = mido.second2tick(first_msg_time, TICKS_PER_BEAT, tempo_us_per_beat)
        track.append(quantized_msgs[0].copy(time=max(0, int(round(delta_ticks)))))
        last_event_abs_time = first_msg_time
        
        for i in range(1, len(quantized_msgs)):
            msg = quantized_msgs[i]
            delta_time = msg.time - last_event_abs_time
            last_event_abs_time = msg.time
            ticks = mido.second2tick(delta_time, TICKS_PER_BEAT, tempo_us_per_beat)
            track.append(msg.copy(time=max(0, int(round(ticks)))))

    midi_out = os.path.join(input_dir, f"input_{session_idx}{rec_number-1}.mid")
    input_save = os.path.join("/home/shy/DrumRobot/DrumSound/midi_save_for_duet/midi_input", f"input__{session_idx}{rec_number-1}.mid")
    mid.save(midi_out)
    mid.save(input_save)
    print(f"💾 MIDI 저장: {midi_out}")

# --- 메인 루프 ---
input_ports = mido.get_input_names()
if not input_ports:
    print("❌ MIDI 입력 장치 미발견")
    sys.exit(1)

port_index = 1 if len(input_ports) > 1 else 0
port_name  = input_ports[port_index]
print(f"✅ MIDI 장치: {port_name}")

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

        if session_idx == 0:
            print("\n⏳ 3초 카운트다운")
            for i in (3, 2, 1):
                print(f"{i}...")
                wait_start_time = time.time()
                while time.time() - wait_start_time < 1.0:
                    inport.poll()
                    time.sleep(0.01)

            for _ in inport.iter_pending():
                pass
            print("------------- 첫 타격을 기다립니다 -------------")

        half_rec = total_record / 2.0
        half_make = make_time / 2.0

        record_session(inport, session_idx, half_rec, rec_number=1)
        t1 = threading.Thread(target=generate_with_magenta, args=(session_idx, 1, half_make))
        t1.start()
        record_session(inport, session_idx, half_rec, rec_number=2)
        t2 = threading.Thread(target=generate_with_magenta, args=(session_idx, 2, half_make))
        t2.start()
        t1.join()
        t2.join()

        print("=" *20 + f"Session {session_idx} 모든 작업 완료" + "=" * 20 + "\n")

        if session_idx < num_sessions - 1:
            print(f"⏸ 다음 세션까지 {delay_time}s 대기합니다...")
            flush_for_nsec(inport, delay_time)

print("\n🎉 모든 세션이 완료되었습니다! 🎉")