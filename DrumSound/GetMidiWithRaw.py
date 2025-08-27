import mido
from mido import MidiFile, MidiTrack, MetaMessage
import time
import sys
import threading
import csv
import os
import argparse
import datetime
from collections import defaultdict

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

# --- [추가] 경로 설정 ---
base_dir = os.path.dirname(os.path.abspath(__file__))
preprocess_dir = os.path.join(base_dir, "record_preprocess_steps")
os.makedirs(preprocess_dir, exist_ok=True)
velo_dir = os.path.join(base_dir, "record_velocity")
os.makedirs(velo_dir, exist_ok=True)
input_dir = os.path.join(base_dir, "record_input")
os.makedirs(input_dir, exist_ok=True)
output_dir = os.path.join(base_dir, "record_output")
os.makedirs(output_dir, exist_ok=True)
sync_dir = os.path.abspath(os.path.join(base_dir, "..", "include", "sync"))
os.makedirs(sync_dir, exist_ok=True)
sync_file = os.path.join(sync_dir, "sync.txt")

# --- 유틸리티 함수들 ---

def save_step_to_csv(data, filepath, step_type):
    try:
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            if step_type == 'cluster':
                writer.writerow(['cluster_id', 'original_time', 'note', 'velocity'])
                for i, cluster in enumerate(data):
                    for msg in cluster:
                        writer.writerow([i, msg.time, msg.note, msg.velocity])
            else:
                writer.writerow(['time', 'note', 'velocity'])
                for msg in data:
                    writer.writerow([msg.time, msg.note, msg.velocity])
        print(f"    > CSV 저장 완료: {os.path.basename(filepath)}")
    except Exception as e:
        print(f"    > CSV 저장 실패: {e}")

# --- [신규] MIDI 저장을 위한 헬퍼 함수 ---
def save_messages_as_midi(messages, filepath, bpm, ticks_per_beat):
    """Mido 메시지 리스트를 MIDI 파일로 저장하는 공통 함수"""
    tempo = mido.bpm2tempo(bpm)
    mid = MidiFile(ticks_per_beat=ticks_per_beat)
    track = MidiTrack()
    mid.tracks.append(track)
    track.append(MetaMessage('set_tempo', tempo=tempo, time=0))

    last_abs_time = 0
    # 델타 타임 계산을 위해 시간순 정렬
    messages.sort(key=lambda x: x.time)

    for msg in messages:
        delta_time_sec = msg.time - last_abs_time
        delta_ticks = mido.second2tick(delta_time_sec, ticks_per_beat, tempo)
        track.append(msg.copy(time=max(0, int(round(delta_ticks)))))
        last_abs_time = msg.time

    mid.save(filepath)

# --- MIDI 전처리 파이프라인 함수들 ---

def apply_debouncing(messages, threshold, filepath):
    if not messages: return []
    print(f"  [파이프라인 1/3] 디바운싱 시작 (임계값: {threshold*1000:.0f}ms)...")
    filtered_notes = []
    last_hit_times = {}
    for msg in messages:
        note_id = msg.note
        if note_id not in last_hit_times or msg.time - last_hit_times[note_id] > threshold:
            filtered_notes.append(msg)
            last_hit_times[note_id] = msg.time
    print(f"    > 원본 {len(messages)}개 노트 -> 필터링 후 {len(filtered_notes)}개")
    save_step_to_csv(filtered_notes, filepath, 'debouce')
    return filtered_notes

def apply_clustering(messages, threshold, filepath):
    if not messages: return []
    print(f"  [파이프라인 2/3] 클러스터링 시작 (임계값: {threshold*1000:.0f}ms)...")
    clusters = []
    current_cluster = [messages[0]]
    clusters.append(current_cluster)
    for i in range(1, len(messages)):
        if messages[i].time - messages[i-1].time < threshold:
            current_cluster.append(messages[i])
        else:
            current_cluster = [messages[i]]
            clusters.append(current_cluster)
    print(f"    > {len(messages)}개 노트 -> {len(clusters)}개 클러스터로 그룹화")
    save_step_to_csv(clusters, filepath, 'cluster')
    return clusters

def apply_grid_quantization(clusters, bpm, subdivisions_per_beat, filepath):
    if not clusters: return []
    print(f"  [파이프라인 3/3] 그리드 양자화 시작 (BPM: {bpm}, 단위: 1/{subdivisions_per_beat*4})...")
    quantized_messages = []
    seconds_per_beat = 60.0 / bpm
    quantize_step_duration = seconds_per_beat / subdivisions_per_beat
    for cluster in clusters:
        if not cluster: continue
        average_time = sum(msg.time for msg in cluster) / len(cluster)
        quantized_steps = round(average_time / quantize_step_duration)
        quantized_time = quantized_steps * quantize_step_duration
        for msg in cluster:
            quantized_messages.append(msg.copy(time=quantized_time))
    print(f"    > {len(clusters)}개 클러스터 -> 양자화 완료")
    save_step_to_csv(quantized_messages, filepath, 'quantize')
    return quantized_messages

def preprocess_midi_pipeline(messages, params, base_filepath):
    print("🚀 MIDI 전처리 파이프라인 시작...")
    path_step1 = f"{base_filepath}_step1_debounced.csv"
    path_step2 = f"{base_filepath}_step2_clustered.csv"
    path_step3 = f"{base_filepath}_step3_quantized.csv"
    debounced_msgs = apply_debouncing(messages, params['debounce_threshold'], path_step1)
    clusters = apply_clustering(debounced_msgs, params['cluster_threshold'], path_step2)
    final_msgs = apply_grid_quantization(clusters, params['bpm'], params['subdivisions'], path_step3)
    print("✅ MIDI 전처리 파이프라인 완료!")
    return final_msgs

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

preprocess_dir = os.path.join(base_dir, "record_preprocess_steps")
os.makedirs(preprocess_dir, exist_ok=True)

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
    output_save = os.path.join("/home/shy/DrumRobot/DrumSound/midi_save/midi_output", f"output__{session_idx}{rec_number-1}.mid")                # 
    primer_sequence = midi_file_to_sequence_proto(input_path)
    start_gen = primer_sequence.total_time
    end_gen = start_gen + generate_duration

    # 생성 설정
    generator_options = generator_pb2.GeneratorOptions()
    section = generator_options.generate_sections.add()
    section.start_time = start_gen
    section.end_time = end_gen
    generator.temperature = 0.8
    generator.steps_per_quarter = 4

    # output.mid 저장
    musicBPM = 100
    generated_full = generator.generate(primer_sequence, generator_options)
    generated_only = extract_subsequence(generated_full, start_gen, end_gen)
    set_tempo_in_sequence(generated_only, bpm=musicBPM)
    sequence_proto_to_midi_file(generated_only, output_path)
    sequence_proto_to_midi_file(generated_only, os.path.join(output_save))      # midi output 보관용 저장
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
# ⏳ 3초 카운트다운
    print(f"(버퍼 플러시: MIDI 이벤트 {flushed}개 무시)")

# --- [수정] record_session 함수 ---
def record_session(inport, session_idx, rec_duration, rec_number):
    global is_sync_made
    function_start_time = time.time()
    is_first_recording = not is_sync_made
    record_start = False
    recording_end_time = None
    recording_start_time = function_start_time if not is_first_recording else None
    recorded_msgs = []
    events = []
    
    BPM = 100
    TICKS_PER_BEAT = 960

    print(f"🎙️ Session {session_idx}-{rec_number-1} 녹음 시작 (길이 {rec_duration}s)...")
    if is_first_recording:
        print("   (시작 신호가 될 첫 타격을 기다리는 중...)")

    # 녹음 루프
    while True:
        if recording_end_time and time.time() >= recording_end_time:
            print(f"🛑 Session {session_idx}-{rec_number-1} 녹음 종료 🛑")
            break
        
        if not record_start and (time.time() - function_start_time > 90):
            print("⌛️ 첫 타격 대기 시간 초과.")
            break # 루프 종료

        for msg in inport.iter_pending():
            if is_saveable_message(msg) and msg.type == 'note_on' and msg.velocity > 0:
                if not is_sync_made:
                    is_sync_made = True
                    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    with open(sync_file, "w") as f: f.write(ts)
                    print(f"▶️ 첫 노트 감지 및 sync.txt 생성 ({ts}).")
                    
                    record_start = True
                    recording_start_time = time.time()
                    recording_end_time = recording_start_time + rec_duration
                    print(f"녹음 시작")

                now = time.time()
                elapsed = round(now - recording_start_time, 4)
                mapped_note = map_drum_note(getattr(msg, 'note', 0))
                events.append([elapsed, mapped_note, getattr(msg, 'velocity', 0)])
                recorded_msgs.append(msg.copy(time=elapsed))
                print(f"✅ 저장됨: {msg.copy(time=elapsed)}")
        
        time.sleep(0.001)

    # --- [수정] is_sync_made 플래그 리셋 위치 변경 ---
    # 루프 밖에서, 두 번째 녹음 세션이 끝난 후에만 리셋
    if rec_number == 2:
        is_sync_made = False

    # --- [수정] 빈 녹음 파일 처리 및 저장 로직 개선 ---
    if not recorded_msgs:
        print("텅 빈 녹음입니다. MIDI 파일을 생성하지 않습니다.")
        # 빈 CSV는 생성
        events.append([-1, 0, 0])
        csv_out = os.path.join(velo_dir, f"drum_events_{session_idx}{rec_number-1}.csv")
        with open(csv_out, "w", newline='') as f:
            csv.writer(f, delimiter='\t').writerows(events)
        print(f"💾 빈 CSV 저장: {csv_out} 💾")
        return # 함수 종료

    # C++ 마커 추가 및 CSV 저장
    events.append([-1, 0, 0])
    csv_out = os.path.join(velo_dir, f"drum_events_{session_idx}{rec_number-1}.csv")
    with open(csv_out, "w", newline='') as f:
        csv.writer(f, delimiter='\t').writerows(events)
    print(f"💾 CSV 저장: {csv_out} 💾")

    # 1. 원본(raw) MIDI 데이터 저장
    midi_out_raw = os.path.join(input_dir, f"input_{session_idx}{rec_number-1}_raw.mid")
    save_messages_as_midi(recorded_msgs, midi_out_raw, BPM, TICKS_PER_BEAT)
    print(f"💾 [원본] MIDI 저장: {midi_out_raw}")

    # 2. 전처리 파이프라인 실행
    pipeline_params = {
        'debounce_threshold': 0.04,
        'cluster_threshold': 0.02,
        'bpm': BPM,
        'subdivisions': 2,
    }
    base_filename = f"preprocess_{session_idx}{rec_number-1}"
    base_filepath = os.path.join(preprocess_dir, base_filename)
    processed_msgs = preprocess_midi_pipeline(recorded_msgs, pipeline_params, base_filepath)

    # 3. 전처리된 MIDI 데이터 저장
    midi_out_processed = os.path.join(input_dir, f"input_{session_idx}{rec_number-1}.mid")
    save_messages_as_midi(processed_msgs, midi_out_processed, BPM, TICKS_PER_BEAT)
    print(f"💾 [전처리] MIDI 저장: {midi_out_processed}")
    
    # Magenta 입력용과 별도로 보관용 사본 저장
    input_save = os.path.join("/home/shy/DrumRobot/DrumSound/midi_save/midi_input", f"input__{session_idx}{rec_number-1}.mid")
    save_messages_as_midi(processed_msgs, input_save, BPM, TICKS_PER_BEAT)


# --- 메인 루프 ---
input_ports = mido.get_input_names()
if not input_ports:
    print("❌ MIDI 입력 장치 미발견")
    sys.exit(1)

port_index = 1 if len(input_ports) > 1 else 0
port_name  = input_ports[port_index]
print(f"✅ MIDI 장치: {port_name}")

is_sync_made = False

num_sessions = len(rec_seq) // 3

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