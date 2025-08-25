import mido
from mido import MidiFile, MidiTrack, MetaMessage
import time
import sys
import threading
import csv
import os
import argparse
import datetime
from collections import defaultdict # [추가] 디바운싱에 사용

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

# --- [추가] CSV 저장 유틸리티 함수 ---
def save_step_to_csv(data, filepath, step_type):
    """전처리 단계별 결과를 CSV 파일로 저장합니다."""
    try:
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            # 단계별로 헤더와 데이터 형식을 다르게 저장
            if step_type == 'cluster':
                writer.writerow(['cluster_id', 'original_time', 'note', 'velocity'])
                for i, cluster in enumerate(data):
                    for msg in cluster:
                        writer.writerow([i, msg.time, msg.note, msg.velocity])
            else: # 'debouce' 또는 'quantize'
                writer.writerow(['time', 'note', 'velocity'])
                for msg in data:
                    writer.writerow([msg.time, msg.note, msg.velocity])
        print(f"    > CSV 저장 완료: {os.path.basename(filepath)}")
    except Exception as e:
        print(f"    > CSV 저장 실패: {e}")

# --- MIDI 전처리 파이프라인 함수들 ---

def apply_debouncing(messages, threshold, filepath):
    """1단계: 디바운싱 필터. 악기별로 잔타격을 제거합니다."""
    print(f"  [파이프라인 1/3] 디바운싱 시작 (임계값: {threshold*1000:.0f}ms)...")
    filtered_notes = []
    last_hit_times = defaultdict(float)
    for msg in messages:
        note_id = msg.note
        if msg.time - last_hit_times[note_id] > threshold:
            filtered_notes.append(msg)
            last_hit_times[note_id] = msg.time
    print(f"    > 원본 {len(messages)}개 노트 -> 필터링 후 {len(filtered_notes)}개")
    save_step_to_csv(filtered_notes, filepath, 'debouce') # [추가]
    return filtered_notes

def apply_clustering(messages, threshold, filepath):
    """2단계: 노트 클러스터링. 동시 타격 노트를 그룹화합니다."""
    print(f"  [파이프라인 2/3] 클러스터링 시작 (임계값: {threshold*1000:.0f}ms)...")
    clusters = []
    if messages:
        current_cluster = [messages[0]]
        clusters.append(current_cluster)
        for i in range(1, len(messages)):
            prev_msg_time = messages[i-1].time
            current_msg_time = messages[i].time
            if current_msg_time - prev_msg_time < threshold:
                current_cluster.append(messages[i])
            else:
                current_cluster = [messages[i]]
                clusters.append(current_cluster)
    print(f"    > {len(messages)}개 노트 -> {len(clusters)}개 클러스터로 그룹화")
    save_step_to_csv(clusters, filepath, 'cluster') # [추가]
    return clusters

def apply_grid_quantization(clusters, bpm, subdivisions_per_beat, filepath):
    """3단계: 그리드 기반 양자화. 클러스터를 그리드에 맞춥니다."""
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
    save_step_to_csv(quantized_messages, filepath, 'quantize') # [추가]
    return quantized_messages

def preprocess_midi_pipeline(messages, params, base_filepath): # [수정] base_filepath 파라미터 추가
    """전체 전처리 파이프라인을 순서대로 실행하고 각 단계 결과를 CSV로 저장합니다."""
    print("🚀 MIDI 전처리 파이프라인 시작...")
    
    # 각 단계별 파일 경로 생성
    path_step1 = f"{base_filepath}_step1_debounced.csv"
    path_step2 = f"{base_filepath}_step2_clustered.csv"
    path_step3 = f"{base_filepath}_step3_quantized.csv"
    
    # 파이프라인 1: 디바운싱
    debounced_msgs = apply_debouncing(messages, params['debounce_threshold'], path_step1)
    
    # 파이프라인 2: 클러스터링
    clusters = apply_clustering(debounced_msgs, params['cluster_threshold'], path_step2)
    
    # 파이프라인 3: 그리드 양자화
    final_msgs = apply_grid_quantization(clusters, params['bpm'], params['subdivisions'], path_step3)
    
    print("✅ MIDI 전처리 파이프라인 완료!")
    return final_msgs

# -----------------------------------------------------------------

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
    output_save = os.path.join("/home/shy/DrumRobot/DrumSound/midi_save_for_duet/midi_output", f"output__{session_idx}{rec_number-1}.mid")

    # NoteSequence 로딩
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
    if flushed:
        print(f"(버퍼 플러시: MIDI 이벤트 {flushed}개 무시)")

# --- 녹음 함수 ---
def record_session(inport, session_idx, rec_duration, rec_number):
    global is_sync_made
    # ... (기존 녹음 루프 시작 부분 코드와 동일) ...
    function_start_time = time.time()
    is_first_recording = not is_sync_made
    record_start = False
    recording_end_time = None
    recording_start_time = function_start_time if not is_first_recording else None
    recorded_msgs = []
    events = []
    
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

    # --- [수정] MIDI 전처리 파이프라인 호출 ---
    if recorded_msgs:
        # 1. 파이프라인 파라미터 설정 (여기서 값 조절 가능)
        pipeline_params = {
            'debounce_threshold': 0.04,  # 40ms. 잔타격 무시 시간
            'cluster_threshold': 0.02,   # 20ms. 동시타격 그룹화 시간
            'bpm': BPM,
            'subdivisions': 2,           # 16분음표 단위로 양자화
        }
        
        # 2. 파이프라인 실행
        # [추가] 각 단계별 CSV 파일 저장을 위한 기본 파일 경로 생성
        base_filename = f"preprocess_{session_idx}{rec_number-1}"
        base_filepath = os.path.join(preprocess_dir, base_filename)

        # [수정] 파이프라인 실행 시 파일 경로 전달
        processed_msgs = preprocess_midi_pipeline(recorded_msgs, pipeline_params, base_filepath)
    else:
        processed_msgs = []
    # ---------------------------------------------
    
    mid = MidiFile(ticks_per_beat=TICKS_PER_BEAT)
    track = MidiTrack(); mid.tracks.append(track)
    track.append(MetaMessage('set_tempo', tempo=tempo_us_per_beat, time=0))
    
    last_event_abs_time = 0
    # [수정] 원본(recorded_msgs) 대신 전처리된(processed_msgs) 메시지 사용
    if processed_msgs:
        processed_msgs.sort(key=lambda x: x.time)
        
        # 중복된 시간의 노트가 있을 경우, note 번호가 낮은 순으로 정렬 (선택적)
        # 이는 MIDI 파일 표준에서 권장하는 방식은 아니나, 일관성을 위해 추가 가능
        # processed_msgs.sort(key=lambda x: (x.time, x.note))

        first_msg_time = processed_msgs[0].time
        delta_ticks = mido.second2tick(first_msg_time, TICKS_PER_BEAT, tempo_us_per_beat)
        track.append(processed_msgs[0].copy(time=max(0, int(round(delta_ticks)))))
        last_event_abs_time = first_msg_time
        
        for i in range(1, len(processed_msgs)):
            msg = processed_msgs[i]
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