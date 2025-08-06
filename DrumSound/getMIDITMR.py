import mido
from mido import MidiFile, MidiTrack
import time
import sys
import subprocess
import csv
import os
import argparse
import datetime

# Magenta + Note Sequence 관련 import
import tensorflow.compat.v1 as tf
from magenta.models.drums_rnn import drums_rnn_sequence_generator
from magenta.music import sequence_generator_bundle, midi_file_to_sequence_proto, sequence_proto_to_midi_file
from magenta.music.sequences_lib import extract_subsequence
from magenta.protobuf import generator_pb2

tf.disable_v2_behavior()

parser = argparse.ArgumentParser()
parser.add_argument("--rec_times", nargs='+', type=float, required=True, help="[대기1, 녹음1, 생성1, 대기2, 녹음2, 생성2 ...] 순서의 시간들 (초 단위)")
args = parser.parse_args()


rec_seq = args.rec_times
num_sessions = len(rec_seq) // 3


# MIDI 입력 필터링 함수
def is_saveable_message(msg):
    return (
        not msg.is_meta and
        msg.type in {
            'note_on', 'note_off',
            'control_change', 'program_change',
            'pitchwheel', 'aftertouch', 'polytouch'
        } and msg.channel == 9
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
    sequence.tempos.add(qpm=bpm)
    sequence.tempos[0].time = 0.0

# MIDI 포트 연결
input_ports = mido.get_input_names()
if not input_ports:
    print("❌ MIDI 입력 장치가 감지되지 않았습니다.")
    sys.exit(1)

port_index = 1  # 사용자가 원하는 포트 인덱스
port_name = input_ports[port_index]
print(f"\n✅ MIDI 장치 연결 확인됨: {port_name}")

# 현재 스크립트 기준 base 디렉토리
base_dir = os.path.dirname(os.path.abspath(__file__))

# include/sync 디렉토리 경로 설정
sync_dir = os.path.abspath(os.path.join(base_dir, "..", "include", "sync"))
os.makedirs(sync_dir, exist_ok=True)  # 없으면 생성
sync_file = os.path.join(sync_dir, "sync.txt")

for session_idx in range(num_sessions):

    wait_time = rec_seq[session_idx * 3]
    record_duration = rec_seq[session_idx * 3 + 1]
    generate_duration = rec_seq[session_idx * 3 + 2]
    
    # 녹음 전 미디 입력 Flush
    def flush_during_recording(inport):
        for _ in range(5):
            for _ in inport.iter_pending():
                pass
            time.sleep(0.01)

    # 녹음 전 미디 입력 무시
    def flush_with_live_wait(inport, duration_sec=2.0):
        flushed = 0
        start = time.time()
        while (time.time() - start) < duration_sec:
            for msg in inport.iter_pending():
                flushed += 1
            time.sleep(0.005)
        print(f"MIDI 이벤트 {flushed}개 무시됨.")

    # 녹음 시간 설정 (초)
    record_duration = record_duration

    print(f"\n▶️ {session_idx+1}번째 세션 준비 중... (대기 {wait_time}초)")
    time.sleep(wait_time)

    print(f"\n🎙 녹음 시작! ({record_duration}초 동안 진행됩니다...)")

    # MIDI 설정
    mid = MidiFile(ticks_per_beat=960)
    track = MidiTrack()
    mid.tracks.append(track)
    tempo_bpm = 60
    tempo_us_per_beat = mido.bpm2tempo(tempo_bpm)
    track.append(mido.MetaMessage('set_tempo', tempo=tempo_us_per_beat, time=0))

    # 녹음 시작
    first_note_time_saved = False
    recording_started = False
    start_time = None

    final_events = []
    recorded_msgs = []
    ticks_per_beat = 960


    rec_seq = args.rec_times
    num_sessions = len(rec_seq) // 2


    # ✅ 양자화 설정: True = 켜짐 / False = 꺼짐
    ENABLE_QUANTIZATION = False
    quantize_step = 0.05  # 단위: 초 (예: 0.05초 = 50ms)


    prev_time = time.time()
    with mido.open_input(port_name) as inport:
        flush_during_recording(inport)

        print("\n▶️ 첫 MIDI 입력을 기다리는 중...")

        prev_time = time.time()

        while True:
            now = time.time()

            for msg in inport.iter_pending():
                delta = now - prev_time
                prev_time = now
                ticks = int(round(delta * ticks_per_beat))

                if ENABLE_QUANTIZATION:
                    quantize_ticks = int(round(quantize_step * ticks_per_beat))
                    quantized_ticks = max(round(ticks / quantize_ticks) * quantize_ticks, 1)
                    msg.time = quantized_ticks
                else:
                    msg = msg.copy(time=max(ticks, 1))  # time도 이 시점에 설정
                    recorded_msgs.append(msg)

                if is_saveable_message(msg):
                    if msg.type == 'note_on' and msg.velocity > 0:
                        if not first_note_time_saved:
                            # ⏱ 첫 입력 시각 기록 및 시작
                            current_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            # with open(sync_file.replace("sync.txt", f"sync_{session_idx}.txt"), "w") as f:
                            #     f.write(current_time)
                            with open(sync_file, "w") as f:
                                f.write(current_time)
                            print(f"\n✅ 첫 입력 감지됨: {current_time}")

                            start_time = time.time()
                            recording_started = True
                            first_note_time_saved = True
                            print(f"녹음 시작! ({record_duration}초 동안 진행됩니다...)")

                        elapsed_time = round(time.time() - start_time, 3)
                        
                        final_events.append((elapsed_time, map_drum_note(msg.note)))

                    if recording_started:
                        recorded_msgs.append(msg)
                        print(f"✅ 저장됨: {msg}")

            if recording_started and (time.time() - start_time) >= record_duration:
                print("녹음 끝")
                break

            time.sleep(0.001)

    # 시간 차이 저장
    time_diffs = [[final_events[0][0], final_events[0][1]]] if final_events else []
    for i in range(1, len(final_events)):
        diff = round(final_events[i][0] - final_events[i-1][0], 3)
        time_diffs.append([diff, final_events[i][1]])


    # 파일명 접미사
    suffix = f"_{session_idx}"

    # MIDI 저장
    input_file = os.path.join(base_dir, f"input{suffix}.mid")
    track.extend(recorded_msgs)
    mid.save(input_file)
    print(f"입력 저장 완료: {input_file}")

    # CSV 저장
    csv_file = os.path.join(base_dir, f"drum_hits{suffix}.csv")
    with open(csv_file, "w", newline='') as f:
        writer = csv.writer(f, delimiter='\t')
        writer.writerows(time_diffs)
    print(f"드럼 이벤트 CSV 저장 완료: {csv_file}")

    # Magenta 모델 로딩
    print("Magenta Drums RNN 로딩 중...")
    bundle_file = os.path.join(base_dir, "drum_kit_rnn.mag")
    bundle = sequence_generator_bundle.read_bundle_file(bundle_file)
    generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](
        checkpoint=None, bundle=bundle)
    generator.initialize()

    # NoteSequence 로딩
    primer_sequence = midi_file_to_sequence_proto(input_file)
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
    outputFile = os.path.join(base_dir, f"output{suffix}.mid")
    generated_full = generator.generate(primer_sequence, generator_options)
    generated_only = extract_subsequence(generated_full, start_gen, end_gen)
    set_tempo_in_sequence(generated_only, bpm=musicBPM)
    sequence_proto_to_midi_file(generated_only, outputFile)
    print(f"✅ 저장 완료: {outputFile}")

# # 필요시 timidity 재생 함수 (주석 해제 시 사용 가능)
# def play_with_timidity(midi_file):
#     print(f"🎧 timidity로 {midi_file} 재생 중...")
#     subprocess.run(["timidity", midi_file])
