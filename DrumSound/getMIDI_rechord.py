import mido
from mido import MidiFile, MidiTrack
import time
import sys
import subprocess
import csv
import os

# Magenta + Note Sequence 관련 import
import tensorflow.compat.v1 as tf
from magenta.models.drums_rnn import drums_rnn_sequence_generator
from magenta.music import sequence_generator_bundle, midi_file_to_sequence_proto, sequence_proto_to_midi_file
from magenta.music.sequences_lib import extract_subsequence
from magenta.protobuf import generator_pb2

tf.disable_v2_behavior()

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

# MIDI 포트 연결
input_ports = mido.get_input_names()
if not input_ports:
    print("❌ MIDI 입력 장치가 감지되지 않았습니다.")
    sys.exit(1)

port_index = 1  # 사용자가 원하는 포트 인덱스
port_name = input_ports[port_index]
print(f"\n✅ MIDI 장치 연결 확인됨: {port_name}")

def flush_midi_input():
    try:
        with mido.open_input(port_name) as inport:
            for _ in inport.iter_pending():
                pass
    except Exception as e:
        print(f"⚠️ MIDI flush 중 에러 발생: {e}")
flush_midi_input()

# 녹음 시간 설정 (초)
record_duration = 10.0

# 카운트 다운 추가
print("\n▶️ 녹음 준비...")
for i in range(3, 0, -1):
    print(f"⏳ {i} ...")
    time.sleep(1)

print(f"\n🎙 녹음 시작! ({record_duration}초 동안 진행됩니다...)")

# MIDI 설정
mid = MidiFile(ticks_per_beat=960)
track = MidiTrack()
mid.tracks.append(track)
tempo_bpm = 60
tempo_us_per_beat = mido.bpm2tempo(tempo_bpm)
track.append(mido.MetaMessage('set_tempo', tempo=tempo_us_per_beat, time=0))

# 녹음 시작
start_time = time.time()
final_events = []
recorded_msgs = []
quantize_step = 0.05
ticks_per_beat = 960
quantize_ticks = int(round(quantize_step * ticks_per_beat))

prev_time = time.time()
with mido.open_input(port_name) as inport:
    while True:
        now = time.time()
        elapsed_record_time = now - start_time

        if elapsed_record_time >= record_duration:
            print("🛑 녹음 끝!")
            break

        for msg in inport.iter_pending():
            delta = now - prev_time
            prev_time = now
            ticks = int(round(delta * ticks_per_beat))
            quantized_ticks = max(round(ticks / quantize_ticks) * quantize_ticks, 1)
            msg.time = quantized_ticks

            if is_saveable_message(msg):
                if msg.type == 'note_on' and msg.velocity > 0:
                    elapsed_time = round(now - start_time, 3)
                    final_events.append((elapsed_time, map_drum_note(msg.note)))
                recorded_msgs.append(msg)
                print(f"✅ 저장됨: {msg}")
        time.sleep(0.001)

# 시간 차이 저장
time_diffs = [[final_events[0][0], final_events[0][1]]] if final_events else []
for i in range(1, len(final_events)):
    diff = round(final_events[i][0] - final_events[i-1][0], 3)
    time_diffs.append([diff, final_events[i][1]])

# MIDI 저장
input_file = "input.mid"
track.extend(recorded_msgs)
mid.save(input_file)
print(f"💾 입력 저장 완료: {input_file}")

# CSV 저장
with open("drum_hits.csv", "w", newline='') as f:
    writer = csv.writer(f, delimiter='\t')
    writer.writerows(time_diffs)
print("📄 드럼 이벤트 CSV 저장 완료: drum_hits.csv")

# Magenta 모델 로딩
print("🤖 Magenta Drums RNN 로딩 중...")
bundle = sequence_generator_bundle.read_bundle_file('/home/shy/DrumSound/drum_kit_rnn.mag')
generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](
    checkpoint=None, bundle=bundle)
generator.initialize()

# NoteSequence 로딩
primer_sequence = midi_file_to_sequence_proto(input_file)
start_gen = primer_sequence.total_time
end_gen = start_gen + 20.0

# 생성 설정
generator_options = generator_pb2.GeneratorOptions()
section = generator_options.generate_sections.add()
section.start_time = start_gen
section.end_time = end_gen
generator.steps_per_quarter = 4

# Temperature 별로 반복 생성
versions = [
    (0.3, "output_temp_03.mid"),
    (0.8, "output_temp_08.mid")
]

for temp, filename in versions:
    print(f"\n🎵 Temperature={temp} 로 생성 중 → 파일명: {filename}")

    generator.temperature = temp

    generated_full = generator.generate(primer_sequence, generator_options)
    generated_only = extract_subsequence(generated_full, start_gen, end_gen)

    sequence_proto_to_midi_file(generated_only, filename)
    print(f"✅ 저장 완료: {filename}")

# 필요시 timidity 재생 함수 (주석 해제 시 사용 가능)
def play_with_timidity(midi_file):
    print(f"🎧 timidity로 {midi_file} 재생 중...")
    subprocess.run(["timidity", midi_file])

# 예시 → 0.8 버전 재생
# play_with_timidity("output_temp_08.mid")
