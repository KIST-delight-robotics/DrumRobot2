import mido
from mido import MidiFile
import csv
import os
import subprocess

# Magenta + Note Sequence 관련 import
import tensorflow.compat.v1 as tf
from magenta.models.drums_rnn import drums_rnn_sequence_generator
from magenta.music import sequence_generator_bundle, midi_file_to_sequence_proto, sequence_proto_to_midi_file
from magenta.music.sequences_lib import extract_subsequence
from magenta.protobuf import generator_pb2

tf.disable_v2_behavior()

# 드럼 노트 매핑
def map_drum_note(note):
    mapping = {
        38: 1, 41: 2, 45: 3,
        47: 4, 48: 4, 50: 4,
        42: 5, 51: 6, 49: 7,
        57: 8, 36: 10, 46: 11
    }
    return mapping.get(note, 0)

# 입력 MIDI 파일 설정
# input_file = "/home/shy/DrumRobot/DrumSound/input.mid"
input_file = "/home/shy/DrumRobot/DrumSound/record_midi_by_people/7input_00_raw.mid"
mid = MidiFile(input_file)

# final_events 및 시간 차이 추출
ticks_per_beat = mid.ticks_per_beat
tempo_us_per_beat = 500000  # default 120 BPM

for msg in mid.tracks[0]:
    if msg.type == 'set_tempo':
        tempo_us_per_beat = msg.tempo

tick_time = tempo_us_per_beat / 1_000_000 / ticks_per_beat
final_events = []
current_time = 0.0

for msg in mid.tracks[0]:
    current_time += msg.time * tick_time
    if not msg.is_meta and msg.type == 'note_on' and msg.velocity > 0 and msg.channel == 9:
        final_events.append((round(current_time, 3), map_drum_note(msg.note)))

# 시간 차이 저장
time_diffs = [[final_events[0][0], final_events[0][1]]] if final_events else []
for i in range(1, len(final_events)):
    diff = round(final_events[i][0] - final_events[i - 1][0], 3)
    time_diffs.append([diff, final_events[i][1]])

# CSV 저장
with open("drum_hits.csv", "w", newline='') as f:
    writer = csv.writer(f, delimiter='\t')
    writer.writerows(time_diffs)
print("📄 드럼 이벤트 CSV 저장 완료: drum_hits.csv")

# Magenta 모델 로딩
print("🤖 Magenta Drums RNN 로딩 중...")
bundle = sequence_generator_bundle.read_bundle_file('/home/shy/DrumRobot/DrumSound/drum_kit_rnn.mag')
generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](
    checkpoint=None, bundle=bundle)
generator.initialize()

# NoteSequence 로딩
primer_sequence = midi_file_to_sequence_proto(input_file)
start_gen = primer_sequence.total_time
end_gen = start_gen + 9.6

# 생성 옵션 설정
generator_options = generator_pb2.GeneratorOptions()
section = generator_options.generate_sections.add()
section.start_time = start_gen
section.end_time = end_gen
# generator.steps_per_quarter = 4

# # Temperature 별 생성
# versions = [
#     (0.3, "/home/shy/DrumRobot/DrumSound/output_temp_03.mid"),
#     (0.8, "/home/shy/DrumRobot/DrumSound/output_temp_08.mid")
#     #(0.3, "/home/shy/DrumRobot/DrumSound/output03/output_duet2_03_4.mid"),
#     #(0.8, "/home/shy/DrumRobot/DrumSound/output08/output_duet2_08_4.mid")
# ]

# for temp, filename in versions:
#     print(f"\n🎵 Temperature={temp} 로 생성 중 → 파일명: {filename}")
#     generator.temperature = temp
#     generated_full = generator.generate(primer_sequence, generator_options)
#     generated_only = extract_subsequence(generated_full, start_gen, end_gen)
#     sequence_proto_to_midi_file(generated_only, filename)
#     print(f"✅ 저장 완료: {filename}")

configurations = [
    (1, 0.8, "/home/shy/DrumRobot/DrumSound/mgt_output/7_sq1_temp08_raw.mid"),
    (1, 0.3, "/home/shy/DrumRobot/DrumSound/mgt_output/7_sq1_temp03_raw.mid"),
    (4, 0.3, "/home/shy/DrumRobot/DrumSound/mgt_output/7_sq4_temp03_raw.mid"),
    (4, 0.8, "/home/shy/DrumRobot/DrumSound/mgt_output/7_sq4_temp08_raw.mid")
]

# 설정값 리스트를 순회하며 MIDI 파일을 생성합니다.
for spq, temp, filename in configurations:
    print(f"\n🎵 steps_per_quarter={spq}, Temperature={temp} 로 생성 중 → 파일명: {filename}")
    
    # 현재 설정값으로 생성기 파라미터를 업데이트합니다.
    generator.steps_per_quarter = spq
    generator.temperature = temp
    
    # 시퀀스 생성
    generated_full = generator.generate(primer_sequence, generator_options)
    generated_only = extract_subsequence(generated_full, start_gen, end_gen)
    
    # MIDI 파일로 저장
    sequence_proto_to_midi_file(generated_only, filename)
    print(f"✅ 저장 완료: {filename}")

# MIDI 재생 (선택적 사용)
def play_with_timidity(midi_file):
    print(f"🎧 timidity로 {midi_file} 재생 중...")
    subprocess.run(["timidity", midi_file])

# play_with_timidity("output_temp_08.mid")
