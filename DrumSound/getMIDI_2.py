import os
import subprocess
import tensorflow.compat.v1 as tf
from magenta.models.drums_rnn import drums_rnn_sequence_generator
from magenta.music import sequence_generator_bundle, midi_file_to_sequence_proto, sequence_proto_to_midi_file
from magenta.music.sequences_lib import extract_subsequence
from magenta.protobuf import generator_pb2

tf.disable_v2_behavior()

input_file = "input.mid"
output_file = "output.mid"
bundle_file = "drum_kit_rnn.mag"

# Magenta 모델 로딩
print("🤖 Magenta Drums RNN 로딩 중...")
bundle = sequence_generator_bundle.read_bundle_file(bundle_file)
generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](
    checkpoint=None, bundle=bundle)
generator.initialize()

# NoteSequence 로딩
primer_sequence = midi_file_to_sequence_proto(input_file)
start_gen = primer_sequence.total_time
end_gen = start_gen + 20.0

# 생성 옵션 설정
generator_options = generator_pb2.GeneratorOptions()
section = generator_options.generate_sections.add()
section.start_time = start_gen
section.end_time = end_gen
generator.temperature = 0.8
generator.steps_per_quarter = 4

# 생성 및 추출
generated_full = generator.generate(primer_sequence, generator_options)
generated_only = extract_subsequence(generated_full, start_gen, end_gen)

# 저장
sequence_proto_to_midi_file(generated_only, output_file)
print(f"✅ 생성된 MIDI 저장 완료: {output_file}")

# timidity 재생 함수 (옵션)
def play_with_timidity(midi_file):
    print(f"🎧 timidity로 {midi_file} 재생 중...")
    subprocess.run(["timidity", midi_file])

# play_with_timidity(output_file)  # 필요 시 주석 해제
