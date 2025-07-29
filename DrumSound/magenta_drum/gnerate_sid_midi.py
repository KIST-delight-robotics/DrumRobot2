import note_seq
from magenta.models.drums_rnn import drums_rnn_sequence_generator
from magenta.models.shared import sequence_generator_bundle
from note_seq.protobuf import generator_pb2
from magenta.music import midi_file_to_sequence_proto, sequence_proto_to_midi_file
import tensorflow.compat.v1 as tf
import subprocess
import sys
import os

tf.disable_v2_behavior()

# 사용자 입력
input_midi = input("🎼 시드로 사용할 MIDI 파일명을 입력하세요 (예: seed.mid): ").strip()
if not os.path.exists(input_midi):
    print(f"❌ 파일을 찾을 수 없습니다: {input_midi}")
    sys.exit(1)

# 모델 로드
bundle = sequence_generator_bundle.read_bundle_file('drum_kit_rnn.mag')
generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](
    checkpoint=None, bundle=bundle)
generator.initialize()

# MIDI 파일 → NoteSequence
primer_sequence = midi_file_to_sequence_proto(input_midi)

# 시퀀스 전체 길이 추정
total_time = primer_sequence.total_time
print(f"🕒 입력 시퀀스 길이: {total_time:.2f}초")

# 생성 구간 설정: 입력 끝나고 이어서 10초 생성
generator_options = generator_pb2.GeneratorOptions()
generate_section = generator_options.generate_sections.add()

generate_section.start_time = total_time
generate_section.end_time = total_time + 10.0

# 온도, 세밀도 조정
generator.temperature = 0.7
generator.steps_per_quarter = 4

# 드럼 생성
generated_sequence = generator.generate(primer_sequence, generator_options)

# 저장
output_path = "generated_from_input.mid"
sequence_proto_to_midi_file(generated_sequence, output_path)
print(f"✅ 드럼 시퀀스 생성 완료 → {output_path}")

# 재생 함수
def play_midi_with_cli(midi_path):
    sf2_path = "/usr/share/sounds/sf2/FluidR3_GM.sf2"
    cmd = [
        "fluidsynth",
        "-a", "alsa",
        "-o", "audio.alsa.device=default",
        "-g", "1.5",
        "-ni", sf2_path,
        midi_path
    ]
    print("🎧 실행 커맨드:", " ".join(cmd))
    subprocess.run(cmd)

    print(f"🧪 생성된 총 노트 수: {len(generated_sequence.notes)}")
    print(f"🧪 생성 시퀀스 전체 길이: {generated_sequence.total_time:.2f}초")

    starts = [n.start_time for n in generated_sequence.notes]
    ends = [n.end_time for n in generated_sequence.notes]

    print(f"🧪 노트 시작 시간 범위: {min(starts):.2f} ~ {max(starts):.2f}")
    print(f"🧪 노트 종료 시간 범위: {min(ends):.2f} ~ {max(ends):.2f}")


print("🔊 생성된 드럼 시퀀스 재생 중...")
play_midi_with_cli(output_path)
print("🏁 재생 종료.")
