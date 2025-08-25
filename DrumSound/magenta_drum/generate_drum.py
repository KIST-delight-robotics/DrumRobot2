import note_seq
from magenta.models.drums_rnn import drums_rnn_sequence_generator
from magenta.models.shared import sequence_generator_bundle
from note_seq.protobuf import generator_pb2
from magenta.music import sequence_proto_to_midi_file
import tensorflow.compat.v1 as tf
import subprocess

# source venv/bin/activate

tf.disable_v2_behavior()

# 모델 로드
bundle = sequence_generator_bundle.read_bundle_file('drum_kit_rnn.mag')
generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](
    checkpoint=None, bundle=bundle)
generator.initialize()

# 시드 시퀀스 초기화
primer_sequence = note_seq.NoteSequence()

# 악기 pitch
HIHAT = 42
SNARE = 38
KICK = 36
TAMBORINE = 54
CRASH = 49
MIDTOM = 47

# 8비트 리듬 2마디
for i in range(2):
    offset = i * 1.0
    primer_sequence.notes.add(pitch=KICK, start_time=offset + 0.0, end_time=offset + 0.1, velocity=120)
    primer_sequence.notes.add(pitch=HIHAT, start_time=offset + 0.25, end_time=offset + 0.35, velocity=120)
    primer_sequence.notes.add(pitch=SNARE, start_time=offset + 0.5, end_time=offset + 0.6, velocity=120)
    primer_sequence.notes.add(pitch=HIHAT, start_time=offset + 0.5, end_time=offset + 0.6, velocity=120)
    primer_sequence.notes.add(pitch=HIHAT, start_time=offset + 0.75, end_time=offset + 0.85, velocity=120)

primer_sequence.total_time = 4.0

# 생성 옵션 설정
generator_options = generator_pb2.GeneratorOptions()
generator_options.generate_sections.add(start_time=2.0, end_time=12.0)

# Magenta 설정
generator.temperature = 1.0
generator.steps_per_quarter = 4

# 드럼 생성
generated_sequence = generator.generate(primer_sequence, generator_options)

# 저장
output_path = "generated.mid"
sequence_proto_to_midi_file(generated_sequence, output_path)
print(f"✅ 드럼 생성 완료 → {output_path}")

# 재생
def play_midi_with_cli(midi_path):
    sf2_path = "/usr/share/sounds/sf2/FluidR3_GM.sf2"
    cmd = [
        "fluidsynth",
        "-a", "alsa",
        "-o", "audio.alsa.device=default",
        "-g", "1.5",  # 볼륨 증폭
        "-ni", sf2_path,
        midi_path
    ]
    print("🎧 실행 커맨드:", " ".join(cmd))
    subprocess.run(cmd)

print("🔊 생성된 드럼 시퀀스 재생 중...")
play_midi_with_cli(output_path)
print("🏁 재생 종료.")
