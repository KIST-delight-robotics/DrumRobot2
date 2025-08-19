import note_seq

from magenta.models.drums_rnn import drums_rnn_sequence_generator
# ✅ 1. magenta.music 쪽으로
from magenta.music import sequence_generator_bundle
# from magenta.models.shared import sequence_generator_bundle
from note_seq.protobuf import generator_pb2
from magenta.music import midi_file_to_sequence_proto, sequence_proto_to_midi_file
import tensorflow.compat.v1 as tf
import subprocess
import sys
import os

tf.disable_v2_behavior()

# ✅ 시드 MIDI에서 드럼 노트만 필터링
def filter_drum_notes(seq: note_seq.NoteSequence) -> note_seq.NoteSequence:
    new_seq = note_seq.NoteSequence()
    for note in seq.notes:
        if note.is_drum:
            new_note = new_seq.notes.add()
            new_note.pitch = note.pitch
            new_note.start_time = note.start_time
            new_note.end_time = note.end_time
            new_note.velocity = note.velocity
            new_note.instrument = 0  # 단일 트랙
            new_note.is_drum = True
    new_seq.total_time = seq.total_time
    return new_seq

# ✅ 생성된 시퀀스를 단일 드럼 트랙으로 flatten
def flatten_to_single_drum_track(seq: note_seq.NoteSequence) -> note_seq.NoteSequence:
    new_seq = note_seq.NoteSequence()
    for note in seq.notes:
        new_note = new_seq.notes.add()
        new_note.pitch = note.pitch
        new_note.start_time = note.start_time
        new_note.end_time = note.end_time
        new_note.velocity = note.velocity
        new_note.instrument = 0  # 단일 트랙
        new_note.is_drum = True
    new_seq.total_time = seq.total_time
    return new_seq

# ✅ MIDI 재생 함수
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

# ✅ 사용자 입력
input_midi = input("🎼 시드로 사용할 MIDI 파일명을 입력하세요 (예: seed.mid): ").strip()
if not os.path.exists(input_midi):
    print(f"❌ 파일을 찾을 수 없습니다: {input_midi}")
    sys.exit(1)

# ✅ 모델 로드
bundle = sequence_generator_bundle.read_bundle_file('drum_kit_rnn.mag')
generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](
    checkpoint=None, bundle=bundle)
generator.initialize()

# ✅ 시드 불러오기 및 전처리
primer_sequence = midi_file_to_sequence_proto(input_midi)
primer_sequence = filter_drum_notes(primer_sequence)

# ✅ 시퀀스 길이 확인
total_time = max([n.end_time for n in primer_sequence.notes]) + 0.01
primer_sequence.total_time = total_time
print(f"🕒 입력 시퀀스 길이: {total_time:.2f}초")

# ✅ 생성 옵션 설정
generator_options = generator_pb2.GeneratorOptions()
generate_section = generator_options.generate_sections.add()
generate_section.start_time = total_time
generate_section.end_time = total_time + 10.0

generator.temperature = 0.8
generator.steps_per_quarter = 4

# ✅ 드럼 시퀀스 생성
generated_sequence = generator.generate(primer_sequence, generator_options)
generated_sequence = flatten_to_single_drum_track(generated_sequence)

# ✅ 저장
output_path = "4_out1.mid"
sequence_proto_to_midi_file(generated_sequence, output_path)
print(f"✅ 드럼 시퀀스 생성 완료 → {output_path}")

# ✅ 재생
print("🔊 생성된 드럼 시퀀스 재생 중...")
play_midi_with_cli(output_path)
print("🏁 재생 종료.")
