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

# ✅ Temperature 설정 (여기만 수정하면 됨)
temp1 = 0.7  # 0~5초
temp2 = 1.0  # 5~10초
temp3 = 0.2  # 10~15초

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
            new_note.instrument = 0
            new_note.is_drum = True
    new_seq.total_time = seq.total_time
    return new_seq

# ✅ 단일 트랙으로 정리
def flatten_to_single_drum_track(seq: note_seq.NoteSequence) -> note_seq.NoteSequence:
    new_seq = note_seq.NoteSequence()
    for note in seq.notes:
        new_note = new_seq.notes.add()
        new_note.pitch = note.pitch
        new_note.start_time = note.start_time
        new_note.end_time = note.end_time
        new_note.velocity = note.velocity
        new_note.instrument = 0
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

# ✅ 시드 전처리
primer_sequence = midi_file_to_sequence_proto(input_midi)
primer_sequence = filter_drum_notes(primer_sequence)

# ✅ 시퀀스 길이 기준
total_time = max([n.end_time for n in primer_sequence.notes]) + 0.01
primer_sequence.total_time = total_time
print(f"🕒 입력 시퀀스 길이: {total_time:.2f}초")

# ✅ 3구간 생성
generated_sequence = note_seq.NoteSequence()

# ▶ 첫 5초
generator.temperature = temp1
opt1 = generator_pb2.GeneratorOptions()
opt1.generate_sections.add(start_time=total_time, end_time=total_time + 5.0)
gen1 = generator.generate(primer_sequence, opt1)
for n in gen1.notes:
    generated_sequence.notes.add().CopyFrom(n)

# ▶ 다음 5초
generator.temperature = temp2
opt2 = generator_pb2.GeneratorOptions()
opt2.generate_sections.add(start_time=total_time + 5.0, end_time=total_time + 10.0)
gen2 = generator.generate(primer_sequence, opt2)
for n in gen2.notes:
    generated_sequence.notes.add().CopyFrom(n)

# ▶ 마지막 5초
generator.temperature = temp3
opt3 = generator_pb2.GeneratorOptions()
opt3.generate_sections.add(start_time=total_time + 10.0, end_time=total_time + 15.0)
gen3 = generator.generate(primer_sequence, opt3)
for n in gen3.notes:
    generated_sequence.notes.add().CopyFrom(n)

generated_sequence.total_time = total_time + 15.0

# ✅ 전체 시퀀스 (시드 + 생성)
full_sequence = note_seq.NoteSequence()
for n in primer_sequence.notes:
    full_sequence.notes.add().CopyFrom(n)
for n in generated_sequence.notes:
    full_sequence.notes.add().CopyFrom(n)
full_sequence.total_time = total_time + 15.0

# ✅ 저장
sequence_proto_to_midi_file(flatten_to_single_drum_track(full_sequence), "all_output.mid")
sequence_proto_to_midi_file(flatten_to_single_drum_track(generated_sequence), "gen_only_output.mid")
print("✅ all_output.mid (시드+생성) 저장 완료")
print("✅ gen_only_output.mid (생성만) 저장 완료")

# ✅ 재생
print("🔊 전체 시퀀스 재생 중.../////////////////////////////////////////////////////////////////////////////")
play_midi_with_cli("all_output.mid")

print("🔊 생성 부분만 재생 중.../////////////////////////////////////////////////////////////////////////////")
play_midi_with_cli("gen_only_output.mid")
