import mido
from mido import MidiFile, MidiTrack
import time
import threading
import sys
import os
import subprocess

# 🔧 Magenta imports
import note_seq
from note_seq.protobuf import generator_pb2
from note_seq import midi_file_to_sequence_proto, sequence_proto_to_midi_file, concatenate_sequences
from magenta.models.drums_rnn import drums_rnn_sequence_generator
from magenta.models.shared import sequence_generator_bundle
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()

# ✅ 저장 가능한 메시지 필터
def is_saveable_message(msg):
    return (
        not msg.is_meta and
        msg.type in {
            'note_on', 'note_off',
            'control_change', 'program_change',
            'pitchwheel', 'aftertouch', 'polytouch'
        }
    )

# 🎹 MIDI 입력 포트 확인
input_ports = mido.get_input_names()
print("🎹 Available MIDI input ports:")
for i, name in enumerate(input_ports):
    print(f"{i}: {name}")

if not input_ports:
    print("❌ MIDI 입력 장치가 감지되지 않았습니다.")
    sys.exit(1)

port_index = 1
port_name = input_ports[port_index]
print(f"\n✅ MIDI 장치 연결 확인됨: {port_name}")
input("\n▶️ 엔터를 누르면 녹음을 시작합니다...")

# 🎼 MIDI 파일 초기 설정
ticks_per_beat = 960
tempo_bpm = 60
tempo_us_per_beat = mido.bpm2tempo(tempo_bpm)
mid = MidiFile(ticks_per_beat=ticks_per_beat)
track = MidiTrack()
mid.tracks.append(track)
track.append(mido.MetaMessage('set_tempo', tempo=tempo_us_per_beat, time=0))

# ⏱️ 녹음 시작
print("🎙 녹음 중입니다... (엔터를 누르면 종료됩니다)")
stop_recording = False
def wait_for_stop():
    global stop_recording
    input()
    stop_recording = True
threading.Thread(target=wait_for_stop).start()

prev_time = time.time()
recorded_msgs = []

with mido.open_input(port_name) as inport:
    while not stop_recording:
        for msg in inport.iter_pending():
            now = time.time()
            delta = now - prev_time
            prev_time = now
            ticks = int(round(delta * ticks_per_beat))
            msg.time = max(ticks, 1)
            if is_saveable_message(msg):
                msg.channel = 9
                recorded_msgs.append(msg)
                print(f"✅ 저장됨: {msg}")
            else:
                print(f"⚠️ 제외됨: {msg}")
        time.sleep(0.001)

# 📝 입력 저장
input_file = "input.mid"
track.extend(recorded_msgs)
try:
    mid.save(input_file)
    print(f"💾 입력 저장 완료: {input_file}")
except ValueError as e:
    print(f"❌ 저장 실패: {e}")
    sys.exit(1)

# 🎼 Magenta 모델 로딩
print("🤖 Magenta Drums RNN 로딩 중...")
bundle = sequence_generator_bundle.read_bundle_file('drum_kit_rnn.mag')
generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](
    checkpoint=None, bundle=bundle)
generator.initialize()

# 🧠 입력 MIDI → NoteSequence
primer_sequence = midi_file_to_sequence_proto(input_file)
total_time = primer_sequence.total_time
print(f"🕒 입력 길이: {total_time:.2f}초")

# 🎯 편곡 영역 설정
generator_options = generator_pb2.GeneratorOptions()
section = generator_options.generate_sections.add()
section.start_time = total_time
section.end_time = total_time + 10.0  # 10초 생성
generator.temperature = 0.7
generator.steps_per_quarter = 4

# 🥁 생성
generated_sequence = generator.generate(primer_sequence, generator_options)
full_sequence = concatenate_sequences([primer_sequence, generated_sequence])

# 💾 전체 결과 저장
output_file = "arranged_output.mid"
sequence_proto_to_midi_file(full_sequence, output_file)
print(f"✅ 편곡된 MIDI 저장 완료: {output_file}")

# ▶️ 재생
def play_with_timidity(midi_file):
    print(f"🎧 timidity로 {midi_file} 재생 중...")
    subprocess.run(["timidity", midi_file])

play_with_timidity(output_file)
