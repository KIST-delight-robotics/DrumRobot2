import mido
from mido import MidiFile, MidiTrack
import time, threading, sys, os, subprocess
import tensorflow.compat.v1 as tf
from note_seq import (
    midi_file_to_sequence_proto,
    sequence_proto_to_midi_file,
    concatenate_sequences,
    sequences_lib
)
from note_seq.protobuf import generator_pb2
from magenta.models.drums_rnn import drums_rnn_sequence_generator
from magenta.models.groovae import groovae_sequence_generator
from magenta.models.music_vae import configs
from magenta.music import sequence_generator_bundle

tf.disable_v2_behavior()

# === MIDI 입력 설정 ===
def is_saveable_message(msg):
    return not msg.is_meta and msg.type in {
        'note_on', 'note_off', 'control_change', 'program_change',
        'pitchwheel', 'aftertouch', 'polytouch'
    }

input_ports = mido.get_input_names()
port_index = 1
port_name = input_ports[port_index]
print(f"✅ 연결된 MIDI 입력: {port_name}")
input("▶️ 엔터를 누르면 녹음 시작...")

# === MIDI 녹음 ===
ticks_per_beat = 960
tempo_bpm = 60
tempo_us_per_beat = mido.bpm2tempo(tempo_bpm)
mid = MidiFile(ticks_per_beat=ticks_per_beat)
track = MidiTrack()
mid.tracks.append(track)
track.append(mido.MetaMessage('set_tempo', tempo=tempo_us_per_beat, time=0))

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
            msg.time = max(int(round(delta * ticks_per_beat)), 1)
            if is_saveable_message(msg):
                msg.channel = 9
                recorded_msgs.append(msg)
        time.sleep(0.001)

input_midi = "input.mid"
track.extend(recorded_msgs)
mid.save(input_midi)
print(f"💾 입력 저장 완료: {input_midi}")

# === Drums RNN 생성 ===
print("🤖 Drums RNN 로딩 중...")
rnn_bundle = sequence_generator_bundle.read_bundle_file('drum_kit_rnn.mag')
rnn_generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](checkpoint=None, bundle=rnn_bundle)
rnn_generator.initialize()

primer = midi_file_to_sequence_proto(input_midi)
total_time = primer.total_time
rnn_options = generator_pb2.GeneratorOptions()
rnn_section = rnn_options.generate_sections.add()
rnn_section.start_time = total_time
rnn_section.end_time = total_time + 10.0

rnn_sequence = rnn_generator.generate(primer, rnn_options)
combined_sequence = concatenate_sequences([primer, rnn_sequence])
rnn_midi = "rnn_output.mid"
sequence_proto_to_midi_file(combined_sequence, rnn_midi)
print(f"🥁 Drums RNN 생성 완료: {rnn_midi}")

# === Drums RNN 출력 저장 ===
combined_sequence = concatenate_sequences([primer, rnn_sequence])
rnn_midi = "rnn_output.mid"
sequence_proto_to_midi_file(combined_sequence, rnn_midi)
print(f"🥁 Drums RNN 생성 완료 (GrooVAE 전): {rnn_midi}")

# === GrooVAE 적용 ===
print("🎛️ GrooVAE 로딩 중...")

# 모델 구성 (GrooVAE 4bar config)
config = configs.CONFIG_MAP['groovae_4bar']

# 체크포인트 경로 설정
checkpoint_dir = 'groovae_4bar'
checkpoint_path = os.path.join(checkpoint_dir, 'model.ckpt')

# GrooVAE Generator 수동 생성
groovae_generator = groovae_sequence_generator.GroovaeSequenceGenerator(
    model=config.model,
    details=config.details,
    steps_per_quarter=config.hparams.steps_per_quarter,
    checkpoint=checkpoint_path,
    bundle=None)

groovae_generator.initialize()

# 4마디 (4 * 4 steps = 64 step = 4초) 추출
groove_input = sequences_lib.extract_subsequence(combined_sequence, total_time, total_time + 4)
groove_input = sequences_lib.quantize_note_sequence(groove_input, steps_per_quarter=4)

vae_options = generator_pb2.GeneratorOptions()
vae_options.args['temperature'].float_value = 1.0
vae_sequence = groovae_generator.generate(groove_input, vae_options)

# GrooVAE 결과 붙이기
groove_output_sequence = concatenate_sequences([primer, vae_sequence])

# === GrooVAE 적용 결과 저장 ===
groove_output_midi = "output.mid"
sequence_proto_to_midi_file(groove_output_sequence, groove_output_midi)
print(f"✅ GrooVAE 적용 결과 저장 완료: {groove_output_midi}")

# ▶️ 재생은 GrooVAE 결과 기준
def play_with_timidity(midi_path):
    print(f"🎧 timidity로 재생 중: {midi_path}")
    subprocess.run(["timidity", midi_path])

play_with_timidity(groove_output_midi)

