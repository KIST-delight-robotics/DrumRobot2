import note_seq
from magenta.models.drums_rnn import drums_rnn_sequence_generator
from magenta.models.shared import sequence_generator_bundle
from note_seq.protobuf import generator_pb2
from magenta.music import sequence_proto_to_midi_file
import tensorflow.compat.v1 as tf
import time
import subprocess
from pynput import keyboard
from datetime import datetime

tf.disable_v2_behavior()

# 모델 로드
bundle = sequence_generator_bundle.read_bundle_file('drum_kit_rnn.mag')
generator = drums_rnn_sequence_generator.get_generator_map()['drum_kit'](
    checkpoint=None, bundle=bundle)
generator.initialize()

# 키 매핑 (0~9까지 모두 포함)
key_to_pitch = {
    '0': 46,  # 오픈 하이햇
    '1': 38,  # 스네어
    '2': 41,  # 플로우 탐
    '3': 45,  # 미드 탐
    '4': 47,  # 타탐
    '5': 42,  # 하이햇
    '6': 51,  # 라이드 심볼
    '7': 49,  # 크래시 심볼 오른쪽
    '8': 57,  # 크래시 심볼 왼쪽
    '9': 36   # 킥
}

primer_sequence = note_seq.NoteSequence()
recording = False
start_time = 0
MIN_INTERVAL = 0.1
last_pressed = {"key": None, "time": 0}

def on_press(key):
    global recording, start_time, last_pressed

    try:
        k = key.char
    except:
        return

    now = time.time()

    if k == 's' and not recording:
        recording = True
        start_time = now
        print("🎬 녹음 시작 (s)")

    elif k == 'e' and recording:
        recording = False
        print("🛑 녹음 종료 (e)")

    elif recording and k in key_to_pitch:
        if k == last_pressed["key"] and (now - last_pressed["time"]) < MIN_INTERVAL:
            return
        last_pressed["key"] = k
        last_pressed["time"] = now
        t = now - start_time
        pitch = key_to_pitch[k]
        primer_sequence.notes.add(pitch=pitch, start_time=t, end_time=t + 0.1, velocity=120)
        print(f"🎵 입력된 키: {k} → Pitch: {pitch}, 시간: {t:.3f}s")

def play_midi_with_cli(midi_path):
    sf2_path = "/usr/share/sounds/sf2/FluidR3_GM.sf2"
    cmd = [
        "fluidsynth",
        "-a", "alsa",
        "-o", "audio.alsa.device=default",
        "-g", "1.5",  # 🔊 볼륨 증폭
        "-ni", sf2_path,
        midi_path
    ]
    print("🎧 실행 커맨드:", " ".join(cmd))
    subprocess.run(cmd)

print("🎹 's' 누르면 녹음 시작, 'e' 누르면 종료. 0~9 키로 드럼 입력")

listener = keyboard.Listener(on_press=on_press)
listener.start()

while not recording:
    time.sleep(0.1)
while recording:
    time.sleep(0.01)

primer_sequence.total_time = time.time() - start_time
print(f"📦 입력된 리듬 길이: {primer_sequence.total_time:.2f}초")

generator_options = generator_pb2.GeneratorOptions()
generator_options.generate_sections.add(
    start_time=primer_sequence.total_time,
    end_time=primer_sequence.total_time + 12.0
)
generator.temperature = 1.0
generator.steps_per_quarter = 4

print("🎶 Magenta로 드럼 생성 중...")
generated_sequence = generator.generate(primer_sequence, generator_options)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_path = f"generated_{timestamp}.mid"
sequence_proto_to_midi_file(generated_sequence, output_path)
print(f"✅ 생성 완료! 저장 위치: {output_path}")

print("🔊 재생 시작...")
play_midi_with_cli(output_path)
print("🏁 재생 종료.")
