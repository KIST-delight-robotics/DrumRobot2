# ==========================================
# [중요] TLS 메모리 에러 해결을 위한 코드
# 반드시 다른 import보다 가장 먼저 와야 합니다.
import sklearn
# ==========================================

# 경고 메시지 억제
import warnings
import os
import sys
import threading
import mido # pip install mido

warnings.filterwarnings("ignore")   # 경고 메시지 억제
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'    # TensorFlow의 로그 레벨 설정

import tensorflow as tf
tf.get_logger().setLevel('ERROR')   # TensorFlow 경고 메시지 억제

from task_manager import taskManager

# ==========================================
# [추가됨] Raw MIDI Reader Class
# ALSA Sequencer 없이 장치 파일에서 직접 읽어오는 클래스
# ==========================================
# ★ 중요: ls -l /dev/snd/ 확인 후 경로 수정 (또는 udev rule 적용 후 /dev/drum_pad 사용)
FIXED_MIDI_PATH = '/dev/snd/midiC3D0' # **실제 장치**에 맞게 수정 필요

class DrumPadInterface:
    def __init__(self, device_path):
        self.device_path = device_path
        self.running = False
        self.parser = mido.Parser()
        self.latest_msg = None
        self._thread = None
        self._lock = threading.Lock()

    def _read_loop(self):
        print(f"[MIDI] Opening Raw Device: {self.device_path}")
        try:
            # buffering=0 (Unbuffered) 필수
            with open(self.device_path, 'rb', buffering=0) as f:
                print("[MIDI] Device Opened Successfully.")
                while self.running:
                    byte = f.read(1)
                    if byte:
                        self.parser.feed(byte)
                        for message in self.parser:
                            # Note On + Velocity > 0 인 경우만 캡처
                            if message.type == 'note_on' and message.velocity > 0:
                                with self._lock:
                                    self.latest_msg = message
        except Exception as e:
            print(f"[MIDI Error] {e}")
            print(f"[Check] 'sudo chmod 666 {self.device_path}' or Check Connection.")

    def start(self):
        if not self.running:
            self.running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            print("[MIDI] Reader Stopped.")

    def get_event(self):
        """taskManager가 호출할 메서드"""
        with self._lock:
            if self.latest_msg:
                msg = self.latest_msg
                self.latest_msg = None # 읽은 후 비움 (One-shot)
                return msg
            return None

# ==========================================
# [기존 로직] Argument Parsing
# ==========================================
program_name = sys.argv[0]
print(f"[Python] program name : {program_name}")

bpm = 120
base_path = None
record = False # Default value explicitly set
num_repeats = 1 # Default value needed if not passed

num_args = len(sys.argv) - 1
for i in range(num_args):
    arg = sys.argv[i+1]
    
    if arg == "--sync": 
        print(f"[Python] program mode : {arg}")   
        record = False
    
    elif arg == "--record":
        print(f"[Python] program mode : {arg}")
        record = True
    
    elif arg == "--repeat":
        try:
            num_repeats = int(sys.argv[i+2])
        except IndexError:
            pass # Handle error appropriately
    
    elif arg == "--param":
        num_param = len(sys.argv) - i - 2
        # param parsing logic kept as is...
        if num_param >= num_repeats * 3:
            wait_times_sec = []
            recording_times_bar = []
            creation_times_bar = []
            for j in range(num_repeats):
                wait_times_sec.append(float(sys.argv[i + 2 + 3*j]))
                recording_times_bar.append(float(sys.argv[i + 3 + 3*j]))
                creation_times_bar.append(float(sys.argv[i + 4 + 3*j]))
    
    elif arg == "--bpm":
        bpm = float(sys.argv[i+2])
        pass
    
    elif arg == "--path":
        base_path = sys.argv[i+2]
        pass
    
    else:
        pass

# ==========================================
# [통합] MIDI Interface 생성 및 taskManager 실행
# ==========================================

# 1. MIDI Reader 인스턴스 생성
midi_interface = DrumPadInterface(FIXED_MIDI_PATH)

try:
    # 2. MIDI 읽기 시작 (백그라운드 스레드)
    midi_interface.start()

    # 3. taskManager 생성
    # [중요] taskManager 내부에서 mido.open_input()을 하는 대신
    # 이 midi_interface 객체를 받아서 쓰도록 수정해야 합니다.
    # 일단 생성자에 넘겨주거나, setter를 사용한다고 가정합니다.
    tm = taskManager(bpm=bpm, base_path=base_path)
    
    # taskManager에 MIDI 인터페이스 주입 (task_manager.py 수정 필요)
    if hasattr(tm, 'set_midi_interface'):
        tm.set_midi_interface(midi_interface)
    else:
        print("[Warning] taskManager에 set_midi_interface 메서드가 없습니다.")
        print("task_manager.py를 수정하여 MIDI 객체를 받도록 해야 합니다.")
        # 임시로 속성 강제 주입 (taskManager 코드 구조에 따라 다름)
        tm.midi_interface = midi_interface 

    # 4. 모드에 따른 실행
    if record:
        # make_midi 내부에서도 self.midi_interface.get_event()를 호출해야 함
        tm.make_midi(num_repeats, wait_times_sec, recording_times_bar)
        # tm.make_midi_from_folder(num_repeats, recording_times_bar)
    else:
        tm.make_sync()

except KeyboardInterrupt:
    print("\n[Python] Interrupted by User")

except Exception as e:
    print(f"[Python] Error occurred: {e}")

finally:
    # 프로그램 종료 시 MIDI 스레드도 안전하게 종료
    midi_interface.stop()
    print("[Python] System Shutdown Complete.")