import mido
from mido import MidiFile, MidiTrack
import time
import sys
import datetime

# MIDI 입력 필터링 함수
def is_saveable_message(msg):
    return (
        not msg.is_meta and
        msg.type in {
            'note_on', 'note_off',
            'control_change', 'program_change',
            'pitchwheel', 'aftertouch', 'polytouch'
        } and msg.channel == 9
    )

# MIDI 포트 연결
input_ports = mido.get_input_names()
if not input_ports:
    print("MIDI 입력 장치가 감지되지 않았습니다.")
    sys.exit(1)

port_index = 1  # 사용자가 원하는 포트 인덱스
port_name = input_ports[port_index]
print(f"\n✅ MIDI 장치 연결 확인됨: {port_name}", flush=True)

def flush_during_recording(inport):
    for _ in range(5):
        for _ in inport.iter_pending():
            pass
        time.sleep(0.01)

# 카운트다운
print("\n3초 후 녹음이 시작됩니다...")
for i in [3, 2, 1]:
    print(i)
    time.sleep(1)

print("녹음 시작... 첫 입력 대기 중")

msgs = [] 

# 첫 note_on 감지 및 시간 저장
first_note_time_saved = False
with mido.open_input(port_name) as inport:
    flush_during_recording(inport)
    while True:
        for msg in inport.iter_pending():
            if is_saveable_message(msg):
                if msg.type == 'note_on' and msg.velocity > 0:
                    if not first_note_time_saved:
                        current_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        with open("/home/shy/DrumRobot/include/sync/sync.txt", "w") as f:
                            f.write(current_time)
                        print(f"✅ 첫 입력 감지됨: {current_time} → 저장 완료")
                        first_note_time_saved = True
                        sys.exit(0)
        time.sleep(0.001)
