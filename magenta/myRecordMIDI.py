import mido
from mido import MidiFile, MidiTrack, Message
import time
import datetime

class MyRecord:
    def __init__(self):
        self.bpm = 120
        
    def print_midi_input(self):
        # 연결된 MIDI 입력 장치 이름 확인
        print(mido.get_input_names())

    # MIDI 입력 버퍼 비우기
    def clear_input_buffer(self, midi_input):
        # 현재 입력 포트에서 대기 중인 모든 메시지를 읽어 버립니다.
        print("\nClearing input buffer...")
        start = time.time()
        while time.time() < start + 3:
            for msg in midi_input.iter_pending():
                print(f"Message discarded: {msg}")

            time.sleep(0.01)   # CPU 사용률을 낮추기 위해 짧은 딜레이 추가
    
    def make_sync_file(self):
        current_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with open("/home/shy/DrumRobot/include/sync/sync.txt", "w") as f:
            f.write(current_time)

    # MIDI 신호를 실시간으로 받아와서 MIDI 파일로 저장
    def record_midi_second(self, input_port_name, output_file, record_duration_second):
        bpm = self.bpm  # 템포
        ticks_per_beat = 220  # 1 비트(quarter note) 당 틱 수
        seconds_per_beat = 60 / bpm  # 1 비트의 시간(초 단위)
        
        # MIDI 입력 포트 연결
        midi_input = mido.open_input(input_port_name)
        print(f"\nConnected to {input_port_name}")

        # 새로운 MIDI 파일 생성
        mid = MidiFile(ticks_per_beat=ticks_per_beat)
        track = MidiTrack()
        mid.tracks.append(track)

        # 트랙 이름 설정과 템포 설정
        track.append(mido.MetaMessage('track_name', name='Drum Track'))
        track.append(mido.MetaMessage('set_tempo', tempo=mido.bpm2tempo(bpm)))  # 템포 설정
        track.append(mido.MetaMessage('time_signature', numerator=4, denominator=4))
        track.append(Message('program_change', channel=9, program=0, time=0))
        
        self.clear_input_buffer(midi_input)

        print(f"\nRecording for {record_duration_second} seconds after the first note...")
        
        first_note_received = False
        start_time = None
        last_message_time = None

        # MIDI 입력을 받아 메시지를 파일에 기록
        while True:

            # 입력 메세지가 들어올 때마다 실행
            for msg in midi_input.iter_pending():

                print(f"Message Received: {msg}")

                if msg.type == 'note_on' and not first_note_received:
                    first_note_received = True  # 첫 번째 note_on 메시지 수신 시
                    start_time = time.time()  # 녹음 시작 시간 기록
                    last_message_time = time.time()
                    t = 0
                    print("First note received, starting recording...")
                    self.make_sync_file()   # sync 파일 생성
                elif first_note_received:
                    now = time.time()
                    time_since_last_message = now - last_message_time  # 메시지 간의 시간 차이(초)
                    ticks_since_last_message = time_since_last_message / seconds_per_beat * ticks_per_beat  # 틱 단위로 변환
                    t = int(ticks_since_last_message)  # t를 틱 단위로 설정
                    last_message_time = time.time()
                        
                if first_note_received:
                    # 받은 메시지를 트랙에 추가
                    if msg.type == 'note_on':
                        track.append(Message('note_on', channel=9, note=msg.note, velocity=msg.velocity, time=t))
                    else:
                        track.append(Message('note_on', channel=9, note=msg.note, velocity=0, time=t))  # 'note_off'는 velocity가 0으로 처리

            # 주어진 시간만큼 녹음하고 탈출
            if first_note_received:
                elapsed_time = time.time() - start_time  # 첫 타격 이후 경과 시간
                if elapsed_time >= record_duration_second:
                    print(f"Recording stopped after {record_duration_second} seconds.")
                    break
                
            time.sleep(0.01)   # CPU 사용률을 낮추기 위해 짧은 딜레이 추가
        
        # MIDI 파일 저장
        track.append(mido.MetaMessage('end_of_track', time=1))
        mid.save(output_file)
        print(f"Recording saved to {output_file}")