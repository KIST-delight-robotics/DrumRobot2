import mido
import pretty_midi
import note_seq
from mido import MidiFile, MidiTrack, Message
import time
import datetime

def print_midi_mido(midi_path):
    midi_file = mido.MidiFile(midi_path)

    # 트랙 내의 모든 메시지 출력
    for i, track in enumerate(midi_file.tracks):
        print(f"Track {i}: {track.name}")
        for msg in track:
            print(msg)

def print_midi_pretty_midi(midi_path):
    midi_data = pretty_midi.PrettyMIDI(midi_path)
    print(f"Resolution (Ticks per Quarter): {midi_data.resolution}")

    # Tempo 정보 출력
    tempo, _ = midi_data.get_tempo_changes()
    print(f"Tempo: {tempo}")

    # 악기와 노트 수 출력
    num_notes = 0
    for instrument in midi_data.instruments:
        num_notes += len(instrument.notes)
        print(f"Instrument: {instrument.program}, Number of Notes: {len(instrument.notes)}")
        for note in instrument.notes:
            print(f"Pitch: {note.pitch}, Start: {note.start}, End: {note.end}")
    
    print(f"Total Number of Notes: {num_notes}")

def print_midi_sequence(midi_path):

    # 입력 MIDI 파일을 로드
    input_sequence = note_seq.midi_file_to_sequence_proto(midi_path)
    
    for note in input_sequence.notes:
        print(f"Pitch: {note.pitch}, Start Time: {note.start_time}, End Time: {note.end_time}, Is Drum: {note.is_drum}")

# 미디 분석 및 출력
# midi_path = 'record/drum_recording_0_1_quantizer.mid'
# print("sequence----------------------------------------------------")
# print_midi_sequence(midi_path)
# print("mido----------------------------------------------------")
# print_midi_mido(midi_path)
# print("pretty----------------------------------------------------")
# print_midi_pretty_midi(midi_path)

# 녹음 오브젝트
class RecordingManager:
    def __init__(self, input_port_name, bpm=120, base_path=None):
        self.bpm = bpm
        self.input_port_name = input_port_name

        print(f"\n[Python] Connected to {self.input_port_name}")

        self.base_path = base_path
        
    def print_device_name(self):
        # 연결된 MIDI 입력 장치 이름 확인
        print(mido.get_input_names())

    # MIDI 입력 버퍼 비우기
    def clear_input_buffer(self, midi_input, wait_second=3):
        # 현재 입력 포트에서 대기 중인 모든 메시지를 읽어 버립니다.
        print("\n[Python] Clearing input buffer...")
        print(f"[Python] Waiting for {wait_second} seconds.")
        print(f"[Python] {wait_second}")
        start = time.time()
        cnt_second = 1
        while time.time() < start + wait_second:
            for msg in midi_input.iter_pending():
                # print(f"[Python] Message discarded: {msg}")    # 지워진 메세지 출력
                pass

            if time.time() > start + cnt_second:
                print(f"[Python] {wait_second - cnt_second}")
                cnt_second = cnt_second + 1

            time.sleep(0.01)   # CPU 사용률을 낮추기 위해 짧은 딜레이 추가
    
    def make_sync_file(self):
        current_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        if self.base_path is None:
            with open("sync/sync.txt", "w") as f:
                f.write(current_time)
        else:
            with open(self.base_path + "sync/sync.txt", "w") as f:
                f.write(current_time)

    # 첫 타격 감지
    def detect_first_hit(self):
        bpm = self.bpm  # 템포
        ticks_per_beat = 480  # 1 비트(quarter note) 당 틱 수
        
        # MIDI 입력 포트 연결
        midi_input = mido.open_input(self.input_port_name)

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
        
        first_note_received = False

        print("[Python] Waiting for the first hit...")
        # MIDI 입력을 받아 메시지를 파일에 기록
        while True:

            # 입력 메세지가 들어올 때마다 실행
            for msg in midi_input.iter_pending():

                if msg.type == 'note_on' and not first_note_received:
                    first_note_received = True  # 첫 번째 note_on 메시지 수신 시
                    print("\n[Python] First note received, make sync file")
                    self.make_sync_file()   # sync 파일 생성

            # 주어진 시간만큼 녹음하고 탈출
            if first_note_received:
                break
        
        midi_input.close()

    # 일정 시간동안 MIDI 신호를 실시간으로 받아와서 MIDI 파일로 저장
    def record_for_time(self, output_file, recording_second, buffer_clear_flag=False):
        base_bpm = 120
        bpm = self.bpm  # 템포
        ticks_per_beat = 480  # 1 비트(quarter note) 당 틱 수
        seconds_per_beat = 60 / bpm  # 1 비트의 시간(초 단위)
        
        # MIDI 입력 포트 연결
        midi_input = mido.open_input(self.input_port_name)

        # 새로운 MIDI 파일 생성
        mid = MidiFile(ticks_per_beat=ticks_per_beat)
        track = MidiTrack()
        mid.tracks.append(track)

        # 트랙 이름 설정과 템포 설정
        track.append(mido.MetaMessage('track_name', name='Drum Track'))
        track.append(mido.MetaMessage('set_tempo', tempo=mido.bpm2tempo(base_bpm)))  # 템포 설정
        track.append(mido.MetaMessage('time_signature', numerator=4, denominator=4))
        track.append(Message('program_change', channel=9, program=0, time=0))
        
        if buffer_clear_flag:
            self.clear_input_buffer(midi_input)

        print(f"\n[Python] Recording for {recording_second} seconds after the first note...")
        
        start_time = time.time()  # 녹음 시작 시간 기록
        last_message_time = time.time()

        # MIDI 입력을 받아 메시지를 파일에 기록
        while True:

            # 입력 메세지가 들어올 때마다 실행
            for msg in midi_input.iter_pending():

                print(f"[Python] Recording - Message Received: {msg}")

                now = time.time()
                time_since_last_message = now - last_message_time  # 메시지 간의 시간 차이(초)
                ticks_since_last_message = time_since_last_message / seconds_per_beat * ticks_per_beat  # 틱 단위로 변환
                t = int(ticks_since_last_message)  # t를 틱 단위로 설정
                last_message_time = time.time()

                # 받은 메시지를 트랙에 추가
                if msg.type == 'note_on':
                    track.append(Message('note_on', channel=9, note=msg.note, velocity=msg.velocity, time=t))
                else:
                    track.append(Message('note_on', channel=9, note=msg.note, velocity=0, time=t))  # 'note_off'는 velocity가 0으로 처리
                        
            # 주어진 시간만큼 녹음하고 탈출
            elapsed_time = time.time() - start_time  # 첫 타격 이후 경과 시간
            if elapsed_time >= recording_second:
                print(f"[Python] Recording stopped after {recording_second} seconds.")
                break
                
            time.sleep(0.01)   # CPU 사용률을 낮추기 위해 짧은 딜레이 추가
        
        # MIDI 파일 저장
        track.append(mido.MetaMessage('end_of_track', time=1))
        mid.save(output_file)
        print(f"\n[Python] Recording saved to {output_file}")
        midi_input.close()

    # 첫 타격 감지 후 일정 시간동안 MIDI 신호를 실시간으로 받아와서 MIDI 파일로 저장
    def record_after_first_hit(self, output_file, wait_second, recording_second):
        base_bpm = 120
        bpm = self.bpm  # 템포
        ticks_per_beat = 480  # 1 비트(quarter note) 당 틱 수
        seconds_per_beat = 60 / bpm  # 1 비트의 시간(초 단위)
        
        # MIDI 입력 포트 연결
        midi_input = mido.open_input(self.input_port_name)

        # 새로운 MIDI 파일 생성
        mid = MidiFile(ticks_per_beat=ticks_per_beat)
        track = MidiTrack()
        mid.tracks.append(track)

        # 트랙 이름 설정과 템포 설정
        track.append(mido.MetaMessage('track_name', name='Drum Track'))
        track.append(mido.MetaMessage('set_tempo', tempo=mido.bpm2tempo(base_bpm)))  # 템포 설정
        track.append(mido.MetaMessage('time_signature', numerator=4, denominator=4))
        track.append(Message('program_change', channel=9, program=0, time=0))
        
        self.clear_input_buffer(midi_input, wait_second)

        print(f"\n[Python] Recording for {recording_second} seconds after the first note...")
        
        first_note_received = False
        start_time = None
        last_message_time = None

        # MIDI 입력을 받아 메시지를 파일에 기록
        while True:

            # 입력 메세지가 들어올 때마다 실행
            for msg in midi_input.iter_pending():

                print(f"[Python] Recording - Message Received: {msg}")

                if msg.type == 'note_on' and not first_note_received:
                    first_note_received = True  # 첫 번째 note_on 메시지 수신 시
                    start_time = time.time()  # 녹음 시작 시간 기록
                    last_message_time = time.time()
                    t = 0
                    print("[Python] First note received, starting recording...")
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
                if elapsed_time >= recording_second:
                    print(f"[Python] Recording stopped after {recording_second} seconds.")
                    break
                
            time.sleep(0.01)   # CPU 사용률을 낮추기 위해 짧은 딜레이 추가
        
        # MIDI 파일 저장
        track.append(mido.MetaMessage('end_of_track', time=1))
        mid.save(output_file)
        print(f"\n[Python] Recording saved to {output_file}")
        midi_input.close()
