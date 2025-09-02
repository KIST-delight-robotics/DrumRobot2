import mido
from mido import MidiFile, MidiTrack, Message
import time

# 연결된 MIDI 입력 장치 이름 확인
# print(mido.get_input_names())

# MIDI 신호를 실시간으로 받아와서 MIDI 파일로 저장
def record_midi(input_port_name, output_file):

    bpm = 120  # 템포
    ticks_per_beat = 480  # 1 비트(quarter note) 당 틱 수
    seconds_per_beat = 60 / bpm  # 1 비트의 시간(초 단위)
    
    # MIDI 입력 포트 연결
    midi_input = mido.open_input(input_port_name)
    print(f"Connected to {input_port_name}")

    # 새로운 MIDI 파일 생성
    mid = MidiFile()
    track = MidiTrack()
    mid.tracks.append(track)

    # 트랙 이름 설정과 템포 설정
    track.append(mido.MetaMessage('track_name', name='Drum Track'))
    track.append(mido.MetaMessage('set_tempo', tempo=mido.bpm2tempo(bpm)))  # 템포 설정
    track.append(mido.MetaMessage('time_signature', numerator=4, denominator=4))
    track.append(Message('program_change', channel=9, program=0, time=0))
    
    print("Recording... Press Ctrl+C to stop.")
    firstNote = True
    start = time.time()

    try:
        # MIDI 입력을 받아 메시지를 파일에 기록
        for msg in midi_input:
            print(f"Message Received: {msg}")

            if firstNote:
                start = time.time()
                t = 0

                firstNote = False
            else:
                now = time.time()
                time_since_last_message = now - start  # 메시지 간의 시간 차이(초)
                ticks_since_last_message = time_since_last_message / seconds_per_beat * ticks_per_beat  # 틱 단위로 변환

                t = int(ticks_since_last_message)  # t를 틱 단위로 설정
                start = time.time()
            
            if msg.type == 'note_on':
                # 받은 메시지를 트랙에 추가
                track.append(Message('note_on', channel=9, note=msg.note, velocity=msg.velocity, time=t))
            else:
                track.append(Message('note_on', channel=9, note=msg.note, velocity=0, time=t))    # 'note_off'면 velocity가 0으로 처리

    except KeyboardInterrupt:
        print("Recording stopped.")
    
    # MIDI 파일 저장
    track.append(mido.MetaMessage('end_of_track', time=1))
    mid.save(output_file)
    print(f"Recording saved to {output_file}")
