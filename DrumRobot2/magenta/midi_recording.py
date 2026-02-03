import mido
import pretty_midi
import note_seq
from mido import MidiFile, MidiTrack, Message
import time
import datetime

# =================================================================
# [사용자 설정] 모드 스위치
# False: AI 합주 모드 (즉시 녹음) -> 지금 테스트할 때 사용
# True : 'This Is Me' 공연 모드 (15초 대기 기능 켜짐)
# =================================================================
IS_PERFORMANCE_MODE = False 

# ====================================================
# [디버깅용 출력 함수들은 그대로 유지]
# ====================================================
def print_midi_mido(midi_path):
    midi_file = mido.MidiFile(midi_path)
    for i, track in enumerate(midi_file.tracks):
        print(f"Track {i}: {track.name}")
        for msg in track:
            print(msg)

def print_midi_pretty_midi(midi_path):
    midi_data = pretty_midi.PrettyMIDI(midi_path)
    print(f"Resolution (Ticks per Quarter): {midi_data.resolution}")
    tempo, _ = midi_data.get_tempo_changes()
    print(f"Tempo: {tempo}")
    num_notes = 0
    for instrument in midi_data.instruments:
        num_notes += len(instrument.notes)
        print(f"Instrument: {instrument.program}, Number of Notes: {len(instrument.notes)}")
        for note in instrument.notes:
            print(f"Pitch: {note.pitch}, Start: {note.start}, End: {note.end}")
    print(f"Total Number of Notes: {num_notes}")

def print_midi_sequence(midi_path):
    input_sequence = note_seq.midi_file_to_sequence_proto(midi_path)
    for note in input_sequence.notes:
        print(f"Pitch: {note.pitch}, Start Time: {note.start_time}, End Time: {note.end_time}, Is Drum: {note.is_drum}")


# ====================================================
# [수정됨] RecordingManager 클래스
# ====================================================
class RecordingManager:
    def __init__(self, midi_interface, bpm=120, base_path=None):
        self.bpm = bpm
        self.midi_interface = midi_interface  # 객체 저장
        self.base_path = base_path
        print(f"\n[Python] RecordingManager Initialized (Raw MIDI Mode)")
        
        if IS_PERFORMANCE_MODE:
            print("★ [Mode] PERFORMANCE MODE ON (15초 대기 기능 활성화)")
        else:
            print("★ [Mode] AI JAM MODE ON (즉시 녹음)")

    def print_device_name(self):
        print("[Python] Device: Raw MIDI Interface (/dev/snd/midiC2D0)")

    def clear_input_buffer(self, wait_second=3):
        print("\n[Python] Clearing input buffer...")
        print(f"[Python] Waiting for {wait_second} seconds.")
        
        start = time.time()
        cnt_second = 1
        
        while time.time() < start + wait_second:
            _ = self.midi_interface.get_event()

            if time.time() > start + cnt_second:
                print(f"[Python] {wait_second - cnt_second}")
                cnt_second = cnt_second + 1

            time.sleep(0.01)

    def make_sync_file(self):
        current_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        if self.base_path is None:
            path = "sync/sync.txt"
        else:
            path = self.base_path + "sync/sync.txt"
            
        with open(path, "w") as f:
            f.write(current_time)

    def detect_first_hit(self):
        bpm = self.bpm
        ticks_per_beat = 480
        
        mid = MidiFile(ticks_per_beat=ticks_per_beat)
        track = MidiTrack()
        mid.tracks.append(track)
        track.append(mido.MetaMessage('track_name', name='Drum Track'))
        track.append(mido.MetaMessage('set_tempo', tempo=mido.bpm2tempo(bpm)))
        track.append(mido.MetaMessage('time_signature', numerator=4, denominator=4))
        track.append(Message('program_change', channel=9, program=0, time=0))
        
        self.clear_input_buffer(wait_second=1)
        
        first_note_received = False
        print("[Python] Waiting for the first hit...")

        while True:
            msg = self.midi_interface.get_event()

            if msg:
                if msg.type == 'note_on' and not first_note_received:
                    first_note_received = True
                    print("\n[Python] First note received, make sync file")
                    self.make_sync_file()

            if first_note_received:
                break
            
            time.sleep(0.001)

    def record_for_time(self, output_file, buffer_clear_flag=False):
        bpm = self.bpm
        ticks_per_beat = 480
        seconds_per_beat = 60 / bpm
        recording_second = 8 * seconds_per_beat
        
        mid = MidiFile(ticks_per_beat=ticks_per_beat)
        track = MidiTrack()
        mid.tracks.append(track)

        track.append(mido.MetaMessage('track_name', name='Drum Track'))
        track.append(mido.MetaMessage('set_tempo', tempo=mido.bpm2tempo(bpm)))
        track.append(mido.MetaMessage('time_signature', numerator=4, denominator=4))
        track.append(Message('program_change', channel=9, program=0, time=0))
        
        if buffer_clear_flag:
            self.clear_input_buffer()

        print(f"\n[Python] Recording for {recording_second} seconds...")
        
        start_time = time.time()
        last_message_time = time.time()
        total_ticks_recorded = 0

        while True:
            msg = self.midi_interface.get_event()

            if msg:
                print(f"[Python] Recording - Message Received: {msg}")

                now = time.time()
                time_since_last_message = now - last_message_time
                ticks = int(time_since_last_message / seconds_per_beat * ticks_per_beat)
                t = ticks

                if msg.type == 'note_on':
                    track.append(Message('note_on', channel=9, note=msg.note, velocity=msg.velocity, time=t))
                    last_message_time = time.time()
                    total_ticks_recorded += t
                elif msg.type == 'note_off':
                    track.append(Message('note_on', channel=9, note=msg.note, velocity=0, time=t))
                    last_message_time = time.time()
                    total_ticks_recorded += t
                        
            elapsed_time = time.time() - start_time
            if elapsed_time >= recording_second:
                print(f"[Python] Recording stopped after {recording_second} seconds.")
                break
                
            time.sleep(0.001)
        
        # 공백 채우기 (AI 에러 방지용 - 항상 작동)
        target_total_ticks = int(recording_second / seconds_per_beat * ticks_per_beat)
        remaining_ticks = target_total_ticks - total_ticks_recorded
        if remaining_ticks < 0: remaining_ticks = 0
        track.append(mido.MetaMessage('end_of_track', time=remaining_ticks))

        mid.save(output_file)
        print(f"\n[Python] Recording saved to {output_file}")


    # ★ 핵심 수정 함수
    # midi_recording.py 전체 함수 교체

    def record_after_first_hit(self, output_file, wait_second, sample_i):
        bpm = self.bpm
        ticks_per_beat = 480
        seconds_per_beat = 60 / bpm
        recording_second = 8 * seconds_per_beat  # 2마디 (4/4박자)
        
        # ★ [핵심] 최대 허용 틱 수 계산 (이걸 넘으면 안됨)
        target_total_ticks = int(recording_second / seconds_per_beat * ticks_per_beat)

        mid = MidiFile(ticks_per_beat=ticks_per_beat)
        track = MidiTrack()
        mid.tracks.append(track)

        track.append(mido.MetaMessage('track_name', name='Drum Track'))
        track.append(mido.MetaMessage('set_tempo', tempo=mido.bpm2tempo(bpm)))
        track.append(mido.MetaMessage('time_signature', numerator=4, denominator=4))
        track.append(Message('program_change', channel=9, program=0, time=0))
        
        # 버퍼 비우기
        self.clear_input_buffer(wait_second)

        print(f"\n[Python] Recording for {recording_second} seconds (Max Ticks: {target_total_ticks})...")
        
        first_note_received = False
        start_time = None
        last_message_time = None
        total_ticks_recorded = 0 

        while True:
            msg = self.midi_interface.get_event()

            if msg:
                print(f"[Python] Recording - Message Received: {msg}")

                if msg.type == 'note_on' and msg.velocity > 0 and not first_note_received:
                    first_note_received = True
                    
                    # Performance Mode Check
                    if IS_PERFORMANCE_MODE:
                        if sample_i == 0: self.clear_input_buffer(15)
                        if sample_i == 2: self.clear_input_buffer(10)

                    start_time = time.time()
                    last_message_time = time.time()
                    
                    # 첫 노트 기록
                    track.append(Message('note_on', channel=9, note=msg.note, velocity=msg.velocity, time=0))
                    # 강제 Note Off (60틱)
                    track.append(Message('note_off', channel=9, note=msg.note, velocity=0, time=60))
                    
                    total_ticks_recorded += 60
                    
                    print("[Python] First note received, starting recording...")
                    self.make_sync_file()
                
                elif first_note_received and msg.type == 'note_on' and msg.velocity > 0:
                    now = time.time()
                    time_since_last_message = now - last_message_time
                    ticks = int(time_since_last_message / seconds_per_beat * ticks_per_beat)
                    
                    # ---------------------------------------------------------
                    # ★ [핵심 수정] 오버플로우 방지 (칼같이 자르기)
                    # ---------------------------------------------------------
                    # 이번에 추가할 틱(ticks + 60)이 한도를 넘는지 검사
                    if total_ticks_recorded + ticks + 60 > target_total_ticks:
                        print(f"[Python] Time limit reached! discarding extra note.")
                        break # 루프 즉시 탈출
                    # ---------------------------------------------------------

                    current_delta = ticks
                    if current_delta < 0: current_delta = 0

                    track.append(Message('note_on', channel=9, note=msg.note, velocity=msg.velocity, time=current_delta))
                    track.append(Message('note_off', channel=9, note=msg.note, velocity=0, time=60))
                    
                    total_ticks_recorded += (current_delta + 60)
                    last_message_time = time.time()

            # 시간 체크 (이중 안전장치)
            if first_note_received:
                elapsed_time = time.time() - start_time
                if elapsed_time >= recording_second:
                    print(f"[Python] Recording timer finished.")
                    break
                
            time.sleep(0.001)

        # ==========================================================
        # ★ [필수] 모자란 시간 채우기 (Underflow 방지)
        # ==========================================================
        remaining_ticks = target_total_ticks - total_ticks_recorded
        
        if remaining_ticks < 0: 
            remaining_ticks = 0 # 이미 넘쳤으면 0으로
        
        print(f"[Python] Finalizing track. Total: {total_ticks_recorded}, Padding: {remaining_ticks}")
        track.append(mido.MetaMessage('end_of_track', time=remaining_ticks))
        
        mid.save(output_file)
        print(f"\n[Python] Recording saved to {output_file}")