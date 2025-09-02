import pretty_midi
import mido
import note_seq

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

def print_midi_mido(midi_path):
    midi_file = mido.MidiFile(midi_path)

    # 트랙 내의 모든 메시지 출력
    for i, track in enumerate(midi_file.tracks):
        print(f"Track {i}: {track.name}")
        for msg in track:
            print(msg)

# Start sequence와 End sequence의 노트 출력하기
def print_midi_sequence(midi_path):

    # 입력 MIDI 파일을 로드
    input_sequence = note_seq.midi_file_to_sequence_proto(midi_path)
    
    for note in input_sequence.notes:
        print(f"Pitch: {note.pitch}, Start Time: {note.start_time}, End Time: {note.end_time}, Is Drum: {note.is_drum}")