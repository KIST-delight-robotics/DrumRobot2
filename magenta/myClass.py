from myRecordMIDI import MyRecord
from myMagenta import MyMagenta
from myPrintMIDI import print_midi_sequence
from myPrintMIDI import print_midi_mido
from myPrintMIDI import print_midi_pretty_midi

class taskManager:
    def __init__(self):
        print("1")

    def make_sync(self):
        rec = MyRecord()

    def make_midi(self, num_repeats, wait_times, recording_times, creation_times):
        print(f"[Python] number of repeats : {num_repeats}")
        
        for i in range(num_repeats):
            print(f"[Python] {i+1} wait times : {wait_times[i]}")
            print(f"[Python] {i+1} recording times : {recording_times[i]}")
            print(f"[Python] {i+1} creation times : {creation_times[i]}")