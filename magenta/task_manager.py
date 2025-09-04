from midi_recording import RecordingManager
from magenta_model import MagentaManager
from midi_match import match_best_from_cache

class taskManager:
    def __init__(self):
        print("1")

    def make_sync(self):
        rec = RecordingManager()

    def make_midi(self, num_repeats, wait_times, recording_times, creation_times):
        print(f"[Python] number of repeats : {num_repeats}")
        
        for i in range(num_repeats):
            print(f"[Python] {i+1} wait times : {wait_times[i]}")
            print(f"[Python] {i+1} recording times : {recording_times[i]}")
            print(f"[Python] {i+1} creation times : {creation_times[i]}")