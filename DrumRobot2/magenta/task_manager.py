import threading
import mido
# RecordingManager도 수정이 필요합니다 (아래 설명 참조)
from midi_recording import RecordingManager
from magenta_music_vae import MagentaManager
from midi_match import match_best_from_cache
from midi_quantizer import quantize_drum_midi
from quantize_pretty_midi import quantize_midi

class taskManager:
    def __init__(self, bpm, base_path=None):
        self.bpm = bpm
        self.base_path = base_path
        
        # [수정 1] 장치 이름 찾기(get_device_name) 삭제
        # ALSA가 없으므로 이름으로 찾는 것은 불가능합니다.
        self.device_name = None 
        self.midi_interface = None # script.py에서 주입받을 객체
        
        self.config_name = 'cat-drums_2bar_small'

    # [수정 2] 외부에서 Raw MIDI 인터페이스를 주입받는 함수 추가
    def set_midi_interface(self, interface):
        self.midi_interface = interface
        print("[taskManager] MIDI Interface Registered.")

    def set_path(self):
        if self.base_path is None:
            self.checkpoint_path = 'model/cat-drums_2bar_small.hikl.tar'
            self.recording_file_path = 'record/drum_recording_'
            self.magenta_output = 'generated/output'
            self.base_folder = 'basic/'
            self.cache_path = "library_cache.npz"
        else:
            self.checkpoint_path = self.base_path + 'model/cat-drums_2bar_small.hikl.tar'
            self.recording_file_path = self.base_path + 'record/drum_recording_'
            self.magenta_output = self.base_path + 'generated/output'
            self.base_folder = self.base_path + 'basic/'
            self.cache_path = self.base_path + "library_cache.npz"

    def make_sync(self):
        # [수정 3] RecordingManager에 이름 대신 '인터페이스 객체' 전달
        if self.midi_interface is None:
            print("[Error] MIDI Interface not set!")
            return

        # self.device_name 대신 self.midi_interface를 넘김
        rec = RecordingManager(self.midi_interface, self.bpm, self.base_path)
        rec.detect_first_hit()

    def make_midi(self, num_repeats, wait_times_sec, recording_times_bar):
        self.set_path()
        
        if self.midi_interface is None:
            print("[Error] MIDI Interface not set!")
            return

        # [수정 4] RecordingManager 생성 시 인터페이스 전달
        rec = RecordingManager(self.midi_interface, self.bpm, self.base_path)
        mgt = MagentaManager(self.config_name, self.checkpoint_path)
        
        print(f"[Python] number of repeats : {num_repeats}")

        threads = []
        for i in range(num_repeats):
            num_recording = int(recording_times_bar[i] / 2)
            
            for j in range(num_recording):
                recording_file_name = self.recording_file_path + str(i) + '_' + str(j+1) + '.mid'
                
                if j == 0:
                    rec.record_after_first_hit(recording_file_name, wait_times_sec[i], i)
                else:
                    buffer_clear_flag = False
                    rec.record_for_time(recording_file_name, buffer_clear_flag)

                quantize_midi_file = quantize_drum_midi(recording_file_name)
                base_midi = self.base_folder + match_best_from_cache(midi_path=quantize_midi_file, cache_path=self.cache_path)
                print(f"\n[Python] matching : {base_midi}")

                output_filename = self.magenta_output + str(i) + '_' + str(j+1)
                thread = threading.Thread(target=mgt.interpolate_music, args=(quantize_midi_file, base_midi, output_filename))
                threads.append(thread)
                thread.start()

            for thread in threads:
                thread.join()