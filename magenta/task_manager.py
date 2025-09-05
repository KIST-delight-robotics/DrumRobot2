import threading
from midi_recording import RecordingManager
from magenta_music_vae import MagentaManager
from midi_match import match_best_from_cache
from midi_quantizer import quantize_drum_midi

from quantize_pretty_midi import quantize_midi

class taskManager:
    def __init__(self, base_path):
        self.base_path = base_path
        self.device_name = 'NUX DP-2000:NUX DP-2000 MIDI 1 20:0' # 연결된 장치 이름
        self.config_name = 'cat-drums_2bar_small'

    def set_path(self):
        if self.base_path == 'null':
            self.checkpoint_path = 'model/cat-drums_2bar_small.hikl.tar'
            self.recording_file_path = 'record/drum_recording_'      # 녹음 저장할 위치
            self.magenta_output = 'generated/output'           # 마젠타 출력 위치
            self.base_folder = 'basic/'
            self.cache_path = "library_cache.npz"
        else:
            self.checkpoint_path = self.base_path + 'model/cat-drums_2bar_small.hikl.tar'
            self.recording_file_path = self.base_path + 'record/drum_recording_'      # 녹음 저장할 위치
            self.magenta_output = self.base_path + 'generated/output'           # 마젠타 출력 위치
            self.base_folder = self.base_path + 'basic/'
            self.cache_path = self.base_path + "library_cache.npz"

    def make_sync(self):
        
        rec = RecordingManager(self.device_name, self.base_path)

        rec.detect_first_hit()

    def make_midi(self, num_repeats, wait_times, recording_times, creation_times):
        
        # 경로 설정
        self.set_path()

        # 오브젝트 생성
        rec = RecordingManager(self.device_name, self.base_path)
        mgt = MagentaManager(self.config_name, self.checkpoint_path)
        
        print(f"[Python] number of repeats : {num_repeats}")
        
        for i in range(num_repeats):
            print(f"\n[Python] wait times : {wait_times[i]}")
            print(f"[Python] recording times : {recording_times[i]}")
            print(f"[Python] creation times : {creation_times[i]}")

            # 첫 번째 녹음
            recording_file_name = self.recording_file_path + str(i) + '_1.mid'
            rec.record_after_first_hit(recording_file_name, wait_times[i], recording_times[i]/2)

            # 전처리
            quantize_midi_file = quantize_drum_midi(recording_file_name)        # 양자화
            base_midi = self.base_folder + match_best_from_cache(midi_path=quantize_midi_file, cache_path=self.cache_path)    # 가장 유시한 리듬
            print(f"\n[Python] matching : {base_midi}")
            
            # 마젠타
            output_filename = self.magenta_output + '_1'
            thread1 = threading.Thread(target=mgt.interpolate_music, args=(quantize_midi_file, base_midi, output_filename))
            thread1.start()

            # 두 번째 녹음
            recording_file_name = self.recording_file_path + str(i) + '_2.mid'
            buffer_clear_flag = False   # 두 번째 녹음은 버퍼 안비움
            rec.record_for_time(recording_file_name, recording_times[i]/2, buffer_clear_flag)

            # 전처리
            quantize_midi_file = quantize_drum_midi(recording_file_name)        # 양자화
            base_midi = self.base_folder + match_best_from_cache(midi_path=quantize_midi_file, cache_path=self.cache_path)   # 가장 유시한 리듬
            print(f"\n[Python] matching : {base_midi}")
            
            # 마젠타
            output_filename = self.magenta_output + '_2'
            thread2 = threading.Thread(target=mgt.interpolate_music, args=(quantize_midi_file, base_midi, output_filename))
            thread2.start()

            thread1.join()
            thread2.join()