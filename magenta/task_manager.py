import os
import threading
import mido
from midi_recording import RecordingManager
from magenta_music_vae import MagentaManager
from midi_match import match_best_from_cache
from midi_quantizer import quantize_drum_midi

from quantize_pretty_midi import quantize_midi

class taskManager:
    def __init__(self, bpm, base_path=None):
        self.bpm = bpm
        self.base_path = base_path
        self.get_device_name()
        self.config_name = 'cat-drums_2bar_small'

    def get_device_name(self):
        # 연결된 장치 이름
        device_names = mido.get_input_names()
        
        self.device_name = None
        for device in device_names:
            if device == 'TD-17:TD-17 MIDI 1 20:0':
                self.device_name = 'TD-17:TD-17 MIDI 1 20:0'
            elif device == 'NUX DP-2000:NUX DP-2000 MIDI 1 20:0':
                self.device_name = 'NUX DP-2000:NUX DP-2000 MIDI 1 20:0'

    def set_path(self):
        if self.base_path is None:
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
        
        rec = RecordingManager(self.device_name, self.bpm, self.base_path)

        rec.detect_first_hit()

    def make_midi(self, num_repeats, wait_times_sec, recording_times_bar):
        
        # 경로 설정
        self.set_path()

        # 오브젝트 생성
        rec = RecordingManager(self.device_name, self.bpm, self.base_path)
        mgt = MagentaManager(self.config_name, self.checkpoint_path)
        
        print(f"[Python] number of repeats : {num_repeats}")

        threads = []
        for i in range(num_repeats):
            num_recording = (int) (recording_times_bar[i] / 2)  # 녹음할 횟수 (2마디씩 N번)
            
            for j in range(num_recording):
                recording_file_name = self.recording_file_path + str(i) + '_' + str(j+1) + '.mid'
                
                if j == 0:
                    # 첫 번째 녹음    
                    rec.record_after_first_hit(recording_file_name, wait_times_sec[i], i)
                else:
                    buffer_clear_flag = False   # 두 번째 녹음부터 버퍼 안비움
                    rec.record_for_time(recording_file_name, buffer_clear_flag)

                # 전처리
                quantize_midi_file = quantize_drum_midi(recording_file_name)        # 양자화
                base_midi = self.base_folder + match_best_from_cache(midi_path=quantize_midi_file, cache_path=self.cache_path)    # 가장 유시한 리듬
                print(f"\n[Python] matching : {base_midi}")

                # 마젠타
                output_filename = self.magenta_output + str(i) + '_' + str(j+1)
                thread = threading.Thread(target=mgt.interpolate_music, args=(quantize_midi_file, base_midi, output_filename))
                threads.append(thread)
                thread.start()

            # 모든 쓰레드가 종료될 때까지 대기
            for thread in threads:
                thread.join()

    def make_midi_from_folder(self, num_repeats, recording_times_bar):
        # 경로 설정
        self.set_path()

        # 오브젝트 생성
        rec = RecordingManager(self.device_name, self.bpm, self.base_path)
        mgt = MagentaManager(self.config_name, self.checkpoint_path)
        
        print(f"[Python] number of repeats : {num_repeats}")

        threads = []
        for i in range(num_repeats):
            num_recording = (int) (recording_times_bar[i] / 2)  # 녹음할 횟수 (2마디씩 N번)
            
            for j in range(num_recording):
                quantize_midi_file = self.recording_file_path + str(i) + '_' + str(j+1) + '_quantized.mid'

                # 전처리
                base_midi = self.base_folder + match_best_from_cache(midi_path=quantize_midi_file, cache_path=self.cache_path)    # 가장 유시한 리듬
                print(f"\n[Python] matching : {base_midi}")

                # 마젠타
                output_filename = self.magenta_output + str(i) + '_' + str(j+1)
                thread = threading.Thread(target=mgt.interpolate_music, args=(quantize_midi_file, base_midi, output_filename))
                threads.append(thread)
                thread.start()

            # 모든 쓰레드가 종료될 때까지 대기
            for thread in threads:
                thread.join()