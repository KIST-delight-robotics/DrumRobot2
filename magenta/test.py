from myRecordMIDI import MyRecord
from myMagenta import MyMagenta
from myPrintMIDI import print_midi_sequence
from myPrintMIDI import print_midi_mido
from myPrintMIDI import print_midi_pretty_midi

# 경고 메시지 억제
import warnings
import tensorflow as tf
import os

warnings.filterwarnings("ignore")   # 경고 메시지 억제
tf.get_logger().setLevel('ERROR')   # TensorFlow 경고 메시지 억제
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'    # TensorFlow의 로그 레벨 설정 (CUDA 관련 경고 및 TensorFlow 경고 숨기기)

input_device_name = 'NUX DP-2000:NUX DP-2000 MIDI 1 20:0'  # 연결된 장치 이름
output_file_path = 'record/drum_recording.mid'

rec = MyRecord()
# rec.record_midi_second(input_device_name, output_file_path, 4.0)
# print_midi_sequence(output_file_path)

config_name = 'cat-drums_2bar_small'
# checkpoint_path = 'model/cat-drums_2bar_small.lokl.tar'
checkpoint_path = 'model/cat-drums_2bar_small.hikl.tar'
# input_midi = 'generated/basic.mid'
input_midi = 'MuseScore/basic_4beats_1.mid'
output_file_name = 'test_'

mgt = MyMagenta(config_name, checkpoint_path)

mgt.print_model()
# mgt.generate_music(output_file_name)
# mgt.generate_music_from_input(input_midi, output_file_name)
# mgt.interpolate_music(output_file_path, input_midi, output_file_name)
