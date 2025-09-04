# 경고 메시지 억제
import warnings
import os
warnings.filterwarnings("ignore")   # 경고 메시지 억제
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'    # TensorFlow의 로그 레벨 설정 (CUDA 관련 경고 및 TensorFlow 경고 숨기기)

import tensorflow as tf
tf.get_logger().setLevel('ERROR')   # TensorFlow 경고 메시지 억제

import sys
from myClass import taskManager

program_name = sys.argv[0]
program_mode = sys.argv[1]
print(f"[Python] program name : {program_name}")
print(f"[Python] program mode : {program_mode}")

tm = taskManager()

if program_mode == "--sync":

    tm.make_sync()

elif program_mode == "--record":
    
    if sys.argv[2] == "--repeat":
        
        num_repeats = int(sys.argv[3])
        num_args = len(sys.argv) - 4
        
        if num_args >= num_repeats * 3:
            
            wait_times = []
            recording_times = []
            creation_times = []
            
            for i in range(num_repeats):
                wait_times.append(float(sys.argv[3*(i+1) + 1]))
                recording_times.append(float(sys.argv[3*(i+1) + 2]))
                creation_times.append(float(sys.argv[3*(i+1) + 3]))

            tm.make_midi(num_repeats, wait_times, recording_times, creation_times)
        
        else:
            print("[Python] [ERROR] args3")
    
    else:
        print("[Python] [ERROR] args2")

else:
    print("[Python] [ERROR] args1")






