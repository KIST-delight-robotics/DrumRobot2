# 경고 메시지 억제
import warnings
import os
warnings.filterwarnings("ignore")   # 경고 메시지 억제
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'    # TensorFlow의 로그 레벨 설정 (CUDA 관련 경고 및 TensorFlow 경고 숨기기)

import tensorflow as tf
tf.get_logger().setLevel('ERROR')   # TensorFlow 경고 메시지 억제

import sys
from task_manager import taskManager

program_name = sys.argv[0]
print(f"[Python] program name : {program_name}")

base_path = 'null'
num_args = len(sys.argv) - 1
for i in range(num_args):
    arg = sys.argv[i+1]
    
    if arg == "--sync": 
        print(f"[Python] program mode : {arg}")   
        record = False
    
    elif arg == "--record":
        print(f"[Python] program mode : {arg}")
        record = True
    
    elif arg == "--repeat":
        num_repeats = int(sys.argv[i+2])
    
    elif arg == "--param":
        num_param = len(sys.argv) - i - 2
        
        if num_param >= num_repeats * 3:
            
            wait_times = []
            recording_times = []
            creation_times = []
            
            for j in range(num_repeats):
                wait_times.append(float(sys.argv[i + 2 + 3*j]))
                recording_times.append(float(sys.argv[i + 3 + 3*j]))
                creation_times.append(float(sys.argv[i + 4 + 3*j]))
    
    elif arg == "--path":
        base_path = sys.argv[i+2]
        pass
    
    else:
        pass

tm = taskManager(base_path)

if record:
    tm.make_midi(num_repeats, wait_times, recording_times, creation_times)
else:
    tm.make_sync()