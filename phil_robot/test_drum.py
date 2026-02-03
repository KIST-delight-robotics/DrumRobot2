# [main.py]

# 1. sklearn 자리 선점 (TLS 에러 방지)
import sklearn
# 2. 시스템 라이브러리 강제 로드 (TLS 에러 방지)
import os
import ctypes
try:
    ctypes.CDLL("/home/shy/miniforge3/envs/drum/lib/libgomp.so.1")
    print(">>> [System] libgomp Pre-loaded successfully!")
except:
    pass

import sys
import torch
import soundfile as sf  # soundfile 직접 임포트

# =================================================================
# 🏥 [긴급 수술] Torchaudio 로보토미 (뇌수술)
# 멍청한 torchaudio가 자꾸 torchcodec을 찾으니까,
# 아예 soundfile을 직접 써서 파일을 읽도록 함수를 바꿔치기합니다.
# =================================================================
import torchaudio

def emergency_audio_load(filepath, **kwargs):
    # torchaudio.load 대신 soundfile.read를 직접 사용
    data, samplerate = sf.read(filepath)
    
    # soundfile은 numpy array를 반환하므로 torch tensor로 변환
    tensor = torch.FloatTensor(data)
    
    # 차원 맞추기 (Mono일 경우 [Time] -> [1, Time])
    if tensor.ndim == 1:
        tensor = tensor.unsqueeze(0)
    else:
        # Stereo일 경우 [Time, Channel] -> [Channel, Time] (Torchaudio 규격)
        tensor = tensor.transpose(0, 1)
        
    return tensor, samplerate

# 원래 함수를 덮어씌웁니다. (이제 torchaudio.load를 호출하면 위 함수가 실행됨)
torchaudio.load = emergency_audio_load
print(">>> [System] Torchaudio has been patched to use 'soundfile' directly.")
# =================================================================

# 🚨 [기존 패치] PyTorch 2.6 보안 에러 우회
_original_load = torch.load
def safe_load(*args, **kwargs):
    if 'weights_only' not in kwargs:
        kwargs['weights_only'] = False
    return _original_load(*args, **kwargs)
torch.load = safe_load
print(">>> [System] PyTorch 'weights_only' security check bypassed.")


import sounddevice as sd
import numpy as np
import ollama
import whisper
from TTS.api import TTS
import tempfile
import time

# --- 설정 ---
ROBOT_NAME = "phil-bot"     
VOICE_REF = "phil_voice1.wav" 
SAMPLE_RATE = 16000         

# CPU 모드로 실행
DEVICE = "cpu" 

print(f">>> [1/3] 👂 귀(Whisper) 장착 중... ({DEVICE})")
stt_model = whisper.load_model("small", device=DEVICE)

print(f">>> [2/3] 👄 입(XTTS) 장착 중... ({DEVICE})")
tts = TTS("tts_models/multilingual/multi-dataset/xtts_v2").to(DEVICE)

print(f">>> [3/3] 🤖 {ROBOT_NAME} 깨우는 중...")

def record_audio(duration=5):
    print("\n🎤 듣고 있어요... (말씀하세요!)")
    audio = sd.rec(int(duration * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype='float32')
    sd.wait()
    print("⏹️ 녹음 끝!")
    return audio.flatten()

def speak(text):
    if not text: return
    print(f"🤖 Phil: {text}")
    
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        output_path = f.name
        
    os.environ["COQUI_TOS_AGREED"] = "1"
    
    tts.tts_to_file(
        text=text,
        file_path=output_path,
        speaker_wav=VOICE_REF, 
        language="ko"
    )
    
    os.system(f"aplay {output_path} > /dev/null 2>&1")
    os.remove(output_path)

# --- 메인 실행 ---
try:
    speak("안녕하세요! 들리시나요?")
    
    while True:
        input("\n[Enter]를 누르고 말씀하세요...") 
        
        # 1. 듣기
        audio_data = record_audio(duration=4) 
        
        # 2. 받아적기
        result = stt_model.transcribe(audio_data, language="ko", fp16=False)
        user_text = result['text'].strip()
        
        if not user_text:
            print("?? (안 들려요)")
            continue
            
        print(f"👤 User: {user_text}")
        
        if "잘 가" in user_text or "종료" in user_text:
            speak("안녕히 계세요!")
            break

        # 3. 생각하기
        response = ollama.chat(model=ROBOT_NAME, messages=[
            {'role': 'user', 'content': user_text},
        ])
        reply = response['message']['content']

        # 4. 말하기
        speak(reply)

except KeyboardInterrupt:
    print("\n시스템 종료.")
except Exception as e:
    print(f"\n❌ 에러 발생: {e}")