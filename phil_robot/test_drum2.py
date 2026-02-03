# [main.py 맨 윗부분]
import os

# 1. CUDA 라이브러리 경로 강제 지정 (젯슨 표준 경로)
os.environ["PATH"] += os.pathsep + '/usr/local/cuda/bin'
os.environ["LD_LIBRARY_PATH"] = '/usr/local/cuda/lib64' + (os.pathsep + os.environ.get("LD_LIBRARY_PATH", "") if os.environ.get("LD_LIBRARY_PATH") else "")

import torch

# GPU가 정말 인식되는지 확인하는 코드 (터미널에 출력됨)
cuda_available = torch.cuda.is_available()
print(f">>> [System] CUDA Available: {cuda_available}")
if cuda_available:
    print(f">>> [System] GPU Device: {torch.cuda.get_device_name(0)}")
else:
    print(">>> [Critical] GPU가 아직 인식되지 않습니다. CPU로 전환합니다.")
    # 임시로 실행이라도 되게 하려면 아래 주석 해제 (하지만 느려짐)
    # DEVICE = "cpu"

# --- 아까 그 'weights_only' 패치도 이 환경에 다시 필요합니다 ---
_original_load = torch.load
def safe_load(*args, **kwargs):
    if 'weights_only' in kwargs:
        kwargs['weights_only'] = False
    return _original_load(*args, **kwargs)
torch.load = safe_load
print(">>> [System] PyTorch 'weights_only' security check bypassed.")
import sys
import ctypes
import time
import tempfile
import sounddevice as sd
import numpy as np
import ollama
import whisper  # openai-whisper
from TTS.api import TTS
import torchaudio

# ==========================================
# 🔧 [시스템 설정] 젯슨 GPU 환경 최적화
# ==========================================

# 1. libgomp 경로 수정 (drum -> drum2)
# TLS 에러 방지를 위해 미리 로드합니다.
try:
    # 박사님 현재 환경 이름이 'drum2'인 것을 반영했습니다.
    ctypes.CDLL("/home/shy/miniforge3/envs/drum2/lib/libgomp.so.1")
    print(">>> [System] libgomp Pre-loaded successfully!")
except OSError:
    try:
        # 혹시 경로가 다를까봐 시스템 기본 경로도 시도
        ctypes.CDLL("/usr/lib/aarch64-linux-gnu/libgomp.so.1")
    except:
        pass

# 2. 오디오 백엔드 'soundfile' 강제 고정
# torchaudio가 제멋대로 굴지 않게 딱 잡아둡니다.
try:
    torchaudio.set_audio_backend("soundfile")
except:
    pass

# ==========================================
# 🤖 [로봇 설정]
# ==========================================
ROBOT_NAME = "phil-bot"      # Ollama 모델 이름
VOICE_REF = "phil_voice1.wav" # 복제할 목소리 파일 (같은 폴더에 있어야 함)
SAMPLE_RATE = 16000          
DEVICE = "cuda"              # 🔥 상남자의 GPU 모드

print(f">>> [1/3] 👂 귀(Whisper) 장착 중... ({DEVICE})")
# OpenAI Whisper 로드 (GPU 가속)
stt_model = whisper.load_model("small", device=DEVICE)

print(f">>> [2/3] 👄 입(XTTS) 장착 중... ({DEVICE})")
# XTTS 로드 (GPU 가속)
tts = TTS("tts_models/multilingual/multi-dataset/xtts_v2").to(DEVICE)

print(f">>> [3/3] 🤖 {ROBOT_NAME} 깨어나는 중...")

def record_audio(duration=5):
    print("\n🎤 듣고 있어요... (말씀하세요!)")
    # 젯슨 마이크 녹음
    audio = sd.rec(int(duration * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype='float32')
    sd.wait()
    print("⏹️ 녹음 끝!")
    return audio.flatten()

def speak(text):
    if not text: return
    print(f"🤖 Phil: {text}")
    
    # 임시 파일 생성
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        output_path = f.name
        
    # 약관 동의 자동 패스
    os.environ["COQUI_TOS_AGREED"] = "1"
    
    # GPU로 음성 합성
    tts.tts_to_file(
        text=text,
        file_path=output_path,
        speaker_wav=VOICE_REF, 
        language="ko"
    )
    
    # 재생 (리눅스 aplay 사용)
    os.system(f"aplay {output_path} > /dev/null 2>&1")
    os.remove(output_path)

# --- 메인 실행 루프 ---
try:
    speak("박사님! 저 필이에요. GPU로 돌아오니까 힘이 넘쳐요!")
    
    while True:
        # 엔터 키 입력 대기 (마이크 잡음 방지)
        input("\n[Enter]를 누르고 말씀하세요...") 
        
        # 1. 듣기
        audio_data = record_audio(duration=4) 
        
        # 2. 받아적기 (OpenAI Whisper 방식)
        # fp16=True로 설정하여 GPU 텐서 코어를 사용 (속도 향상)
        result = stt_model.transcribe(audio_data, language="ko", fp16=True)
        user_text = result['text'].strip()
        
        if not user_text:
            print("?? (안 들려요)")
            continue
            
        print(f"👤 User: {user_text}")
        
        # 종료 명령어
        if "잘 가" in user_text or "종료" in user_text:
            speak("안녕히 계세요! 쿵치따!")
            break

        # 3. 생각하기 (Ollama)
        # Ollama는 알아서 GPU를 쓰니 걱정 마세요
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