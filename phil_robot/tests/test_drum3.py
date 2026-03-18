import os
import sys
import subprocess
import sounddevice as sd
import numpy as np
import whisper
import ollama
import torch

# ==========================================
# ⚙️ 설정
# ==========================================
ROBOT_NAME = "phil-bot"     
MIC_SAMPLE_RATE = 16000

# 경로 설정
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PIPER_BINARY = os.path.join(BASE_DIR, "piper/piper")       
PIPER_MODEL = os.path.join(BASE_DIR, "phil_voice.onnx") # 현리 모델

# GPU 체크
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
print(f"========================================")
print(f"시스템: {DEVICE.upper()} 모드")
print(f"========================================")

# 1. Whisper 로드
print(f">>> [1/2] 👂 Whisper 장착 중...")
stt_model = whisper.load_model("small", device=DEVICE)
print(f">>> [2/2] 👄 Piper TTS 준비 완료.")

# ==========================================
# 🛠️ 함수 정의
# ==========================================

def record_audio(duration=5):
    print("\n🎤 말씀하세요...")
    try:
        audio = sd.rec(int(duration * MIC_SAMPLE_RATE), samplerate=MIC_SAMPLE_RATE, channels=1, dtype='float32')
        sd.wait()
        return audio.flatten()
    except Exception as e:
        print(f"❌ 마이크 에러: {e}")
        return np.zeros(1)

def speak(text):
    """
    [핵심 수정] 
    쉘(echo)을 통하지 않고 Python에서 직접 UTF-8 데이터를 Piper에게 전달합니다.
    중국어처럼 들리는 현상(인코딩 깨짐)을 완벽하게 해결합니다.
    """
    if not text: return
    print(f"🤖 Phil: {text}")
    
    clean_text = text.replace("\n", " ")
    output_wav = "temp_voice.wav"
    
    try:
        # 1. Piper 프로세스 열기 (stdin으로 텍스트 받을 준비)
        # 쉘(Shell)=False로 설정하여 쉘의 인코딩 간섭을 차단
        command = [PIPER_BINARY, '--model', PIPER_MODEL, '--output_file', output_wav]
        
        process = subprocess.Popen(
            command, 
            stdin=subprocess.PIPE, 
            stdout=subprocess.DEVNULL, 
            stderr=subprocess.DEVNULL
        )
        
        # 2. 한글 텍스트를 UTF-8 바이트로 변환해서 직접 주입
        process.communicate(input=clean_text.encode('utf-8'))
        
        # 3. 재생 (aplay 사용 - 파일 헤더 기반 재생)
        if os.path.exists(output_wav):
            os.system(f"aplay -q {output_wav}")
            os.remove(output_wav)
            
    except Exception as e:
        print(f"⚠️ 말하기 실패: {e}")

# ==========================================
# 🚀 메인 실행
# ==========================================
try:
    speak("박사님, 이제 제 발음이 정확하게 들리시나요?")
    
    while True:
        input("\n[Enter]를 누르면 듣습니다...") 
        
        # 듣기
        audio_data = record_audio(duration=4) 
        
        # STT
        result = stt_model.transcribe(audio_data, language="ko", fp16=True)
        user_text = result['text'].strip()
        
        if not user_text:
            continue
            
        print(f"👤 User: {user_text}")
        
        if "잘 가" in user_text or "종료" in user_text:
            speak("안녕히 계세요!")
            break

        # LLM
        response = ollama.chat(model=ROBOT_NAME, messages=[
            {'role': 'user', 'content': user_text},
        ])
        reply = response['message']['content']

        # TTS
        speak(reply)

except KeyboardInterrupt:
    print("\n종료합니다.")