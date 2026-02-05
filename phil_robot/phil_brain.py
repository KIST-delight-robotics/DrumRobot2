import sounddevice as sd
import whisper
import ollama
#import json
import numpy as np
# TCP 소켓 통신(command 전송)
from phil_client import RobotClient
# TTS 엔진
from melo_engine import TTS_Engine

# ==========================================
# ⚙️ 설정값 (Config)
# ==========================================
SAMPLE_RATE = 16000      # Whisper 권장 샘플링 레이트
RECORD_SECONDS = 3       # 한 번에 들을 시간 (3초)
LLM_MODEL = "phil-bot"     # ⚠️ 사용 중인 모델명으로 변경 필수
HOST = '127.0.0.1'
PORT = 9999


# ==========================================
# 🔧 녹음 함수
# ==========================================
def record_audio():
    """마이크로 소리를 듣고 Array로 반환"""
    print(f"\n🎤 듣는 중... ({RECORD_SECONDS}초)")
    try:
        # float32로 녹음 후 1차원으로 펴서(flatten) 반환
        audio = sd.rec(int(RECORD_SECONDS * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype='float32')
        sd.wait()

        return audio.flatten()
        
    except Exception as e:
        print(f"❌ 마이크 녹음 실패: {e}")
        return None

def main():
    # 1. 통신 모듈(전화기) 준비
    bot = RobotClient(host=HOST, port=PORT)
    
    # 2. 연결 시도 (연결 안 되면 뇌를 켤 필요도 없음)
    if not bot.connect():
        print(f"연결 실패: {e}")
        return 
    
    # 3. 뇌(AI) 로딩
    tts = TTS_Engine() # TTS 엔진 시동
    print("[STT] Whisper 모델 로딩 중...")
    stt_model = whisper.load_model("small", device="cuda")
    
    # 🔥 [중요] 모델 워밍업 (Warm-up)
    # 가짜(0으로 채워진) 오디오를 한번 돌려서 GPU 초기화 문제를 방지함
    print("🔥 모델 예열 중... (잠시만 기다려주세요)")
    try:
        dummy_audio = np.zeros(16000, dtype=np.float32) # 2초짜리 무음
        stt_model.transcribe(dummy_audio, fp16=False)
    except:
        pass # 워밍업 에러는 무시

    print("✅[STT] 준비 완료!")
    # 첫 인사
    tts.speak("대화 준비가 되었습니다. 엔터 키를 누르고 말씀해 주세요.")

    try:
        while True:
            key = input("\n⌨️ [Enter] 듣기 / 'q' 종료 >> ")
            if key.lower() == 'q':
                print("에이전트 종료")
                break

            # --- A. 듣기 ---
            audio_data = record_audio()
            if audio_data is None: continue
            
            print("텍스트 변환 중...")
            result = stt_model.transcribe(audio_data, fp16=False, language="ko")
            user_text = result['text'].strip()
            
            print(f"🗣️ User: {user_text}")

            if not user_text: continue

            # --- B. 생각하기 ---
            print("🧠 생각 중...")

            response = ollama.chat(
                model=LLM_MODEL,
                messages=[{'role': 'user', 'content': user_text}],
                #format='json'
            )
            
            # 리스트 파싱
            ai_data = response['message']['content']
            # ai_msg = ai_data.get("response", "모르겠어요")
            # ai_cmd = ai_data.get("command", None)

            ai_cmd = None

            if ">>" in ai_data:
                # ">>" 기준으로 메시지와 명령 분리
                parts = ai_data.split(">>", 1)

                # 앞부분: "[p]" -> 대괄호랑 공백 제거 -> "p"
                cmd_part = parts[0].strip()
                ai_cmd = cmd_part.replace("[", "").replace("]", "")

                # 뒷부분: AI 메시지
                ai_msg = parts[1].strip()


            # --- C. 명령 전송 (분리된 파일의 함수 사용) ---
            if ai_cmd:
                print(f"📡 명령 전송: {ai_cmd}")
                bot.send_command(ai_cmd)

            print(f"🤖 Phil: {ai_msg}")
            tts.speak(ai_msg)
            
            
    except KeyboardInterrupt:
        print("\n종료합니다.")
    finally:
        bot.close()

if __name__ == "__main__":
    main()