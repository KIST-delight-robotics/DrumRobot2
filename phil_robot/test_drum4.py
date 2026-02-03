import sounddevice as sd
import whisper
import ollama
# TTS 엔진
from melo_engine import TTS_Engine

# ==========================================
# ⚙️ 설정값 (Config)
# ==========================================
SAMPLE_RATE = 16000      # Whisper 권장 샘플링 레이트
RECORD_SECONDS = 5       # 한 번에 들을 시간 (5초)
LLM_MODEL = "phil-bot"     # ⚠️ 사용 중인 모델명으로 변경 필수

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

# ==========================================
# 🚀 메인 함수
# ==========================================
def main():
    print("========== [AI CONVERSATION MODE] ==========")

    # 1. [초기화] TTS & STT 로딩
    # ----------------------------------------------
    tts = TTS_Engine() # TTS 엔진 시동
    
    print("[STT] Whisper 모델 로딩 중... (GPU)")
    # small 모델 사용
    stt_model = whisper.load_model("small", device="cuda")
    print("[STT] 준비 완료!")
    
    # 첫 인사
    tts.speak("대화 준비가 되었습니다. 엔터 키를 누르고 말씀해 주세요.")

    # 2. [루프] 대화 반복
    # ----------------------------------------------
    while True:
        try:
            # --- RESTART ---
            key = input("\n⌨️ [Enter]를 누르면 듣습니다 (종료: q) >> ")
            if key.lower() == 'q':
                print("시스템을 종료합니다.")
                break
            
            # --- A. 듣기 (STT) ---
            audio_data = record_audio()
            if audio_data is None: continue
            
            # Whisper로 변환
            print("📜 텍스트 변환 중...")
            result = stt_model.transcribe(audio_data, fp16=False, language="ko", initial_prompt="자기소개, 필봇")
            user_text = result['text'].strip()
            
            print(f"🗣️ 사용자: {user_text}")

            if not user_text:
                print("⚠️ 소리가 감지되지 않았습니다.")
                continue

            # --- B. 생각하기 (LLM) ---
            print("🧠 생각 중...")
            
            # Ollama에게 질문
            response = ollama.chat(model=LLM_MODEL, messages=[
                {'role': 'user', 'content': user_text},
            ])
            ai_response = response['message']['content']

            # --- C. 말하기 (TTS) ---
            print(f"🤖 AI: {ai_response}")
            tts.speak(ai_response)

        except KeyboardInterrupt:
            print("\n시스템 강제 종료")
            break
        except Exception as e:
            print(f"❌ 에러 발생: {e}")

if __name__ == "__main__":
    main()