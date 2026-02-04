import sounddevice as sd
import whisper
import ollama
import re
import numpy as np
import time
# TTS 엔진
from melo_engine import TTS_Engine

# ==========================================
# ⚙️ 설정값 (Config)
# ==========================================
SAMPLE_RATE = 16000      # Whisper 권장 샘플링 레이트
RECORD_SECONDS = 4       # 한 번에 들을 시간 (4초)
LLM_MODEL = "phil-speech"     # ⚠️ 사용 중인 모델명으로 변경 필수

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
    
    # 🔥 [중요] 모델 워밍업 (Warm-up)
    # 가짜(0으로 채워진) 오디오를 한번 돌려서 GPU 초기화 문제를 방지함
    print("🔥 모델 예열 중... (잠시만 기다려주세요)")
    try:
        dummy_audio = np.zeros(16000, dtype=np.float32) # 2초짜리 무음
        stt_model.transcribe(dummy_audio, fp16=True)
    except:
        pass # 워밍업 에러는 무시



    # 📌 [수정 1] 대화 기억장치(History) 초기화
    # ---------------------------------------------------------
    # 시스템 프롬프트는 Modelfile에 있으므로 여기선 빈 리스트로 시작해도 됩니다.
    history = [] 
    # ---------------------------------------------------------


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
            
            stt_start_time = time.time()

            # Whisper로 변환
            print("📜 텍스트 변환 중...")
            result = stt_model.transcribe(audio_data, fp16=True, language="ko", initial_prompt="자기소개, 필봇")
            user_text = result['text'].strip()
            
            print(f"🗣️ 사용자: {user_text}")
            
            stt_end_time = time.time()
            print(f"⏱️ STT 처리 시간: {stt_end_time - stt_start_time:.2f}초")

            if not user_text:
                print("⚠️ 소리가 감지되지 않았습니다.")
                continue



            # 📌 [수정 2] 사용자 말을 기억장치에 저장 + 오래된 기억 삭제
            # ---------------------------------------------------------
            history.append({'role': 'user', 'content': user_text})
            
            # [Jetson 보호] 기억이 너무 길어지면(10턴 이상) 앞부분 삭제 (Sliding Window)
            if len(history) > 10:
                history = history[-10:] 
            # ---------------------------------------------------------


            # --- B. 생각하기 (LLM) ---
            print("🧠 생각 중...")
            
            llm_start_time = time.time()


            # 📌 [수정 3] messages에 방금 한 말이 아니라 'history' 전체를 넣음
            # Ollama에게 질문
            response = ollama.chat(
                model=LLM_MODEL,
                messages=history,
                stream=True
           )
            
            ai_raw = ""   # 전체 대화 기록용 (history 저장용)
            buffer = ""   # TTS 말하기용 임시 바구니 (한 문장씩 담음)
            
            # 문장이 끝나는 기호 정규식 (. ! ? ;)
            sentence_endings = re.compile(r'[.!?;]')

            for chunk in response:
                part = chunk['message']['content']
                
                # 1. 화면 출력 & 전체 기록
                print(part, end='', flush=True)
                ai_raw += part
                
                # 2. TTS 바구니에 일단 담기
                buffer += part

                # 3. [핵심] 방금 들어온 글자(part)가 마침표나 물음표인가?
                if sentence_endings.search(part):
                    # 바구니에 실질적인 내용이 있을 때만 말하기 (공백이나 점만 있는 경우 방지)
                    if len(buffer.strip()) > 1:
                        # 🗣️ 문장 단위로 끊어서 말하기!
                        tts.speak(buffer) 
                        buffer = "" # 말했으니 바구니 비우기

            # 4. 반복문이 끝났는데 바구니에 남은 말이 있다면? (마침표 안 찍고 끝난 경우)
            if buffer.strip():
                tts.speak(buffer)

            print() # 줄바꿈

            # 📌 [수정 4] 로봇의 대답도 기억장치에 저장해야 다음 턴에 기억함
            # ---------------------------------------------------------
            history.append({'role': 'assistant', 'content': ai_raw})
            # ---------------------------------------------------------


            llm_end_time = time.time()
            print(f"⏱️ LLM 처리 시간: {llm_end_time - llm_start_time:.2f}초")
            
            # --- C. 말하기 (TTS) ---
            print(f"🤖 AI: {ai_raw}")

            #tts.speak(ai_raw)

        except KeyboardInterrupt:
            print("\n시스템 강제 종료")
            break
        except Exception as e:
            print(f"❌ 에러 발생: {e}")

if __name__ == "__main__":
    main()