import sounddevice as sd
import whisper
import ollama
#import json
import numpy as np
import time
# TTS 엔진

import os
import psutil

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PHIL_ROBOT_DIR = os.path.dirname(CURRENT_DIR)
PROJECT_ROOT = os.path.dirname(PHIL_ROBOT_DIR)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from phil_robot.runtime.melo_engine import TTS_Engine


def get_mem_usage():
    """현재 프로세스의 메모리 사용량을 MB 단위로 반환"""
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / (1024 * 1024)

# ==========================================
# ⚙️ 설정값 (Config)
# ==========================================
SAMPLE_RATE = 16000      # Whisper 권장 샘플링 레이트
RECORD_SECONDS = 3       # 한 번에 들을 시간 (3초)
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

    # 시작 시점 메모리
    base_mem = get_mem_usage()
    print(f"[{time.strftime('%H:%M:%S')}] 초기 메모리: {base_mem:.2f} MB")

    # 1. [초기화] TTS 로딩
    # ----------------------------------------------
    tts = TTS_Engine() 
    tts_mem = get_mem_usage()
    print(f"[{time.strftime('%H:%M:%S')}] TTS 로드 후: {tts_mem:.2f} MB (증가량: {tts_mem - base_mem:.2f} MB)")
    
    # 2. [초기화] STT 로딩
    # ----------------------------------------------
    print("[STT] Whisper 모델 로딩 중... (GPU)")
    stt_model = whisper.load_model("small", device="cuda")
    stt_mem = get_mem_usage()
    print(f"[{time.strftime('%H:%M:%S')}] STT 로드 후: {stt_mem:.2f} MB (증가량: {stt_mem - tts_mem:.2f} MB)")
    
    print(f"\n✅ 모델 로딩 완료! 총 점유: {stt_mem:.2f} MB")
    print("----------------------------------------------")

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
                #format='json'
            )
            
            # JSON 파싱
            ai_raw_json = response['message']['content'] # 원본 JSON 문자열
            #ai_data = json.loads(ai_raw_json)
            #ai_msg = ai_data.get("response", "모르겠어요")


            # 📌 [수정 4] 로봇의 대답도 기억장치에 저장해야 다음 턴에 기억함
            # ---------------------------------------------------------
            # 중요: 'ai_msg'(텍스트)가 아니라 'ai_raw_json'(JSON형식)을 저장해야 
            # 로봇이 다음번에도 JSON 포맷을 유지하려고 노력합니다.
            history.append({'role': 'assistant', 'content': ai_raw_json})
            # ---------------------------------------------------------


            llm_end_time = time.time()
            print(f"⏱️ LLM 처리 시간: {llm_end_time - llm_start_time:.2f}초")
            
            # --- C. 말하기 (TTS) ---
            #print(f"🤖 AI: {ai_msg}")
            print(f"🤖 AI (원본 JSON): {ai_raw_json}")

            #tts.speak(ai_msg)
            tts.speak(ai_raw_json)

        except KeyboardInterrupt:
            print("\n시스템 강제 종료")
            break
        except Exception as e:
            print(f"❌ 에러 발생: {e}")

if __name__ == "__main__":
    main()
