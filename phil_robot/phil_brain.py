# phil_robot/phil_brain.py

import sounddevice as sd
import whisper
import ollama
import json
import numpy as np
# TCP 소켓 통신(command 전송)
from phil_client import RobotClient, ROBOT_STATE
# TTS 엔진
from melo_engine import TTS_Engine
from response_parser import parse_llm_response 

# ==========================================
# ⚙️ 설정값 (Config)
# ==========================================
SAMPLE_RATE = 16000      # Whisper 권장 샘플링 레이트
RECORD_SECONDS = 3       # 한 번에 들을 시간 (3초)
LLM_MODEL = "phil-bot"     # ⚠️ 사용 중인 모델명으로 변경 필수
HOST = '127.0.0.1'
PORT = 9999

JOINT_LIMITS = {
    "waist": (-89.0, 89.0),
    "R_arm1": (0.0, 149.0),
    "L_arm1": (29.0, 179.0),
    "R_arm2": (-59.0, 89.0),
    "R_arm3": (0.0, 139.0),
    "L_arm2": (-59.0, 89.0),
    "L_arm3": (0.0, 139.0),
    "R_wrist": (-107.0, 134.0), # 135도에서 과전류
    "L_wrist": (-107.0, 134.0), # 135도에서 과전류
    "R_foot": (-89.0, 199.0),
    "L_foot": (-89.0, 199.0)
}

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

            raw_state_json = json.dumps(ROBOT_STATE, ensure_ascii=False) # 현재 로봇 상태를 JSON 문자열로 변환

            # 사용자 프롬프트에 조립            
            context_injected_user_text = (
                f"[System Info: {raw_state_json}]\n"
                f"사용자: {user_text}\n\n"
            )

            # 디버깅용 프롬프트 출력 (터미널에서 LLM이 뭘 보고 대답하는지 확인)
            print(f"🧐 [프롬프트 확인]\n{context_injected_user_text}")
            
            response = ollama.chat(
                model=LLM_MODEL,
                messages=[{'role': 'user', 'content': context_injected_user_text}],
            )
            
            # 리스트 파싱
            ai_data = response['message']['content']

            commands, message = parse_llm_response(ai_data)

            # --- C. 명령 전송 (분리된 파일의 함수 사용) ---
            for cmd in commands:
                print(f"📡 명령 전송: {cmd}")
                bot.send_command(cmd)

            print(f"🤖 Phil: {message}")
            tts.speak(message)
            
            
    except KeyboardInterrupt:
        print("\n종료합니다.")
    finally:
        bot.close()

if __name__ == "__main__":
    main()