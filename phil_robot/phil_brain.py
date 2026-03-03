# phil_robot/phil_brain.py

import sounddevice as sd
import whisper
import ollama
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

            current_state = ROBOT_STATE["state"]

            # 1. ROBOT_STATE에서 상태값들 안전하게 꺼내기 (없으면 기본값 반환)
            current_state = ROBOT_STATE.get("state", 0)
            is_fixed = ROBOT_STATE.get("is_fixed", True)
            error_detail = ROBOT_STATE.get("error_detail", "None")
            current_song = ROBOT_STATE.get("current_song", "None")
            progress = ROBOT_STATE.get("progress", "None")
            last_action = ROBOT_STATE.get("last_action", "None")

            # 2. 상태에 따른 아주 디테일한 맥락 주입
            if current_state == 2:
                state_context = f"현재 당신(로봇)은 '{current_song}' 곡을 신나게 연주(Play) 중입니다. (진행률: {progress}) 사용자가 손을 들라거나 다른 행동을 요구하면 지금은 연주 중이라 어렵다고 정중히 거절하세요."
            
            elif current_state == 4:
                # ★ C++ 미들웨어가 보내준 진짜 에러 원인을 프롬프트에 주입!
                state_context = f"현재 당신의 신체에 에러(Error)가 발생해 멈춰있습니다. (시스템 에러 원인: {error_detail}) 사용자에게 이 에러 원인을 설명하면서 핑계를 대고 사과하세요."
            
            elif current_state == 0 and not is_fixed:
                state_context = f"현재 당신은 지정된 자세로 바쁘게 이동(Moving) 중입니다. (최근 수행 명령: {last_action})"
            
            else:
                state_context = f"현재 당신은 대기(Ideal) 중이며, 사용자의 어떤 명령이든 받을 준비가 되어 있습니다. (최근 수행 명령: {last_action})"

            # 3. 사용자 프롬프트에 조립
            context_injected_user_text = f"[System Info: {state_context}]\n사용자: {user_text}\n\n[명령어는 항상 >> 명령 형식으로 응답해주세요. 예시: >> move_forward]"
            response = ollama.chat(
                model=LLM_MODEL,
                messages=[{'role': 'user', 'content': context_injected_user_text}],
                #format='json'
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