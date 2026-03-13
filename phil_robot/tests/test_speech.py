import sounddevice as sd
import whisper
import ollama
import numpy as np
import time
import threading
import queue
from collections import deque
import os
import sys

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PHIL_ROBOT_DIR = os.path.dirname(CURRENT_DIR)
PROJECT_ROOT = os.path.dirname(PHIL_ROBOT_DIR)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from phil_robot.runtime.melo_engine import TTS_Engine

# ==========================================
# ⚙️ 설정값
# ==========================================
SAMPLE_RATE = 16000
LLM_MODEL = "phil-speech"
WAKE_WORDS = ["필", "필봇", "피리", "안녕", "로봇", "일봇", "빌봇", "삘봇", "Phil", "필보사"]

START_THRESHOLD = 15
STOP_THRESHOLD = 8
PRE_RECORD_SECONDS = 0.5
CONVERSATION_TIMEOUT = 20
TRASH_TEXTS = ["MBC", "뉴스", "구독", "좋아요", "시청", "감사", "여러분"]

# ==========================================
# 🚦 전역 변수 (스레드 간 통신용)
# ==========================================
audio_queue = queue.Queue()  # 귀가 들은 걸 뇌로 보내는 택배 상자
is_speaking = False          # "지금 말하는 중이니?" (True면 듣기 중단)
is_running = True            # 프로그램 종료 신호

# ==========================================
# 👂 [스레드 1] 귀 (Listening Thread)
# ==========================================
def listener_thread_func():
    global is_speaking, is_running
    
    print("👂 [Thread] 귀가 열렸습니다 (백그라운드 감지 시작)")
    
    pre_buffer_len = int(PRE_RECORD_SECONDS / 0.1)
    pre_buffer = deque(maxlen=pre_buffer_len)
    
    # 여기서 스트림을 한 번 열어서 프로그램 끝날 때까지 절대 닫지 않음
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=1) as stream:
        while is_running:
            try:
                # 1. 0.1초씩 읽으면서 '호출어' 대기
                indata, _ = stream.read(1600)
                pre_buffer.append(indata)
                volume = np.linalg.norm(indata) * 10
                
                # 🚨 핵심: 로봇이 말하고 있을 때는 듣지 않음 (Echo 방지)
                if is_speaking:
                    sd.sleep(100) # CPU 낭비 방지
                    continue

                # 2. 소리가 감지되면?
                if volume > START_THRESHOLD:
                    print(f"\n⚡ 소리 감지 (Vol: {volume:.1f}) -> 녹음 시작")
                    
                    # --- 스마트 녹음 로직 (함수 안 쓰고 풀어서 작성) ---
                    recorded_frames = list(pre_buffer)
                    silent_chunks = 0
                    max_silent = 12 # 1.2초
                    
                    while is_running and not is_speaking: # 말하기 시작하면 즉시 중단
                        data, _ = stream.read(1600)
                        recorded_frames.append(data)
                        vol = np.linalg.norm(data) * 10
                        
                        if vol > STOP_THRESHOLD: silent_chunks = 0
                        else: silent_chunks += 1
                        
                        if silent_chunks > max_silent: break # 말 끝남
                        if len(recorded_frames) > 100: break # 10초 초과
                    
                    # 3. 다 들었으면 큐(Queue)에 던짐
                    if len(recorded_frames) * 0.1 > 1.0: # 1초 이상만
                        final_audio = np.concatenate(recorded_frames).flatten().astype(np.float32)
                        audio_queue.put(final_audio)
                        print("📦 오디오 배송 완료 (Queue)")
                    else:
                        print("🧹 너무 짧아서 버림")

            except Exception as e:
                print(f"❌ 귀 스레드 에러: {e}")
                time.sleep(1)

# ==========================================
# 🧠 [메인 스레드] 뇌 & 입 (Main Logic)
# ==========================================
def main():
    global is_speaking, is_running
    print("========== [AI THREADED MODE] ==========")

    tts = TTS_Engine()
    print("[STT] Whisper 로딩 중...")
    stt_model = whisper.load_model("small", device="cuda")
    
    # 워밍업
    try: stt_model.transcribe(np.zeros(16000, dtype=np.float32), fp16=True)
    except: pass

    # 🧵 스레드 시작 (귀를 독립시킴)
    listener = threading.Thread(target=listener_thread_func, daemon=True)
    listener.start()

    history = []
    is_active_mode = False
    last_active_time = 0

    tts.speak("준비 완료.")

    while is_running:
        try:
            # 1. 큐에서 오디오가 올 때까지 대기 (Blocking)
            # 귀 스레드가 뭔가를 듣고 큐에 넣으면 여기서 깨어남
            try:
                audio_data = audio_queue.get(timeout=1) # 1초마다 체크
            except queue.Empty:
                continue # 오디오 없으면 계속 대기

            # 2. STT 변환 (메인 스레드가 담당)
            print("📜 변환 중...", end=" ")
            result = stt_model.transcribe(audio_data, fp16=True, language="ko", initial_prompt="안녕하세요 필봇입니다.")
            user_text = result['text'].strip()
            print(f"-> [{user_text}]")

            # 유효성 검사
            if len(user_text) < 2: continue
            trash_found = False
            for trash in TRASH_TEXTS:
                if trash in user_text: trash_found = True
            if trash_found: continue

            # 3. 대화 로직 (기존과 동일)
            current_time = time.time()
            
            if is_active_mode:
                if current_time - last_active_time > CONVERSATION_TIMEOUT:
                    print("💤 대기 모드 전환")
                    is_active_mode = False
            
            if not is_active_mode:
                is_wake_up = False
                for word in WAKE_WORDS:
                    if word in user_text:
                        is_wake_up = True
                        break
                if is_wake_up:
                    print("✅ 호출어 감지!")
                    is_active_mode = True
                    
                    # 🚨 말하기 시작 -> 귀 막기
                    is_speaking = True 
                    tts.speak("네?")
                    is_speaking = False # 다 말했으면 귀 열기
                    
                    last_active_time = time.time()
                    if len(user_text) < 5: continue
                else:
                    print("🔇 무시함")
                    continue

            start_time = time.time()
            # 4. LLM 생각
            last_active_time = time.time()
            history.append({'role': 'user', 'content': user_text})
            if len(history) > 10: history = history[-10:]

            print("🧠 생각 중...")
            response = ollama.chat(model=LLM_MODEL, messages=history)
            ai_msg = response['message']['content']
            print(f"🤖 AI: {ai_msg}")

            time_taken = time.time() - start_time
            print(f"⏱️ 처리 시간: {time_taken:.2f}초")
            # 5. 말하기 (입 열기)
            # 🚨 핵심: 말하는 동안 is_speaking = True로 만들어서 귀 스레드를 멈춤
            is_speaking = True
            tts.speak(ai_msg)
            is_speaking = False # 말 끝나면 다시 듣기 시작
            
            history.append({'role': 'assistant', 'content': ai_msg})

        except KeyboardInterrupt:
            print("\n시스템 종료 중...")
            is_running = False
            break
        except Exception as e:
            print(f"❌ 메인 에러: {e}")

if __name__ == "__main__":
    main()
