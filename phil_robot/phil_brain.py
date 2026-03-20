# phil_robot/phil_brain.py

import os
import time

import numpy as np
import psutil
import sounddevice as sd
import whisper

from config import PLANNER_MODEL, CLASSIFIER_MODEL
from pipeline.brain_pipeline import run_brain_turn
from pipeline.executor import execute_validated_plan
from runtime.melo_engine import TTS_Engine
from runtime.phil_client import RobotClient, get_robot_state_snapshot

# ==========================================
# Config
# ==========================================
SAMPLE_RATE = 16000
RECORD_SECONDS = 3
HOST = "127.0.0.1"
PORT = 9999


def get_mem_usage():
    """현재 프로세스의 메모리 사용량을 MB 단위로 반환"""
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / (1024 * 1024)


def record_audio():
    """마이크로 소리를 듣고 1차원 float 배열로 반환"""
    print(f"\n🎤 듣는 중... ({RECORD_SECONDS}초)")
    try:
        audio = sd.rec(
            int(RECORD_SECONDS * SAMPLE_RATE),
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="float32",
        )
        sd.wait()
        return audio.flatten()
    except Exception as exc:
        print(f"❌ 마이크 녹음 실패: {exc}")
        return None


def warm_up_stt_model(stt_model):
    """초기 추론 지연을 줄이기 위해 무음 샘플로 모델을 예열"""
    print("🔥 모델 예열 중... (잠시만 기다려주세요)")
    try:
        dummy_audio = np.zeros(16000, dtype=np.float32)
        stt_model.transcribe(dummy_audio, fp16=False)
    except Exception:
        pass


def load_runtime():
    """
    대화 루프에 필요한 런타임 객체를 준비한다.
    phil_brain.py 는 객체 생성과 메인 루프 orchestration에만 집중한다.
    """
    bot = RobotClient(host=HOST, port=PORT)
    if not bot.connect():
        print("연결 실패")
        return None, None, None

    base_mem = get_mem_usage()
    print(f"[{time.strftime('%H:%M:%S')}] 초기 메모리: {base_mem:.2f} MB")

    tts = TTS_Engine()
    tts_mem = get_mem_usage()
    print(f"[{time.strftime('%H:%M:%S')}] TTS 로드 후: {tts_mem:.2f} MB (증가량: {tts_mem - base_mem:.2f} MB)")

    print("[STT] Whisper 모델 로딩 중...")
    stt_model = whisper.load_model("small", device="cuda")

    stt_mem = get_mem_usage()
    print(f"[{time.strftime('%H:%M:%S')}] STT 로드 후: {stt_mem:.2f} MB (증가량: {stt_mem - tts_mem:.2f} MB)")
    print(f"\n✅ 모델 로딩 완료! 총 점유: {stt_mem:.2f} MB")

    warm_up_stt_model(stt_model)
    print("✅[STT] 준비 완료!")
    return bot, tts, stt_model


def transcribe_user_speech(stt_model, audio_data):
    """STT 단계만 별도 함수로 분리해 메인 루프 가독성을 높인다."""
    stt_start_time = time.time()
    print("텍스트 변환 중...")
    result = stt_model.transcribe(audio_data, fp16=False, language="ko")
    user_text = result["text"].strip()
    print(f"🗣️ User: {user_text}")
    print(f"⏱️ STT 처리 시간: {time.time() - stt_start_time:.2f}초")
    return user_text


def debug_brain_result(brain_result):
    """LLM 입력, 내부 thinking, validator 경고를 한 곳에서 출력"""
    print(f"🧭 [Classifier 입력]\n{brain_result.classifier_input_json}")
    print(f"🧭 [Classifier 결과] {brain_result.classifier_result}")
    print(f"🧭 [Planner Domain] {brain_result.planner_domain}")
    print(f"🧐 [Planner 입력]\n{brain_result.planner_input_json}")
    print(f"🗺️ [Planner 결과] {brain_result.planner_result}")
    print(
        f"⏱️ LLM 처리 시간: 총 {brain_result.llm_duration_sec:.2f}초 "
        f"(classifier {brain_result.classifier_duration_sec:.2f}초 + planner {brain_result.planner_duration_sec:.2f}초)"
    )

    if brain_result.validated_plan.reason:
        print(f"\n[Phil's Brain 🧠]\n{brain_result.validated_plan.reason}\n")

    if brain_result.validated_plan.expanded_op_cmds:
        print(f"[Planner Expanded] {brain_result.validated_plan.expanded_op_cmds}")
    if brain_result.validated_plan.resolved_op_cmds:
        print(f"[Planner Resolved] {brain_result.validated_plan.resolved_op_cmds}")
    if brain_result.validated_plan.valid_op_cmds:
        print(f"[Validator Accepted] {brain_result.validated_plan.valid_op_cmds}")
    if brain_result.validated_plan.rejected_op_cmds:
        print(f"[Validator Rejected] {brain_result.validated_plan.rejected_op_cmds}")

    for warning in brain_result.validated_plan.warnings:
        print(f"[Validator] {warning}")


def main():
    bot, tts, stt_model = load_runtime()
    if not all([bot, tts, stt_model]):
        return

    tts.speak("대화 준비가 되었습니다. 엔터 키를 누르고 말씀해 주세요.")

    try:
        while True:
            key = input("\n⌨️ [Enter] 듣기 / 'q' 종료 >> ")
            if key.lower() == "q":
                print("에이전트 종료")
                break

            audio_data = record_audio()
            # 녹음에 실패했으면 이번 턴의 나머지 처리는 건너뛰고 다음 입력을 받는다.
            if audio_data is None:
                continue

            user_text = transcribe_user_speech(stt_model, audio_data)
            # 음성 인식 결과가 비어 있으면 이번 턴을 종료하고 다시 입력 대기로 돌아간다.
            if not user_text:
                continue

            print("🧠 생각 중...")

            # 상태 수신 스레드가 갱신 중인 값을 그대로 넘기지 않도록 스냅샷을 만든다.
            robot_state = get_robot_state_snapshot()
            brain_result = run_brain_turn(
                user_text=user_text,
                raw_robot_state=robot_state,
                classifier_model_name=CLASSIFIER_MODEL,     # classifier은 현재 고정
                planner_model_name=PLANNER_MODEL,
            )

            debug_brain_result(brain_result)
            execute_validated_plan(bot, brain_result.validated_plan)

            print(f"🤖 Phil: {brain_result.validated_plan.speech}")
            tts.speak(brain_result.validated_plan.speech)

    except KeyboardInterrupt:
        print("\n종료합니다.")
    finally:
        bot.close()


if __name__ == "__main__":
    main()
