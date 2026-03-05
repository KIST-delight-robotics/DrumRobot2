# 파일명: melo_engine.py
import torch
from melo.api import TTS
import os

import time  # 상단에 추가

class TTS_Engine:
    def __init__(self):
        print("\n[TTS] 엔진 시동 거는 중... (모델 로딩)")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        try:
            # 여기서 시간 좀 걸림 (최초 1회)
            self.model = TTS(language='KR', device=self.device)
            self.speaker_ids = self.model.hps.data.spk2id
            print(f"[TTS] 로딩 완료 (장치: {self.device})")
            
            # 웜업 (중요: 처음 한 번은 버벅이므로 빈 소리 재생)
            print("[TTS] 성대 푸는 중 (Warm-up)...")
            self.speak("준비 완료", play=False) 
            
        except Exception as e:
            print(f"[TTS] ❌ 치명적 오류: {e}")
            self.model = None

    def preprocess(self, text):
        # 젯슨이 못 읽는 단어 처리
        replacements = {
            "GPU": "지피유", "CPU": "씨피유", "LLM": "엘엘엠", 
            "AI": "에이아이", "Jetson": "젯슨", "Orin": "오린",
            "MeloTTS": "멜로 티티에스", "CUDA": "쿠다", "80": "팔십",
            "100": "백", "120": "백이십", "160": "백육십", "96": "구십육",
            "88": "팔십팔", "64": "육십사", "16": "열여섯", "32": "서른둘"
        }
        for k, v in replacements.items():
            text = text.replace(k, v).replace(k.lower(), v)
        return text

    def speak(self, text, output_path="temp_speech.wav", play=True):
        if not self.model: return

        clean_text = self.preprocess(text)
        
        # 1. 순수 합성 시간 측정 시작
        inference_start = time.time()

        # 생성 (이미 로딩돼서 빠름)
        self.model.tts_to_file(clean_text, self.speaker_ids['KR'], output_path, speed=1.0)
        
        inference_end = time.time()
        print(f"  └ [TTS Inference] {inference_end - inference_start:.2f}s (텍스트 → 오디오 파일)")
        if play:

            # 2. 재생 지연 시간 측정 (aplay 호출 오버헤드 확인용)
            play_start = time.time()

            # -q: 로그 숨김, aplay: 리눅스 기본 재생기
            os.system(f"aplay -q {output_path}")

            play_end = time.time()
            print(f"  └ [TTS Playback] {play_end - play_start:.2f}s (오디오 장치 재생)")