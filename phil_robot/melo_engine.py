# 파일명: melo_engine.py
import torch
from melo.api import TTS
import os
import time

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
        # 젯슨이 못 읽는 영어 약어 처리
        replacements = {
            "GPU": "지피유", "CPU": "씨피유", "LLM": "엘엘엠", 
            "AI": "에이아이", "Jetson": "젯슨", "Orin": "오린",
            "MeloTTS": "멜로 티티에스", "CUDA": "쿠다"
        }
        for k, v in replacements.items():
            text = text.replace(k, v).replace(k.lower(), v)
        return text

    def speak(self, text, output_path="temp_speech.wav", play=True):
        if not self.model: return

        clean_text = self.preprocess(text)
        
        # 생성 (이미 로딩돼서 빠름)
        self.model.tts_to_file(clean_text, self.speaker_ids['KR'], output_path, speed=1.0)
        
        if play:
            # -q: 로그 숨김, aplay: 리눅스 기본 재생기
            os.system(f"aplay -q {output_path}")