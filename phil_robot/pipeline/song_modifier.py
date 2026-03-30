from typing import Dict, List, Set
from dataclasses import dataclass

@dataclass
class SongModifier:
    """
    사용자의 명령을 통해 드럼 비트의 박자와 강세를 조정하는 클래스입니다.
    """
    tempo_scale: float = 1.0  # 박자 조정 비율 (예: 1.0은 원래 속도, 0.5는 절반 속도)
    velocity_delta: int = 0  # 강세 조정 값 (예: 0은 원래 강세, 양수는 강세 증가, 음수는 강세 감소)

    def is_identity(self) -> bool:
        
        return self.tempo_scale == 1.0 and self.velocity_delta == 0

def parse_song_modifier(user_text: str) -> SongModifier:
    """
    사용자의 명령을 분석하여 SongModifier 객체를 생성합니다.
    예시 명령: "빠르게 연주해줘", "느리게 연주해줘", "강세를 20만큼 줄여줘"
    """
    # 인스턴스 초기화
    mod = SongModifier()
    
    if "빠르게" in user_text:
        mod.tempo_scale = 1.5
    elif "느리게" in user_text:
        mod.tempo_scale = 0.5
    
    if "세게" in user_text or "강하게" in user_text:
        mod.velocity_delta = 1
    elif "약하게" in user_text or "살살" in user_text:
        mod.velocity_delta = -1
    
    return mod