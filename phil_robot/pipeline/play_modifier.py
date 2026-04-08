from dataclasses import dataclass
from typing import Optional

@dataclass
class PlayModifier:
    """
    연주 방식을 조정하는 클래스입니다.
    """
    tempo_scale: float = 1.0  # 박자 조정 비율 (예: 1.0은 원래 속도, 0.5는 절반 속도)
    velocity_delta: int = 0  # 강세 조정 값 (예: 0은 원래 강세, 양수는 강세 증가, 음수는 강세 감소)
    source: Optional[str] = None  # 수정의 출처 (예: explicit / context / memory / inferred 등)
    apply_scope: Optional[str] = None  # 수정이 적용되는 범위 (예: current_play / next_play etc.)

    # 모든 값이 기본값인 경우
    def is_identity(self) -> bool:
        return self.tempo_scale == 1.0 and self.velocity_delta == 0

def parse_play_modifier(user_text: str) -> PlayModifier:
    """
    사용자의 명령을 분석하여 PlayModifier 객체를 생성합니다.
    예시 명령: "빠르게 연주해줘", "느리게 연주해줘", "세게 연주해줘", "약하게 연주해줘"
    """
    # 인스턴스 초기화
    mod = PlayModifier()
    
    if "빠르게" in user_text or "빠르고" in user_text or "빨리" in user_text or "빠른" in user_text or "답답" in user_text:
        mod.tempo_scale = 1.1
    elif "느리게" in user_text or "느리고" in user_text or "천천히" in user_text or "느린" in user_text or "느림" in user_text or "느려" in user_text:
        mod.tempo_scale = 0.9
    
    if "세게" in user_text or "세고" in user_text or "강하게" in user_text or "강하고" in user_text or "세진" in user_text or "강한" in user_text:
        mod.velocity_delta = 1
    elif "약하게" in user_text or "약하고" in user_text or "살살" in user_text:
        mod.velocity_delta = -1

    if not mod.is_identity():
        mod.source = "explicit"
        mod.apply_scope = "next_play"
    
    return mod
