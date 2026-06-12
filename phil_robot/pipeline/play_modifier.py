from dataclasses import dataclass
from typing import Optional

# 한 번의 "빠르게/느리게"가 직전 배속에 곱해지는 비율과 허용 범위.
TEMPO_STEP = 1.1
TEMPO_MAX = 2.0
TEMPO_MIN = 0.5

# "세게/약하게"가 직전 강세 보정에 더해지는 값과 허용 범위.
VELOCITY_STEP = 1
VELOCITY_MAX = 3
VELOCITY_MIN = -3


def _clamp(value: float, low: float, high: float) -> float:
    if value < low:
        return low
    if value > high:
        return high
    return value


@dataclass
class PlayModifier:
    """
    연주 방식을 조정하는 클래스입니다.
    """
    tempo_scale: float = 1.0  # 박자 조정 비율 (예: 1.0은 원래 속도, 0.5는 절반 속도)
    velocity_delta: int = 0  # 강세 조정 값 (예: 0은 원래 강세, 양수는 강세 증가, 음수는 강세 감소)
    source: Optional[str] = None  # 수정의 출처 (예: explicit / context / memory / inferred 등)
    apply_scope: Optional[str] = None  # 수정이 적용되는 범위 (예: current_play / next_play etc.)
    requested: bool = False  # 이번 발화에서 실제로 속도/세기 변경을 요청했는지 여부

    # 모든 값이 기본값인 경우
    def is_identity(self) -> bool:
        return self.tempo_scale == 1.0 and self.velocity_delta == 0


def _is_reset_request(user_text: str) -> bool:
    """'원래 속도로', '보통 빠르기로' 처럼 기본값 복귀를 요청했는지 판단한다."""
    if "원래대로" in user_text:
        return True
    has_target = "속도" in user_text or "빠르기" in user_text or "템포" in user_text
    has_reset = "원래" in user_text or "보통" in user_text or "정상" in user_text or "기본" in user_text
    return has_target and has_reset


def parse_play_modifier(
    user_text: str,
    base_tempo_scale: float = 1.0,
    base_velocity_delta: int = 0,
) -> PlayModifier:
    """
    사용자의 명령을 분석하여 PlayModifier 객체를 생성합니다.
    예시 명령: "더 빠르게 연주해줘", "느리게 연주해줘", "세게 연주해줘", "약하게 연주해줘"

    속도/세기는 절대값이 아니라 직전 상태(base_*)에 누적해서 적용한다.
    그래서 "더 빠르게"를 반복하면 배속이 계속 커진다.
    """
    # 직전 상태를 기본값으로 들고 시작한다(변경 요청이 없으면 그대로 유지/전달).
    mod = PlayModifier()
    mod.tempo_scale = base_tempo_scale
    mod.velocity_delta = base_velocity_delta

    tempo_requested = False
    velocity_requested = False

    if _is_reset_request(user_text):
        mod.tempo_scale = 1.0
        mod.velocity_delta = 0
        tempo_requested = True
        velocity_requested = True
    else:
        faster = "빠르게" in user_text or "빠르고" in user_text or "빨리" in user_text or "빠른" in user_text or "답답" in user_text
        slower = "느리게" in user_text or "느리고" in user_text or "천천히" in user_text or "느린" in user_text or "느림" in user_text or "느려" in user_text

        if faster:
            mod.tempo_scale = round(_clamp(base_tempo_scale * TEMPO_STEP, TEMPO_MIN, TEMPO_MAX), 2)
            tempo_requested = True
        elif slower:
            mod.tempo_scale = round(_clamp(base_tempo_scale / TEMPO_STEP, TEMPO_MIN, TEMPO_MAX), 2)
            tempo_requested = True

        stronger = "세게" in user_text or "세고" in user_text or "강하게" in user_text or "강하고" in user_text or "세진" in user_text or "강한" in user_text
        softer = "약하게" in user_text or "약하고" in user_text or "살살" in user_text

        if stronger:
            mod.velocity_delta = int(_clamp(base_velocity_delta + VELOCITY_STEP, VELOCITY_MIN, VELOCITY_MAX))
            velocity_requested = True
        elif softer:
            mod.velocity_delta = int(_clamp(base_velocity_delta - VELOCITY_STEP, VELOCITY_MIN, VELOCITY_MAX))
            velocity_requested = True

    mod.requested = tempo_requested or velocity_requested
    if mod.requested:
        mod.source = "explicit"
        mod.apply_scope = "next_play"

    return mod
