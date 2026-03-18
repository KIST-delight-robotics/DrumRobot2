"""
phil_robot 공용 설정 상수.

엔트리포인트(`phil_brain.py`)와 하위 pipeline 계층이 함께 참조하는 값을
한 곳에 모아 의존성 방향을 단순하게 유지한다.
"""

CLASSIFIER_MODEL = "qwen3:4b-instruct-2507-q4_K_M"
PLANNER_MODEL = "qwen3:30b-a3b-instruct-2507-q4_K_M"
