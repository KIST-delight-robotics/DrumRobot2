# Phil Robot 평가 파이프라인

## 목적
이 디렉토리는 `phil_robot`의 Python LLM 제어 스택을 위한 오프라인 평가 파이프라인을 담고 있습니다.

평가 흐름은 실제 로봇 런타임 경로와 의도적으로 분리되어 있습니다.

운영 경로:
- 실제 로봇 상호작용을 위한 경로
- 지연시간과 운영 안정성이 가장 중요함

평가 경로:
- replay, 회귀 테스트, 메트릭 집계를 위한 경로
- 더 느려도 괜찮음
- 더 많은 구조화 데이터를 저장함
- 실제 단계별 출력이 기대값과 일치하는지 비교함

## 평가 레이어
각 평가 케이스는 여러 레이어를 동시에 채점할 수 있습니다.

1. `classifier`
2. `planner`
3. `validator`
4. `e2e`

### Classifier 레이어
검사 항목:
- `intent`
- `needs_motion`
- `needs_dialogue`
- `risk_level`

### Planner 레이어
검사 항목:
- `planner_domain`
- 선택된 `skills`
- 생성된 `commands`

### Validator 레이어
검사 항목:
- `valid_commands`
- `rejected_commands`
- 최종 `speech`

### End-to-End 레이어
한 턴 전체 결과를 검사합니다.
- classifier 결과
- planner routing
- validator 결과
- 최종 speech 제약

## 케이스 포맷
케이스 파일은 JSON 배열입니다. 각 케이스는 다음 구조를 가집니다.

```json
{
  "id": "play_tim_basic",
  "tags": ["play", "smoke"],
  "user_text": "This Is Me 연주해줘",
  "robot_state": {
    "state": 0,
    "bpm": 100,
    "is_fixed": true,
    "current_song": "None",
    "progress": "0/1",
    "last_action": "None",
    "is_lock_key_removed": true,
    "error_message": "None",
    "current_angles": {
      "waist": 0.0,
      "L_wrist": 75.0
    }
  },
  "expected": {
    "intent": "play_request",
    "planner_domain": "play",
    "skills_any_of": [["play_tim"], ["ready_pose", "play_tim"]],
    "valid_commands_any_of": [["r", "p:TIM", "led:play"]],
    "speech_contains_any": ["This Is Me", "연주"]
  }
}
```

## 실행 방법
`~/robot_project`에서 실행:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py --suite smoke
```

혹은 파일을 직접 지정:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py --cases phil_robot/eval/cases_smoke.json
```

`~/robot_project/phil_robot/eval` 안에 들어와 있다면:

```bash
python run_eval.py --suite smoke
```

## 출력
러너는 다음을 출력합니다.
- 케이스별 pass/fail 요약
- 레이어별 mismatch 상세
- 전체 pass 집계

JSON 리포트로 저장할 수도 있습니다.

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py \
  --suite smoke \
  --report phil_robot/eval/reports/smoke_report.json
```

`eval` 폴더 안에 있을 때:

```bash
python run_eval.py --suite smoke --report reports/smoke_report.json
```

## 설계 메모
- 이 평가 경로는 실제 classifier / planner 모델을 그대로 호출합니다.
- 실제 로봇 소켓 명령은 전송하지 않습니다.
- production과 동일한 `run_brain_turn(...)` 경로를 재사용해서 현실적인 채점을 합니다.
- 이후 회귀 테스트와 replay 평가의 기반이 되도록 설계했습니다.
