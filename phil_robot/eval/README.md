# Phil Robot 평가 파이프라인

## 관련 문서
- planner 후보군의 향후 비교 절차는 [PLANNER_MODEL_BENCHMARK_PLAN_KR.md](/home/shy/robot_project/phil_robot/eval/PLANNER_MODEL_BENCHMARK_PLAN_KR.md)에 정리되어 있습니다.
- round-1 자동 실행 manifest 는 [planner_benchmark_round1_manifest.json](/home/shy/robot_project/phil_robot/eval/planner_benchmark_round1_manifest.json)에 있습니다.

현재 planner benchmark 기본 원칙:
- planner benchmark 는 `JSON production path`만 사용합니다.
- `legacy_str` 대 `json` 비교 스크립트는 과거 형식 비교 실험용이며, planner 모델 후보 선정용 benchmark 에서는 사용하지 않습니다.

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
    "valid_commands_any_of": [["r", "p:TIM"]],
    "speech_contains_any": ["This Is Me", "연주"]
  }
}
```

## 실행 방법
`~/robot_project`에서 실행:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py --suite smoke
```

`config.py`를 바꾸지 않고 모델만 임시 override 하려면:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py \
  --suite smoke \
  --classifier-model qwen3:4b-instruct-2507-q4_K_M \
  --planner-model qwen3:30b-a3b-instruct-2507-q4_K_M
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
  --report phil_robot/eval/reports/smoke_report_q3-4b-q4km_q3-30b-a3b-q4km.json
```

`eval` 폴더 안에 있을 때:

```bash
python run_eval.py --suite smoke --report reports/smoke_report_q3-4b-q4km_q3-30b-a3b-q4km.json
```

표준 파일명 규칙으로 자동 저장하려면 `--save-report`를 사용합니다.

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py \
  --suite smoke \
  --save-report
```

`eval` 폴더 안에 있을 때:

```bash
python run_eval.py --suite smoke --save-report
```

planner round-1 전체 배치를 순차 실행하려면:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_planner_benchmark.py
```

기본 동작:
- manifest 의 후보 순서를 그대로 따라 prep -> smoke 를 순차 실행합니다.
- classifier 는 케이스당 한 번만 실행해 `classifier_result` 와 `planner_input_json` 을 고정하고, 각 planner 모델은 같은 JSON fixture 위에서만 비교합니다.
- `config.py`는 수정하지 않고 benchmark 러너 내부에서 classifier/planner 모델을 주입합니다.
- round summary 는 `PLANNER_MODEL_BENCHMARK_ROUND1.json`, `PLANNER_MODEL_BENCHMARK_ROUND1_KR.md`에 저장됩니다.

planner latency isolation benchmark 를 따로 실행하려면:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_planner_latency_isolation.py \
  --suite smoke \
  --classifier-model qwen3:4b-instruct-2507-q4_K_M \
  --planner-model qwen3:30b-a3b-instruct-2507-q4_K_M \
  --save-report
```

기본 동작:
- classifier 는 케이스당 한 번만 실행해 fixture 를 만들고, 같은 `planner_input_json` 위에서 planner 만 반복 호출합니다.
- 기본값은 case 당 warm-up `1`회 제외, warm run `3`회 측정입니다.
- 리포트에는 planner input 길이, response 길이, avg / median / p95 latency 와 output variability 가 들어갑니다.

## 리포트 파일명 규칙
자동 생성되는 리포트 파일명은 다음 규칙을 따릅니다.

```text
<suite>_report_<classifier약어>_<planner약어>_<YYYYMMDD_HHMM>.json
```

예:

```text
smoke_report_q3-4b-q4km_q3-30b-a3b-q4km_20260317_1530.json
```

약어 규칙:
- classifier 약어: 현재 `CLASSIFIER_MODEL` 값에서 생성
- planner 약어: 현재 `PLANNER_MODEL` 값에서 생성
- provider, 파라미터 규모, quant 정보 위주로 압축

동일한 분에 같은 모델 조합으로 다시 저장하면 뒤에 순번을 붙입니다.

```text
smoke_report_q3-4b-q4km_q3-30b-a3b-q4km_20260317_1530_1.json
smoke_report_q3-4b-q4km_q3-30b-a3b-q4km_20260317_1530_2.json
```

즉:
- 첫 실행은 모델 조합 + 측정 시각이 들어간 이름으로 저장
- 같은 분 안에 같은 모델 조합으로 다시 저장하면 `_1`, `_2` ... 가 자동 증가

## 리포트 메타데이터
새로 저장되는 JSON 리포트에는 다음 메타데이터가 함께 기록됩니다.
- `generated_at`
- `suite`
- `cases_path`
- `classifier_model`
- `planner_model`

이 정보로 나중에 어떤 모델 조합과 어떤 케이스 파일로 측정했는지 추적할 수 있습니다.

## 기존 리포트 파일에 대한 참고
현재 `reports/` 안에 있는 기존 `smoke_report*.json` 파일 중 일부는
이 규칙을 도입하기 전에 수동으로 저장된 legacy 리포트일 수 있습니다.
새 규칙은 `--save-report`로 생성한 리포트부터 일관되게 적용됩니다.

## 설계 메모
- 이 평가 경로는 실제 classifier / planner 모델을 그대로 호출합니다.
- 실제 로봇 소켓 명령은 전송하지 않습니다.
- production과 동일한 `run_brain_turn(...)` 경로를 재사용해서 현실적인 채점을 합니다.
- 이후 회귀 테스트와 replay 평가의 기반이 되도록 설계했습니다.
