# Planner Model Benchmark Plan

## 목적
이 문서는 `phil_robot`의 planner 모델 후보군을 같은 평가 하네스에서 순차적으로 비교하기 위한 실행 계획서다.

이번 라운드의 목표는 다음 두 가지다.

1. 현재 JSON production planner 경로에서 회귀 없이 동작하는 후보를 먼저 추린다.
2. 살아남은 후보들 중에서 지연시간, 출력 길이, 명령 안정성 면에서 가장 실용적인 모델을 고른다.

## 이번 1차 후보군
아래 태그 문자열을 이번 라운드의 고정 후보군으로 사용한다.

| 모델 태그 | 계열 | 이번 라운드에서 보고 싶은 것 |
| --- | --- | --- |
| `qwen3:30b-a3b-q4_K_M` | Qwen MoE | 현재 baseline family. MoE planner 기준점 |
| `qwen3:32b-q4_K_M` | Qwen dense | 같은 family 안에서 MoE 대 dense 비교 |
| `nemotron-cascade-2:30b-a3b-q4_K_M` | NVIDIA Nemotron MoE | Qwen MoE 대체재 가능성 |
| `gpt-oss:20b` | GPT-OSS | 더 작은 급에서 지연시간 이득이 있는지 |
| `mistral-small3.2:24b-instruct-2506-q4_K_M` | Mistral | instruction 안정성과 compact output |
| `exaone3.5:32b-instruct-q4_K_M` | EXAONE | 한국어 planner 품질과 bilingual 강점 |
| `lfm2:24b-q4_K_M` | Liquid | 효율형 대안으로서 latency 이득 |

## 비교 원칙
이번 planner 비교에서는 다음 항목을 고정한다.

- classifier 모델은 `qwen3:4b-instruct-2507-q4_K_M`로 고정한다.
- planner system prompt와 planner input JSON 구조는 production 코드 그대로 사용한다.
- planner benchmark 는 `JSON production path`만 사용한다.
- 케이스 파일은 [cases_smoke.json](/home/shy/robot_project/phil_robot/eval/cases_smoke.json)을 사용한다.
- 현재 smoke suite는 총 `13`개 케이스다.
- 한 번에 하나의 planner 모델만 메모리에 올려 순차 실행한다.
- 모델 간 비교에서는 `latest` alias보다 가능한 한 명시 태그를 우선 사용한다.
- alias 태그를 썼더라도 결과 기록에는 실제 digest를 함께 남긴다.
- first-call warm-up은 측정치에 포함하지 않는다.

## 기록해야 하는 공통 메타데이터
모든 모델 리포트에는 아래 정보를 같이 남긴다.

- planner model tag
- planner model digest
- classifier model tag
- 실행 시각
- 케이스 파일 경로
- Ollama 버전
- Python 환경
- warm-up 제외 여부
- JSON production path 여부

## 전체 진행 순서
이번 1차 벤치는 아래 4단계로 진행한다.

1. 설치 및 태그 확인
2. JSON production correctness smoke
3. JSON production latency 집계
4. 상위 후보에 대한 추가 비교

## 0단계: 설치 및 태그 확인
각 후보를 돌리기 전에 아래를 먼저 확인한다.

1. 모델 pull
2. 태그 문자열 확인
3. 실제 digest 확인
4. 같은 태그가 다른 alias를 가리키는지 확인

권장 명령:

```bash
ollama pull <planner-model-tag>
ollama show <planner-model-tag>
ollama list
```

이 단계에서 확인할 것:

- pull이 정상적으로 끝나는지
- 모델 크기가 현재 머신에서 현실적인지
- 같은 family 내 alias 충돌이 없는지
- `q4_K_M` 표기가 실제로 해당 digest와 대응하는지

## 1단계: JSON production correctness smoke
1차 게이트는 정확도와 회귀 여부다.

이 단계의 목적:

- planner가 current JSON production path를 깨지 않는지 확인
- classifier 결과를 받아 planner domain routing이 정상인지 확인
- validator까지 갔을 때 명령과 speech가 smoke expectation을 만족하는지 확인

현재 코드 기준 최소 실행 방식:

1. [run_eval.py](/home/shy/robot_project/phil_robot/eval/run_eval.py)의 `--planner-model`, `--classifier-model` override를 사용한다.
2. 아래 명령으로 smoke 평가를 실행한다.

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py \
  --suite smoke \
  --classifier-model qwen3:4b-instruct-2507-q4_K_M \
  --planner-model <planner-model-tag> \
  --save-report
```

이 단계에서 반드시 보는 것:

- `total_cases`, `passed_cases`, `failed_cases`
- 실패 케이스 id
- planner layer mismatch
- validator layer mismatch
- safety 관련 케이스 회귀 여부

즉시 탈락 기준:

- safety 케이스에서 명령이 살아남아서는 안 되는 상황에 명령이 살아남는 경우
- `stop`, `play`, `status`처럼 도메인이 명확한 케이스에서 planner domain이 반복적으로 틀리는 경우
- smoke suite에서 구조적으로 같은 유형의 실패가 2건 이상 반복되는 경우
- JSON 출력이 깨져 planner fallback이 반복적으로 나타나는 경우

보류 기준:

- 총 실패가 1건뿐이지만 wording 차이나 경계 케이스로 보이는 경우
- command는 맞는데 speech 표현만 약간 다른 경우

통과 기준:

- 현재 smoke suite 기준 회귀가 없거나, 사람이 봤을 때 production 위험이 없는 아주 경미한 차이만 있는 경우

## 2단계: JSON production latency 집계
정확도 게이트를 통과한 모델만 지연시간 비교 대상으로 올린다.

이 단계의 목적:

- correctness를 해치지 않는 범위에서 planner wall-clock 시간을 비교
- prompt prefill과 eval decode 중 어디서 시간이 늘어나는지 분리
- 출력 토큰이 길어서 느린 모델인지, prefill 자체가 느린 모델인지 구분

현재 하네스 기준 추천 방법:

1. 대상 모델을 `run_eval.py` 또는 `run_planner_benchmark.py`의 planner override로 설정한다.
2. warm-up 1회 후 아래 명령으로 JSON mode 기준 벤치를 돈다.
3. 결과 JSON에서 planner timing을 집계한다.

현재 저장소에는 format compare 전용 러너 [run_format_compare.py](/home/shy/robot_project/phil_robot/eval/run_format_compare.py), round batch 러너 [run_planner_benchmark.py](/home/shy/robot_project/phil_robot/eval/run_planner_benchmark.py), planner latency isolation 러너 [run_planner_latency_isolation.py](/home/shy/robot_project/phil_robot/eval/run_planner_latency_isolation.py)가 있다. planner 모델 후보 비교에는 `run_format_compare.py`를 쓰지 않고, JSON benchmark 러너들만 사용한다.

현재 round-1 자동 배치 러너는 [run_planner_benchmark.py](/home/shy/robot_project/phil_robot/eval/run_planner_benchmark.py)와 [planner_benchmark_round1_manifest.json](/home/shy/robot_project/phil_robot/eval/planner_benchmark_round1_manifest.json)을 사용한다.

권장 측정 항목:

- mean planner sec
- p95 planner sec
- slowest case
- mean prompt tokens
- mean eval tokens
- mean output chars
- parse success rate

권장 집계 원칙:

- first-call warm-up 제외
- 한 모델당 동일 suite 2회 이상 반복
- 최종 비교표에는 평균과 최솟값이 아니라 평균과 최악값을 같이 적기

실무 해석 원칙:

- `avg_wall_sec`가 가장 중요하다.
- `avg_eval_tokens`가 큰 모델은 response verbosity로 느릴 가능성이 높다.
- `avg_prompt_sec`가 지나치게 큰 모델은 긴 planner prompt에 약할 수 있다.
- `avg_overhead_sec`가 작다면 병목은 Python이 아니라 모델 추론 자체다.

## 모델별로 남겨야 하는 관찰 메모
숫자만으로 결정하지 말고 아래 qualitative 메모를 반드시 남긴다.

- 응답이 너무 장황한지
- 명령을 skill 위주로 깔끔하게 주는지
- 직접 low-level `move:`를 남발하는지
- 한국어가 자연스러운지
- 안전 차단 상황에서 과감하게 `no-op + 설명`으로 가는지
- `reason`이 쓸데없이 길어지는지

## 최종 선택 규칙
최종 선택은 아래 우선순위를 따른다.

1. smoke correctness
2. safety regression 없음
3. planner latency
4. output compactness
5. 한국어 품질

즉, 가장 빠른 모델이 아니라 `안전하게 맞고 충분히 빠른 모델`을 선택한다.

## 권장 파일 저장 규칙
Raw report는 `phil_robot/eval/reports/` 아래에 저장한다.

권장 규칙:

```text
smoke_report_<classifier약어>_<planner약어>_<YYYYMMDD_HHMM>.json
```

모델 비교 요약 문서는 이번 문서와 별도로 아래 형식으로 저장한다.

```text
phil_robot/eval/PLANNER_MODEL_BENCHMARK_ROUND1_KR.md
phil_robot/eval/PLANNER_MODEL_BENCHMARK_ROUND2_KR.md
```

## 이번 라운드의 실제 실행 순서
이번 1차 실행 순서는 아래 순서를 권장한다.

1. `qwen3:30b-a3b-q4_K_M`
2. `qwen3:32b-q4_K_M`
3. `nemotron-cascade-2:30b-a3b-q4_K_M`
4. `gpt-oss:20b`
5. `mistral-small3.2:24b-instruct-2506-q4_K_M`
6. `exaone3.5:32b-instruct-q4_K_M`
7. `lfm2:24b-q4_K_M`

이 순서를 추천하는 이유:

- 먼저 현재 baseline family와 가장 가까운 후보를 재측정한다.
- 그 다음 same-family dense 대안을 본다.
- 그 뒤에 타 family 대안을 비교한다.
- 마지막에는 효율형 후보를 본다.

## 이번 문서의 해석 범위
이 문서는 benchmark 실행 계획서다.

- 아직 모든 후보를 실제로 돌렸다는 뜻은 아니다.
- 이번 문서는 “무엇을 어떤 순서로 어떻게 기록하며 탈락시킬지”를 고정하는 문서다.
- 실제 결과는 이후 round report 문서와 raw JSON report로 따로 남긴다.
