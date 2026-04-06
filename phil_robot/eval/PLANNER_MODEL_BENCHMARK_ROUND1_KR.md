# Planner Model Benchmark Round 1

- generated_at: `2026-04-02T15:49:45+09:00`
- cases_path: `/home/shy/robot_project/phil_robot/eval/cases_smoke.json`
- classifier_model: `qwen3:4b-instruct-2507-q4_K_M`
- ollama_version: `ollama version is 0.14.1`
- python_env: `/home/shy/miniforge3/envs/drum4/bin/python (Python 3.8.20)`
- latency_repeats: `2`
- stop_before_compare: `True`

## Status Table

| order | family | requested_tag | resolved_tag | model_id | prep | gate | smoke_pass | latency_mean | latency_p95 | latency_worst | smoke_report | latency_report |
| ---: | --- | --- | --- | --- | --- | --- | --- | ---: | ---: | ---: | --- | --- |
| 1 | qwen3_moe | qwen3:30b-a3b-q4_K_M | qwen3:30b-a3b-instruct-2507-q4_K_M | 19e422b02313 | ready | fail | 11/13 | N/A | N/A | N/A | reports/smoke_report_q3-4b-q4km_q3-30b-a3b-q4km_20260402_1505.json | - |
| 2 | qwen3_dense | qwen3:32b-q4_K_M | qwen3:32b-q4_K_M | 030ee887880f | ready | fail | 10/13 | N/A | N/A | N/A | reports/smoke_report_q3-4b-q4km_q3-32b-q4km_20260402_1527.json | - |
| 3 | nemotron_moe | nemotron-cascade-2:30b-a3b-q4_K_M | - | - | prep_fail | prep_fail | N/A | N/A | N/A | N/A | - | - |
| 4 | gpt_oss | gpt-oss:20b | gpt-oss:20b | 17052f91a42e | ready | fail | 11/13 | N/A | N/A | N/A | reports/smoke_report_q3-4b-q4km_gptoss-20b_20260402_1534.json | - |
| 5 | mistral_small | mistral-small3.2:24b-instruct-2506-q4_K_M | mistral-small3.2:24b-instruct-2506-q4_K_M | 5a408ab55df5 | ready | fail | 11/13 | N/A | N/A | N/A | reports/smoke_report_q3-4b-q4km_mistra-24b-q4km_20260402_1541.json | - |
| 6 | exaone | exaone3.5:32b-instruct-q4_K_M | exaone3.5:32b-instruct-q4_K_M | f2f69abac3da | ready | fail | 11/13 | N/A | N/A | N/A | reports/smoke_report_q3-4b-q4km_exaone-32b-q4km_20260402_1549.json | - |
| 7 | lfm2 | lfm2:24b-q4_K_M | - | - | prep_fail | prep_fail | N/A | N/A | N/A | N/A | - | - |

## Pending Human Review

- 없음

## Notes

- 이 문서는 factual round summary 만 담고 있으며, 모델 간 우열 비교나 최종 추천은 의도적으로 넣지 않았다.
- `hold_review` 모델은 사람이 raw output 과 speech 를 보고 경계 케이스인지 확인한 뒤 다음 라운드 비교 후보에 올린다.