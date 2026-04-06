# Format Compare Benchmark

- generated_at: `2026-04-02T11:12:28+09:00`
- case_source: `/home/shy/robot_project/phil_robot/eval/cases_smoke.json`
- case_count: `10`
- classifier_model: `qwen3:4b-instruct-2507-q4_K_M`
- planner_model: `qwen3:30b-a3b-instruct-2507-q4_K_M`
- method: classifier는 케이스마다 한 번만 실행하고, 같은 `intent_result`를 재사용해 planner 단계만 `legacy_str`와 `json`으로 비교했다.
- method: smoke 케이스 앞의 10개를 사용했고, 각 모드는 케이스당 1회 실행했다. 즉 모드별 총 10회다.
- method: Ollama 호출은 `temperature=0`으로 고정했다.
- method: 측정 전에 classifier와 planner 두 모드를 각각 1회씩 warm-up 하고, 그 호출은 통계에서 제외했다.
- raw_report: `format_compare_benchmark_smoke_20260402_111228.json`

## Average Table

| mode | cases | parse_ok | avg_wall_sec | avg_prompt_tokens | avg_prompt_sec | avg_prompt_tps | avg_eval_tokens | avg_eval_sec | avg_eval_tps | avg_meta_sec | avg_overhead_sec | avg_output_chars | avg_cmd_count |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| legacy_str | 10 | 100.00% | 3.95 | 1461.40 | 2.48 | 597.47 | 37.30 | 1.31 | 29.82 | 3.79 | 0.16 | 64.70 | 2.50 |
| json | 10 | 100.00% | 4.88 | 1529.40 | 2.53 | 611.96 | 57.10 | 2.03 | 28.30 | 4.56 | 0.32 | 126.20 | 1.10 |
| json - legacy_str | 0 | 0.00%p | 0.93 | 68.00 | 0.05 | 14.49 | 19.80 | 0.72 | -1.52 | 0.77 | 0.17 | 61.50 | -1.40 |

## Per-Case Table

| case_id | planner_domain | str_wall_sec | json_wall_sec | str_eval_tokens | json_eval_tokens | str_eval_tps | json_eval_tps | str_parse | json_parse |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| chat_greeting_basic | chat | 2.64 | 4.01 | 8.00 | 34.00 | 35.54 | 27.62 | True | True |
| motion_wave_allowed | motion | 3.35 | 3.94 | 26.00 | 44.00 | 28.55 | 32.36 | True | True |
| motion_wave_blocked_by_lock | motion | 4.10 | 4.02 | 44.00 | 44.00 | 31.46 | 28.65 | True | True |
| motion_arm_up_basic | motion | 3.06 | 4.32 | 17.00 | 53.00 | 32.91 | 32.83 | True | True |
| motion_left_arm_out_basic | motion | 4.27 | 4.97 | 45.00 | 56.00 | 27.92 | 25.88 | True | True |
| play_tim_basic | play | 5.93 | 5.19 | 85.00 | 59.00 | 27.29 | 25.14 | True | True |
| status_question_basic | status | 3.65 | 5.69 | 40.00 | 67.00 | 31.42 | 26.04 | True | True |
| stop_request_basic | stop | 5.19 | 5.01 | 61.00 | 48.00 | 26.78 | 25.68 | True | True |
| relative_wrist_raise_success | motion | 4.42 | 6.02 | 27.00 | 70.00 | 23.67 | 26.65 | True | True |
| relative_wrist_raise_blocked | motion | 2.84 | 5.62 | 20.00 | 96.00 | 32.66 | 32.17 | True | True |

## Notes

- `prompt_*`는 system prompt + user JSON 전체를 읽는 prefill 구간이다.
- `eval_*`는 실제 출력 토큰을 생성하는 decode 구간이다.
- `wall_sec`는 Python 바깥에서 잰 전체 planner 호출 시간이고, `meta_sec`는 Ollama 내부 메타데이터 합이다.
- `overhead_sec`는 `wall_sec - meta_sec`로 계산한 Python/Ollama 바깥 오버헤드 추정치다.
- 이 문서는 STT를 포함하지 않는다. smoke 텍스트 입력만으로 planner 출력 형식 차이를 비교했다.
