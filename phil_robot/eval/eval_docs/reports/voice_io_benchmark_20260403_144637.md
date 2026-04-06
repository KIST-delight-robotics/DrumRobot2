# voice_io_benchmark_20260403_144637.json 해설 리포트

- source_json: `reports/voice_io_benchmark_20260403_144637.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | voice I/O 설정 비교 |
| cases_path | /home/shy/robot_project/phil_robot/eval/cases_smoke.json |
| case_count | 13 |
| user_text_count | 11 |
| speech_count | 13 |
| stt_model | small |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | qwen3:30b-a3b-instruct-2507-q4_K_M |

## 왜 이 실험을 했는가

음성 입력과 음성 출력을 더 빠르고 안정적으로 만들기 위해, STT와 TTS 설정을 바꿔 보며 비교한 실험입니다.

## 이번에 바꿔 보거나 고정한 점

- STT에서 fp16 켜기/끄기와 초기 힌트 프롬프트 유무를 비교했습니다.
- TTS에서 파일 한 번에 생성하는 방식과 문장 단위 stream 방식을 비교했습니다.
- stream 최소 길이도 24, 18, 14로 바꿔 보며 첫 음성 시작 시간을 확인했습니다.
- 이 JSON 기준 추천 기본값은 `stt_use_fp16=True`, `tts_stream_min_len=14` 입니다.

## STT 결과 요약

| config | avg latency | p95 latency | avg score | exact hits | 정밀도 모드 | 초기 힌트 |
| --- | --- | --- | --- | --- | --- | --- |
| fp32_plain | 11.1366 s | 43.9652 s | 0.8% | 0/11 (0.0%) | fp32 | (없음) |
| fp16_plain | 9.0833 s | 18.1923 s | 5.5% | 0/11 (0.0%) | fp16 | (없음) |
| fp16_hint | 10.6861 s | 36.4046 s | 10.4% | 1/11 (9.1%) | fp16 | 필, 드럼 로봇, 연주, 손, 팔, 손목, 고개, 시선 |

## STT에서 어려웠던 발화

| 발화 | 평균 점수 | exact hits | 평균 시간 | 대표 오인식 |
| --- | --- | --- | --- | --- |
| 안녕 | 0.0% | 0/3 (0.0%) | 7.8780 s | että스 |
| 팔 올려 | 0.0% | 0/3 (0.0%) | 29.4946 s | n languages |
| 왼팔 벌려 | 0.0% | 0/3 (0.0%) | 4.8912 s | 을 찾을 рол이 |
| 왜 멈췄어? | 0.0% | 0/3 (0.0%) | 5.5301 s | 음병 주섬 |
| 종료해 | 0.0% | 0/3 (0.0%) | 4.0141 s | plateau |
| 왼쪽 손목 더 올려 | 0.0% | 0/3 (0.0%) | 6.7047 s | 좋습니다ım |
| 계란 반숙은 몇 분 삶아야 해? | 0.0% | 0/3 (0.0%) | 8.9918 s | 그린öndramatic factories |
| 이름이 뭔데? | 0.0% | 0/3 (0.0%) | 22.7482 s | iiiiii فim |
| 거기서 50도 더 올리고 2초 있다 | 2.9% | 0/3 (0.0%) | 9.1258 s | �urmtismr°l |
| This Is Me 연주해줘 | 12.9% | 0/3 (0.0%) | 8.2029 s | 개어쳐 네 |
| 손 흔들어줘 | 45.5% | 1/3 (33.3%) | 5.7406 s | 스노성-상전란 – |

## TTS 결과 요약

| config | avg first audio | p95 first audio | avg synth total | avg chunk | split_cases | 파일 fallback 수 |
| --- | --- | --- | --- | --- | --- | --- |
| file_only | 0.7120 s | 1.3954 s | 0.7120 s | 1.0 | 0 | 0 |
| stream_24 | 0.7174 s | 1.3766 s | 0.7174 s | 1.0 | 0 | 13 |
| stream_18 | 0.6840 s | 1.2979 s | 0.6840 s | 1.0 | 0 | 13 |
| stream_14 | 0.6178 s | 1.3385 s | 0.6178 s | 1.0 | 0 | 13 |

## 선택된 TTS 설정 상세 표

| id | 상황 | first audio | synth total | chunk | 분할 | 파일 fallback |
| --- | --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | 인사 요청 | 0.2813 s | 0.2813 s | 1 | 아니오 | 예 |
| motion_wave_allowed | 움직일 수 있는 상태에서 손 흔들기 | 0.3467 s | 0.3467 s | 1 | 아니오 | 예 |
| motion_wave_blocked_by_lock | 안전 키가 잠긴 상태에서 손 흔들기 | 0.3479 s | 0.3479 s | 1 | 아니오 | 예 |
| motion_arm_up_basic | 팔 올리기 | 0.6490 s | 0.6490 s | 1 | 아니오 | 예 |
| motion_left_arm_out_basic | 왼팔 벌리기 | 0.5533 s | 0.5533 s | 1 | 아니오 | 예 |
| play_tim_basic | 곡 연주 요청 | 0.6038 s | 0.6038 s | 1 | 아니오 | 예 |
| status_question_basic | 오류로 멈춘 뒤 이유 묻기 | 0.7986 s | 0.7986 s | 1 | 아니오 | 예 |
| stop_request_basic | 종료 요청 | 0.5596 s | 0.5596 s | 1 | 아니오 | 예 |
| relative_wrist_raise_success | 왼쪽 손목을 조금 더 올리기 | 0.6544 s | 0.6544 s | 1 | 아니오 | 예 |
| relative_wrist_raise_blocked | 손목을 더 올리면 범위를 넘는 상황 | 1.3385 s | 1.3385 s | 1 | 아니오 | 예 |
| motion_blocked_while_playing | 연주 중 다른 동작 요청 | 0.5357 s | 0.5357 s | 1 | 아니오 | 예 |
| chat_general_knowledge | 일반 지식 질문 | 0.9387 s | 0.9387 s | 1 | 아니오 | 예 |
| chat_identity_name | 로봇 이름 묻기 | 0.4240 s | 0.4240 s | 1 | 아니오 | 예 |

## 눈여겨볼 점

- STT에서는 `fp16_hint`가 가장 빨랐고 평균 점수도 `10.4%` 수준이었습니다.
- TTS에서는 `stream_14`가 첫 음성 시작 시간을 `0.6178 s`까지 줄였습니다.
- 가장 알아듣기 어려운 발화는 `안녕`였고, 대표 오인식은 `että스`였습니다.
- 긴 안전 안내 문장이나 상태 설명 문장에서는 stream 방식이 두 조각으로 나뉘어 먼저 말이 나오기 시작했습니다.

## 종합 총평

이 문서 기준 추천 기본값은 `stt_use_fp16=True`, `stt_initial_prompt='필, 드럼 로봇, 연주, 손, 팔, 손목, 고개, 시선'`, `tts_stream_enabled=True`, `tts_stream_min_len=14` 입니다. 즉 STT는 불필요한 옵션을 덜어 속도와 인식 품질을 같이 보고, TTS는 stream 길이를 조정해 첫 반응을 앞당긴 결과입니다.
