# voice_io_benchmark_20260403_145124.json 해설 리포트

- source_json: `reports/voice_io_benchmark_20260403_145124.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | voice I/O 설정 비교 |
| cases_path | /home/shy/robot_project/phil_robot/eval/cases_smoke.json |
| case_count | 13 |
| user_text_count | 11 |
| speech_count | 18 |
| stt_model | small |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | qwen3:30b-a3b-instruct-2507-q4_K_M |

## 왜 이 실험을 했는가

음성 입력과 음성 출력을 더 빠르고 안정적으로 만들기 위해, STT와 TTS 설정을 바꿔 보며 비교한 실험입니다.

## 이번에 바꿔 보거나 고정한 점

- STT에서 fp16 켜기/끄기와 초기 힌트 프롬프트 유무를 비교했습니다.
- TTS에서 파일 한 번에 생성하는 방식과 문장 단위 stream 방식을 비교했습니다.
- stream 최소 길이도 24, 18, 14로 바꿔 보며 첫 음성 시작 시간을 확인했습니다.
- 이 JSON 기준 추천 기본값은 `stt_use_fp16=False`, `tts_stream_min_len=18` 입니다.

## STT 결과 요약

| config | avg latency | p95 latency | avg score | exact hits | 정밀도 모드 | 초기 힌트 |
| --- | --- | --- | --- | --- | --- | --- |
| fp32_plain | 0.9336 s | 1.2676 s | 81.7% | 6/11 (54.5%) | fp32 | (없음) |
| fp16_plain | 1.0280 s | 1.4895 s | 81.7% | 6/11 (54.5%) | fp16 | (없음) |
| fp16_hint | 1.0322 s | 1.8078 s | 82.7% | 6/11 (54.5%) | fp16 | 필, 드럼 로봇, 연주, 손, 팔, 손목, 고개, 시선 |

## STT에서 어려웠던 발화

| 발화 | 평균 점수 | exact hits | 평균 시간 | 대표 오인식 |
| --- | --- | --- | --- | --- |
| This Is Me 연주해줘 | 38.7% | 0/3 (0.0%) | 1.2582 s | 빗을 이제 미 연주해 줘. |
| 팔 올려 | 44.4% | 0/3 (0.0%) | 0.7302 s | 탈 몰려 |
| 왼팔 벌려 | 66.7% | 0/3 (0.0%) | 0.9483 s | 웬탈 벌려 |
| 계란 반숙은 몇 분 삶아야 해? | 72.7% | 0/3 (0.0%) | 1.4615 s | 베란 단속은 몇 분 삶아야 해? |
| 손 흔들어줘 | 80.0% | 0/3 (0.0%) | 0.8717 s | 손 흔들어져 |
| 안녕 | 100.0% | 3/3 (100.0%) | 0.5973 s | 안녕 |
| 왜 멈췄어? | 100.0% | 3/3 (100.0%) | 0.9787 s | 왜 멈췄어? |
| 종료해 | 100.0% | 3/3 (100.0%) | 0.9371 s | 종료해. |
| 왼쪽 손목 더 올려 | 100.0% | 3/3 (100.0%) | 1.1344 s | 왼쪽 손목 더 올려. |
| 거기서 50도 더 올리고 2초 있다 | 100.0% | 3/3 (100.0%) | 1.1815 s | 거기서 50도 더 올리고 2초 있다. |
| 이름이 뭔데? | 100.0% | 3/3 (100.0%) | 0.8781 s | 이름이 뭔데? |

## TTS 결과 요약

| config | avg first audio | p95 first audio | avg synth total | avg chunk | split_cases | 파일 fallback 수 |
| --- | --- | --- | --- | --- | --- | --- |
| file_only | 1.1193 s | 1.8473 s | 1.1193 s | 1.0 | 0 | 0 |
| stream_24 | 0.7329 s | 1.4996 s | 0.8755 s | 1.2 | 4 | 14 |
| stream_18 | 0.7141 s | 1.5865 s | 0.8559 s | 1.2 | 4 | 14 |
| stream_14 | 0.7365 s | 1.6128 s | 0.8681 s | 1.2 | 4 | 14 |

## 선택된 TTS 설정 상세 표

| id | 상황 | first audio | synth total | chunk | 분할 | 파일 fallback |
| --- | --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | 인사 요청 | 0.2904 s | 0.2904 s | 1 | 아니오 | 예 |
| motion_wave_allowed | 움직일 수 있는 상태에서 손 흔들기 | 0.6442 s | 0.6442 s | 1 | 아니오 | 예 |
| motion_wave_blocked_by_lock | 안전 키가 잠긴 상태에서 손 흔들기 | 0.3666 s | 0.3666 s | 1 | 아니오 | 예 |
| motion_arm_up_basic | 팔 올리기 | 0.4899 s | 0.4899 s | 1 | 아니오 | 예 |
| motion_left_arm_out_basic | 왼팔 벌리기 | 0.3460 s | 0.3460 s | 1 | 아니오 | 예 |
| play_tim_basic | 곡 연주 요청 | 0.6001 s | 0.6001 s | 1 | 아니오 | 예 |
| status_question_basic | 오류로 멈춘 뒤 이유 묻기 | 0.9177 s | 0.9177 s | 1 | 아니오 | 예 |
| stop_request_basic | 종료 요청 | 0.3316 s | 0.3316 s | 1 | 아니오 | 예 |
| relative_wrist_raise_success | 왼쪽 손목을 조금 더 올리기 | 0.6413 s | 0.6413 s | 1 | 아니오 | 예 |
| relative_wrist_raise_blocked | 손목을 더 올리면 범위를 넘는 상황 | 1.5865 s | 1.5865 s | 1 | 아니오 | 예 |
| motion_blocked_while_playing | 연주 중 다른 동작 요청 | 0.2877 s | 0.2877 s | 1 | 아니오 | 예 |
| chat_general_knowledge | 일반 지식 질문 | 0.8840 s | 0.8840 s | 1 | 아니오 | 예 |
| chat_identity_name | 로봇 이름 묻기 | 0.4651 s | 0.4651 s | 1 | 아니오 | 예 |
| probe_1 | 짧은 안내 문장 | 1.4115 s | 1.4115 s | 1 | 아니오 | 예 |
| probe_2 | 안전 키 안내 문장 | 0.8555 s | 1.6100 s | 2 | 예 | 아니오 |
| probe_3 | 연주 중 제한 안내 문장 | 0.9323 s | 1.6891 s | 2 | 예 | 아니오 |
| probe_4 | 관절 한계 안내 문장 | 0.8878 s | 1.4742 s | 2 | 예 | 아니오 |
| probe_5 | 긴 일반 답변 문장 | 0.9161 s | 1.3700 s | 2 | 예 | 아니오 |

## 눈여겨볼 점

- STT에서는 `fp32_plain`가 가장 빨랐고 평균 점수도 `81.7%` 수준이었습니다.
- TTS에서는 `stream_18`가 첫 음성 시작 시간을 `0.7141 s`까지 줄였습니다.
- 가장 알아듣기 어려운 발화는 `This Is Me 연주해줘`였고, 대표 오인식은 `빗을 이제 미 연주해 줘.`였습니다.
- 긴 안전 안내 문장이나 상태 설명 문장에서는 stream 방식이 두 조각으로 나뉘어 먼저 말이 나오기 시작했습니다.

## 종합 총평

이 문서 기준 추천 기본값은 `stt_use_fp16=False`, `stt_initial_prompt=''`, `tts_stream_enabled=True`, `tts_stream_min_len=18` 입니다. 즉 STT는 불필요한 옵션을 덜어 속도와 인식 품질을 같이 보고, TTS는 stream 길이를 조정해 첫 반응을 앞당긴 결과입니다.
