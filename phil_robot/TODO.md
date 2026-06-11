# Phil Robot TODO

이 파일은 `phil_robot`의 실행 가능한 작업 보드다.
아이디어를 전부 같은 레벨에 두지 않고, `Now / Next / Later / Parking Lot`으로 강제 분리한다.

운영 규칙:
- `Now`에는 최대 3개만 둔다.
- 새 아이디어가 떠오르면 바로 구현하지 말고 먼저 `Parking Lot`에 적는다.
- `Now`가 비워질 때만 `Next`에서 끌어올린다.
- 체크 완료 시 체크박스를 `x`로 바꾸고, 필요하면 한 줄 메모를 덧붙인다.
- 실험/측정은 가능하면 결과 파일 경로를 같이 남긴다.

## Now

> 아래 3개는 한 묶음의 `decision graph refactor` 단계다. 확정된 설계 전제:
> - planner는 명령을 "잘못" 만들지 않는다. 못 하는 요청(범위초과/정보없음/상태막힘)은 명령 대신 자연어 되묻기를 낸다.
> - 따라서 **턴 내부 planner↔validator 순환은 없다 (턴당 planner 1회).** 되돌아감은 전부 사람이 다음 턴에 다시 시도하는 cross-turn 스레드 한 종류로 통일한다.
> - validator는 speech 작가 역할을 버리고 **안전망(safety net)** 으로만 남는다.
> - 각 단계 sub-checkbox를 구현 진행하며 직접 체크한다. 의미 있는 변경마다 루트 `log.md`도 갱신한다.

- [x] `decision graph refactor 1단계: 그래프 노드 분해 + BrainTurnResult 제거`
  - 목표: `run_brain_turn`이 classifier+planner+validator를 한 함수로 묶고 `BrainTurnResult`로 반환하던 이중 래퍼를 없애고, `PhilState` 하나가 노드 사이를 굴러다니게 한다.
  - 배경:
    - 지금 `robot_graph.py`의 `process_node`는 state machine가 아니라 `run_brain_turn()`을 한 번 부르고 결과를 PhilState로 옮겨담는 얇은 래퍼다.
    - classifier/planner/validator가 `run_brain_turn` 안에 융합돼 있어 노드 단위 분기를 표현할 수 없다.
  - 체크리스트:
    - [x] `run_brain_turn` 분해 → step 함수(`build_prefilter_plan`/`classify_step`/`build_direct_answer_plan`/`planner_step`) + graph 노드(`ingest`/`classify`/`state`/`direct_answer`/`planner`/`validator`/`execute`)
    - [x] `BrainTurnResult` 런타임에서 제거 — 각 노드가 `PhilState` 필드(classifier_output / robot_state / planner_output / validated / speech / commands)를 직접 채운다. (BrainTurnResult/run_brain_turn은 삭제가 아니라 `eval/brain_probe.py`로 이주 — eval/benchmark 진단 어댑터로만 유지)
    - [x] `phil_brain.py`는 최종 `PhilState["speech"]` 와 `PhilState["commands"]` 만 읽도록 정리 (어느 노드가 채웠는지 무관). 인터럽트는 ingest_node, 상태 fetch는 state_node로 이동
    - [x] classifier 입력에서 `robot_state` 제거 — `CLASSIFIER_SYSTEM_PROMPT`가 `system_info`를 한 번도 참조하지 않는 죽은 토큰이었다 (`build_classifier_input(user_text)` 1-arg)
    - [x] `state_node`를 classifier 뒤·planner 앞에 배치 — planner 직전 fresh fetch가 cross-turn 복구의 핵심 메커니즘이다
    - [x] `direct_answer_node`로 status/identity/repertoire/ambiguous-followup/wave-play 직답을 한 곳(`build_direct_answer_plan`)에 모아 planner 우회
  - 완료 기준:
    - [x] `run_brain_turn` / `BrainTurnResult`가 런타임(`pipeline/`)에서 빠지고 `eval/brain_probe.py`로 이주됐다. eval·test import re-point 완료, `tests/test_planner_benchmark.py` 27건 통과.
    - [x] classifier 입력 JSON에 robot_state가 더 이상 없다.
    - [x] stub LLM/state로 graph e2e 흐름(prefilter/planner/direct_answer 3경로) 검증. 단, 실제 모델 위 eval smoke는 ollama 필요 → Jetson에서 최종 확인 필요.
  - 메모(eval 처리 방침): eval은 "돌아갈 정도"로만 맞추고 multi-turn 재설계는 3단계 이후로 미룸. 현 상태는 `eval/README.md`의 `현재 상태` 섹션에 상세 기록.

- [x] `decision graph refactor 2단계: planner↔validator repair 루프 (self-refuse에서 선회)`
  - 선회 경위: 처음엔 "planner가 범위/막힘을 prompt로 보고 스스로 거절(self-refuse)" 방향으로 갔으나, Jetson smoke 실측에서 (1) planner가 좋은 거절 speech를 내면서도 명령을 같이 내 validator가 그 speech를 generic으로 덮는 버그, (2) 무거운 범위표 prompt가 단순 wave/play를 흔드는 문제가 드러남. 그래서 사용자 원안인 **planner↔validator 내부 repair 루프**로 선회했다.
  - 확정 구조:
    - [x] planner prompt **슬림화** — 관절범위표/self-refuse/q 규칙 제거. planner는 검증 안 하고 계획만. (`JOINT_LIMITS` import 제거)
    - [x] `repair` planner 도메인 신설 (`PLANNER_DOMAIN_REPAIR`, skill 카테고리 set()). validator 거부 사유를 받아 설명/되묻기 speech만 생성. zero-shot(예시 문장 없음).
    - [x] validator = 안전망. 거부 시 speech를 덮지 않고 `RepairHint`(failure_code+reason+rejected) 반환. `block_reason_of`(state_adapter 공용)로 상태차단 코드, `_content_failure_code`로 내용오류 코드.
    - [x] graph 내부 루프: `planner → validator →(거부 & repair_attempt<MAX_REPAIR=2)→ planner(repair 도메인+hint)`. 통과하면 execute, 소진되면 `fallback` 노드(명령 버리고 `FALLBACK_MESSAGE`). 조건부 엣지 `route_after_validator`.
    - [x] repair_attempt(per-turn 로컬)와 recovery_count(per-thread, 3단계)는 독립 — repair는 명령 거부 시 같은 턴 핑퐁.
    - [x] `build_motion_block_message`/`build_partial_execution_message`/clobber 제거. q 기반 session pending 비활성화.
  - 검증:
    - [x] stub e2e 3경로: blocked→repair 1회 수렴, repair도 명령 고집→소진→fallback, 정상→repair 0회 실행.
    - [x] **실제 graph + 30b 구동**: 안전키→"안전 키를 뽑아 달라고 안내."(동작 정상, 문장은 약간 echo), 연주중→"현재 연주 중이라 손을 흔드는 동작은 할 수 없습니다."(완벽), 정상→gesture:wave 실행. repair 루프 런타임 동작 확인.
    - [x] R1 smoke: 흔들리던 wave_allowed/play_tim 안정화(prompt 슬림 효과).
  - 한계/메모:
    - smoke(`run_eval`)는 graph가 아니라 `brain_probe`(단일 패스)를 써서 **repair 루프를 못 탄다.** 따라서 blocked 케이스는 smoke에서 첫 시도 speech만 보여 "실패"로 뜨지만 런타임(graph)은 정상. 제대로 된 검증은 graph 구동 multi-turn scenario eval 필요(후속).
    - repair speech echo("~안내.")는 zero-shot 유지하며 남겨둠(경미).

- [x] `decision graph refactor 3단계: cross-turn recovery 스레드 + pending 의도 context`
  - 목표: 막힘/되묻기/범위안내를 다음 턴으로 잇는 cross-turn 복구를 한 종류로 통일하고, 무한 반복은 회차 캡으로 끊는다.
  - 체크리스트:
    - [x] `SessionContext.recovery_count`/`pending_intent`/`pending_classifier` 추가 (기존 `pending_user_text`/`pending_clarification_q` 교체). update_session: actionable인데 실행 명령 없음→미해결(count+1, pending 고정), 그 외→리셋.
    - [x] 회차 1~MAX_RECOVERY(=4)는 planner가 돈다. `run_turn` 진입에서 `recovery_count >= MAX_RECOVERY`면 planner 없이 deterministic giveup("죄송해요, 잘 이해하지 못했어요...") + 리셋(이 턴은 chat 으로 분류돼 update_session 이 리셋).
    - [x] `resolve_clarification_text` 문자열 결합 폐기 완료(Step3).
    - [x] continuation: pending 활성이면 classify 건너뛰고 `pending_classifier` 재사용 + `pending_domain` 고정, `pending_intent`를 session_summary로 planner에 전달해 이번 발화와 합쳐 해석.
    - [x] 취소어(취소/아니/됐어 등)는 pending 폐기 + 안내.
    - [x] missing-info: 첫 시도가 actionable인데 빈 계획이면 repair 도메인으로 "무엇을/몇 도?" 되묻기(repair 도메인 출력은 제외해 무한루프 방지).
  - 검증:
    - [x] stub 시퀀스: `허리 돌려`→"몇 도로?"(count1) → `30도`→`move:waist,30` 실행(리셋). giveup: 5턴째 deterministic 리셋. 취소: 폐기+ack. + 23 단위테스트 통과.
    - [x] **실제 30b 거부경로 full**: `허리 200도 돌려`→validator 거부→repair "다른 각도를?"(count1, pending 저장) → `30도`→continuation→`move:waist,30` "허리를 30도 돌릴게요"(리셋). cross-turn 복구 런타임 동작 확인.
  - 한계/메모 (추후 수정):
    - **bare 미정보 요청("허리 돌려", 각도 없음) — null 규칙으로 해소(확률적, 실측 3/3)**: planner prompt(PLANNER_SHARED_RULES)에 "미명시 파라미터는 지어내지 말고 null" 규칙을 넣자, 이전엔 기본각(예: waist 90)을 지어내던 30b가 빈 계획/`move:waist,null`을 낸다. 빈 계획은 missing-info 트리거가, null은 `validate_move_command`의 "목표 각도가 지정되지 않음" 거부(→`_content_failure_code`가 `missing_info`로 정규화)가 잡아 repair "몇 도로?"로 되묻는다. "허리 돌려" 3/3 모두 되묻기 확인(이전엔 지어내 실행). LLM이라 100% 보장은 아니며(가끔 지어낼 수 있음), 완전한 구조적 해결은 resolver 분리에서 null 슬롯을 1급으로 다루는 것.
    - **복구 중 화제 전환 미지원**: pending=motion 복구 중 "연주해줘" 같은 다른 명령이 와도 v1은 pending_domain 고정. validator가 의도-명령 일치를 검사하지 않아 깔끔히 못 거르고, 최악의 경우 몇 턴 돌다 giveup으로 리셋된다. 취소어로 즉시 폐기 가능. 정교한 화제전환 감지는 추후. (`robot_fsm.run_turn` continuation 분기에 동일 취지 주석)

## Next

- [ ] `scenario eval` 확장 (graph 구동 multi-turn) — 설계 확정, 구현 대기
  - 왜: 현재 `run_eval`(smoke)은 `brain_probe`(단일 패스)라 graph 의 repair 루프·cross-turn recovery 를 못 탄다. blocked/range/되묻기는 smoke 에서 "실패"로 떠도 런타임(graph)은 정상. 제대로 검증하려면 graph 를 턴마다 굴려야 한다.
  - 구동 방식(확정): `build_phil_graph(...)` app 을 turn 마다 `app.invoke({"user_text":...})`, 세션 하나 공유, robot_state 는 turn 별 주입(`get_state_fn`), bot/executor 는 fake, LLM 은 integration(실모델)/unit(stub) 두 모드. 상세는 `eval/README.md`의 `scenario eval 설계안` 섹션.
  - 케이스 포맷: `{id, turns:[{user, state(patch), expect}]}`. expect 후보 `commands_has/commands_empty/speech_has/repair_attempt/recovery_count/planner_domain`.
  - 새 파일(예정): `eval/run_scenario.py`, `eval/cases_scenario.json`. 기존 `run_eval.py`는 단일턴/benchmark 용 유지.
  - 커버 시나리오:
    - blocked → repair 메시지(명령 0) → 다음 턴 해제 → 실행 (cross-turn)
    - missing(각도 없음) → 되묻기 → 다음 턴 각도 → 실행
    - range(200도) → 범위 안내 되묻기 → 다음 턴 정상값 → 실행
    - giveup: 5턴 연속 미해결 → deterministic 리셋
    - happy-path: 정상 motion/play/chat/status (repair 0)
    - `안녕 하고 고개 끄덕여`
    - `그대에게 연주하고 끝나면 인사해` -> 현재는 pending task 미지원으로 명확히 미지원 또는 Later 항목 참조

- [ ] `planner 의미 해석 / resolver 계산 분리`
  - 목표: planner는 `absolute / delta / sequence` 의미만 정하고, resolver는 state snapshot 기준 수치 계산만 맡는다.
  - 이유:
    - `scenario_20`, `scenario_21`처럼 연속 상대동작에서 `move`를 다시 해석하다가 명령이 꼬이는 문제를 줄인다.
    - planner가 최종 각도까지 추측하기보다 관절, 방향, 단계 수, 대기 시간만 구조화해 넘기게 한다.
  - 구현 방향:
    - planner 출력에 `joint`, `mode`, `value`, `wait` 같은 중간 표현을 둔다.
    - resolver는 이를 누적 절대각 시퀀스로 변환한다.
    - validator는 최종 절대각 명령만 검증한다.
  - 진행 메모:
    - `2026-04-14` 기준으로 `motion_resolver.py`에 `N초 뒤에 N도 더`와 `N도씩 두번` 텍스트를 직접 읽는 다단계 상대이동 파서를 먼저 추가했다.
    - decision graph refactor 이후에는 resolver가 `missing_slots`와 `failure_code`를 더 명확히 넘기도록 조정한다.

- [ ] `classifier routing / shortcut` 보강
  - 목표: 짧은 social-motion 발화가 `chat`으로 빠지지 않게 classifier routing을 먼저 안정화한다.
  - 우선 대상:
    - `만세`
    - `손 흔들어`
    - `고개 끄덕여`
    - `고개 저어봐`
  - 메모:
    - `scenario_08`은 failure taxonomy보다 classifier가 `만세`를 `chat`으로 본 문제가 더 직접적인 원인으로 보인다.
    - 저모호도 motion/social command는 prefilter 또는 shortcut으로 먼저 잡는 쪽이 낫다.
    - 단, robot_state 판단은 classifier가 아니라 `state_gate`로 분리한다.

- [x] `planner latency isolation benchmark` 추가
  - 목표: classifier 영향 없이 planner 자체의 latency와 output variability를 따로 측정한다.
  - 조건:
    - 동일 `classifier_output`
    - 동일 `planner_input`
    - warm/cold 조건 분리 가능하면 기록
  - 기록 항목:
    - planner input JSON 길이
    - planner response 길이
    - avg / median / p95 latency
  - 메모:
    - `eval/run_planner_latency_isolation.py`를 추가해 JSON production planner path만 대상으로 같은 fixture 위에서 planner만 반복 측정하도록 구성했다.

- [ ] `state_adapter` 강화
  - raw state를 planner-friendly feature로 가공한다.
  - 후보 feature:
    - `currently_busy`
    - `can_accept_motion_command`
    - `can_accept_play_command`
    - `safety_lock`
    - `recoverable_error`
    - `recent_action_summary`
  - decision graph refactor 연계:
    - `state_gate`가 쓰는 `can_accept_motion_command`, `can_accept_play_command`, `block_reason`을 여기서 만들지 검토한다.

- [ ] `task planner + dialogue planner` 분리
  - 현재 domain planner가 동시에 다루는 `행동 계획`과 `말하기 계획`을 분리한다.
  - 목표 흐름:
    - `intent_classifier`
    - `task_planner`
    - `dialogue_planner`
    - `validator`
    - `executor`
  - 최신 방향:
    - 먼저 `clarify/repair` node로 실패 문장 생성을 분리한 뒤, 필요하면 별도 모델 분리로 확장한다.

- [ ] `approval / clarification checkpoint` 1급 시민화
  - 목표: 위험하거나 모호한 명령은 일반 실패가 아니라 명시적 승인/되묻기 단계로 승격한다.
  - 예:
    - `허리 돌려` -> `몇 도로 움직일까요?`
    - `연주해줘` -> `어떤 곡을 연주할까요?`
    - `종료해` -> 필요 시 확인 후 실행
  - 구현 방향:
    - validator가 `missing_info` / `confirmation_required` 상태를 반환
    - graph의 `clarify` / `confirm` node가 질문 문장을 만든다.
    - 다음 턴에서 pending action을 이어서 처리
  - 메모:
    - normal planner의 `q` 생성 책임은 이 항목으로 흡수한다.

- [ ] `session memory` 초안
  - 목표: 바로 직전 문맥을 기억해 `거기서 더`, `아까처럼`, `그 노래` 같은 지시를 안정적으로 처리한다.
  - 범위:
    - 직전 intent
    - 직전 `planner_output`
    - 직전 confirmed target
    - 직전 clarification state
  - 현재 구현:
    - `pending_user_text`와 `pending_clarification_q`로 되묻기 다음 턴을 deterministic하게 이어붙인다.
  - 확장 방향:
    - `pending_task`는 session에 넣을 수 있지만, 상태 감시와 실행은 별도 scheduler가 맡게 한다.

- [x] `planner model benchmark` 설계
  - 목표: 여러 planner 모델을 같은 조건에서 비교해 planner 후보를 고른다.
  - 비교 조건:
    - 동일 `cases`
    - 동일 `classifier_output` 또는 동일 `planner_input`
    - 동일 validator / executor 규칙
  - 비교 항목:
    - planner pass rate
    - skill selection quality
    - valid command quality
    - speech quality
    - avg / median / p95 planner latency
  - 메모:
    - `eval/run_planner_benchmark.py`를 JSON-only fixed fixture 방식으로 바꿔, classifier 를 케이스당 한 번만 실행하고 모든 planner 모델을 같은 `planner_input` 위에서 비교하도록 정리했다.

- [ ] `classifier prefilter` 도입
  - 목표: 자주 나오는 저모호도 발화를 rule-based shortcut으로 먼저 처리해 classifier latency를 줄인다.
  - 대상 예:
    - 인사
    - 이름/정체 질문
    - 관절 각도 질문
    - 단순 상태 질의

- [ ] `clarification flow` 설계
  - 목표: `허리 돌려`, `연주해줘`, `팔 올려` 같은 모호한 요청에 대해 부족한 슬롯별 질문을 생성한다.
  - 원칙:
    - 저위험 상대이동은 기본 step 허용 가능
    - 중/고위험 모호 명령은 clarification 우선
    - 질문 문장은 planner prompt의 `q`가 아니라 `clarify_node`가 생성한다.
  - missing slot 후보:
    - `song_code`: 어떤 곡을 연주할지 모름
    - `joint_angle`: 절대 목표각이 없음
    - `relative_direction`: 올릴지 내릴지 불명확
    - `target_side`: 왼쪽/오른쪽/양쪽 대상이 불명확
    - `duration_or_repeat`: 순차 동작의 시간/반복 수가 불명확

- [ ] `skill abstraction` 강화
  - 목표: LLM이 저수준 `move:<motor>,<angle>`를 직접 만드는 비율을 더 낮춘다.
  - 방향:
    - `greet_user`
    - `wave_hi`
    - `nod_yes`
    - `shake_no`
    - `celebrate`
    - `arm_up`
    - `arm_down`
    - `start_play_song(TIM)`
    - `explain_joint_limit`
    - `look_at_user`
    - `return_home`
  - 원칙:
    - 이미 `AgentAction` 기본 제어 함수로 안정적으로 수행 가능한 긴 시퀀스부터 먼저 위 계층으로 올린다

## Later

- [ ] `planner model search`
  - 조건: planner 구조를 먼저 안정화한 뒤 진행
  - 주의: classifier나 planner 구조가 흔들리는 동안의 모델 비교는 해석이 어렵다.

- [ ] `URDF 기반 SIL` 구축
  - 목표: 실제 로봇을 덜 건드리고 scenario test를 반복 가능한 환경에서 돌린다.
  - 기대 효과:
    - 연속 시나리오 테스트
    - 복구 정책 검증
    - 저수준 command 안전성 확인

- [ ] `recovery / resume flow` 설계
  - 목표: 중간 실패 후 안전 상태로 돌아가거나, 가능한 경우 작업을 재개한다.
  - 예:
    - joint limit 차단 후 대안 제시
    - 실행 중 실패 후 `home` 복귀
    - recoverable error 해소 후 pending task 재개

- [ ] `TaskScheduler / pending task runtime` 설계
  - 목표: `연주 끝나면 인사해`처럼 한 턴 밖에서 robot state 또는 시간 조건을 기다리는 작업을 관리한다.
  - 현재 상태:
    - 아직 구현 없음.
    - `SessionContext`에는 clarification pending만 있으며, 상태 조건 기반 pending task는 TODO 주석 수준이다.
    - 현재 graph는 `app.invoke()` 1회로 끝나는 per-turn graph이므로 장시간 상태 감시에 적합하지 않다.
  - 권장 구조:
    - `LangGraph`: 이번 턴 판단, 즉시 실행 commands 생성, pending task 등록.
    - `TaskScheduler`: graph 밖에서 계속 robot_state snapshot을 감시.
    - `Executor`: scheduler가 넘긴 commands를 단순 실행.
  - pending state 후보:
    - `task_id`: 중복 실행 방지용 id.
    - `source_user_text`: 원래 사용자 발화.
    - `trigger`: `after_motion`, `after_play`, `after_delay`, `when_idle` 등.
    - `plan_type`, `skills`, `commands`, `speech`: 실행 후보.
    - `created_at`, `expires_at`, `consumed`: 만료와 1회 실행 제어.
    - `reason`: 왜 즉시 실행하지 않고 pending으로 보냈는지.
  - pending task 예:
    - `trigger=after_play`, `skills=["wave_hi"]`
    - `trigger=after_motion`, `skills=["idle_home"]`
    - `trigger=after_delay`, `delay_sec=30`, `skills=[...]`
    - `오른손 올리는 중 "오른손 내려"` 같은 후속 motion은 정책 결정 전에는 즉시 큐잉하지 않고 명확히 미지원 또는 차단 응답을 우선한다.
  - 상태 감시 예:
    - `state=2` 연주 상태를 한 번 관찰한 뒤 `state=0` 또는 idle로 돌아오면 `after_play` 실행.
    - `is_fixed=false` 움직임 상태를 한 번 관찰한 뒤 `is_fixed=true`로 돌아오면 `after_motion` 실행.
    - 같은 task가 반복 실행되지 않도록 task id와 consumed flag를 둔다.
  - 주의:
    - scheduler도 명령 후보를 직접 실행하지 말고 validator를 다시 통과시킨다.
    - 안전키/에러/연주 중 motion 제한은 scheduler 실행 시점에도 다시 확인한다.
    - Executor를 상시 worker로 키우는 것보다 scheduler와 executor를 분리하는 편이 응집도가 좋다.

- [ ] `fallback planner` / `recovery planner` 분리 검토
  - 목표: 기본 planner 실패나 실행 실패 시 별도 계획기로 대응한다.
  - fallback planner:
    - planner JSON 실패
    - skill 선택 실패
    - domain 불확실
  - recovery planner:
    - 실행 실패 후 다음 안전 행동 결정
    - 사용자 설명 문구와 복구 행동 분리

- [ ] `perception` 결합 지점 설계
  - 목표: 향후 비전/센서 인식 결과를 planner-friendly feature로 넣을 수 있게 한다.
  - 예:
    - 사용자가 어느 방향에 있는지
    - 현재 타겟이 감지되는지
    - 시선/팔 동작의 실제 성공 여부

- [ ] `TAU-style task benchmark` 설계
  - 그대로 원본을 들여오기보다, 로봇 제어 태스크 중심으로 adaptation 한다.
  - 지표 후보:
    - task completion rate
    - safety violation rate
    - recovery success rate
    - multi-turn consistency

- [ ] `deterministic status shortcut` 확장
  - 현재 일부 관절 각도 질문 외에도 직접 답할 수 있는 상태 질의를 늘린다.
  - 후보:
    - 현재 곡
    - 현재 BPM
    - 직전 행동
    - 에러 상태 / 에러 원인

- [x] `LangGraph-style state graph` 도입 여부 검토
  - 목표: 장기적으로 pause/resume, confirmation, recovery, multi-step routing을 상태 그래프로 관리할지 판단한다.
  - 메모:
    - `2026-04-16` 기준으로 LangGraph 스타일 상태 기계 도입 완료.
    - Python 3.8 + aarch64 제약으로 langgraph 패키지 대신 `pipeline/state_graph.py` 경량 호환 구현 사용. API 동일.
    - 현재 `pipeline/robot_graph.py`는 `process -> execute -> END` 구성이다.
    - `pipeline/exec_thread.py`에 `Executor.execute()` 구현 (cancel 시 stop_event 설정으로 wait 및 미전송 명령 즉시 중단, 로봇 전송 없음).
    - 동작 완료 후 홈 복귀(`h`)는 graph node가 아니라 `Executor`의 `on_done(cancelled=False)`에서 `plan_type == "motion"`일 때 `home()`을 호출해 처리한다.
    - Enter 누름 시 이전 동작 즉시 중단, TTS는 `app.invoke()` 반환 후 메인 스레드에서 호출.
    - 현재 graph는 장기 task scheduler가 아니라 한 사용자 턴의 `PhilState`를 노드 사이 전달하는 per-turn graph다.
    - 상세 설계: `docs/LANGGRAPH_STATE_MACHINE_KR.md`

- [ ] `memory / RAG` 장기 구조 검토
  - 목표: 제어 path와 분리된 knowledge path를 설계한다.
  - memory:
    - session memory
    - long-term memory
  - RAG:
    - 매뉴얼/에러문서/레퍼토리 설명 retrieval
  - 원칙:
    - safety-critical control은 code/validator 중심 유지
    - RAG는 knowledge augmentation 용도로만 사용

- [ ] `multi-agent / supervisor` 필요성 재평가
  - 목표: 현재 단일 pipeline으로 충분한지, 장기적으로 supervisor가 필요한지 판단한다.
  - 후보 구조:
    - intent agent
    - task planner agent
    - dialogue agent
    - supervisor / coordinator
  - 비고:
    - 현재 시점에는 과도할 수 있으므로 실제 병목이 보일 때만 추진

- [ ] `legacy C++ control revival review`
  - 목표: 예전 C++ 제어 기능 중 재활용 가능한 범위와 폐기할 범위를 다시 나눈다.
  - 재검토 대상:
    - `DrumRobot.cpp`의 magenta mode
    - sync 맞추기 기능
    - `TestManager`
  - 판단 기준:
    - blocking behavior가 현재 Python/LLM 파이프라인과 충돌하는지
    - low-level primitive / timing logic으로 재사용 가능한지
    - legacy orchestration 자체는 버리고 test harness 또는 하위 제어기로만 남길지
  - 권장 시점:
    - scenario eval
    - recovery / resume flow
    - URDF 기반 SIL
    이후에 다시 검토

## Parking Lot

- [ ] `DrumRobot2 박자 단위 pause/resume` (파일 단위 재개의 다음 단계)
  - 현재: 파일 처음부터 재연주. 더 정밀한 재개를 원하면 버퍼에 파일 경계 마커를 심어야 함
  - 방향: `TMotorData`에 마커 필드 추가 → send 스레드가 마커 소비 시 임시 파일에 위치 기록 → pause 시 읽기
- [~] `wait 명령 제거` 완료 / `시간·조건 지연은 TaskScheduler로 이관`은 추후
  - 완료: executor에서 wait 완전 제거(`wait:<seconds>` → validator가 unknown 거부). cancel/stop_event/cancelled/`_interruptible_wait`도 함께 제거(끊을 wait가 없으므로). planner/skills/motion_resolver(`wait_then_more` 파서)에서 wait 생성 끊음. "N도씩 두번"(repeat)은 생존, "N초 뒤에"만 사라짐.
  - 추후: "몇 초 뒤에 ~해" 같은 시간/조건 지연 실행의 정식 자리는 `TaskScheduler / pending task runtime`의 `after_delay` trigger — "연주 끝나고 인사해"(`after_play`)와 같은 부류. 즉시 실행=executor, 지연 실행=scheduler. (Later `TaskScheduler` 항목과 연계)
- [ ] `failure taxonomy` 정의
  - 메모:
    - 지금은 `raw_op_cmds / resolved_op_cmds / valid_op_cmds`만으로도 1차 원인 분리가 가능하다.
    - 당장은 taxonomy 확장보다 `planner 의미 해석 / resolver 계산 분리`와 `classifier routing` 안정화가 우선으로 보인다.
- [ ] `task planner`와 `dialogue planner`를 서로 다른 모델로 분리할지 검토
- [ ] TTS 숫자 읽기 정책 세분화
  - 각도
  - 시간
  - BPM
  - 퍼센트
- [ ] `Executor` 상시 worker 전환 필요성 검토
  - 현재: 명령 시퀀스마다 daemon thread를 만들고, 끝나면 thread가 종료된다.
  - 당장 유지 이유:
    - 단순 `move -> wait -> move` 시퀀스는 현재 구조로 충분하다.
    - STT/LLM/TTS 지연에 비해 thread 생성 비용은 작다.
  - 전환 조건:
    - command queue, 우선순위, 예약 명령, 장기 pending task 실행 충돌 조정이 필요해질 때 검토한다.
  - `wait` 위치 메모:
    - 현재 `wait:<seconds>`는 `valid_op_cmds`에 섞여 있지만, 실제로는 로봇 TCP 명령이 아니라 Python 실행 지연 지시어다.
    - 단기적으로는 유지한다. 장기적으로는 `ExecutionStep` 같은 실행 계획 표현으로 `send command`와 `wait`를 분리할지 검토한다.
    - `5초 뒤에 실행` 같은 예약 대기는 Executor가 아니라 `TaskScheduler / pending task runtime` 쪽 책임으로 둔다.
  - 주의:
    - `command_executor.py`와 `executor.py`는 미사용 레이어로 삭제된 상태이므로 되살리지 않는다.
- [ ] import fallback 정리
  - 장기적으로 실행 방식을 하나로 통일할지 검토
- [ ] `ValidatedPlan` / `BrainTurnResult` 시각화용 디버그 출력 정리
- [ ] `unused import` / 구조 정리 리팩토링

## Done

- [x] `DrumRobot2 파일 단위 pause/resume` 구현
  - `runPlayProcess()` 파일별 처리 블록 안에 실행 완료 대기 루프 추가
  - pause 시 `play_file_index`가 현재 파일을 가리킴 → resume 시 해당 파일 처음부터 재연주
  - 박자 단위 재개는 Parking Lot 참조
- [x] `gesture / play abstraction uplift`
  - 목표: 긴 `op_cmd` 시퀀스를 위 계층 skill/gesture로 끌어올리고, `AgentAction`은 저수준 실행 primitive로 고정한다.
  - 1차 범위:
    - 기존 `wave / nod / shake / hurray` 계열을 상위 skill 카탈로그 기준으로 다시 정리
    - `arm_up`, `arm_down` 같은 팔 동작 gesture 추가
    - `tempo:fast|slow`, `strength:strong|soft`를 다음 연주용 pre-play modifier로 설계
  - 원칙:
    - planner는 긴 `move:` 나열보다 skill / symbolic command를 우선 사용
    - calibration과 실제 joint 시퀀스는 C++ `AgentAction` 쪽에 둔다
    - 연주 speed / intensity modifier는 `readMeasure()`에서 적용한다
  - 메모:
    - 현재 기준으로 baseline uplift와 pre-play modifier/readMeasure 적용 경로까지는 사실상 정리 완료로 본다.
    - 이후 확장 기준과 계층 재정리는 `phil_robot/docs/DECISION_LAYER_ROADMAP_KR.md`를 중심으로 따라간다.
- [x] 자유형 문자열 출력에서 `JSON-only` 출력 계약으로 전환
- [x] parser fallback 추가
- [x] classifier / planner 2단계 분리
- [x] domain-specific planner 도입
- [x] `ValidatedPlan` 기반 validator / executor 분리
- [x] `skill-first` planning 도입
- [x] `relative motion resolver` 추가
- [x] 일부 상태 질의에 대한 deterministic direct answer 도입
- [x] `smoke` multi-layer eval 구축
- [x] classifier benchmark 수행 및 `qwen3:4b-instruct-2507-q4_K_M` 채택
- [x] eval report 자동 파일명 규칙 도입
- [x] `phil_robot` 폴더 구조 정리

## Active Notes

- 현재 classifier:
  - `qwen3:4b-instruct-2507-q4_K_M`
- 현재 planner:
  - `qwen3:30b-a3b-instruct-2507-q4_K_M`
- 현재 기본 smoke suite:
  - `eval/cases_smoke.json`
- 최신 규칙 기반 자동 리포트 파일명:
  - `<suite>_report_<classifier약어>_<planner약어>_<YYYYMMDD_HHMM>.json`
- 현재 런타임 구조 (Now 1~3단계 + simplify pass 완료 후):
  - langgraph/StateGraph는 폐기. `pipeline/robot_fsm.py`의 `run_turn`(imperative)이 한 턴을 처리:
    `preprocess → classify → state → direct_answer → (planner⇄validator repair 루프) → execute`.
  - `brain_pipeline.py` = 각 step의 실제 로직(엔진), `robot_fsm.py` = 그 step들을 한 턴 FSM으로 엮음.
    eval(`brain_probe.run_brain_turn`)도 같은 step을 호출(엔진 하나, 입구 둘).
  - 턴 **내부** repair 루프: planner가 명령을 내면 validator가 검증, 거부 시 사유(RepairHint)를
    repair 도메인 planner로 돌려 재생성. `MAX_REPAIR=2`, 소진 시 fallback. (validator=안전망, speech override 없음)
  - 턴 **사이** recovery: session의 `recovery_count`/`pending_intent`. `MAX_RECOVERY=4` 넘으면 giveup.
    continuation은 classify 건너뛰고 pending 도메인 이어감. 취소어 폐기.
  - motion 홈 복귀는 `robot_fsm.home()`(execute의 on_done에서 데몬 스레드).
  - `exec_thread`는 wait/cancel 없이 "보내고 on_done()"만. wait 명령 자체가 제거됨.
  - 장기 조건부 작업(연주 끝나고 등)은 향후 `TaskScheduler` 계층.
- 미해결 한계(코드 주석/3단계 메모 참조):
  - planner가 "막힘/정보없음에 빈 계획" 지시를 항상 지키진 않음. 거부 가능한 위반(범위초과/상태차단)은 repair로 잡고,
    bare "허리 돌려"(각도 미지정)는 null 규칙(미명시→null/빈 계획)으로 대부분 되묻기로 유도(실측 3/3). LLM이라 가끔 지어낼 수 있어 완전 보장은 resolver 분리(null 슬롯 1급)에서.
  - eval `run_eval`은 graph가 아니라 `brain_probe`(단일 패스)라 repair/recovery 루프 미검증. graph(run_turn) 구동 scenario eval 필요(Next).
