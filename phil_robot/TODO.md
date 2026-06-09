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

- [ ] `decision graph refactor 1단계: state gate + validator status contract`
  - 목표: planner가 헛도는 상황을 planner 호출 전에 막고, validator가 사용자 문장을 override 하는 대신 구조화된 실패 상태를 반환하게 한다.
  - 배경:
    - 현재 `planner.py`의 `q` 규칙은 각도 없는 관절 이동처럼 일부 빈 슬롯만 planner prompt 안에서 처리한다.
    - `연주해줘`처럼 곡 슬롯이 비었거나, 안전키 미해제처럼 상태가 막는 경우까지 prompt 규칙으로 계속 늘리면 planner가 계획/질문/실패 설명을 모두 떠안게 된다.
    - 장기 목표는 `classifier -> state_gate -> planner -> validator -> graph routing` 으로 책임을 나누는 것이다.
  - 상태 모델 초안:
    - `ok`: 실행 가능. `valid_op_cmds`가 있거나 정상 chat/status speech가 있다.
    - `missing_info`: 사용자의 추가 입력이 필요하다. 예: `song_code`, `waist_angle`, `target_side`.
    - `blocked_state`: 현재 robot state 때문에 어떤 motion/play 후보도 실행하면 안 된다. 예: safety key, error state, busy/play state.
    - `recoverable_rejected`: planner 후보가 잘못됐지만 재계획하면 고칠 수 있다. 예: joint limit 초과, unknown song code, unknown skill/command.
    - `partial`: 일부 명령만 통과했다. 실행할지, 줄일지, 되물을지 graph 정책이 필요하다.
    - `empty`: speech도 command도 없는 진짜 빈 결과.
  - 구현 방향:
    - `ValidatedPlan`에 `plan_status`, `failure_code`, `missing_slots`, `repairable` 필드를 추가한다.
    - `validator.py`는 `speech`를 덮어쓰기보다 `rejected_op_cmds`, `warnings`, `failure_code`를 채운다.
    - `command_validator.py`의 문자열 warning만으로 판단하지 않도록 실패 reason을 code로 정규화한다.
    - `state_gate.py`를 새로 두거나 `robot_graph.py`의 process 단계 직후 helper로 시작한다.
    - `state_gate`는 classifier 결과와 최신 `robot_state`를 보고 hard block을 planner 호출 전에 반환한다.
  - hard block 예:
    - `motion_request`인데 `is_lock_key_removed=false` -> `blocked_state / safety_key`
    - `motion_request`인데 `state=2` -> `blocked_state / playing`
    - `motion_request`인데 `is_fixed=false` -> `blocked_state / moving`
    - `play_request`인데 error state -> `blocked_state / error_state`
  - 완료 기준:
    - hard block 상황에서 planner 호출 횟수가 0회임을 debug log 또는 eval fixture로 확인한다.
    - 기존 안전 차단 동작은 유지하되 사용자 문장 생성은 graph 후속 node가 맡을 수 있게 데이터가 구조화된다.

- [ ] `decision graph refactor 2단계: clarify / repair node 도입`
  - 목표: `speech/commands`가 비었거나 rejected 된 결과를 LangGraph 안에서 라우팅하고, normal planner prompt의 `q` 책임을 graph node로 이동한다.
  - 현재 인식:
    - 현재 `robot_graph.py`는 `process -> execute -> END`만 수행하는 얇은 per-turn wrapper에 가깝다.
    - 진짜 decision graph가 되려면 한 턴 안에서 `missing_info`, `blocked_state`, `confirmation_required`, `recoverable_rejected` 같은 상태를 명시적으로 분기해야 한다.
    - 동작 완료 후 홈 복귀는 graph node가 아니라 `Executor`의 `on_done(cancelled=False)`에서 `plan_type == "motion"`일 때 `home()`을 호출하는 경로가 담당한다.
  - graph 초안:
    - `process`: classifier, state gate, planner, validator를 실행해 `ValidatedPlan`을 만든다.
    - `route_after_process`: `plan_status`를 보고 다음 node를 고른다.
    - `clarify`: `missing_slots`를 보고 질문 문장과 `clarification_question`을 만든다.
    - `confirm`: 위험하거나 중요한 실행 후보에 대해 사용자 승인을 묻는다.
    - `repair`: recoverable failure에 한해 planner를 제한 횟수로 다시 호출한다.
    - `execute_now`: `valid_op_cmds`를 `Executor`에 넘긴다.
  - routing 규칙:
    - `ok` -> `execute_now` 또는 `end`
    - `missing_info` -> `clarify`
    - `blocked_state` -> deterministic fallback speech 후 `end`
    - `confirmation_required` -> `confirm`
    - `recoverable_rejected` -> `repair`
    - `partial` -> 우선은 deterministic partial fallback, 이후 정책 확장
    - `empty` -> `repair` 1회, 실패 시 fallback speech
    - `pending_candidate` -> 현재는 명확히 미지원 응답. `TaskScheduler / pending task runtime` 설계 이후에만 `enqueue_pending`으로 확장한다.
  - clarify node 원칙:
    - full planner를 그대로 다시 부르지 말고 `question generation mode`로 제한한다.
    - 입력은 `user_text`, `planner_domain`, `missing_slots`, `robot_state`, `session_summary`로 제한한다.
    - 출력은 `speech`와 `clarification_question`만 허용하고 `skills/op_cmd`는 항상 비운다.
    - 기존 `SessionContext.pending_user_text/pending_clarification_q` 경로를 그대로 재사용한다.
  - confirm node 원칙:
    - 승인 대기 상태는 session에 저장하되, 실행 전에는 반드시 최신 `robot_state`로 validator를 다시 통과시킨다.
    - `멈춰`, `취소`, `아니` 같은 응답은 pending approval을 폐기한다.
  - repair node 원칙:
    - `max_repair_attempts=1`부터 시작한다.
    - repair가 만든 후보도 반드시 다시 `build_validated_plan()` 또는 동등한 validator 경로를 통과한다.
    - 같은 `failure_code`가 반복되면 즉시 fallback으로 종료한다.
    - `blocked_state`와 `missing_info`는 repair 대상이 아니다.
  - 완료 기준:
    - `허리 돌려` -> clarify node가 질문 생성, command 미전송.
    - `연주해줘` -> clarify node가 곡 선택 질문 생성, 임의 곡 선택 금지.
    - 안전키 미해제 motion -> planner 호출 없이 fallback speech, command 미전송.
    - joint limit 초과 -> repair 1회 또는 fallback으로 종료, 무한 루프 없음.

- [ ] `planner prompt slim-down: q 제거와 plan 후보 생성 전용화`
  - 목표: planner가 되묻기 문장 생성까지 담당하지 않도록 하고, plan 후보 생성에 집중하게 한다.
  - 변경 방향:
    - `PLANNER_SHARED_RULES`의 `q` 필드 요구를 제거하거나 deprecated로 낮춘다.
    - normal planner 출력은 `skills`, `op_cmd`, `speech`, `reason` 중심으로 유지한다.
    - speech는 정상 chat/status/play 안내에는 허용하되, 실패/차단/되묻기 override는 graph node로 옮긴다.
    - planner가 필수 슬롯을 모르면 임의로 채우지 말고 빈 plan 또는 structured hint를 남기게 한다.
  - 스키마 후보:
    - 1단계 호환: 기존 `q/clarification_question` parser는 유지하되 prompt에서 적극 요구하지 않는다.
    - 2단계 정리: `clarification_question`은 `ValidatedPlan`/session에는 남기고 planner raw output에서는 제거한다.
  - 유의점:
    - 기존 eval이 `clarification_question`을 기대하는 경우 fixture를 같이 갱신한다.
    - `planner`가 빈 plan을 냈을 때 실패인지 정상 chat인지 validator/graph가 구분할 수 있어야 한다.
    - 안전 판단은 planner가 아니라 `state_gate`와 validator가 최종 책임진다.

## Next

- [ ] `scenario eval` 확장
  - 목표: smoke(기본 확인)에서 벗어나 decision graph refactor를 검증할 복합 시나리오를 추가한다.
  - 범위:
    - 정상 명령
    - missing slot clarification
    - blocked state
    - recoverable rejected plan
    - 연속 대화 명령
  - 우선 케이스:
    - `허리 돌려` -> 질문만 생성, 명령 없음
    - `30도` -> pending clarification과 합쳐져 `허리 돌려 30도`로 재계획
    - `연주해줘` -> 곡 선택 질문
    - 안전키 미해제 상태에서 `손 들어줘` -> planner 호출 없이 차단 응답
    - `허리 200도 돌려` -> limit failure code 확인
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
- 현재 LangGraph 역할:
  - `robot_graph.py`는 장기 상태 관리자라기보다 per-turn routing graph다.
  - 현재 노드는 `process -> execute -> END`이며, 진짜 decision graph 분기는 아직 TODO다.
  - motion 홈 복귀는 `return_home` node가 아니라 `Executor.on_done -> home()` 경로에서 처리한다.
  - 장기 조건부 작업은 향후 `TaskScheduler` 계층에서 다룬다.
- 현재 clarification:
  - session은 `pending_user_text + next user_text`를 deterministic하게 합친다.
  - 향후 질문 문장 생성 책임은 planner prompt의 `q`에서 `clarify_node`로 옮긴다.
