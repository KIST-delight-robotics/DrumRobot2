# Phil Robot 결정 계층 로드맵

## 왜 이 문서가 필요한가

`modifier layer`를 이야기할 때 가장 자주 헷갈리는 지점은
`이 계층이 규칙 기반인가, ML인가, LLM인가`를 먼저 떠올리게 된다는 점이다.

하지만 실제로 더 중요한 질문은 그보다 앞선다.

- 이 계층은 무엇을 결정하는가
- 이 결정은 어디까지 책임지는가
- 어떤 입력을 받아 어떤 출력 계약으로 넘기는가

즉 `계층(layer)`은 먼저 `결정 경계(decision boundary)`이고,
그 뒤에야 그 안의 구현이 규칙 기반일지, ML/DL일지, LLM일지가 정해진다.

이 문서는 현재 `phil_robot` 제어 파이프라인을
`기술 스택`이 아니라 `결정 계층` 기준으로 다시 정리하고,
향후 어디에 ML/LLM을 붙일 수 있는지 로드맵 형태로 남겨 두기 위한 문서다.

## 핵심 요약

- `interrupt gate`는 고우선 즉시 제어를 결정한다.
- `intent classifier`는 사용자의 상위 의도를 분류한다.
- `planner`는 무엇을 할지 정한다.
- `play modifier resolver`는 어떻게 연주할지 정한다.
- `validator`는 그 결정을 실제로 허용할지 막을지 판단한다.
- `executor`는 transport command로 직렬화한다.
- C++ `DrumRobot`은 실제 적용 시점과 적용 방식을 책임진다.

중요한 점은 `play modifier resolver`가
`LLM layer`나 `ML layer`를 뜻하는 것이 아니라,
`연주 방식 보강을 최종 결정하는 계층`이라는 점이다.

즉 같은 resolver라도 시기에 따라 다음처럼 구현이 달라질 수 있다.

- 지금: explicit keyword 규칙 기반
- 다음: context / history / memory 기반 deterministic 보강
- 이후: ML 분류/회귀 보조
- 필요 시: LLM 추론 fallback

## 현재 추천 결정 계층

### 1. Interrupt Gate

역할:
- `멈춰`, `정지`, `스톱`, `잠깐` 같은 고우선 제어를 바로 잡는다.

입력:
- `user_text`
- `robot_state`

출력:
- 즉시 stop/pause command 또는 `pass`

현재 추천:
- 규칙 기반 fast path

이유:
- 지연이 짧아야 한다.
- 잘못 해석하면 위험하다.
- LLM보다 deterministic 우선순위가 중요하다.

향후 확장:
- synonym 확장
- play 중에도 통과되는 별도 gate
- 필요하면 작은 classifier 보조

### 2. Intent Classifier

역할:
- 사용자의 상위 intent를 분류한다.

예:
- `chat`
- `motion_request`
- `play_request`
- `status_question`
- `stop_request`

입력:
- `user_text`
- 요약된 `robot_state`

출력:
- intent label
- optional confidence

현재 상태:
- LLM classifier

향후 추천:
- 닫힌 집합 분류 문제이므로 ML/DL 후보가 가장 강하다.

후보:
- rules + keyword baseline
- embedding + logistic regression
- small encoder classifier
- low-confidence 시 LLM fallback

판단:
- 이 계층은 ML/DL을 가장 먼저 붙여 보기 좋은 자리다.

### 3. Planner

역할:
- 무엇을 할지 결정한다.

예:
- 어떤 곡을 틀지
- 어떤 skill을 쓸지
- 어떤 응답 문장을 말할지

입력:
- `user_text`
- `classifier_result`
- adapted state

출력:
- planner result
- skill / symbolic command
- speech

현재 상태:
- domain-specific LLM planner

향후 추천:
- planner는 여전히 LLM 친화적이다.
- 단, low-level `move:` 나열보다 skill / symbolic command 우선 원칙은 유지한다.

판단:
- planner는 `무엇을 할지`를 정하고,
- modifier resolver는 `어떻게 할지`를 정한다.

둘은 같은 축이 아니다.

### 4. Play Modifier Resolver

역할:
- 같은 `p:TIM`이라도 어떤 연주 느낌으로 칠지 결정한다.

예:
- 빠르게
- 느리게
- 세게
- 약하게
- 나중에는 `답답하지 않게`, `아까처럼`, `조금 더 세게`

입력 후보:
- `user_text`
- `classifier_result`
- `planner_result`
- expanded `op_cmds`
- `robot_state`
- `turn_history`
- `memory`

출력:
- `PlayModifier | None`

현재 상태:
- explicit keyword parser
- `tempo_scale`
- `velocity_delta`

현재 구현 해석:
- `parse_play_modifier(user_text)`는 parser다.
- 이 함수는 문장에서 명시 표현을 읽는다.
- 최종 적용 여부는 validator가 gate를 건다.

중요한 구분:
- parser: 문장에서 보이는 modifier를 읽는 단계
- resolver: 여러 근거를 합쳐 최종 modifier를 결정하는 단계

즉 resolver는 기술 종류가 아니라 책임 이름이다.

향후 진화 방향:

1. Phase 1: Explicit Rule-Based
- `"빠르게"`, `"세게"` 같은 명시 표현만 처리

2. Phase 2: Context / Memory Resolver
- `"아까처럼"`
- `"조금 더 세게"`
- `"답답한데"`

3. Phase 3: ML 보조
- 분류: `tempo_up / tempo_down / strength_up / strength_down / none`
- 회귀: `tempo_scale`, `velocity_delta` 직접 예측

4. Phase 4: LLM Fallback
- 애매한 표현이나 긴 문맥이 필요한 경우만 추론 사용

추천 원칙:
- explicit > memory > inferred > default

즉 사용자가 직접 말한 modifier가 항상 최우선이고,
추론이나 기억이 explicit를 덮어쓰면 안 된다.

### 5. Validator

역할:
- plan을 실제 실행 가능한 형태로 검증한다.

현재 역할:
- skill expansion
- motion resolution
- command validation
- play modifier gate

현재 modifier gate:
- `classifier_result.intent == "play_request"`
- 실제 valid command 안에 `p:`가 살아남음
- parsed modifier가 identity가 아님

판단:
- validator는 modifier를 추론하는 곳이 아니라
- modifier를 이번 턴에 실어도 되는지 결정하는 곳이다.

향후에도 이 계층은 가능하면 deterministic하게 유지한다.

### 6. Executor

역할:
- validated plan을 transport command로 직렬화한다.

현재 역할:
- `requested_op_cmds`
- `requested_transport_cmds`
- `executed_transport_cmds`

modifier 관련 역할:
- `PlayModifier` 객체를 직접 보내지 않는다.
- transport command로 serialize해서 보낸다.

예:
- `tempo_scale:1.10`
- `velocity_delta:1`
- `p:TIM`

판단:
- executor는 의미 해석 계층이 아니라
- 직렬화와 송신 순서 계층이다.

### 7. C++ DrumRobot Apply Layer

역할:
- 실제 연주 시 modifier를 언제 어떻게 적용할지 결정한다.

현재 목표:
- `processInput()`에서 pending play modifier 저장
- `readMeasure()`에서 `bpm`과 velocity에 적용

판단:
- calibration과 실제 joint sequence는 C++ 쪽에 둔다.
- 연주 speed / intensity modifier도 최종 적용은 C++ `readMeasure()`가 맡는다.

이 계층은 끝까지 deterministic하게 유지하는 것이 좋다.

## 현재 추천 아키텍처

```text
user_text
  -> interrupt gate
  -> intent classifier
  -> planner
  -> play modifier resolver
  -> validator
  -> executor
  -> C++ apply
```

기술 선택으로 다시 쓰면 다음과 같이 섞일 수 있다.

```text
rule-based interrupt
  -> ML or LLM classifier
  -> LLM planner
  -> rule-based / ML / LLM-assisted modifier resolver
  -> deterministic validator
  -> deterministic executor
  -> deterministic C++ apply
```

중요한 점은 여기서도 계층 정의는 그대로고,
계층 내부 구현만 바뀐다는 것이다.

## Play Modifier Resolver에 대한 현재 추천 결론

현재는 `modifier resolver`를 크게 잡되,
실제 구현은 얇게 시작하는 것이 좋다.

즉:
- 파일 이름과 책임은 `resolver` 쪽으로 열어 둔다.
- 실제 내부 구현은 지금은 simple rule-based parser여도 된다.

이 접근이 좋은 이유:
- 지금 당장 복잡도를 늘리지 않는다.
- 나중에 context/memory/ML/LLM을 붙일 자리를 미리 확보한다.
- validator / executor / C++ 경계를 다시 뜯지 않아도 된다.

## 현실적인 단계별 로드맵

### Step 1. 현재 상태 고정

- explicit keyword parser 유지
- validator gate 유지
- executor transport contract 유지
- C++ `readMeasure()` apply 추가

### Step 2. Resolver 인터페이스 승격

- `parse_play_modifier()`를 resolver 내부 함수로 내린다.
- 외부에서는 `resolve_play_modifier(...)`만 보게 한다.

예:

```python
def resolve_play_modifier(
    user_text,
    classifier_result,
    planner_result,
    robot_state,
    turn_history=None,
):
    ...
```

### Step 3. Context / Memory 추가

- 직전 modifier 기억
- 직전 곡 기억
- `아까처럼`, `조금 더 세게` 지원

### Step 4. Intent Classifier ML 실험

- classifier dataset 축적
- closed-set ML/DL 모델 실험
- low-confidence LLM fallback

### Step 5. Modifier ML/LLM 보조

- ambiguous modifier inference 추가
- 필요하면 regression 또는 LLM fallback 도입

## 어떤 계층에 무엇이 잘 맞는가

| 계층 | 규칙 기반 | ML/DL | LLM |
| --- | --- | --- | --- |
| interrupt gate | 매우 적합 | 보조 가능 | 비추천 |
| intent classifier | baseline 가능 | 매우 적합 | 현재 사용 가능 |
| planner | 일부 제한적 | 덜 적합 | 매우 적합 |
| play modifier resolver | 초기 매우 적합 | 보조 적합 | fallback 적합 |
| validator | 매우 적합 | 비추천 | 비추천 |
| executor | 매우 적합 | 비추천 | 비추천 |
| C++ apply | 매우 적합 | 비추천 | 비추천 |

## 한 줄 결론

`modifier 계층`은
`규칙 기반인지 LLM인지`를 뜻하는 이름이 아니라,
`연주 방식을 최종 결정하는 책임 계층`을 뜻한다.

그래서 지금은 rule-based parser로 시작해도 되고,
나중에는 같은 계층 안에 context, memory, ML, LLM을 순서대로 얹어 갈 수 있다.
