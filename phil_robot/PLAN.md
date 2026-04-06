# Phil Robot Online STT Plan

## 요약

이 문서는 `phil_robot`의 현재 `3초 고정 녹음 -> 전체 Whisper 1회 -> planner` 흐름을 `연속 입력 스트림 -> chunk별 Whisper -> overlap merge -> 최종 문장 1회 planner` 구조로 바꾸기 위한 handoff용 구현 계획서다.

v1 원칙은 아래와 같이 고정한다.

- partial STT는 내부 인지와 로그용으로만 사용한다.
- planner, validator, executor는 최종 병합 문장 1회만 사용한다.
- 의미 보정 규칙은 넣지 않는다.
- chunk 간 텍스트는 prefix/suffix overlap 제거만 수행한다.
- TTS 재생 중에는 청취를 막아 self-echo를 차단한다.
- v1 목표는 `발화 종료 후 final text 확정`의 보통 지연을 `0.5초 이내`로 낮추는 것이다.

## 구현 변경

### 1. 메인 루프와 STT 입구

- `phil_robot/phil_brain.py`의 `RECORD_SECONDS=3` 기반 blocking 녹음 루프를 제거한다.
- 새 모듈은 `phil_robot/runtime/online_stt.py` 하나로 추가한다.
- `phil_brain.py`는 새 모듈의 최종 인터페이스만 호출한다.
- 최종 인터페이스는 아래처럼 고정한다.

```python
user_text = listen_user_text(stt_model=stt_model, is_speaking_fn=is_tts_active)
```

- 반환값은 `str` 또는 빈 문자열로 한다.
- 빈 문자열이면 현재와 동일하게 이번 턴을 스킵한다.
- `run_brain_turn(user_text, raw_robot_state, ...)` 호출 계약은 바꾸지 않는다.
- TTS streaming 경로는 유지하고, final text가 확정된 뒤에만 실행한다.

### 2. online_stt.py 책임 범위

`online_stt.py`는 아래 책임을 모두 가진다.

- `InputStream` 기반 연속 오디오 수집
- VAD 기반 발화 시작/종료 판단
- chunk 스케줄링
- chunk별 Whisper 호출
- partial segment 버퍼링
- overlap merge
- final utterance 확정

v1은 파일을 더 쪼개지 않고 이 모듈 하나에 모은다. 다른 agent가 추가 분해를 고민하지 않도록 이 점을 고정한다.

### 3. 고정 파라미터

v1 기본값은 아래처럼 확정한다.

- sample rate: `16000`
- frame size: `100ms` (`1600` samples)
- pre-roll: `300ms`
- start threshold: 현재 `test_speech.py` 계열 값에 맞춰 `15`
- stop threshold: 현재 `test_speech.py` 계열 값에 맞춰 `8`
- chunk stride: `800ms`
- audio overlap: `200ms`
- final tail window: 마지막 미확정 구간 기준 `1.2초`
- post-silence commit: `300ms`
- min utterance length: `700ms`
- max utterance length: `10초`
- language: `"ko"`
- Whisper 호출 옵션: `fp16=True`, `initial_prompt` 없음

`0.5초 이내` 목표와 충돌하지 않도록 post-silence commit은 `0.7~1.0초`가 아니라 `300ms`로 고정한다.

### 4. TTS 중 청취 차단

- 현재 `test_speech.py`가 쓰는 `is_speaking` gate 방식을 본 구현에도 그대로 넣는다.
- TTS가 재생 중이면 `listen_user_text()`는 새 utterance를 시작하지 않는다.
- 이미 진행 중인 utterance도 TTS 시작 시 즉시 abort하지 않고, TTS 시작 전 final commit이 끝난 턴만 speak로 넘어가게 메인 루프 순서를 유지한다.
- v1에는 별도 AEC나 마이크/스피커 신호 분리 기능을 넣지 않는다.

### 5. chunk별 Whisper 호출 정책

- Whisper는 chunk마다 독립 호출한다.
- stateful decoder cache 재사용은 v1 범위에 넣지 않는다.
- chunk 입력은 `chunk stride + audio overlap` 기준의 오디오 조각이다.
- 새 chunk를 디코드할 때는 최근 `200ms` 오디오를 다시 포함해 경계 손실을 줄인다.
- partial segment는 시간순으로 `segment_text_list`에 저장한다.
- partial segment는 planner나 executor로 전달하지 않는다.
- partial segment는 디버그 출력에서만 보여도 된다.

### 6. 최종 문장 생성 규칙

최종 문장은 `segment_text_list`를 시간순으로 병합해 만든다.

- 인접 두 segment만 비교한다.
- 앞 segment suffix와 뒤 segment prefix의 최대 공통 문자열 overlap를 찾는다.
- overlap 길이가 `2글자 이상`이면 중복 부분을 제거하고 병합한다.
- overlap가 `2글자 미만`이면 공백 1개로 이어 붙인다.
- trim과 중복 공백 정리만 허용한다.
- `아니`, `말고`, self-correction을 해석해 문장을 다시 쓰는 규칙은 넣지 않는다.

최종 commit 직전에는 마지막 tail window에 대해 Whisper를 한 번 더 돌려 final tail text를 갱신한다. 이 재디코드 결과를 마지막 segment로 사용한 뒤 전체 merge를 실행한다.

### 7. 발화 종료와 abort 규칙

`listen_user_text()`는 아래 조건을 모두 만족할 때 final text를 commit한다.

- speech active 상태에서 start threshold를 넘긴 뒤 utterance가 시작되었음
- 마지막 speech frame 이후 `300ms` 이상 silence가 이어짐
- 마지막 tail Whisper 호출이 완료됨
- 병합 결과 텍스트가 비어 있지 않음
- utterance 길이가 `700ms` 이상임

아래 조건이면 빈 문자열을 반환한다.

- 너무 짧은 utterance
- 무음
- Whisper 결과 공백
- trash utterance

v1 trash utterance는 기존 `phil_brain.py`의 메인 루프가 아닌 `online_stt.py` 안에서 걸러도 되고, 메인 루프에서 한 번 더 걸러도 된다. 단, 최소 한 곳에는 반드시 있어야 한다.

## acceptance와 범위

### 1. in scope

- 현재 3초 고정 녹음 제거
- final-only planner 경계 유지
- partial STT를 내부 처리로만 사용
- overlap merge 기반 최종 문장 생성
- 발화 종료 후 체감 지연 단축

### 2. out of scope

- partial마다 planner를 먼저 돌리는 구조
- self-correction 의미 해석기
- 복합 의도를 planner가 여러 action으로 분해하는 기능
- Whisper decoder cache 재사용
- AEC, beamforming, diarization

### 3. 복합 발화 acceptance 해석

`오른팔 들고 This Is Me 연주해줘` 같은 복합 발화는 v1에서 아래 기준으로만 통과로 본다.

- STT final text가 안정적으로 한 문장으로 생성됨
- planner에는 그 final text 1개만 전달됨
- partial text가 executor로 유입되지 않음

즉 이 케이스는 `복합 action 실행 성공`이 아니라 `복합 발화가 STT 단계에서 깨지지 않고 하나의 final text로 넘어가는지`를 검증하는 용도다.

## 테스트 계획

### 1. 새 테스트 파일

- `phil_robot/tests/test_online_stt.py`를 추가한다.

### 2. 단위 테스트

- overlap merge
  - 완전 중복
  - 부분 중복
  - overlap 없음
  - 공백 정리
- commit 조건
  - 짧은 침묵은 commit 안 됨
  - `300ms` 침묵에서 commit
  - 너무 짧은 utterance abort
- echo gate
  - `is_speaking=True`일 때 새 utterance 시작 안 함
- final-only 보장
  - partial segment가 planner 입력으로 전달되지 않음

### 3. 통합 테스트 시나리오

- 짧은 발화
  - `안녕`
  - `종료해`
  - 1음절 발화 abort
- 일반 요청
  - `손 흔들어줘`
  - `왼팔 벌려`
  - `This Is Me 연주해줘`
- self-correction 포함
  - `왼팔... 아니 오른팔 들어`
  - `디스... This Is Me 연주해줘`
- 복합 발화
  - `오른팔 들고 This Is Me 연주해줘`
  - acceptance는 final text 안정성 기준으로만 본다
- 침묵 경계
  - 짧은 pause에서 split 안 됨
  - 긴 침묵 뒤 commit 됨
- 안전성
  - partial text가 planner, validator, executor로 유입되지 않음
  - final text 공백이면 턴을 스킵함

### 4. 성능 기준

- baseline은 현재 `3초 고정 녹음 + Whisper 후처리` 경로다.
- 비교 지표는 `발화 종료 시점부터 final text 반환까지의 시간`으로 고정한다.
- v1 통과 기준:
  - 보통 케이스 평균 `0.5초 이내`
  - 최악 케이스에서도 기존 방식보다 유의미하게 짧을 것

## 선행 수정

다른 agent가 구현을 시작할 때 아래 항목은 초기 커밋에서 함께 고친다.

- `phil_robot/phil_brain.py`의 Whisper 모델명 오타 `smalll`을 `small`로 수정

이 항목은 STT 구조 변경과 직접 연결된 선행 버그이므로 별도 작업으로 분리하지 않는다.

## 기본 가정

- 문서 언어는 한국어로 유지한다.
- v1은 `human-like online perception`을 지향하되 `human-like semantic correction`은 구현하지 않는다.
- partial은 내부 인지용, final은 외부 실행용이라는 경계를 유지한다.
- 저장소 규칙상 실제 구현 시 루트 `log.md`를 함께 갱신한다.
