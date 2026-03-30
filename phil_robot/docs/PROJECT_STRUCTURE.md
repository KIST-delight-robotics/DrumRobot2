# Phil Robot Project Structure

## 목적

`phil_robot` 디렉토리는 로봇 음성 인터랙션, LLM 제어 파이프라인, 평가 코드, 테스트 코드, 문서, 런타임 자산을 함께 포함하고 있다.  
파일 수가 늘어나면서 탐색 비용이 커졌기 때문에, 대규모 오픈소스 프로젝트에서 자주 쓰는 책임 분리 기준으로 폴더를 재구성했다.

## 현재 구조

```text
phil_robot/
├── phil_brain.py
├── init_phil.sh
├── environment.yml
├── requirements_melo_tts.txt
├── assets/
├── artifacts/
├── docs/
├── eval/
├── pipeline/
├── runtime/
├── tests/
└── third_party/
```

## 폴더별 역할

### 루트

- `phil_brain.py`
  - 대화 시스템의 메인 엔트리포인트
  - STT, state snapshot, pipeline 실행, executor 호출, TTS orchestration 담당
- `init_phil.sh`
  - Jetson/로컬 런타임 초기화 스크립트
- `environment.yml`, `requirements_melo_tts.txt`
  - 환경 구성 정의 파일

### `pipeline/`

LLM 제어 파이프라인의 핵심 로직을 모은 계층이다.

- classifier
- domain planner
- state adapter
- skill expansion
- motion resolver
- command validator
- plan validator
- executor wrapper
- LLM I/O contract

이 폴더에는 “제어 판단”에 해당하는 파일만 둔다.  
즉, `입력을 받아 어떤 의도로 해석하고 어떤 계획을 세울지`를 다루는 코드다.

현재 주요 파일:

- `brain_pipeline.py`
- `intent_classifier.py`
- `planner.py`
- `validator.py`
- `executor.py`
- `skills.py`
- `state_adapter.py`
- `motion_resolver.py`
- `command_validator.py`
- `command_executor.py`
- `llm_interface.py`
- `failure.py`
- `response_parser.py`

### `runtime/`

실제 장치/런타임과 결합되는 코드를 둔다.

- 로봇 소켓 클라이언트
- TTS 엔진 래퍼
- 장치 I/O 성격이 강한 코드

현재 파일:

- `phil_client.py`
- `melo_engine.py`

구분 기준:

- LLM 판단 로직이면 `pipeline/`
- 하드웨어/오디오/네트워크 런타임 연결이면 `runtime/`

### `eval/`

운영 경로와 분리된 평가 파이프라인을 둔다.

- eval case dataset
- eval runner
- saved reports
- eval usage docs

현재 파일:

- `cases_smoke.json`
- `run_eval.py`
- `README.md`
- `reports/`

구분 기준:

- 실제 서비스 루프에서 쓰지 않고, 오프라인 검증/회귀 테스트용이면 `eval/`

### `tests/`

개발 중 실험성 테스트, 수동 점검용 스크립트, 개별 컴포넌트 테스트를 둔다.

현재 파일:

- `test_drum.py`
- `test_drum2.py`
- `test_drum3.py`
- `test_drum4.py`
- `test_speech.py`
- `client_test.py`

구분 기준:

- 정식 eval dataset 기반 채점이 아니라
- 특정 기능을 직접 실행해 보는 보조 스크립트면 `tests/`

### `docs/`

설계 문서, 아키텍처 문서, 벤치마크 결과 문서를 둔다.

현재 파일:

- `LLM_PIPELINE_ARCHITECTURE.md`
- `LLM_PIPELINE_ARCHITECTURE_KR.md`
- `CLASSIFIER_BENCHMARK_REPORT.md`
- `CLASSIFIER_BENCHMARK_REPORT_KR.md`
- `PROJECT_STRUCTURE.md`
- `PROJECT_STRUCTURE_KR.md`

구분 기준:

- 사람이 읽는 설명 문서면 `docs/`
- 코드 실행에 직접 필요하지 않으면 `docs/`

### `assets/`

정적인 자산 파일을 둔다.

- 음성 레퍼런스 파일
- 예시 오디오
- 이미지나 로고 같은 리포지토리 포함 자산

현재 파일:

- `phil_voice1.wav`
- `phil_speak.mp3`

구분 기준:

- 버전 관리되는 정적 리소스면 `assets/`

### `artifacts/`

실행 중 생성되거나 덮어써지는 산출물을 둔다.

예:

- `temp_speech.wav`
- 임시 녹음 파일
- 실행 로그나 캐시 산출물

구분 기준:

- 생성물이고 source of truth가 아니면 `artifacts/`

### `third_party/`

외부 프로젝트나 외부 배포물, 직접 관리하지 않는 번들 의존성을 둔다.

현재 파일:

- `MeloTTS/`
- `downloads_whl/`

구분 기준:

- upstream에서 가져온 코드면 `third_party/`
- 우리 프로젝트 핵심 로직이 아니면 `third_party/`

## 대규모 오픈소스에서 보통 어떻게 나누는가

대규모 오픈소스는 보통 다음 기준을 섞어 쓴다.

1. 실행 코드와 실험 코드를 분리한다.
2. 문서와 코드, 생성물과 원본 자산을 분리한다.
3. 외부 의존성과 내부 핵심 로직을 분리한다.
4. 런타임 계층과 판단 계층을 분리한다.

`phil_robot`에 적용한 현재 구조도 이 원칙을 따른다.

## 앞으로 파일을 어디에 둘지 기준

### `pipeline/`에 둘 것

- intent 분류기
- planner
- validator
- executor wrapper
- LLM contract
- skill catalog

### `runtime/`에 둘 것

- socket client
- TTS/STT runtime wrapper
- 장치 연결 계층

### `eval/`에 둘 것

- JSON case dataset
- benchmark runner
- replay runner
- report generator

### `tests/`에 둘 것

- 특정 장비 확인용 단발성 테스트
- 수동 실험 스크립트
- quick sanity check 스크립트

### `docs/`에 둘 것

- 구조 설명
- 실험 결과
- 아키텍처 기록

### `assets/`에 둘 것

- reference wav
- demo media
- static example files

### `artifacts/`에 둘 것

- temp audio
- generated outputs
- cache-like runtime files

### `third_party/`에 둘 것

- vendored dependency
- external model toolkit
- upstream source snapshot

## 권장 운영 원칙

1. 엔트리포인트는 루트에 얇게 유지한다.
2. 제어 로직은 `pipeline/` 안에서만 확장한다.
3. 하드웨어 종속 코드는 `runtime/` 밖으로 새지 않게 한다.
4. 평가 코드는 운영 루프와 섞지 않고 `eval/`에 유지한다.
5. 실험이 끝난 스크립트는 `tests/`에서 문서화하거나 삭제한다.
6. 생성 파일은 가능하면 `artifacts/`로 보낸다.

이 구조를 유지하면, 이후 `planner`, `eval`, `feedback`, `RAG`, `memory`가 추가되어도 폴더 탐색 비용이 크게 늘어나지 않는다.
