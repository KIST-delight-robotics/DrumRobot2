[Phil Robot (Phil AI Drummer) | 온디바이스 음성 인터랙션 및 제어 아키텍처 설계]
Jetson AGX Orin 기반의 인터랙티브 안드로이드 드러머 로봇에 외부 API 없이 동작하는 로컬 AI 에이전트와 실시간 C++ 제어계를 통합한 프로젝트. 단순 음성 파이프라인 구현을 넘어, Python 기반 AI Brain과 C++ 기반 Robot Body를 분리한 비동기 시스템 아키텍처, 상태 인지형 2-stage LLM 제어 파이프라인, 오프라인 eval pipeline, PyBullet 기반 SIL(Simulation-in-the-Loop)까지 엔드투엔드로 설계·구현.

기간: 2025.12.22-present

기술 스택:
Python 3.8.20, C++17, Jetson AGX Orin, Ollama 0.6.1, Qwen3:4B-Instruct-2507-Q4_K_M, Qwen3:30B-A3B-Instruct-2507-Q4_K_M, Whisper 20250625, MeloTTS, PyTorch 2.1.0a0+41361538.nv23.6, Transformers 4.27.4, NumPy 1.24.4, scikit-learn 1.3.2, SoundDevice 0.5.3, TCP Socket, JSON, CAN, PyBullet, OpenCV4, SFML, librealsense2

- 로봇 음성 인터랙션 아키텍처 도메인에서 단일 LLM 호출 기반 제어 구조의 취약성 문제를 해결하기 위해, `state snapshot -> state adaptation -> intent classifier -> domain planner -> skill expansion -> relative motion resolver -> validator -> executor`로 이어지는 상태 인지형 2-stage LLM 파이프라인을 설계·구현하여 제어 안정성, 디버깅 가능성, 확장성을 높임.
- 실시간 로봇 제어 도메인에서 AI 연산이 모터 제어 루프를 방해하는 문제를 해결하기 위해, Python AI Brain과 C++ Robot Body를 분리하고 `명령은 경량 텍스트`, `상태는 newline-delimited JSON`으로 주고받는 비대칭 TCP 소켓 아키텍처를 설계·구현하여 저지연 제어와 확장 가능한 상태 피드백 구조를 달성.
- 온디바이스 AI 시스템 도메인에서 외부 API 의존성, 지연, 비용, 개인정보 노출 문제를 해결하기 위해, Whisper-STT, Qwen LLM, MeloTTS를 Jetson 상에서 모두 로컬로 구동하는 음성 인터랙션 파이프라인과 모델 warm-up 구조를 구현하여 API-free 실시간 로봇 상호작용 환경을 구축.
- 로봇 제어 UX 도메인에서 기존 키보드 입력 중심의 수동 조작 문제를 해결하기 위해, DrumRobot 제어 흐름을 음성 명령 기반으로 재구성하고 곡 선택·상태 제어·행동 명령을 AgentSocket/AgentAction 경로로 통합하여 자연어 기반 연주 및 제스처 제어가 가능한 인터랙션 구조를 구현.
- 엣지 배포 도메인에서 x86 중심 개발 환경을 실기 환경에 그대로 적용하기 어려운 문제를 해결하기 위해, Jetson arm64 빌드 체인과 aarch64 Python 실행 환경, 로컬 모델 구동 환경을 정비하여 x86 기반 개발 코드베이스를 ARM 기반 실기 환경으로 포팅.
- LLM 로봇 제어 검증 도메인에서 프롬프트·모델 변경 시 회귀를 확인하기 어려운 문제를 해결하기 위해, 운영 경로와 분리된 eval pipeline을 설계하고 classifier/planner/validator/e2e 레이어별 채점, JSON case dataset, 자동 report 생성 구조를 구현하여 반복 가능한 오프라인 평가 체계를 구축했으며 smoke suite 11/11 pass를 달성.
- 실로봇 개발 도메인에서 반복 실험 비용과 하드웨어 안전 리스크가 큰 문제를 해결하기 위해, production socket contract를 유지하는 PyBullet 기반 SIL 환경과 runtime URDF patching 구조를 설계·구현하여 실로봇 없이도 명령 재생, 상태 확인, headless screenshot 검증이 가능한 개발 루프를 구축.


[Phil Robot (Phil AI Drummer) | 온디바이스 로봇 에이전트 아키텍처 설계 및 구현]
Jetson AGX Orin 기반 인터랙티브 안드로이드 드러머 로봇에 대해, 외부 API 없이 동작하는 온디바이스 AI Brain과 실시간 C++ Robot Body를 비동기 소켓으로 결합한 제어 아키텍처를 설계·구현한 프로젝트. 단순 음성 파이프라인 구현을 넘어, 상태 인지형 staged LLM pipeline, AgentSocket/AgentAction 기반 제어 인터페이스, production-compatible SIL(Simulation-in-the-Loop), 오프라인 eval pipeline까지 엔드투엔드로 설계·구축.

기간: 2025.12.22-present

기술 스택: Jetson AGX Orin, Python 3.8.20, C++17, Ollama 0.6.1, Qwen3:4B-Instruct-2507-Q4_K_M, Qwen3:30B-A3B-Instruct-2507-Q4_K_M, Whisper 20250625, MeloTTS, PyTorch 2.1.0a0+41361538.nv23.6, Transformers 4.27.4, NumPy 1.24.4, scikit-learn 1.3.2, SciPy 1.10.1, SoundDevice 0.5.3, SoundFile 0.13.1, PyBullet, TCP Socket, JSON, CAN, OpenCV4, SFML, RealSense2

- 로봇 음성 인터랙션 아키텍처 설계 시, AI 추론과 실시간 모터 제어가 한 프로세스에 얽히는 구조적 문제를 해결하기 위해 Python 기반 AI Brain과 C++ 기반 Robot Body를 분리한 비동기 TCP 소켓 구조를 설계하고, AgentSocket/AgentAction 중심의 명령·상태 인터페이스를 구축하여 저지연 명령 전달과 확장 가능한 JSON 상태 피드백 체계를 구현함.
- LLM 제어 파이프라인 설계 시, `STT -> 단일 LLM 호출 -> 정규식 파싱` 구조의 취약성과 안전성 한계를 해결하기 위해 state adaptation, intent classification, planner domain routing, skill expansion, relative motion resolution, command validation, executor로 이어지는 staged pipeline을 설계·구현하여 제어 안정성, 디버깅 용이성, 유지보수성을 향상시킴.
- 온디바이스 AI 시스템 구축 시, 외부 API 의존으로 인한 지연·비용·개인정보·네트워크 리스크를 해결하기 위해 Whisper-STT, Qwen LLM, MeloTTS를 Jetson 상에서 모두 로컬 구동하는 STT-LLM-TTS 파이프라인과 모델 warm-up 구조를 구현하여 API-free 실시간 음성 상호작용 시스템을 완성함.
- 로봇 제어 UX 개선 시, 기존 키보드 기반 수동 입력 흐름의 한계를 해결하기 위해 곡 선택, 상태 제어, 제스처, 관절 이동을 자연어 명령에서 실행 가능한 command set으로 연결하는 음성 명령 중심 제어 흐름으로 재구성함.
- 엣지 배포 및 실행 환경 이식 시, x86 개발 기준 빌드/의존성을 실기 환경에 그대로 적용하기 어려운 문제를 해결하기 위해 Jetson arm64/aarch64 기반 빌드 체인과 Python 실행 환경을 정비하고 로컬 모델 구동 스택을 맞춰 실제 하드웨어 배포 가능한 형태로 포팅함.
- LLM 로봇 제어 검증 체계 설계 시, 프롬프트·모델 변경에 대한 회귀 확인이 어려운 문제를 해결하기 위해 production `run_brain_turn(...)` 경로를 재사용하는 multi-layer eval pipeline(classifier/planner/validator/e2e), JSON case dataset, 자동 report 생성 체계를 설계·구현하여 smoke suite 11/11 pass 기반의 반복 가능한 오프라인 평가 체계를 구축함.
- 실기 의존 개발 프로세스 개선 시, 하드웨어 접근 비용과 안전 리스크로 반복 실험이 어려운 문제를 해결하기 위해 production socket contract를 그대로 유지하는 PyBullet 기반 SIL을 설계·구현하고, runtime URDF patching, joint mapping, gesture/state simulation, headless screenshot 생성까지 포함한 시뮬레이션 검증 루프를 구축함.


[Phil Robot (Phil AI Drummer) | 온디바이스 로봇 에이전트 아키텍처 설계 및 구현]
Jetson AGX Orin 기반 인터랙티브 안드로이드 드러머 로봇에 대해, 외부 API 없이 동작하는 로컬 AI 에이전트와 실시간 C++ 제어계를 통합한 프로젝트. Python 기반 AI Brain과 C++ 기반 Robot Body를 분리한 비동기 제어 아키텍처, 상태 인지형 staged LLM pipeline, AgentSocket/AgentAction 기반 제어 인터페이스, 오프라인 eval pipeline, PyBullet 기반 SIL(Simulation-in-the-Loop)까지 엔드투엔드로 설계·구현.

기간: 2025.12.22-present

기술 스택: Jetson AGX Orin, ARM64/aarch64, Python 3.8.20, C++17, Ollama 0.6.1, Qwen3:4B-Instruct-2507-Q4_K_M, Qwen3:30B-A3B-Instruct-2507-Q4_K_M, Whisper 20250625, MeloTTS, PyTorch 2.1.0a0+41361538.nv23.6, Transformers 4.27.4, NumPy 1.24.4, scikit-learn 1.3.2, SciPy 1.10.1, SoundDevice 0.5.3, SoundFile 0.13.1, PyBullet, TCP Socket, JSON, CAN, OpenCV4, SFML, RealSense2

- 로봇 제어 아키텍처 설계 시 AI 추론과 실시간 모터 제어가 한 루프에 결합될 때 발생하는 지연 및 안정성 문제를 해결하기 위해, Python AI Brain과 C++ Robot Body를 분리한 비동기 TCP 소켓 구조를 설계하고 AgentSocket/AgentAction 기반 명령·상태 인터페이스를 구축하여 저지연 제어와 확장 가능한 상태 피드백 구조를 구현함.
- LLM 제어 파이프라인 설계 시 기존 단일 `STT -> LLM -> parse -> execute` 구조의 취약성과 안전성 한계를 해결하기 위해 state adaptation, intent classification, planner domain routing, skill expansion, relative motion resolution, command validation, executor로 이어지는 staged pipeline을 설계·구현하여 제어 안정성, 디버깅 용이성, 유지보수성을 향상시킴.
- 온디바이스 AI 시스템 구축 시 외부 API 의존으로 인한 지연, 비용, 네트워크, 개인정보 리스크를 해결하기 위해 Whisper-STT, Qwen LLM, MeloTTS를 Jetson 상에서 모두 로컬 구동하는 STT-LLM-TTS 파이프라인과 warm-up 구조를 구축하여 API-free 실시간 음성 상호작용 환경을 구현함.
- 로봇 상호작용 UX 설계 시 기존 수동 조작 중심 흐름의 한계를 해결하기 위해 곡 선택, 상태 제어, 제스처, 관절 이동을 자연어 명령에서 실행 가능한 command set으로 연결하는 음성 인터랙션 중심 제어 구조를 구현함.
- 엣지 배포 아키텍처 구성 시 x64 중심 개발 의존성을 실기 환경에 적용하기 어려운 문제를 해결하기 위해 C++ 링크 타깃을 Jetson arm64 환경으로 정비하고 aarch64 Python 실행 환경과 로컬 모델 구동 스택을 표준화하여 ARM 기반 실기 배포 가능 구조로 포팅함.
- LLM 로봇 제어 검증 체계 설계 시 프롬프트·모델 변경에 대한 회귀 확인이 어려운 문제를 해결하기 위해 production `run_brain_turn(...)` 경로를 재사용하는 multi-layer eval pipeline(classifier/planner/validator/e2e), JSON case suite, 자동 report 생성 체계를 설계·구현하여 반복 가능한 오프라인 평가 체계를 구축함.
- 모델 선택 및 성능 최적화 시 추론 지연 문제를 해결하기 위해 benchmark 기반으로 classifier 모델을 재선정하여 smoke case 11/11 정합성을 유지하면서 classifier latency를 6.481초에서 1.627초로 74.9% 단축함.
- 실기 의존 개발 프로세스 개선 시 하드웨어 접근 비용과 안전 리스크로 반복 실험이 어려운 문제를 해결하기 위해 production socket contract를 유지하는 PyBullet 기반 SIL을 설계·구현하고 runtime URDF patching, joint mapping, gesture/state simulation, headless screenshot 생성까지 포함한 시뮬레이션 검증 루프를 구축함.
