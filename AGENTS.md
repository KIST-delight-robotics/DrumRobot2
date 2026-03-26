# Repository-wide agent instructions

이 저장소 전체에서 작업할 때는 아래 규칙을 기본으로 따른다.

## 변경 기록 규칙
- 루트의 `log.md`를 작업 로그로 사용한다.
- 코드, 문서, 설정 등 저장소에 의미 있는 변경을 가한 경우 `log.md`에도 함께 기록한다.
- 각 로그 항목에는 반드시 아래 3가지를 포함한다.
  1. 날짜 (`YYYY-MM-DD`)
  2. 시간 (`HH:MM KST (UTC+9)` 형식의 한국시간)
  3. 변경 요약 (무엇을 왜 바꿨는지 한두 줄)
- 최신 기록이 위로 오도록(log reverse-chronological order) 새 항목을 기존 항목 위에 추가한다.
- 커밋 전에는 이번 작업이 `log.md`에 반영되었는지 확인한다.
- 별도 지시가 없으면 모든 작업 로그 시각은 한국시간(KST, UTC+9/GMT+9) 기준으로 적는다.

## 로그 포맷
다음 템플릿을 기준으로 작성한다.

```md
## YYYY-MM-DD
- HH:MM KST (UTC+9) — 변경 요약
  - 수정 파일: `path/to/file`
  - 메모: 필요 시 배경/의도/후속 작업
```

## 범위와 우선순위
- 더 하위 디렉터리에 별도의 `AGENTS.md`가 있으면 그 지침을 우선한다.
- 사용자/시스템/개발자 지침이 있으면 그 지침을 최우선으로 따른다.

## 언어/배치 규칙
- `DrumRobot2/` 아래에 새로 작성하는 코드는 가능하면 C++로 작성한다.
- `DrumRobot2/` 아래에는 Python 파일을 새로 추가하지 않는 방향을 기본 원칙으로 삼는다.
- Python 실행 코드, 보조 프로세스, SIL/시뮬레이터 관련 Python 코드는 가능한 한 `drum_intheloop/`, `phil_intheloop/` 또는 그에 준하는 별도 Python 영역에 둔다.
- 예외적으로 `DrumRobot2/` 아래에 Python 파일을 둬야 한다면, 사용자 요청이나 기존 구조와의 호환 필요성이 분명할 때만 그렇게 한다.

## 현재 통합 컨텍스트
- 현재 `drum_intheloop`는 `DrumRobot2`의 command-level 출력을 `named pipe`로 받아 `PyBullet`에 적용하는 느슨한 SIL 경로를 사용한다.
- 현재 공식 표현은 `LLM 플래너 -> 로봇 제어기 -> command-level 시뮬레이터`이다.
- 현재 SIL 활성화는 자동 검출이 아니라 **명시적 환경변수**로만 켠다.
  - `DRUM_SIL_MODE=1` 이면 `DrumRobot2` C++ 쪽 pipe writer와 disconnected-motor SIL 우회가 켜진다.
  - 환경변수가 없으면 SIL은 꺼진 것으로 본다.
- `/tmp/drum_command.pipe`는 이제 `drum_intheloop/sil/SilCommandPipeReader.py`가 실행 시 삭제 후 재생성하고, 종료 시 정리한다.
  - stale FIFO 존재만으로 SIL on/off를 판단하지 않는다.
  - `DrumRobot2` writer는 더 이상 FIFO를 만들지 않는다.
- 현재 우회 상태:
  - SIL 모드에서는 disconnected CAN 모터를 `motors` 맵에서 지우지 않는다.
  - SIL 모드에서는 disconnected TMotor/Maxon의 실제 CAN 송신은 건너뛰고, command-level export만 유지한다.
  - SIL 모드에서는 `motorSettingCmd`, `maxonMotorEnable`, `setMaxonMotorMode`에서 disconnected Maxon 하드웨어 초기화를 건너뛴다.
- 현재 남아 있는 핵심 이슈:
  - `AgentSocket` 기반 TCP brain 연결은 pipe 경로와 별개다.
  - `main.cpp`는 `initializeDrumRobot()`가 끝난 뒤에야 state/send/recv thread를 시작하므로, TCP brain 연결이 안 되면 body trajectory/pipe export도 시작되지 않을 수 있다.
  - `AgentSocket` gate는 별도라서 TCP가 붙어도 `k` 이전에는 명령이 폐기될 수 있다.
- CSV 파일 생성은 trajectory 생성과 별개다.
  - `openCSVFile()`는 startup 초기에 INIT 메타데이터를 기록하므로, CSV가 있다고 body trajectory가 생성된 것은 아니다.
- 현재 실행 기본 순서:
  1. `drum_intheloop`에서 `python sil/SilCommandPipeReader.py --mode gui`
  2. `DrumRobot2/bin`에서 `sudo env DRUM_SIL_MODE=1 ./main.out`
  3. 이후 `phil_robot`/brain 연결 확인
- 장기 목표는 `vcan` 또는 `struct can_frame` 기반의 frame-accurate SIL이지만, 현재는 command-level pipe 경계를 안정화하는 단계다.
