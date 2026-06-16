# `phil-controller` (DrumRobot2) 작업 지침

이 레포는 드럼 로봇의 **실시간 C++ 제어기(body)** 다. CAN/TMotor/Maxon/DXL 제어, 상태머신,
악보 기반 trajectory 생성, brain 명령 수신, 상태 broadcast를 담당한다.

## 시스템 안에서의 위치

```text
phil-brain (LLM)  ──TCP 9999──▶  [phil-controller]  ──CAN/DXL──▶  phil-sil 또는 실제 하드웨어
```

이 레포는 두 인터페이스 계약의 **허브**다. 두 계약 모두 [CONTRACTS.md](./CONTRACTS.md)에 정의돼 있다.

- **Contract A** (brain ↔ controller, TCP 9999): `src/AgentSocket.cpp`
- **Contract B** (controller ↔ SIL/하드웨어, CAN + DXL): `src/CanManager.cpp`, `src/CommandParser.cpp`

> **계약을 바꾸지 마라 — 합의 없이는.** TCP 명령 문법/상태 JSON 필드, 또는 CAN frame/DXL packet
> byte 레이아웃을 바꾸면 상대 레포(`phil-brain`, `phil-sil`)가 즉시 깨진다. wire format을 바꿔야 하면
> 먼저 `CONTRACTS.md`를 고치고, 세 레포 복사본을 동기화하고, 상대 레포도 함께 수정한다.

## 빌드 / 실행

```bash
make clean && make          # -> bin/main.out
cd bin && sudo ./main.out   # 상대경로 의존성 때문에 bin/에서 실행
```

- `Makefile`은 `opencv4`, `sfml`, `realsense2`, USBIO(`-lUSBIO_arm64`), Dynamixel(`-ldxl_sbc_cpp`),
  ArUco가 있는 Jetson/aarch64 환경을 전제로 한다. `-std=c++17`.
- `src/main.cpp`는 `initializeDrumRobot()`가 끝난 뒤에야 state/send/recv/music/python/broadcast
  thread를 시작한다. brain TCP 연결이 안 되면 그 뒤 경로가 함께 지연될 수 있다.
- thread 우선순위: send(5) > recv(4) > state(3) > music(2) > python(1).
- `openCSVFile()`은 startup 초기에 메타데이터를 기록한다 — CSV가 생겼다고 body trajectory 생성까지
  성공한 것은 아니다.

## 악보(txt) 형식

- 위치: `include/codes/*.txt`. 첫 줄은 `bpm <number>`.
- 각 행: `마디번호  대기시간  오른손악기  왼손악기  오른손velocity  왼손velocity  오른발악기  왼발악기`
  - 대기시간은 이전 행과의 상대 시간(초). 악기번호 0이면 그 손/발은 타격 안 함.

## 코드 규칙

- 이 레포 아래 새 코드는 **C++로 작성**한다. Python 파일을 새로 추가하지 않는 것을 기본 원칙으로 한다
  (보조/SIL Python은 `phil-sil`·`phil-brain` 쪽에 둔다). 예외는 기존 구조 호환이 분명할 때만.
- 변수명은 3단어 이하. 약어는 언더바로 분리(`op_cmd`, `joint_deg`). 약한 이름(`it`, `iter`)보다 역할이
  드러나는 이름(`motor_ptr`, `joint_map`)을 쓴다.
- 새/대규모 리팩터 코드에서는 `auto`를 기본 선택으로 쓰지 말고 명시 타입을 쓴다.
- 람다 후행 반환형(`-> type`)이나 `ptr->field` 축약을 기본으로 쓰지 말고 일반 함수/분기/명시적 역참조로 푼다.

## 재생 제어 / 안전

- 전역 궤적 버퍼링을 배제하고 점진적 파일 기반 실행을 쓴다.
- `s`(정지): pending 버퍼와 `TestManager` 큐를 즉시 flush해 추가 동작 없이 멈춘다.
- `p`(재개): 중단된 마디/위치부터 이어서 재생.
- `k` 게이트: 안전 키 제거 후 콘솔에 `k`를 입력해야 brain 명령 게이트가 열린다([CONTRACTS.md](./CONTRACTS.md) A.4 참조).

## 작업 로그

- 이 레포에서 의미 있는 변경을 하면 레포 루트의 `log.md`에 기록한다.
- 각 항목에 날짜(`YYYY-MM-DD`), 시간(`HH:MM KST (UTC+9)`), 변경 요약을 포함하고, 최신 항목을 위에 추가한다.

```md
## YYYY-MM-DD
- HH:MM KST (UTC+9) — 변경 요약
  - 수정 파일: `path/to/file`
  - 메모: 필요 시 배경/의도/후속 작업
```

## 우선순위

- 더 하위 디렉터리에 별도 `AGENTS.md`가 있으면 그 지침을 우선한다.
- 사용자/시스템/개발자 지침이 있으면 최우선으로 따른다.
