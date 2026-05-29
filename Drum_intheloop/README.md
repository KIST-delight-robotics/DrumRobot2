# Drum_intheloop

`Drum_intheloop`는 `DrumRobot2`가 실제 장치 경계로 내보내는
SocketCAN `can_frame`과 Dynamixel Protocol 2.0 serial packet을 그대로 받아
`PyBullet`에 적용하는 frame-level SIL(Simulation in the Loop) 디렉터리다.

현재 공식 흐름은 아래처럼 이해하면 된다.

```text
LLM 플래너
  -> phil_robot / brain
  -> DrumRobot2 로봇 제어기
  -> SocketCAN can_frame / Dynamixel serial packet
  -> Drum_intheloop frame-level SIL
  -> PyBullet
  -> feedback frame/status packet
  -> DrumRobot2
```

예전 command-level SIL처럼 `/tmp/drum_command.pipe`에 명령을 쓰거나
`DRUM_SIL_MODE` 환경변수로 우회 경로를 켜는 구조가 아니다. 지금 구조는
`DrumRobot2`가 실제 하드웨어를 상대할 때와 최대한 같은 모양의 입출력 경계를
사용한다.

더 자세한 문제 분석과 디버깅 기록은 [Trouble shooting.md](<Trouble shooting.md>)에
정리되어 있다.

## 핵심 목표

- `DrumRobot2`의 C++ trajectory, safety, send loop를 그대로 사용한다.
- 실제 장치 대신 `vcan0..3`와 DXL PTY가 모터 bus 역할을 한다.
- SIL은 frame/packet을 decode해서 PyBullet joint target으로 바꾼다.
- SIL은 다시 실제 모터처럼 feedback frame/status packet을 encode해서 돌려준다.
- 원본 URDF/STL은 수정하지 않고, runtime URDF 복사본만 보정한다.

이 디렉터리는 planner나 TCP brain 연결을 직접 고치지 않는다. 예를 들어
`phil_robot`이 잘못된 gesture를 고르거나, `AgentSocket`이 연결되지 않아
`DrumRobot2`가 명령을 생성하지 않는 문제는 이 디렉터리 바깥 문제다.

## 전체 경계

```text
DrumRobot2
  TMotor command frame
  Maxon CANopen frame
  Dynamixel Protocol 2.0 packet
        |
        v
Linux virtual devices
  vcan0..3
  /dev/ttyUSB0 -> PTY symlink
        |
        v
Drum_intheloop/simul.py
  sil/decoder.py
  sil/router.py
  sil/mapping.py
  sil/pybullet_backend.py
  sil/encoder.py
        |
        v
PyBullet robot
        |
        v
feedback
  TMotor feedback frame
  Maxon TPDO frame
  Dynamixel status packet
        |
        v
DrumRobot2 current angle/state update
```

중요한 점은 `DrumRobot2`가 SIL을 특별한 command pipe로 보는 것이 아니라는 점이다.
`DrumRobot2` 입장에서는 CAN interface와 `/dev/ttyUSB0`이 있을 뿐이다.

## 실행 순서

### 1. 의존성 설치

```bash
sudo apt update
sudo apt install -y iproute2 kmod can-utils socat
cd /home/shy/robot_project/Drum_intheloop
python3 -m pip install -r requirements.txt
```

`can-utils`는 `candump`, `cansend` 같은 디버깅 도구를 위해 사용한다.
`socat`은 DXL용 PTY pair를 만들기 위해 필요하다.

### 2. 터미널 1: SIL device 준비

```bash
cd /home/shy/robot_project/Drum_intheloop
./setup_sil.sh
```

이 스크립트는 다음을 수행한다.

- `vcan0`, `vcan1`, `vcan2`, `vcan3`를 준비한다.
- `socat`으로 DXL PTY pair를 만든다.
- robot-side endpoint를 `/dev/ttyUSB0` symlink로 노출한다.
- simulator-side endpoint를 `/tmp/ttyUSB0_sim`으로 노출한다.
- PTY가 유지되도록 터미널을 계속 점유한다.

따라서 이 터미널은 닫으면 안 된다. 닫으면 DXL PTY도 같이 사라진다.

실제 `/dev/ttyUSB0` 장치가 있거나, SIL이 만든 것이 아닌 symlink가 이미 있으면
스크립트는 덮어쓰지 않고 중단한다. 실제 장치를 보호하기 위한 동작이다.

### 3. 터미널 2: simulator 실행

```bash
cd /home/shy/robot_project/Drum_intheloop
python3 simul.py --mode gui
```

GUI 없이 돌리고 싶으면 다음처럼 실행한다.

```bash
python3 simul.py --mode direct
```

`simul.py`는 CAN socket과 DXL PTY를 열고, PyBullet backend를 시작한 뒤
frame/packet을 계속 처리한다.

### 4. 터미널 3: DrumRobot2 실행

```bash
cd /home/shy/robot_project/DrumRobot2/bin
sudo ./main.out
```

`DrumRobot2`는 real `can*` interface가 하나라도 있으면 real CAN만 사용한다.
real CAN이 없을 때만 `vcan*`로 fallback한다. 따라서 SIL을 쓸 때는 실제 CAN 장치가
활성화되어 있지 않은지 확인해야 한다.

### 5. 터미널 4: brain / voice 경로 실행

음성 명령까지 연결하려면 별도 터미널에서 brain을 실행한다.

```bash
cd /home/shy/robot_project/phil_robot
python phil_brain.py
```

실행 후 `DrumRobot2`에서 brain 연결 로그가 보이고, 안전 키 확인 단계에서 `k`를
입력해야 gesture command가 실제로 처리된다. TCP brain 연결과 `k` gate는 별개다.

## CAN 선택 정책

`DrumRobot2`의 interface 선택은 아래 규칙을 따른다.

```text
1. real can* interface를 찾는다.
2. 하나라도 있으면 real CAN만 사용한다.
3. real CAN이 없으면 vcan* interface를 사용한다.
```

real CAN에는 bitrate 설정이 필요하지만, `vcan`에는 bitrate를 설정하지 않는다.
`setup_sil.sh`도 `vcan`을 up 시키기만 한다.

기본 bus 배치는 다음과 같다.

```text
vcan0: L_arm1, L_arm2, L_arm3, waist
vcan1: R_arm1, R_arm2, R_arm3
vcan2: L_foot, R_foot
vcan3: L_wrist, R_wrist
```

다만 frame-level SIL은 command가 들어온 bus를 motor feedback bus로 동적 바인딩할 수
있다. 그래서 디버깅할 때는 고정 bus 표만 보지 말고 실제 `candump`에서 어떤 bus로
frame이 오가는지 같이 봐야 한다.

## DXL PTY 구조

DXL은 `/dev/ttyUSB0` 하나의 serial bus로 처리한다.

```text
/dev/ttyUSB0
  ID 1: head_pan
  ID 2: head_tilt
```

SIL에서는 `/dev/ttyUSB0`이 실제 USB serial adapter가 아니라 PTY symlink다.

```text
DrumRobot2 / Dynamixel SDK
  -> /dev/ttyUSB0
  -> PTY pair
  -> /tmp/ttyUSB0_sim
  -> simul.py
```

`syncWrite`는 status packet을 기대하지 않는다. `syncRead`는 ID 1, ID 2의 status
packet을 제한 시간 안에 받아야 한다. simulator가 늦게 응답하면 `DrumRobot2` 로그에
`SyncRead failed`가 찍힌다.

현재 simulator는 DXL PTY read/write를 PyBullet/CAN loop에서 분리한 lightweight
thread에서 처리한다. PyBullet GUI나 CAN 처리 때문에 DXL 응답이 늦어지는 것을 줄이기
위한 구조다.

## 모듈 역할

```text
simul.py
  frame-level simulator entrypoint
  CAN/DXL/PyBullet orchestration

setup_sil.sh
  vcan0..3 setup
  DXL PTY pair setup
  /dev/ttyUSB0 symlink protection

sil/decoder.py
  TMotor CAN frame decode
  Maxon CANopen frame decode
  Dynamixel Protocol 2.0 packet decode

sil/encoder.py
  TMotor feedback frame encode
  Maxon TPDO frame encode
  Dynamixel status packet encode

sil/router.py
  CAN ID / DXL ID -> motor name routing
  command -> URDF target dict 변환

sil/mapping.py
  motor catalog
  node id
  production angle -> URDF angle transform
  startup pose

sil/pybullet_backend.py
  PyBullet lifecycle
  URDF loading
  joint state apply/read

sil/visuals.py
  PyBullet world/camera/theme
  drum pad and pedal visualization

sil/urdf_tools.py
  runtime URDF copy 생성
  package:// mesh path rewrite
  link pose patch
```

protocol 처리는 `decoder.py`/`encoder.py`에 둔다. CAN ID나 DXL ID routing은
`router.py`에 둔다. production joint 의미와 URDF joint 의미 사이의 보정은
`mapping.py`에 둔다. `pybullet_backend.py`는 가능하면 단순한 backend로 유지한다.

## Feedback 모델

실제 장치에 맞춘 현재 feedback 모델은 아래와 같다.

### TMotor

```text
command frame 수신
  -> target decode
  -> PyBullet target 적용
  -> 같은 target을 TMotor feedback frame으로 echo
```

TMotor는 command를 받으면 즉시 target echo feedback을 보낸다. 이 echo는
`DrumRobot2`의 다음 safety check에서 current angle이 최신 target 근처로 갱신되도록
돕는다.

또한 discovery/idle을 위해 200Hz status를 유지하되, 어떤 TMotor가 command를 한 번이라도
받은 뒤에는 idle PyBullet feedback을 보내지 않는다. command echo와 idle feedback이
같은 current 값을 서로 덮는 문제를 막기 위해서다.

### Maxon

Maxon은 200Hz 주기 feedback을 계속 뿌리지 않는다. CANopen `0x80` SYNC를 받았을 때,
Operational 상태인 Maxon에 대해서만 TPDO feedback을 보낸다.

```text
0x80 SYNC 수신
  -> Operational Maxon 확인
  -> current position encode
  -> TPDO 전송
```

### DXL

DXL은 주기 feedback이 없다. Dynamixel SDK의 `syncRead` packet을 받았을 때만 status
packet으로 응답한다.

```text
syncWrite(goal)
  -> goal 저장
  -> status 응답 없음

syncRead(ID 1, ID 2)
  -> 최신 goal/current 값을 status packet으로 encode
  -> ID 1 status 전송
  -> ID 2 status 전송
```

`syncRead` 응답은 PyBullet state를 읽지 않고, 마지막으로 받은 DXL goal 또는 startup
pose를 기준으로 만든다. head gesture처럼 짧은 동작에서 PyBullet step이 늦어져도 DXL
status 응답은 빠르게 돌아가야 하기 때문이다.

## Safety와 current angle

`DrumRobot2`의 TMotor send loop는 command를 보내기 전에 safety check를 한다.
즉, simulator가 어떤 command를 받은 뒤 echo를 보내더라도 그 echo는 이미 통과된 command
자신을 구해주는 것이 아니라 다음 command의 current 기준을 갱신한다.

```text
DrumRobot2
  current angle 확인
  desired angle과 차이 검사
  통과하면 CAN command 전송
  이후 SIL이 command를 받고 feedback echo
  다음 command safety에서 갱신된 current 사용
```

그래서 feedback이 밀리거나, 오래된 feedback이 current를 덮으면 다음 command에서
`Set CAN Frame Error : Safety Check`가 발생할 수 있다.

## PyBullet 적용 방식

현재 backend는 actuator dynamics를 시뮬레이션하지 않고 `resetJointState()`로 target을
즉시 적용한다.

장점:

- protocol/frame-level 경로 검증이 단순하다.
- command가 어떤 joint로 들어가는지 빠르게 눈으로 확인할 수 있다.
- dynamics tuning 없이 mapping과 routing 문제를 분리해서 볼 수 있다.

한계:

- 실제 모터의 acceleration, delay, compliance를 재현하지 않는다.
- GUI가 느려지면 시각 표시와 frame 처리 timing이 실제처럼 보장되지 않는다.
- 손목/팔이 순간적으로 꺾여 보이는 현상은 PyBullet backend의 즉시 적용 방식과도 관련될 수 있다.

이 SIL의 목적은 hard realtime physics simulator가 아니라, `DrumRobot2`의 frame-level
입출력 경계를 보존하면서 routing, mapping, feedback timing을 검증하는 것이다.

## 디버깅 명령

CAN frame 확인:

```bash
candump vcan0
candump vcan1
candump vcan2
candump vcan3
```

DXL symlink 확인:

```bash
ls -l /dev/ttyUSB0
ls -l /tmp/ttyUSB0_sim
```

vcan 상태 확인:

```bash
ip link show type vcan
```

Python 문법 확인:

```bash
cd /home/shy/robot_project
python3 -m compileall Drum_intheloop/simul.py Drum_intheloop/sil
```

## 자주 헷갈리는 경계

- `Brain Connected`는 TCP brain 연결을 의미한다. CAN/DXL SIL 준비와는 별개다.
- `k` 입력 전에는 `AgentSocket` gate 때문에 명령이 폐기될 수 있다.
- CSV 파일 생성은 trajectory 생성 완료를 의미하지 않는다. startup metadata만 기록해도 CSV는 생길 수 있다.
- `SyncRead failed`는 DXL 응답 timing/packet 문제일 가능성이 크다. CAN FIFO 문제와는 별개다.
- TMotor safety error는 command를 보내기 전의 current 기준으로 발생한다. 방금 보낼 command의 echo는 아직 존재하지 않는다.
- `vcan` FIFO는 frame 순서를 뒤집지 않는다. 문제가 생기면 queue 순서보다 feedback source와 처리 지연을 먼저 봐야 한다.

## 현재 실행 체크리스트

1. real CAN 장치가 올라와 있지 않은지 확인한다.
2. `./setup_sil.sh`를 먼저 실행하고 터미널을 유지한다.
3. `python3 simul.py --mode gui` 또는 `--mode direct`를 실행한다.
4. `sudo ./main.out`을 실행한다.
5. brain 연결 후 `k`를 입력해 수신 gate를 연다.
6. gesture를 한 번만 보내고 `current_angles`와 `candump`를 본다.
7. 같은 gesture를 연속으로 보낼 때 safety error가 나면 [Trouble shooting.md](<Trouble shooting.md>)의 CAN queue/current 동기화 항목을 확인한다.
