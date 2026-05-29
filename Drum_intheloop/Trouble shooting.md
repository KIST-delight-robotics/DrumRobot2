# Trouble shooting

이 문서는 `Drum_intheloop` frame-level SIL을 붙이면서 실제로 헷갈렸던 문제를 항목별로
정리한 기록이다. 핵심은 “PyBullet에서 움직이는가”보다 “`DrumRobot2`가 받는 feedback이
실제 장치처럼 current angle을 안정적으로 갱신하는가”다.

## 1. 전체 문제를 보는 기준

`DrumRobot2`의 TMotor safety는 command를 보내기 전에 실행된다.

```text
DrumRobot2 send loop
  1. commandBuffer에서 다음 target을 꺼냄
  2. desired angle과 current angle 차이를 검사
  3. 차이가 너무 크면 Set CAN Frame Error
  4. 통과하면 CAN command frame 전송
```

따라서 SIL의 feedback은 “이미 보낸 command”의 결과를 `DrumRobot2` current에 반영해서
그 다음 command safety가 통과하도록 도와야 한다.

```text
command N safety 통과
  -> command N 전송
  -> SIL이 command N 수신
  -> SIL이 command N target을 feedback으로 echo
  -> DrumRobot2 current가 command N 근처로 갱신
  -> command N+1 safety에서 이 current 사용
```

중요한 제한이 있다. command N의 echo는 command N 자신을 통과시키지 못한다. command N의
safety check는 frame이 bus에 나오기 전에 이미 끝났기 때문이다.

## 2. vcan FIFO가 하는 일과 하지 않는 일

`vcan` queue는 frame 순서를 뒤집지 않는다. 먼저 들어간 frame이 먼저 읽힌다.

```text
write order:
  A -> B -> C

read order:
  A -> B -> C
```

그러나 FIFO가 있다고 해서 “항상 최신 current가 유지된다”는 뜻은 아니다. 이유는 두 가지다.

첫째, consumer가 늦으면 queue에 과거 command들이 쌓인다.

```text
DrumRobot2는 5ms마다 command 생성
simul.py가 30ms 동안 밀림

vcan queue:
  [R_arm3 102]
  [R_arm3 98]
  [R_arm3 95]
  [R_arm3 73]
```

둘째, feedback을 만드는 source가 둘 이상이면, 더 늦게 생성된 feedback이 FIFO상으로도
나중에 들어간다. 값이 오래된 PyBullet state에서 왔다고 해도, frame 자체가 나중에
생성되면 FIFO는 그것을 나중 frame으로 취급한다.

즉 “오래된 값이 최신값 뒤로 역전됐다”가 아니라, “늦게 생성된 idle feedback이 낡은 값을
담고 있었다”가 정확한 표현이다.

## 3. 최신 TMotor target만 echo하는 이유

CAN 처리 loop가 밀렸을 때 queue에 있는 모든 target에 대해 feedback을 하나씩 보내면,
`DrumRobot2` current도 과거 trajectory를 뒤늦게 따라간다.

```text
vcan queue:
  [R_arm3 102]
  [R_arm3 98]
  [R_arm3 95]
  [R_arm3 73]

나쁜 처리:
  feedback current=102
  feedback current=98
  feedback current=95
  feedback current=73
```

이 방식은 “모든 과거 sample을 충실하게 재생”한다는 점에서는 그럴듯하지만, 실시간성 관점에서는
이미 늦었다. `DrumRobot2`는 지금 다음 command safety를 봐야 하는데 current가 과거 sample을
순서대로 뒤쫓으면 safety 기준이 흔들린다.

현재 방식은 queue를 비우면서 motor별 최신 target만 남긴다.

```text
vcan queue:
  [R_arm3 102]
  [R_arm3 98]
  [R_arm3 95]
  [R_arm3 73]

simul.py:
  latest_tmotor["R_arm3"] = 102
  latest_tmotor["R_arm3"] = 98
  latest_tmotor["R_arm3"] = 95
  latest_tmotor["R_arm3"] = 73

queue 다 비운 뒤:
  feedback current=73 하나만 보냄
```

그림으로 보면 이렇다.

```text
          밀린 command frame들
                  |
                  v
    +-----------------------------+
    | vcan RX queue               |
    |   R_arm3 102                |
    |   R_arm3 98                 |
    |   R_arm3 95                 |
    |   R_arm3 73                 |
    +-----------------------------+
                  |
                  v
    +-----------------------------+
    | simul.py drain loop         |
    |                             |
    | latest_tmotor[R_arm3] = 102 |
    | latest_tmotor[R_arm3] = 98  |
    | latest_tmotor[R_arm3] = 95  |
    | latest_tmotor[R_arm3] = 73  |
    +-----------------------------+
                  |
                  v
    +-----------------------------+
    | feedback TX                 |
    |   R_arm3 current = 73       |
    +-----------------------------+
```

이건 LIFO stack을 쓰는 것과 비슷해 보이지만 정확히는 “queue를 모두 drain하면서 key별 최신값만
덮어쓰기”다. 여러 모터가 섞여 있으면 모터마다 최신 target 하나씩 남는다.

```text
vcan queue:
  [R_arm1 45]
  [R_arm3 102]
  [R_arm1 46]
  [R_arm3 73]

latest_tmotor:
  R_arm1 = 46
  R_arm3 = 73

feedback:
  R_arm1 current=46
  R_arm3 current=73
```

이렇게 하면 queue가 잠깐 쌓여도 `DrumRobot2` current가 가능한 한 최신 trajectory 지점으로
점프한다. SIL의 목적이 실제 모터 dynamics 재현이 아니라 frame-level integration 확인이므로,
이 편이 safety current 동기화에 더 맞다.

## 4. idle feedback이 command 이후에 문제를 만들 수 있었던 이유

초기 구조에는 TMotor feedback source가 두 개 있었다.

```text
source A: command echo feedback
  command를 받자마자 target을 current로 echo

source B: idle/status feedback
  200Hz 주기로 PyBullet state를 읽어서 current로 전송
```

이 둘이 같은 motor current를 갱신하면 race처럼 보이는 상황이 생긴다.

```text
t = 0ms
  command echo:
    R_arm3 current = 102

t = 5ms
  command echo:
    R_arm3 current = 98

t = 10ms
  idle feedback:
    PyBullet state가 아직 73으로 보임
    R_arm3 current = 73 전송
```

FIFO는 순서를 뒤집지 않았다. idle feedback이 실제로 10ms에 생성됐으니 나중에 들어간 것이
맞다. 문제는 그 값의 source가 command target이 아니라 늦게 읽은 PyBullet state였다는 점이다.

그래서 현재 정책은 아래와 같다.

```text
TMotor가 아직 command를 받은 적 없음:
  discovery/idle용 200Hz status 허용

TMotor가 command를 한 번이라도 받음:
  idle feedback 중단
  command echo만 feedback source로 사용
```

이렇게 하면 `DrumRobot2` current를 갱신하는 source가 하나로 줄어든다.

## 5. “echo 오면 보내는 것”과 “5ms마다 보내는 것”의 차이

`DrumRobot2`의 TMotor command 생성은 대략 5ms sample 기반이다. 즉 C++ send loop가
trajectory sample을 주기적으로 꺼내 CAN command를 보낸다.

SIL feedback은 별개다.

```text
DrumRobot2:
  5ms마다 target command frame 전송

simul.py:
  command frame을 받으면 그 target을 feedback으로 echo
```

따라서 “SIL이 5ms마다 feedback을 만든다”가 아니라 “DrumRobot2가 5ms마다 command를 보내고,
SIL은 받은 command에 반응해서 echo한다”가 더 정확하다.

다만 queue가 밀리면 SIL 입장에서는 여러 command를 한 번에 받는다. 이때 모든 command를
feedback으로 다시 보내면 과거 current가 줄줄이 replay되므로, 현재는 최신값 하나만 보낸다.

## 6. CAN 처리 loop가 왜 밀릴 수 있는가

`vcan` 자체가 느려서라기보다, Python simulator가 항상 hard realtime으로 도는 것이 아니기
때문이다.

밀릴 수 있는 요인:

- PyBullet GUI rendering
- `resetJointState()`와 `stepSimulation()` 호출
- DXL PTY read/write 처리
- Python GIL과 OS scheduler
- 터미널 출력과 debug print
- 여러 bus를 순회하며 CAN frame을 drain하는 비용
- brain/TTS/STT 등 다른 프로세스가 같은 머신 자원을 쓰는 상황

예를 들어 `DrumRobot2`가 5ms마다 R_arm3 target을 보내는데 `simul.py`가 어떤 이유로 25ms 동안
CAN queue를 못 읽으면, R_arm3 sample 5개가 쌓일 수 있다.

```text
시간:
  0ms   R_arm3 110
  5ms   R_arm3 104
  10ms  R_arm3 98
  15ms  R_arm3 90
  20ms  R_arm3 73

simul.py가 25ms에 깨어남:
  queue에 5개가 이미 쌓여 있음
```

이때 가장 중요한 것은 “과거 5개를 모두 시각적으로 재생하는 것”이 아니라
“`DrumRobot2` current를 다음 safety check에 맞는 최신값으로 빨리 갱신하는 것”이다.

## 7. GUI Hz를 올리면 해결되는가

PyBullet GUI의 표시 주기나 rendering 설정을 줄이는 것은 도움이 될 수 있다. 하지만 이것은
근본 해결이라기보다 부하를 줄이는 조치다.

문제의 핵심은 `DrumRobot2`의 control loop와 Python/PyBullet GUI loop가 같은 hard realtime
시간축을 공유하지 않는다는 점이다. GUI가 더 빨라져도 다음 문제는 남는다.

- OS scheduler가 Python process를 늦게 깨울 수 있다.
- DXL status 응답이 deadline 안에 못 갈 수 있다.
- queue가 이미 쌓인 뒤에는 모든 frame을 replay하는 방식이 current를 늦춘다.

그래서 GUI 최적화보다 먼저 적용해야 하는 원칙은 “queue를 drain하고 최신값만 current로
되돌린다”다.

## 8. hard realtime으로 맞추는 방법은 어떤가

가능은 하지만 이 SIL의 목적과 비용을 생각하면 우선순위가 낮다.

hard realtime에 가까워지려면 다음이 필요하다.

- Python/PyBullet 대신 C++ 또는 realtime-friendly process로 bus responder 분리
- GUI와 protocol responder 분리
- thread priority, CPU affinity, realtime kernel 설정
- CAN/DXL deadline 계측
- lock-free queue 또는 shared memory 설계

하지만 지금 문제는 실제 물리 제어기를 만드는 문제가 아니라, `DrumRobot2`가 보내는 frame과
feedback current가 논리적으로 맞는지 보는 문제다. 따라서 현재 단계에서는 다음 정책이 더
단순하고 효과적이다.

```text
CAN:
  queue를 최대한 빨리 drain
  motor별 최신 target만 feedback

DXL:
  PyBullet loop와 분리된 PTY thread에서 즉시 status 응답

Maxon:
  SYNC 때만 TPDO 응답

PyBullet:
  표시/검증용 target 적용
```

## 9. SyncRead failed가 발생하는 이유

`DrumRobot2`의 DXL 흐름은 대략 아래와 같다.

```text
syncWrite(goal)
  -> DXL goal position 쓰기
  -> status packet 기대 안 함

syncRead(present position)
  -> ID 1 status packet 기다림
  -> ID 2 status packet 기다림
  -> timeout 안에 못 받으면 SyncRead failed
```

Dynamixel Protocol 2.0의 `syncRead`는 하나의 broadcast read instruction에 대해 여러 ID가
각자 status packet을 돌려주는 구조다. `DrumRobot2`는 ID 1과 ID 2를 등록해 두었으므로,
SIL도 두 packet을 순서대로 빠르게 써줘야 한다.

현재 기대 흐름:

```text
DrumRobot2 -> /dev/ttyUSB0:
  SyncRead address=132 length=4 ids=[1,2]

simul.py -> /tmp/ttyUSB0_sim:
  Status packet ID 1 present_position
  Status packet ID 2 present_position
```

실패 원인은 보통 셋 중 하나다.

```text
1. SIL이 SyncRead packet을 못 읽음
2. 읽었지만 status packet을 늦게 씀
3. status packet을 썼지만 packet 형식/길이가 SDK 기대와 다름
```

현재 코드에서는 packet 형식보다 timing 가능성이 더 크다고 본다. ping과 기본 status packet
형태가 맞고, local decode/encode 확인도 통과했기 때문이다. 예전 구조에서는 DXL 처리가
PyBullet/CAN main loop 안에 있었기 때문에 GUI나 CAN drain에 밀리면 status packet이 늦었다.

현재는 DXL PTY 응답을 별도 thread로 분리했다.

```text
before:
  PyBullet/CAN/DXL을 하나의 loop에서 처리
  -> GUI나 CAN이 밀리면 SyncRead 응답도 늦음

after:
  DXL PTY thread가 packet read/write 담당
  main loop는 pending DXL target만 PyBullet에 반영
```

변경 후에도 `SyncRead failed`가 계속 뜨면 simulator를 완전히 재시작했는지 먼저 확인한다.
이전 `simul.py` process가 살아 있으면 새 구조가 적용되지 않는다.

## 10. SyncRead failed가 TMotor safety까지 흔드는 이유

`SyncRead failed`는 head/DXL 문제처럼 보이지만, 같은 `DrumRobot2` send loop에서 발생하면
전체 loop timing을 흔들 수 있다.

```text
send loop:
  TMotor send
  Maxon send
  DXL syncWrite
  DXL syncRead  <- 여기서 block/timeout
  다음 loop
```

DXL `syncRead`가 timeout까지 기다리면 그동안 다음 TMotor command 처리도 늦어질 수 있다.
그러면 CAN queue와 current feedback이 다시 밀리고, 결과적으로 TMotor safety error로 이어질
수 있다.

따라서 `SyncRead failed`는 단순히 head가 안 움직이는 문제로만 보면 안 된다. gesture 전체의
timing과 current 동기화에 영향을 줄 수 있다.

## 11. 손목이나 팔이 말이 안 되는 궤적으로 꺾여 보이는 경우

가능한 원인을 층별로 나눠야 한다.

### 11.1 Mapping 문제

팔이 계속 반대 방향으로 꺾이면 `sil/mapping.py`의 production-to-URDF transform을 먼저 본다.

```text
production angle
  -> sign/reference/bias transform
  -> URDF joint angle
```

이 문제는 보통 특정 joint가 항상 반대로 움직인다.

### 11.2 Feedback current 문제

각도 방향은 맞는데 safety error가 나거나 두 번째 gesture에서 터지면 feedback current가 밀렸을
가능성이 높다.

```text
DrumRobot2 desired:
  R_arm3 100

DrumRobot2 current:
  R_arm3 70

safety:
  차이 30도 초과
  -> Set CAN Frame Error
```

이때 PyBullet 화면에서 “움직인 것처럼 보였는가”보다 `DrumRobot2`가 받은 current feedback이
어떤 값이었는지가 더 중요하다.

### 11.3 PyBullet 즉시 적용 문제

현재 backend는 motor dynamics 없이 `resetJointState()`를 쓴다. 그래서 손목/팔이 실제
모터처럼 부드럽게 보간되지 않고 순간적으로 target에 붙어 보일 수 있다.

```text
실제 장치:
  target -> motor profile -> 물리적으로 이동

현재 PyBullet:
  target -> resetJointState -> 즉시 이동
```

따라서 시각적으로 꺾임이 과해 보인다고 해서 항상 C++ trajectory가 틀렸다고 볼 수는 없다.
먼저 feedback current와 mapping을 분리해서 봐야 한다.

## 12. 연속 gesture에서 더 잘 터지는 이유

한 번의 gesture가 끝난 직후 current가 home으로 완전히 동기화되지 않았는데 같은 gesture가 다시
들어오면, 다음 trajectory 첫 target과 current 차이가 크게 잡힐 수 있다.

예:

```text
첫 번째 wave 중간/끝:
  DrumRobot2 current R_arm3 = 70

두 번째 wave 시작:
  desired R_arm3 = 100

safety:
  abs(100 - 70) = 30
  limit 초과 또는 경계
  -> error
```

이 문제는 “gesture 명령 자체가 잘못됐다”기보다 “이전 동작의 feedback current가 다음 동작 시작
시점에 맞게 들어왔는가” 문제로 보는 편이 좋다.

## 13. 현재 코드의 의도

현재 `simul.py` 쪽 의도는 다음과 같다.

```text
TMotor:
  command 수신 시 target echo
  command 이후 idle PyBullet feedback 중단
  CAN queue가 밀리면 motor별 최신 target 하나만 echo

Maxon:
  200Hz feedback 없음
  0x80 SYNC 수신 시 Operational Maxon TPDO 전송

DXL:
  주기 feedback 없음
  SyncRead packet 수신 시 status packet 응답
  PTY 응답은 PyBullet/CAN loop와 분리

PyBullet:
  검증/표시용
  actuator dynamics는 아직 모델링하지 않음
```

이 구조의 핵심은 feedback source를 줄이고, `DrumRobot2` current를 가능한 한 최신 target과
맞추는 것이다.

## 14. 확인 순서

문제가 다시 나면 아래 순서로 보는 것이 좋다.

### 14.1 process 재시작

```bash
pkill -f "python3 simul.py"
```

그 뒤 다시 실행한다.

```bash
cd /home/shy/robot_project/Drum_intheloop
python3 simul.py --mode gui
```

### 14.2 vcan 확인

```bash
ip link show type vcan
candump vcan0
candump vcan1
candump vcan2
candump vcan3
```

### 14.3 DXL PTY 확인

```bash
ls -l /dev/ttyUSB0
ls -l /tmp/ttyUSB0_sim
```

`/dev/ttyUSB0`이 SIL symlink가 아니라 실제 장치이거나 다른 symlink면 `setup_sil.sh`가 의도대로
동작하지 않은 것이다.

### 14.4 Python syntax 확인

```bash
cd /home/shy/robot_project
python3 -m compileall Drum_intheloop/simul.py Drum_intheloop/sil
```

### 14.5 로그에서 볼 것

CAN safety error가 나면 이 세 값을 먼저 본다.

```text
Set CAN Frame Error : Safety Check (R_arm3)
Desired Joint Angle : 100.45deg
Current Joint Angle : 70.2deg
```

이 경우 simulator 화면보다 `Current Joint Angle`이 왜 70.2에 머물렀는지를 추적해야 한다.

`SyncRead failed`가 반복되면 아래를 본다.

```text
DXL SyncRead packet을 SIL이 읽었는가
ID 1/2 status packet을 썼는가
응답이 timeout 전에 갔는가
```

필요하면 다음 단계는 raw DXL packet logging이다.

## 15. 결론

이번 디버깅의 핵심 결론은 다음이다.

- `vcan` FIFO가 순서를 뒤집은 것이 아니다.
- queue가 밀리면 과거 command가 쌓이고, 모든 과거 feedback을 replay하면 current가 늦어진다.
- command echo와 idle PyBullet feedback이 같이 current를 갱신하면 source가 섞인다.
- TMotor는 command 이후 idle feedback을 끄고 최신 target echo만 쓰는 것이 현재 SIL 목적에 맞다.
- DXL `SyncRead failed`는 DXL만의 문제가 아니라 send loop timing 전체를 흔들 수 있다.
- PyBullet GUI는 검증 도구이지 hard realtime motor emulator가 아니다.
