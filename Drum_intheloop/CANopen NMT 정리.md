# CANopen NMT 정리

이 문서는 frame-level SIL에서 Maxon 모터 일부가 확률적으로 `Not Connected`로 떨어지던
문제를 추적하고, 그 근본 원인이 CANopen NMT state machine을 시뮬레이터가 모델링하지 않은
것이라고 결론낸 뒤, 어떻게 해결했는지를 순서대로 정리한 기록이다.

PDO/SDO/NMT가 각자 무엇이고 왜 분리되어 있는지, 그래서 `DrumRobot2`가 실하드웨어에선
잘 잡던 모터를 simulator에선 왜 가끔 못 잡았는지, 그리고 simulator에 무엇만 추가하면
이걸 안정화할 수 있는지를 차례대로 본다.

---

## 1. TL;DR

- **문제**: SIL에서 Maxon 모터 4개 중 1~2개가 매 실행마다 다르게 `Not Connected`로 빠짐.
- **원인**: simulator가 Maxon TPDO(`0x180+node`)를 처음부터 200Hz로 쏘고 있어서 noise가
  많았고, `setMotorsSocket()`의 10-frame read window 안에 정작 발견에 필요한 SDO ack
  (`0x580+node`)가 가끔 들어오지 못함.
- **왜 실하드웨어는 괜찮았나**: 실 Maxon EPOS는 부팅 직후 `Pre-Operational` 상태라
  TPDO 자체를 안 쏜다. 그래서 발견 단계엔 bus가 거의 조용하고 SDO ack가 안정적으로 도착.
- **해결**: simulator에 NMT state(`Pre-Operational`/`Operational`/`Stopped`)를 모터별로
  추가하고, `Operational` 상태가 아닌 Maxon은 주기 feedback을 안 보내도록 게이팅. C++ 쪽
  `getOperational`이 NMT `Start Remote Node`(`0x000, [0x01, nodeId]`)를 보낼 때 그 노드만
  `Operational`로 전이시킴.
- **건드린 곳**: `sil/motor_state.py`(신규), `sil/decoder.py`, `simul.py`,
  `sil/encoder.py`(문서 헤더).

---

## 2. 현상

`DrumRobot2`를 SIL로 띄울 때 콘솔에 찍히는 모터 발견 로그:

```text
--------------> CAN NODE ID 0 Connected. Motor [waist]
--------------> CAN NODE ID 1 Connected. Motor [R_arm1]
--------------> CAN NODE ID 2 Connected. Motor [L_arm1]
--------------> CAN NODE ID 3 Connected. Motor [R_arm2]
--------------> CAN NODE ID 4 Connected. Motor [R_arm3]
--------------> CAN NODE ID 5 Connected. Motor [L_arm2]
--------------> CAN NODE ID 6 Connected. Motor [L_arm3]
CAN NODE ID 7 Not Connected. Motor [R_wrist]
--------------> CAN NODE ID 8 Connected. Motor [L_wrist]
CAN NODE ID 9 Not Connected. Motor [maxonForTest]
CAN NODE ID 10 Not Connected. Motor [R_foot]
--------------> CAN NODE ID 11 Connected. Motor [L_foot]
```

- TMotor 7개(node 0~6)는 항상 잡힌다.
- Maxon은 4개(R_wrist 7, L_wrist 8, R_foot 10, L_foot 11)인데 매번 1~2개가 빠짐.
  어떤 게 빠질지는 실행마다 다름. 즉 **확률적**.
- `maxonForTest`(node 9)는 simulator MAXON_SPEC에 없어서 의도적으로 못 잡는 게 정상.

확률적이라는 게 단서다. 코드 자체에 일관된 누락이 있는 게 아니라 timing/race 문제일
가능성이 높다는 뜻이다.

---

## 3. 배경: CAN과 CANopen

### CAN

CAN(Controller Area Network)은 1980년대에 자동차용으로 만들어진 저수준 직렬 버스다. CAN
프레임 한 개에는 다음만 있다.

```text
- arbitration ID (11-bit 또는 29-bit)
- DLC (0~8 byte payload 길이)
- data 0~8 byte
- CRC
```

여기서 끝이다. CAN 자체는 "이 ID가 무슨 뜻"이라는 약속이 없다. 그건 위에 올라가는
프로토콜이 정한다.

### CANopen

CANopen(공식 명세 CiA DS-301)은 CAN 위에 얹는 application layer 표준이다. 11-bit
arbitration ID를 `function code | node_id`로 쪼개서 누가 누구에게 무슨 종류의 메시지를
보내는지를 표준화한다.

```text
COB-ID (11-bit) = function code (7-bit) + node_id (7-bit)
                  (실제로는 4-bit fc + 7-bit nid)
```

표준 function code 표:

| function code | 용도 | 방향 |
|---|---|---|
| `0x000` | NMT (network management) | master → slave |
| `0x080` | SYNC / EMCY | broadcast / slave → master |
| `0x180 + node` | TPDO1 (state, e.g. position) | slave → master |
| `0x200 + node` | RPDO1 (control word) | master → slave |
| `0x280 + node` | TPDO2 | slave → master |
| `0x300 + node` | RPDO2 (e.g. position target) | master → slave |
| `0x400 + node` | RPDO3 (e.g. velocity target) | master → slave |
| `0x500 + node` | RPDO4 (e.g. torque target) | master → slave |
| `0x580 + node` | SDO response | slave → master |
| `0x600 + node` | SDO request | master → slave |
| `0x700 + node` | heartbeat / boot-up | slave → master |

`sil/mapping.py`의 `maxon_ids()`가 이 표를 그대로 옮겨놓은 것이다. 따라서 simulator는
이미 CANopen ID 규칙을 알고 있다.

### Master / Slave

CANopen은 master 하나 + slave N개 구조다.

- master: 네트워크 관리, NMT 명령 발사, slave 설정. → 우리 경우 `DrumRobot2`.
- slave: 각 모터 컨트롤러(Maxon EPOS). master 명령에 따라 동작하고 자기 상태를 보고.

master는 한 명만 있어야 한다. 같은 bus에 master가 둘이면 NMT 명령이 충돌한다.

---

## 4. PDO vs SDO

### PDO (Process Data Object)

- **목적**: 실시간 cyclic 데이터(현재 위치, 토크 명령 등).
- **방향**: 단방향. "쏘는 쪽이 보내고 끝".
  - TPDO(Transmit PDO): slave가 master에게 (position feedback 등)
  - RPDO(Receive PDO): master가 slave에게 (target position 등)
- **포맷**: 헤더 없이 payload 8 byte가 곧 데이터. 가볍고 빠름.
- **신뢰성**: 응답 없음. "받았겠지" 가정.

### SDO (Service Data Object)

- **목적**: 설정/객체 사전 접근(가속도 한계, 동작 모드 같은 한 번 설정하는 값).
- **방향**: request/response. master가 `0x600+node`로 묻고 slave가 `0x580+node`로 답.
- **신뢰성**: 양쪽 다 ack가 있어서 "상대가 살아 있고 알아들었는지" 확신할 수 있음.
- **속도**: PDO보다 느림(헤더 오버헤드 + 응답 대기).

### 비유

PDO/SDO는 "메시지의 종류"지 "상태 변수"가 아니다.

- PDO = "지금 내 위치는 X" 같은 라디오 방송. 누가 듣든 신경 안 씀.
- SDO = "이 설정값 알려줘 → 알겠다, X야" 같은 전화 통화. 양쪽 다 듣고 답함.

simulator에서 따로 변수로 추적할 필요가 없다. 그냥 메시지 만들어서 보내고/받고 끝.

### NMT (Network Management)

NMT는 PDO/SDO와 달리 **상태 변수**다. 각 slave는 자기 상태 머신을 갖는다.

```text
[powerup]
   │
   ▼
Initialisation
   │ (자동으로 boot)
   ▼
Pre-Operational  ──── SDO 가능, TPDO 불가 ─────┐
   │                                              │
   │ master가 NMT Start (0x000, [0x01, node])     │
   ▼                                              │
Operational     ──── SDO + TPDO 모두 가능        │
   │                                              │
   │ master가 NMT Stop (0x000, [0x02, node])     │
   ▼                                              │
Stopped         ──── 거의 모든 통신 멈춤         │
```

**핵심**: TPDO는 `Operational` 상태에서만 송신된다. Pre-Op에선 slave가 PDO를 한 글자도
안 보낸다. SDO는 두 상태 모두에서 동작한다(설정 작업은 Pre-Op에서 함).

NMT 명령은 한 가지 ID(`0x000`)를 쓰고 payload 첫 byte로 command를 구분한다.

| data[0] | 의미 |
|---|---|
| `0x01` | Start Remote Node → Operational |
| `0x02` | Stop Remote Node → Stopped |
| `0x80` | Enter Pre-Operational |
| `0x81` | Reset Node |
| `0x82` | Reset Communication |

`data[1]`은 target node ID. 0이면 broadcast(모든 노드).

---

## 5. TMotor와 Maxon의 차이

이 차이가 발견 race의 직접 원인이다.

### TMotor (CubeMars AK servo mode)

- **자체 proprietary CAN 프로토콜**. CANopen 아님.
- 11-bit CAN ID에 `packet_id << 8 | node_id` 식으로 회사가 자기 layout을 정함.
  - packet_id 0 (feedback) → `0x000 + nodeId`
  - packet_id 4 (SET_POS) → `0x400 + nodeId`
- NMT 같은 state machine **없음**. 전원 인가하면 그냥 status frame을 자동 broadcast.
- C++ discovery는 `(can_id & 0xFF) == nodeId`만 보면 됨. 어차피 그 노드가 쏜 거니까.

### Maxon EPOS

- **CANopen 표준** 따름.
- `0x180+nodeId`, `0x580+nodeId`, `0x600+nodeId` 같은 ID 규칙 준수.
- **NMT state machine 가짐**. Pre-Op로 부팅 → master가 Start 보낼 때만 Operational.
- 그래서 발견 시점엔 아직 Pre-Op라 TPDO를 안 쏜다. master가 SDO ping(`getCheck`)을
  보내야만 SDO ack로 자기 존재를 알린다.

이 비대칭이 다음 절의 race를 만든다.

---

## 6. 발견 race 원인 분석

### `DrumRobot2`의 발견 로직

`CanManager::setMotorsSocket()`이 발견 루프다. 핵심만 정리하면:

```cpp
for (각 socket(bus)) {
    for (아직 안 잡힌 각 모터) {
        motor->socket = socket_fd;
        if (Maxon) {
            maxoncmd.getCheck(*maxonMotor, &frame);  // 0x600+nodeId 발사
            txFrame(...);
        }
        usleep(50000);  // 50ms 대기
    }

    // 그 bus에서 최대 10 frame만 읽음
    while (readCount < 10) {
        read(socket_fd, &frame, ...);
        tempFrames[socket_fd].push_back(frame);
        readCount++;
    }

    // 각 모터 마다 매칭 검사
    for (각 모터) {
        if (TMotor) {
            if (frame.can_id & 0xFF == nodeId) → Connected
        } else if (Maxon) {
            if (frame.can_id == 0x580 + nodeId) → Connected
        }
    }
}
```

여기서 두 가지를 주목.

1. **TMotor 매칭 조건**: `(can_id & 0xFF) == nodeId`. 그 노드에서 온 어떤 frame이든 OK.
   주기적 status frame이 매 5ms마다 흘러오니까 50ms 대기 동안 충분히 들어옴.
2. **Maxon 매칭 조건**: `can_id == 0x580 + nodeId` (SDO 응답)만 인정. TPDO(`0x180+node`)는
   매칭 안 시킴. 그래서 SDO ack가 반드시 10-frame read window 안에 들어와야 함.

### Simulator의 잘못된 노이즈

기존 simulator는 NMT state machine을 모델링하지 않았다. `_send_feedback`이 처음부터
200Hz로 모든 motor의 feedback을 쏘고 있었다. Maxon도 예외 없이 `0x180+nodeId`(TPDO1)를
200Hz로 박았다.

발견 시점에 같은 bus에 일어나는 트래픽을 vcan2(L_foot, R_foot)로 예를 들면:

- 50ms 대기 동안: L_foot TPDO 10번, R_foot TPDO 10번 → 약 20 frame 쌓임
- 거기에 SDO ack 2개(L_foot용, R_foot용) 더해짐
- C++가 10 frame만 읽음 → SDO ack가 그 10 frame 안에 든다는 보장이 없음

확률적으로 SDO ack가 noise 사이에 묻혀서 발견 실패가 나는 것이다.

### 왜 실하드웨어에서는 안 일어났나

실 Maxon EPOS는 부팅 직후 `Pre-Operational` 상태다. **PDO를 한 글자도 안 쏜다**. 그래서
발견 시점의 bus는 거의 silent.

```text
실하드웨어 vcan2 발견 시점:
  - L_foot TPDO: (없음, Pre-Op)
  - R_foot TPDO: (없음, Pre-Op)
  - SDO ack 2개

→ 10-frame read window에 SDO ack가 무조건 들어옴 → 100% 발견 성공.
```

실하드웨어에선 C++ 코드가 운 좋게 동작했던 게 아니라, CANopen 표준이 정확히 그 시점에
bus를 조용하게 유지해 주는 덕분이었다.

이게 SIL에서 깨졌던 거다.

### 그 외 후보들의 기각

- "10-frame 한도가 너무 작다": 사실이긴 한데 실하드웨어와 일관성을 위해 C++ 코드는
  안 건드리는 게 낫다.
- "TMotor feedback이 시끄럽다": vcan2/3에는 TMotor 없어서 무관함. vcan0(waist + L_arm)에는
  영향 있지만 거기 Maxon 없음.
- "SDO format이 비표준이라 EPOS가 응답 안 함": 비표준이긴 한데 EPOS는 어떤 SDO request에도
  abort 응답을 보내준다. 그래서 매칭은 됨. simulator도 그걸 흉내내고 있음.

---

## 7. 해결: NMT state 추가

원칙은 단순했다. "C++ 쪽이 그 정보를 보고 행동을 바꾸지 않는 protocol 요소는
simulator도 모델링 안 한다." 그래서 heartbeat, EMCY, SDO segmentation, full PDO mapping
등은 그대로 안 하고, **NMT state만** 추가했다.

### 신규: `sil/motor_state.py`

```python
class NmtState:
    def __init__(self) -> None:
        self._state: Dict[str, str] = {motor: PRE_OPERATIONAL for motor in MAXON_SPEC}

    def is_operational(self, motor: str) -> bool:
        return self._state.get(motor) == OPERATIONAL

    def transition(self, kind: str, motor: Optional[str]) -> None:
        next_state = _KIND_TO_STATE.get(kind)
        if next_state is None:
            return

        targets: List[str] = [motor] if motor else list(self._state.keys())
        for target in targets:
            if target in self._state:
                self._state[target] = next_state
```

- Maxon 노드만 추적. TMotor/DXL은 NMT 없으니 안 다룸.
- 시작 시 모두 `Pre-Operational`.
- `transition()`은 kind 문자열("nmt_start", "nmt_stop" 등)과 target motor를 받아 상태 변경.
- target이 `None`이면 broadcast → 모든 Maxon에 적용.

### 변경: `sil/decoder.py`

NMT 프레임을 인식하는 `_decode_nmt()`를 추가하고 `decode_can_frame()` 맨 앞에서 호출.

```python
NMT_COB_ID = 0x000

_NMT_COMMAND_KIND = {
    0x01: "nmt_start",
    0x02: "nmt_stop",
    0x80: "nmt_preop",
    0x81: "nmt_reset",
    0x82: "nmt_reset",
}


def _decode_nmt(can_id, data):
    if can_id != NMT_COB_ID or len(data) < 2:
        return None

    kind = _NMT_COMMAND_KIND.get(data[0])
    if kind is None:
        return None

    node_id = data[1]
    motor = _maxon_by_node(node_id) if node_id != 0 else None
    return CanCommand(kind=kind, motor=motor, can_id=can_id)
```

NMT를 가장 먼저 검사해야 한다. `waist`의 node_id가 0이라서 `_decode_tmotor`가 `0x000`
frame을 packet_id=0의 waist feedback으로 오해할 가능성이 있기 때문이다.

### 변경: `simul.py`

```python
# CAN 프레임 처리 반복
def _poll_can(self):
    ...
    command = decode_can_frame(frame)
    ...
    if command.kind.startswith("nmt_"):
        self.nmt_state.transition(command.kind, command.motor)
        continue
    ...
```

```python
# 모션/피드백 반복
def _send_feedback(self):
    ...
    for motor, can_bus in self.motor_bus.items():
        if motor in MAXON_SPEC and not self.nmt_state.is_operational(motor):
            continue
        frame = motor_feedback(motor, state)
        ...
```

### 전체 흐름

```text
[T=0]  simulator 시작
       모든 Maxon = Pre-Operational
       → Maxon TPDO 송신 안 함

[T=1]  DrumRobot2 setMotorsSocket() 진입
       각 bus에서 SDO check (0x600+node) 발사
       → simulator가 SDO ack (0x580+node) 응답
       → bus가 조용해서 10-frame window에 ack가 안정적으로 들어감
       → 모든 Maxon Connected

[T=2]  DrumRobot2 maxonMotorEnable() 진입
       각 Maxon에 getOperational() (0x000, [0x01, nodeId]) 발사
       → simulator decoder가 "nmt_start"로 인식
       → simulator가 해당 Maxon을 Operational로 전이
       → 그 시점부터 simulator가 그 Maxon의 TPDO를 200Hz로 송신

[T=3+] 정상 동작
       TPDO가 흘러서 DrumRobot2의 current angle 갱신
```

C++ 쪽은 한 줄도 안 건드렸다. simulator가 실 EPOS의 NMT 동작을 흉내내는 것만으로 충분.

---

## 8. 모델링 안 한 것과 그 이유

| 요소 | 모델링 여부 | 이유 |
|---|---|---|
| TPDO1 cyclic feedback | O | `DrumRobot2`가 current angle 갱신에 필수로 사용 |
| SDO ack | O (단발) | 발견과 일부 설정에 필요 |
| NMT state | O (지금 추가) | 발견 race 직접 원인 |
| RPDO (target 수신) | O | DrumRobot2가 보내는 명령 받아야 함 |
| SYNC (`0x80`) | O (TPDO trigger용) | `DrumRobot2`가 `getSync` 사용 |
| Heartbeat (`0x700+node`) | X | `DrumRobot2`가 heartbeat을 안 읽음 |
| EMCY (`0x080+node`) | X | 사용 안 함 |
| SDO segmentation | X | 단발 ack로 충분 |
| Object dictionary | X | kp/kd 등은 어차피 dynamics가 없어서 무의미 |
| PDO mapping 설정 | X | 고정 매핑으로 가정 |
| Actuator dynamics | X | `resetJointState()` 기반 teleport |

판단 기준은 단순하다. "C++ 쪽이 그걸 안 보거나 그것 없이도 동작이 똑같으면 안 만든다."
이 기준은 simulator 복잡도를 낮게 유지하는 데 결정적이다.

---

## 9. 의도된 한계

지금 simulator는 모터의 "위치 viewer"에 가깝지 "actuator simulator"가 아니다.

- **Position 명령**: `resetJointState()`로 즉시 teleport. kp/kd, profile acceleration,
  profile velocity 같은 부드러운 궤적 파라미터는 영향 없음.
- **Velocity 명령**: `router.py`가 자체 Euler 적분 (`joint_deg + velocity * dt`).
- **Torque 명령**: `router.py`가 간이 dynamics (`accel = torque*gain - velocity*damping`).
  단 이건 진짜 관성 행렬/중력 보상이 있는 동역학이 아니라 1자유도 spring-damper 근사.
- **NMT**: Pre-Op/Operational/Stopped 전이만 있고, Reset Communication 후 SDO 재설정
  같은 정밀 시퀀스는 없음.

이 한계들은 발견 race와 무관하다. 필요해지면 그때 별도 의사결정으로 확장한다.

---

## 10. 참고 위치

| 항목 | 파일 |
|---|---|
| NMT state 클래스 | `Drum_intheloop/sil/motor_state.py` |
| NMT frame 디코딩 | `Drum_intheloop/sil/decoder.py` `_decode_nmt` |
| NMT gating | `Drum_intheloop/simul.py` `_send_feedback` |
| Maxon CAN ID 표 | `Drum_intheloop/sil/mapping.py` `maxon_ids()` |
| C++ 발견 로직 | `DrumRobot2/src/CanManager.cpp` `setMotorsSocket()` |
| C++ NMT Start 송신 | `DrumRobot2/src/CommandParser.cpp` `getOperational()` |
| C++ Maxon enable 시퀀스 | `DrumRobot2/src/DrumRobot.cpp` `maxonMotorEnable()` |

---

## 11. 한 줄 요약

CANopen NMT는 PDO/SDO와 달리 진짜 상태 변수다. simulator가 그 상태 머신을 모델링하지
않아서 모터가 "아직 자기 소개도 안 한 상태"인데 자기 위치를 떠들고 있었고, 그 떠드는
소리에 정작 자기 소개(SDO ack)가 묻혀서 `DrumRobot2`가 못 알아들었던 거다. NMT 상태만
흉내내면 simulator는 실 EPOS와 같은 타이밍으로 침묵하다가 발견 후에 떠들기 시작한다.
