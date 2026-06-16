# CONTRACTS — 컴포넌트 간 인터페이스 계약 (단일 소스)

> **이 파일은 단일 소스(single source of truth)다.**
> `phil-brain`(LLM) / `phil-controller`(C++ 제어기) / `phil-sil`(SIL) 세 레포에
> **같은 내용으로 복사**되어 들어간다. 한 곳을 고치면 나머지 두 곳도 같이 고친다.
> 여기 적힌 값은 추출 시점의 코드에서 뽑은 것이며, **최종 권위는 항상 각 레포의 코드**다.
> 코드와 이 문서가 어긋나면 코드를 믿고, 이 문서를 갱신하라.

이 프로젝트는 세 개의 독립 프로세스로 나뉜다. 각 프로세스는 아래 두 계약(Contract A, B)으로만
서로 연결된다. **계약만 지키면 각 컴포넌트는 독립적으로 교체·재작성할 수 있다.**

```text
┌─────────────┐   Contract A          ┌──────────────────┐   Contract B           ┌──────────────┐
│  phil-brain │  TCP 9999 (NDJSON)     │  phil-controller │  SocketCAN + DXL serial │   phil-sil   │
│  (LLM brain)│ ────────────────────▶  │   (C++ 제어기)    │ ─────────────────────▶ │  (또는 실제   │
│             │ ◀──── state JSON ───── │                  │ ◀──── feedback frame ── │   하드웨어)   │
└─────────────┘                        └──────────────────┘                         └──────────────┘
```

- **Contract A**: `phil-brain` ↔ `phil-controller`, TCP 포트 `9999`, 줄바꿈 구분 JSON/명령
- **Contract B**: `phil-controller` ↔ `phil-sil`(또는 실제 모터 bus), SocketCAN `can_frame` + Dynamixel Protocol 2.0 serial packet

`phil-controller`가 두 계약의 **허브**다. CAN/DXL 출력은 실제 하드웨어든 SIL이든 동일하다
(SIL은 환경변수가 아니라 인터페이스 존재 여부로만 결정된다).

---

## Contract A — Brain ↔ Controller (TCP 9999)

### A.1 연결

- **서버**: `phil-controller` (`AgentSocket`). `INADDR_ANY:9999`, `SOCK_STREAM`, listen backlog 3.
- **클라이언트**: `phil-brain` (`runtime/phil_client.py`). 기본 `127.0.0.1:9999`로 접속.
- **방향**: brain(client) → controller(server)로 접속. 별도 handshake 없음. 접속 성공 시 controller가
  `">>> [Agent] Brain Connected!"` 로그를 찍는다.
- **인코딩**: UTF-8.
- **프레이밍**: 줄바꿈(`\n`) 구분. 양방향 모두 한 메시지 = 한 줄(`\n` 종료).
  - 예외: `{`로 시작하는 multi-line JSON 명령은 controller가 중괄호 균형이 맞을 때까지 누적해 파싱한다.

### A.2 Brain → Controller 명령 (한 줄 + `\n`)

| 명령 | 형식 | 의미 |
|------|------|------|
| ready | `r` | ready pose |
| home | `h` | home (게이트 우회) |
| stop | `s` | 즉시 정지(버퍼 flush) |
| play | `p:<song_code>` | 곡 연주. song_code ∈ `{TIM, TY_short, BI, test_one}` |
| move | `move:<joint>,<angle>` | 단일 관절 절대각(도). 예: `move:waist,45` |
| gesture | `gesture:<name>` | name ∈ `{hi, nod, shake, wave, hurray, happy}` |
| look | `look:<pan>,<tilt>` | pan ∈ [-90,90], tilt ∈ [0,120] (도) |
| pause | `pause` | 연주 일시정지 (게이트 우회) |
| resume | `resume` | 중단 위치부터 재개 (게이트 우회) |
| tempo | `tempo_scale:<value>` | 다음/현재 연주 템포 보정 (게이트 우회) |
| velocity | `velocity_delta:<value>` | 타격 세기 보정 (게이트 우회) |

**관절 범위(brain 측 validator가 전송 전 검증, 단위 도):**

```text
waist   (-90, 90)     R_arm1 (0, 150)    L_arm1 (30, 180)
R_arm2  (-60, 90)     R_arm3 (0, 140.1)
L_arm2  (-60, 90)     L_arm3 (0, 140.1)
R_wrist (-108, 90)    L_wrist (-108, 90)
```

> brain 측 범위는 1차 방어선일 뿐이다. 최종 안전 한계는 controller(C++) 측 범위를 따른다.

### A.3 Controller → Brain 상태 broadcast (한 줄 JSON + `\n`)

- 주기 100ms, **직전 전송과 달라졌을 때만** 전송. `state == 2`(연주 중)에는 angle spam 억제.
- 각도는 소수 2자리 float(도).

```json
{
  "state": 0,
  "bpm": 100,
  "is_fixed": true,
  "current_song": "None",
  "progress": "0/0",
  "is_lock_key_removed": false,
  "last_action": "None",
  "current_angles": {
    "waist": 0.00, "R_arm1": 0.00, "L_arm1": 0.00,
    "R_arm2": 0.00, "R_arm3": 0.00, "L_arm2": 0.00, "L_arm3": 0.00,
    "R_wrist": 0.00, "L_wrist": 0.00, "R_foot": 0.00, "L_foot": 0.00
  },
  "error_message": "..."
}
```

- `state`: `0` Idle / `2` Play(게이트 닫힘, motion 명령 거부) / `6` Error(motion·play 거부). (그 외 값은 코드 참조)
- `error_message`: `state == 6`일 때만 포함.
- `is_lock_key_removed`: 안전 키 제거 여부(= 게이트 개방 가능 상태).

### A.4 게이트(gate) 의미 — 안전망

- controller는 `isGateOpen`(초기값 false)으로 brain 명령을 막는다.
- **열기**: Idle에서 콘솔에 `k` 입력 시 `openGate()`. 또한 pause 완료 후·연주 종료 시 자동 개방.
- **닫기**: `p:`(연주 시작)·resume 실행 시 자동으로 `closeGate()` + 큐 flush.
- **게이트가 닫혀 있어도 통과하는 명령**: `pause`, `resume`, `h`, `tempo_scale:*`, `velocity_delta:*`.
- 그 외 명령(`move:`, `gesture:`, `look:`, `p:`, `r`)은 게이트 닫힘 상태에서 폐기되고
  `"[Safeguard] ... 명령 폐기"`로 로깅된다.

> ⚠️ TCP 연결 성공(`Brain Connected`)과 게이트 개방은 별개다. `k` 입력 전에는 motion/play가 폐기될 수 있다.

---

## Contract B — Controller ↔ SIL / 하드웨어 (CAN + DXL)

### B.1 인터페이스 선택 정책

- controller는 실제 `can*` 인터페이스가 **하나라도** 있으면 real CAN만 쓴다. 없으면 `vcan*`로 fallback.
- real CAN bitrate: `1000000`. `vcan`에는 bitrate를 설정하지 않는다.
- DXL은 `/dev/ttyUSB0` 하나의 serial bus. SIL에서는 이 경로가 PTY symlink이며 sim 측 endpoint는 `/tmp/ttyUSB0_sim`.
- **SIL 활성화는 환경변수가 아니라 인터페이스 존재 여부로만 결정된다.**

### B.2 motor ↔ node_id ↔ bus 매핑

| motor | 종류 | node_id | 기본 bus |
|-------|------|---------|----------|
| waist | TMotor | 0x00 | vcan0 |
| L_arm1 | TMotor | 0x02 | vcan0 |
| L_arm2 | TMotor | 0x05 | vcan0 |
| L_arm3 | TMotor | 0x06 | vcan0 |
| R_arm1 | TMotor | 0x01 | vcan1 |
| R_arm2 | TMotor | 0x03 | vcan1 |
| R_arm3 | TMotor | 0x04 | vcan1 |
| R_wrist | Maxon | 0x07 | vcan3 |
| L_wrist | Maxon | 0x08 | vcan3 |
| R_foot | Maxon | 0x0A | vcan2 |
| L_foot | Maxon | 0x0B | vcan2 |
| head_pan | DXL | ID 1 | /dev/ttyUSB0 |
| head_tilt | DXL | ID 2 | /dev/ttyUSB0 |

> frame-level SIL은 command가 들어온 bus를 feedback bus로 동적 바인딩할 수 있다. 디버깅 시 고정 표만
> 보지 말고 실제 `candump`로 어느 bus에 frame이 오가는지 확인하라.

### B.3 TMotor (servo mode) 프레임

**명령 — 위치 (SET_POS):**

```text
CAN ID : (0x04 << 8) | node_id        # 예: waist=0x0400
DLC    : 4
data[0:4] : int32 big-endian = round(position_deg * 10000)
            (입력 radian → degree 변환 후 인코딩)
```

**명령 — 속도 (SET_RPM):**

```text
CAN ID : (0x03 << 8) | node_id
DLC    : 4
data[0:4] : int32 big-endian = erpm (전기 RPM)
```

**피드백 (motor/SIL → controller):**

```text
CAN ID : node_id (0x00..0x06)
DLC    : 8
data[0:2] : int16 BE = position_deg / 0.1   (즉 deg*10)
data[2:4] : int16 BE = velocity * 10
data[4:6] : int16 BE = current / 0.01
data[6]   : int8 temperature
data[7]   : int8 error
```

> TMotor는 명령 수신 시 target echo 피드백을 즉시 보낸다(다음 safety check의 current 기준 갱신).
> 한 번이라도 명령을 받은 모터에는 idle 피드백을 보내지 않는다(echo와 idle이 서로 덮는 것 방지).

### B.4 Maxon (CANopen) 프레임

**COB-ID (node_id 기준):**

```text
SDO 요청   : 0x600 + node_id      SDO 응답   : 0x580 + node_id
TPDO ctrl  : 0x200 + node_id      TPDO pos   : 0x300 + node_id   ← 위치 명령
TPDO vel   : 0x400 + node_id      TPDO torq  : 0x500 + node_id
RPDO state : 0x180 + node_id      ← 피드백
SYNC       : 0x080 (DLC 0)        NMT        : 0x000 (data[0]=cmd, data[1]=node)
```

**위치 명령:**

```text
CAN ID : 0x300 + node_id
DLC    : 4
data[0:4] : uint32 little-endian = round(position_deg * 35.0 * 4096.0 / 360.0)
```

**피드백 (SYNC 0x80 수신 시 Operational Maxon만 응답):**

```text
CAN ID : 0x180 + node_id
DLC    : 8
data[1]   : status byte (예: 0x37)
data[2:6] : int32 little-endian position_enc
            position_deg = position_enc * 360.0 / (35.0 * 4096.0)
data[6:8] : int16 little-endian torque_enc  (torque_Nm = enc/1000 * 31.052)
```

> Maxon은 주기 피드백을 뿌리지 않는다. CANopen `0x80` SYNC를 받았을 때만 TPDO 피드백을 보낸다.

### B.5 Dynamixel Protocol 2.0 (serial)

- 경로 `/dev/ttyUSB0`, baud `4500000`, Protocol `2.0`. ID 1 = head_pan, ID 2 = head_tilt.
- packet 헤더: `FF FF FD 00`, 이후 `ID, len(LE16), instruction, params..., CRC16(LE)`.
- 주요 instruction: `0x01` PING, `0x03` WRITE, `0x55` STATUS(응답), `0x82` SYNC_READ, `0x83` SYNC_WRITE, `0xFE` broadcast.

**goal position 인코딩 (tick ↔ degree):**

```text
tick = round(2048.0 - angle_deg * 4096.0 / 360.0)
angle_deg = (2048.0 - tick) * 360.0 / 4096.0
```

**피드백 규약:**

```text
syncWrite(goal) -> goal 저장, status 응답 없음
syncRead(ID 1, ID 2) -> 마지막 goal(또는 startup pose) 기준 status packet으로 응답
```

> `syncRead` 응답은 PyBullet state가 아니라 마지막 goal/startup 기준으로 빠르게 만든다(응답 지연 시
> controller 로그에 `SyncRead failed`).

### B.6 각도 의미 변환 (SIL 측 책임)

production motor 각도 ↔ URDF joint 각도 변환은 SIL의 `sil/mapping.py`
(`PRODUCTION_TO_URDF_CAN_TRANSFORM`, startup pose 등)가 담당한다. **controller는 production 의미의
각도만 보내고 받으며, URDF 변환은 알 필요가 없다.**

### B.7 safety와 current angle

controller의 TMotor send loop는 명령 전송 전에 current vs desired 차이로 safety check를 한다.
피드백이 밀리거나 오래된 피드백이 current를 덮으면 다음 명령에서
`Set CAN Frame Error : Safety Check`가 날 수 있다. 즉 **피드백 timing도 계약의 일부**다.

---

## 계약 변경 절차

1. 이 `CONTRACTS.md`를 먼저 고친다(어느 레포에서든).
2. 영향받는 두 레포의 코드를 같이 고친다(wire format은 한쪽만 바꾸면 깨진다).
3. 세 레포의 `CONTRACTS.md` 복사본을 동일하게 맞춘다.
4. 각 레포 `log.md`에 변경을 기록한다.
