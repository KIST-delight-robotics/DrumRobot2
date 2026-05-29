이 디렉터리는 `DrumRobot2`의 SocketCAN/DXL frame-level 출력을 `PyBullet`에 적용하는 SIL 디렉터리다.

이 범위에서 작업할 때는 아래 원칙을 기본으로 따른다.

## 이 디렉터리의 현재 역할

- `DrumRobot2`가 실제 장치처럼 내보내는 `struct can_frame`과 Dynamixel Protocol 2.0 serial packet을 읽는다.
- TMotor, Maxon, DXL 프로토콜을 decode/encode한다.
- production motor 이름과 각도 의미를 URDF joint target으로 변환한다.
- 체크인된 URDF/STL을 직접 수정하지 않고 runtime URDF 복사본에 보정을 적용한다.
- `PyBullet` joint state를 다시 CAN/DXL feedback으로 encode해 `DrumRobot2`에 돌려준다.

## 반드시 기억할 현재 사실

- `setup_sil.sh`는 `vcan0..3`와 DXL PTY pair를 준비하고, robot-side endpoint를 `/dev/ttyUSB0` symlink로 노출한다. simulator는 별도 터미널에서 `python3 simul.py`로 실행한다.
- `DrumRobot2`는 실제 `can*`이 하나라도 있으면 real CAN만 사용하고, real CAN이 없을 때만 `vcan*`을 사용한다.
- `vcan`에는 bitrate 설정을 하지 않는다.
- DXL은 `/dev/ttyUSB0` 하나가 bus 하나이며, ID 1은 `head_pan`, ID 2는 `head_tilt`로 처리한다. simulator peer endpoint는 `/tmp/ttyUSB0_sim`이다.
- CAN joint 의미 보정은 `sil/mapping.py`의 `PRODUCTION_TO_URDF_CAN_TRANSFORM`이 담당한다.
- protocol decode/encode는 `sil/decoder.py`와 `sil/encoder.py`에 둔다.
- motor/DXL ID routing은 `sil/router.py`에 둔다.
- arm 외형 보정은 `sil/urdf_tools.py`의 runtime pose patch가 담당하며, 아직 실험 중이다.
- 현재 backend는 `resetJointState()` 기반이라 actuator dynamics 없이 즉시 적용된다.

## 변경 우선순위

문제가 이 디렉터리에서 해결 가능한지 먼저 층을 구분한다.

1. SocketCAN/DXL ingress 문제인가
2. protocol decode/encode 문제인가
3. CAN ID/DXL ID -> motor routing 문제인가
4. production 이름/각도 의미 -> URDF 변환 문제인가
5. runtime URDF visual/collision frame 문제인가
6. 이 디렉터리 바깥(planner, TCP, C++ state JSON, 하드웨어 초기화) 문제인가

이 중 2~5는 가능하면 `Drum_intheloop/` 안에서 해결하는 것을 우선한다.

## 코드 수정 원칙

- `PyBulletBackend`는 가능한 한 단순한 backend로 유지하고, protocol/라우팅/의미 변환 로직을 넣지 않는다.
- protocol 처리는 `decoder.py`/`encoder.py`, routing은 `router.py`, 각도/URDF 매핑은 `mapping.py`에 둔다.
- visual/collision origin 보정은 `sil/urdf_tools.py`의 runtime patch에서 먼저 시도한다.
- 체크인된 `urdf/` 아래 원본 URDF/STL 파일은 직접 수정하지 않는다.
- `package://` 경로 수정, limit 보강, visual/collision origin 보정은 runtime 임시 파일로 처리한다.
- 관절 범위의 기준이 필요하면 Python validator보다 C++ 쪽 범위를 우선 참고한다.
- 변수명은 짧되 의미가 드러나게 쓴다. 애매한 `pkt`, `pos`, `vel`, `mid`, `jid`보다 `packet`, `position`, `velocity`, `motor_id`, `joint_id`를 우선한다.
- protocol 관례 약어인 `kp`, `kd`, `rx_pdo`, `tx_pdo`, `dlc`는 그대로 써도 된다.

## 디버깅할 때의 판단 기준

- 기본 배치에서는 `candump vcan0`에서 left arm/waist feedback이 보인다. 다만 frame-level SIL은 CAN ID/protocol 중심으로 routing하므로 command가 들어온 bus가 motor feedback bus로 동적 바인딩될 수 있다.
- motor discovery가 실패하면 simulator가 해당 bus에서 올바른 CAN ID로 feedback/SDO ack를 보내는지 먼저 본다.
- 팔이 반대 방향으로 돌면 먼저 `mapping.py`의 transform을 본다.
- 팔이 맞는 방향으로 도는데 어깨/손목에서 빠져 보이면 먼저 `urdf_tools.py`의 runtime pose patch를 본다.
- planner가 오른팔이라고 말하면서 `L_arm*`를 생성하면 그건 대개 이 디렉터리 밖 문제다.
- `current_angles`가 비정상적으로 깨져 보이면 C++ state broadcast 경로와 CAN feedback decode 경로를 함께 의심한다.

## 작업 로그

- 이 디렉터리에서 의미 있는 변경을 하면 루트 `log.md`에 반드시 기록한다.
- 기록 시 한국시간(KST, UTC+9)과 수정 파일, 변경 이유를 함께 남긴다.

## 추천 작업 순서

1. `setup_sil.sh`로 vcan/PTY 준비를 확인하고, `simul.py`로 simulator 실행 흐름을 확인한다.
2. `decoder.py`/`encoder.py`로 protocol frame/packet 처리를 확인한다.
3. `router.py`와 `mapping.py`로 routing 및 각도 의미 변환을 확인한다.
4. `pybullet_backend.py`는 마지막에 PyBullet 적용/표시 문제인지 확인하는 용도로 본다.
5. 이 범위를 벗어나면 `DrumRobot2`, `phil_robot`, `phil_intheloop` 중 어디 책임인지 먼저 분리한다.
