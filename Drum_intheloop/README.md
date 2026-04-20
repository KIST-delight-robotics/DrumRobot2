# Drum_intheloop

`Drum_intheloop`는 `DrumRobot2`가 내보내는 command-level body/head 명령을 받아 `PyBullet`에 적용하는 SIL(Simulation in the Loop) 디렉터리입니다.

이 디렉터리의 핵심 목표는 다음과 같습니다.

- `DrumRobot2`가 실제로 소비하는 모터 명령 경계를 그대로 재사용한다.
- production motor 이름과 각도 의미를 `PyBullet`가 읽을 수 있는 URDF joint target으로 바꾼다.
- 체크인된 URDF/STL 원본은 보존하고, 런타임 복사본만 보정한다.
- 이후 `vcan`/`struct can_frame` 기반의 frame-accurate SIL로 가기 전, command-level seam을 먼저 안정화한다.

이 README는 지금 구조를 처음 보는 사람이 “어디서 무엇이 바뀌고, 무엇은 아직 실험 중인지”까지 이해할 수 있도록 현재 상태를 상세히 정리한 문서입니다.

## 한눈에 보는 현재 구조

현재 경계는 아래와 같습니다.

```text
사용자/LLM/phil_robot
-> DrumRobot2
-> SilCommandPipeWriter (C++)
-> /tmp/drum_command.pipe
-> SilCommandPipeReader (Python)
-> CommandApplier
-> PyBulletBackend
-> PyBullet GUI/DIRECT
```

중요한 점:

- 이 디렉터리는 `planner`를 직접 만들지 않습니다.
- 이 디렉터리는 TCP 소켓을 직접 관리하지 않습니다.
- 이 디렉터리는 아직 `frame-accurate` 시뮬레이터가 아닙니다.
- 이 디렉터리는 “C++에서 계산된 command-level 목표 각도”를 받아 눈으로 확인 가능한 PyBullet 상태로 옮기는 역할에 집중합니다.

## 이 디렉터리가 하는 일과 하지 않는 일

이 디렉터리가 하는 일:

- `/tmp/drum_command.pipe` named pipe 생성/정리
- pipe NDJSON 한 줄을 Python 자료구조로 복원
- production motor 이름을 URDF joint 이름으로 매핑
- production 각도 의미를 URDF 기준 각도로 보정
- runtime URDF를 생성해 `package://` 경로, joint limit, arm visual frame을 보정
- PyBullet에서 로봇을 띄우고 joint state를 즉시 반영

이 디렉터리가 하지 않는 일:

- planner의 left/right 의미 해석 수정
- `DrumRobot2`의 TCP brain 연결 보장
- 실기 수준의 시간 기반 DXL/Maxon/TMotor motion profile 재현
- 체크인된 URDF/STL 원본 수정

## 현재 기준으로 알아야 할 중요한 사실

현재 사실로 문서화해 두는 게 좋은 포인트들입니다.

1. `SilCommandPipeReader.py`가 FIFO 생명주기를 소유합니다.
2. pipe writer는 `/tmp/drum_command.pipe`가 이미 있을 때만 쓸 수 있으며, reader가 먼저 떠 있어야 합니다.
3. `DrumRobot2` 쪽 SIL export는 현재 `DRUM_SIL_MODE=1`일 때만 활성화됩니다.
4. reader는 시작 직후 `robot_spec.py`의 startup preset pose를 PyBullet에 한 번 적용합니다.
5. `head_tilt`는 C++/DXL 쪽 정면이 `90deg`, URDF 쪽 정면이 `0deg`라서 Python에서 `-90deg` 보정합니다.
6. body CAN joint는 `joint_map.py`의 per-joint transform 테이블로 각도 의미를 보정합니다.
7. 팔 외형이 실기와 다르게 보이는 문제는 원본 URDF/STL이 아니라 `urdf_tools.py`의 runtime patch에서 실험 중입니다.
8. 현재 backend는 `resetJointState()` 기반이라 actuator interpolation timing은 없습니다. 단, C++ send loop가 매 1ms마다 `tick` 메시지를 보내 같은 구간의 TMotor/Maxon/DXL 명령이 동일한 `stepSimulation()` 안에 atomic하게 적용됩니다.
9. 실제 관절 명령 없이 빈 tick만 `IDLE_RETURN_SEC`(5초) 이상 이어지면 startup pose로 자동 복귀합니다. SIL 전용 기능입니다.
10. `pybullet_backend.py`는 URDF 로봇 외에 페달 2개(R_foot/L_foot)와 드럼 패드를 별도 MultiBody로 시각화합니다. 페달은 `joint_map.py`의 `PEDAL_JOINTS`/`PEDAL_SPEC`, 드럼 패드는 `DRUM_PAD_SPEC`/`DRUM_PAD_OFFSET`/`DRUM_INSTRUMENT_NAMES`로 설정합니다.

## 빠른 실행 순서

### 1. Python 의존성 설치

```bash
cd /home/shy/robot_project/Drum_intheloop
python -m pip install -r requirements.txt
```

현재 `requirements.txt`는 `pybullet`만 요구합니다.

### 2. reader 실행

```bash
cd /home/shy/robot_project/Drum_intheloop
python sil/SilCommandPipeReader.py --mode gui
```

이 명령은 다음을 수행합니다.

- 기존 `/tmp/drum_command.pipe`가 있으면 stale FIFO인지 확인 후 삭제
- 새 FIFO 생성
- runtime URDF 생성
- PyBullet GUI 실행
- startup preset pose 적용 후 `stepSimulation()` 1회 호출
- 이후 pipe NDJSON을 한 줄씩 읽고, 각 줄마다 `apply -> step -> optional sleep` 순서로 반복

headless로 보고 싶으면:

```bash
python sil/SilCommandPipeReader.py --mode direct
```

### 3. `DrumRobot2`를 SIL 모드로 실행

```bash
cd /home/shy/robot_project/DrumRobot2/bin
sudo env DRUM_SIL_MODE=1 ./main.out
```

주의:

- `sudo ./main.out`만 실행하면 `DRUM_SIL_MODE`가 전달되지 않아 pipe writer가 비활성 상태가 됩니다.
- reader가 먼저 켜져 있어야 writer가 FIFO를 열 수 있습니다.

로그 예시는 대략 다음과 같습니다.

- C++ 쪽:
  - `[SIL] DRUM_SIL_MODE=1 -> SIL mode enabled`
- Python 쪽:
  - `Applying startup preset pose: ...`
  - `Listening on named pipe: /tmp/drum_command.pipe`

### 4. 필요 시 brain / planner 연결

`phil_robot` 또는 별도 brain이 붙으면 `DrumRobot2`가 추가 명령을 생성하고, 그 결과가 다시 pipe를 통해 여기로 흘러옵니다.

단, planner/TCP 문제는 이 디렉터리 바깥 이슈일 수 있습니다.  
예를 들어 “오른팔이라고 말했는데 planner가 `L_arm*`를 생성했다”는 문제는 보통 `phil_robot` 경로를 먼저 봐야 합니다.

## 실제 런타임 흐름

### 1. `SilCommandPipeReader.py`

파일: [sil/SilCommandPipeReader.py](/home/shy/robot_project/Drum_intheloop/sil/SilCommandPipeReader.py)

이 파일은 ingress와 런타임 orchestration을 담당합니다.

역할:

- named pipe 준비/정리
- NDJSON 한 줄씩 읽기
- `tmotor`/`maxon`/`dxl` kind에 따라 Python 자료구조로 파싱
- `CommandApplier`를 호출해 URDF joint target dict 생성
- `PyBulletBackend`에 joint target 적용
- startup preset pose 적용

reader가 읽는 NDJSON 예시는 다음과 같은 형태입니다.

```json
{"kind":"tmotor","motor":"L_arm2","position":10.0,"velocityERPM":0.0,"mode":0,"useBrake":0}
{"kind":"maxon","motor":"L_wrist","position":75.0,"mode":1,"kp":0,"kd":0}
{"kind":"dxl","motor":"head_tilt","position":90.0}
```

여기서 중요한 점은 `position`이 raw motor unit이 아니라, C++ 쪽에서 이미 joint angle(deg)로 변환된 값이라는 점입니다.

### 메시지 한 줄이 실제로 어떻게 소비되는가

현재 pipe protocol은 **command 메시지**와 **tick 메시지** 두 종류로 이루어집니다.

- `SilCommandPipeWriter`는 `tmotor`/`maxon`/`dxl` command를 각각 한 줄씩 FIFO에 씁니다.
- C++ send loop는 매 **1ms** 끝마다 `{“kind”:”tick”}` 한 줄을 FIFO에 씁니다. 이 tick이 1ms frame 경계 역할을 합니다.
- `SilCommandPipeReader`는 command를 받을 때마다 `frame_targets`에 누적하고, tick을 받을 때 한꺼번에 apply → step합니다.

즉 현재 의미 단위는 아래와 같습니다.

```text
command NDJSON 줄 수신
  -> Python dict → frame_targets에 누적

tick 수신 (frame_targets 있음)
  -> frame_targets 전체를 resetJointState(...)로 한 번에 적용
  -> stepSimulation() 1회
  -> optional sleep
  -> frame_targets 초기화

tick 수신 (frame_targets 없음 = 빈 tick)
  -> idle 판정 → IDLE_RETURN_SEC(5초) 초과 시 startup pose 복귀
```

중요:

- 같은 1ms 구간 안에 들어온 TMotor/Maxon/DXL 명령은 동일한 `stepSimulation()` 안에 **atomic하게** 적용됩니다.
- tick이 frame 경계 역할을 하므로, DXL과 CAN 모터의 joint target이 같은 step 안에 정렬됩니다.
- tick이 오지 않으면 apply와 step이 일어나지 않습니다. C++ send loop가 살아있는 것이 전제입니다.

### 2. `command_types.py`

파일: [sil/command_types.py](/home/shy/robot_project/Drum_intheloop/sil/command_types.py)

이 파일은 C++에서 건너온 command-level payload를 Python 쪽에서 받기 위한 최소 자료구조를 정의합니다.

- `TMotorData`
- `MaxonData`
- `CanMotorCommand`

현재 의도는 “simulator 내부 상태를 길게 들고 가기”가 아니라, “한 메시지를 해석해 바로 적용할 수 있는 최소 command object”를 두는 것입니다.

### 3. `command_applier.py`

파일: [sil/command_applier.py](/home/shy/robot_project/Drum_intheloop/sil/command_applier.py)

이 파일은 `Drum_intheloop`에서 가장 중요한 의미 변환 계층입니다.

역할:

- production motor 이름 검사
- CAN command 타입 검사
- production angle -> URDF angle 변환
- DXL logical joint -> URDF look joint 변환

특히 다음 사실이 중요합니다.

- `head_tilt`는 여기서 `position - 90` 보정을 합니다.
- body joint 보정은 `joint_map.py`의 `PRODUCTION_TO_URDF_CAN_TRANSFORM` 테이블을 읽습니다.

공통 수식은 아래와 같습니다.

```text
mapped_deg = bias_deg + reference_deg + sign * (target_deg - reference_deg)
```

해석:

- `sign = +1.0`: 같은 방향 유지
- `sign = -1.0`: 기준각을 중심으로 반대 방향
- `reference_deg = 90.0`: 90도 기준 mirror
- `bias_deg`: 영점 offset

예:

- 단순 부호 반전: `reference=0, sign=-1`
- 90도 기준 mirror: `reference=90, sign=-1`

### 4. `joint_map.py`

파일: [sil/joint_map.py](/home/shy/robot_project/Drum_intheloop/sil/joint_map.py)

이 파일은 “이름 매핑”과 “의미 매핑”을 분리합니다.

주요 테이블:

- `PRODUCTION_TO_URDF_JOINT`
  - 예: `L_arm2 -> left_shoulder_2`, `R_foot -> pedal_right`
- `PRODUCTION_TO_URDF_CAN_TRANSFORM`
  - joint별 `reference_deg`, `sign`, `bias_deg`
- `LOOK_JOINTS`
  - `pan -> head`, `tilt -> head_2`
- `URDF_JOINT_LIMITS_DEG`
  - runtime URDF에 다시 써 넣을 joint 범위
- `PEDAL_JOINTS` / `PEDAL_SPEC`
  - 가상 페달 MultiBody 키와 geometry/위치/색상 정의
- `DRUM_PAD_SPEC` / `DRUM_PAD_OFFSET`
  - 드럼 패드 geometry/색상 정의와 전체 위치 보정치
- `DRUM_INSTRUMENT_NAMES` / `DRUM_HEAD_INDICES` / `DRUM_PAD_SKIP_INDICES`
  - `drum_position.txt` 열 순서, 드럼 헤드 인덱스, 배치 제외 인덱스

현재 눈여겨볼 포인트:

- `R_arm1`은 기존 동작을 유지하기 위해 `90deg` 기준 mirror로 남아 있습니다.
- `L_arm2`, `L_arm3`는 현재 sign 반전이 걸려 있습니다.
- 여기 값은 “실기 의미와 sim 의미를 맞추기 위한 해석 계층”이지, 링크 외형 자체를 바꾸지는 않습니다.

즉:

- 관절이 어느 방향으로 도는가는 여기서 많이 결정됩니다.
- 상완/하완 연결부가 바깥을 본다 같은 visual 문제는 여기만 바꿔서는 해결되지 않을 수 있습니다.

### 5. `robot_spec.py`

파일: [sil/robot_spec.py](/home/shy/robot_project/Drum_intheloop/sil/robot_spec.py)

이 파일은 simulator 관점의 motor catalog입니다.

포함 내용:

- CAN 모터 이름, kind, node id, joint index
- DXL 모터 이름, logical joint
- startup preset pose

현재 startup preset은 다음 의도를 가집니다.

- reader가 뜨자마자 URDF 기본 자세 대신 사람이 보기 좋은 시작 자세를 한 번 맞춘다.
- 이후 실제 pipe command가 들어오면 그 흐름이 simulator 자세를 덮어쓴다.

즉 startup pose는 “실기 초기화 흐름을 완전히 대체하는 절대 기준”이 아니라, viewer가 처음 떴을 때의 초기 시각 보정입니다.

### 6. `pybullet_backend.py`

파일: [sil/pybullet_backend.py](/home/shy/robot_project/Drum_intheloop/sil/pybullet_backend.py)

이 파일은 backend에 집중합니다.

역할:

- PyBullet 연결
- runtime URDF 로드
- 바닥 plane 로드
- joint index 읽기
- base orientation 적용
- ground placement
- joint target을 `resetJointState()`로 반영

현재 backend 설계 원칙:

- 가능한 한 “dumb backend”를 유지한다.
- 이름 해석, joint sign, DXL 보정 같은 의미 변환은 `command_applier.py`/`joint_map.py`에서 한다.
- backend는 이미 URDF 기준으로 정리된 `joint_targets_deg`만 적용한다.

현재 base orientation은 아래 의미입니다.

- URDF 원본이 PyBullet 기본 세계축과 바로 맞지 않아서
- `(-pi/2, 0, 0)`의 Euler를 quaternion으로 바꿔 적용해 세워 둡니다.

또 `_place_robot_on_ground()`는 로봇 전체 AABB 최저점을 보고 z축만 들어올리는 함수입니다.  
이 함수는 방향을 바꾸지 않고, 로봇이 바닥 아래로 박히지 않게 높이만 보정합니다.

중요한 한계:

- 현재 joint 적용은 `resetJointState()` 기반이라 즉시 이동합니다.
- 따라서 실기처럼 시간에 따라 부드럽게 움직이는 DXL/Maxon/TMotor profile은 아직 재현하지 않습니다.

## timing을 엄밀하게 이해하기

이 디렉터리에는 서로 다른 “주기”가 섞여 있습니다. 헷갈리지 않게 세 층으로 나눠서 보면 이해가 쉽습니다.

### 1. C++가 명령을 쓰는 주기

`DrumRobot2` 기준으로 현재 알려진 send/recv cadence는 대략 아래와 같습니다.

- send loop 기본 tick: `1ms`
- `TMotor` write/export: `5ms`마다
- `Maxon` write/export: `1ms`마다
- `DXL` 소비/export: 현재 `5ms`마다
- CAN receive loop: `100us`

중요한 점은, 이 주기들이 있다고 해서 pipe에 “5ms짜리 완성된 한 프레임 묶음”이 생기는 것은 아니라는 것입니다. exporter는 명령 하나가 소비될 때마다 NDJSON 한 줄로 씁니다. 단, C++ send loop는 매 1ms 끝마다 `tick` 메시지를 추가로 씁니다.

### 2. Python reader가 읽고 적용하는 주기

현재 reader thread/main loop는 별도 fixed-rate scheduler를 두지 않습니다. 실제 흐름은 아래와 같습니다.

```text
startup:
  prepare_pipe()
  backend.start()
  apply startup preset
  stepSimulation() 1회
  “Listening on named pipe”

runtime per message:
  pipe에서 다음 줄이 들어올 때까지 block
  줄 1개 수신 (command 또는 tick)

  command인 경우:
    JSON parse → production -> URDF joint target 변환 → frame_targets에 누적

  tick인 경우:
    frame_targets가 있으면:
      apply_targets(frame_targets)
      stepSimulation() 1회
      if --sleep > 0: time.sleep(args.sleep)
      frame_targets 초기화, last_motion_time 갱신
    frame_targets가 없으면 (빈 tick):
      idle 판정 → (time - last_motion_time) > IDLE_RETURN_SEC 이면 startup pose 복귀

  다음 줄 읽기
```

즉 reader는 평소에 “100us마다 polling”하는 구조가 아닙니다.

- 데이터가 없으면 FIFO read에서 그냥 block됩니다.
- command는 누적되고, tick이 와야 실제 apply + step이 일어납니다.
- `--sleep`은 tick 처리 후 다음 줄로 넘어가기 전의 추가 지연입니다.

기본값 `--sleep 0.0001`은 `100us`입니다.

### 3. `stepSimulation()`의 실제 의미

현재 `PyBulletBackend.step()`은 `p.stepSimulation()` 한 줄만 호출합니다. 그리고 이 저장소 안에서는 `setTimeStep()`이나 `setRealTimeSimulation()`으로 별도 physics tick을 다시 정의하지 않습니다.

정확히는 다음과 같습니다.

- `resetJointState()`가 target angle을 즉시 joint state에 꽂아 넣는다.
- 그 뒤 `stepSimulation()`을 한 번 호출한다.
- 즉 “모터가 3ms 동안 따라가며 이동한다” 같은 actuator-side dynamics는 없다.
- 하지만 “메시지가 들어온 순서” 자체는 반영된다.

한 줄로 요약하면:

```text
메시지 도착 timing은 있음
하지만 actuator interpolation timing은 없음
```

### 4. 한 1ms 구간에서 실제로 보이는 것

예를 들어 C++ 쪽에서 같은 1ms 구간 안에 아래 command들이 순서대로 pipe에 써졌다고 가정하겠습니다.

```text
t = 0.0 ms   maxon command A write
t = 0.2 ms   tmotor command B write
t = 0.3 ms   dxl command C write
t = 1.0 ms   tick write (C++ send loop 끝)
```

현재 simulator에서의 처리 그림은 아래와 같습니다.

```text
Python reader:
  A 읽음 -> frame_targets에 누적
  B 읽음 -> frame_targets에 누적
  C 읽음 -> frame_targets에 누적
  tick 읽음 -> apply A+B+C 동시 -> step 1회 -> sleep 100us
```

즉 지금 구조는:

- 같은 tick 구간의 `A+B+C`는 하나의 `stepSimulation()` 안에 **atomic하게** 들어갑니다.
- tick 경계가 C++ 1ms send loop와 맞춰져 DXL/TMotor/Maxon frame이 정렬됩니다.
- 따라서 같은 1ms 구간 명령에 한해서는 simulator가 실기와 비슷한 frame 동기성을 가집니다.

### 5. 왜 이 설명이 중요한가

이 차이를 알아야 아래 현상을 올바르게 해석할 수 있습니다.

- READY 자세가 실기와 살짝 다르게 보일 때:
  - startup preset, 첫 pipe command, tick 도착 순서를 같이 봐야 합니다.
- 동작이 뚝뚝 끊겨 보일 때:
  - reader 문제라기보다 `resetJointState()` 기반 즉시 적용 구조 영향일 수 있습니다.
- “한 step 안에 모든 joint가 같이 들어간다”고 기대했는데 다르게 보일 때:
  - tick 경계가 1ms이므로, 다른 1ms 구간에 걸친 명령은 여전히 별도 step으로 들어갑니다.

### 7. `urdf_tools.py`

파일: [sil/urdf_tools.py](/home/shy/robot_project/Drum_intheloop/sil/urdf_tools.py)

이 파일은 체크인된 URDF/STL 원본을 건드리지 않고, PyBullet용 runtime 복사본을 생성합니다.

현재 수행하는 일:

- `package://drumrobot_RL_urdf/...` 경로를 절대 경로로 변환
- 0으로 비어 있던 joint limit를 runtime URDF에 다시 써 넣기
- arm visual/collision origin에 runtime pose patch 적용

### arm visual runtime patch가 필요한 이유

최근 실험에서 확인된 문제:

- 실기에서는 상완/하완 연결부가 안쪽을 보는 게 자연스러운데
- simulator에서는 양팔 모두 연결부가 바깥을 보는 그림이 나왔습니다.

이 문제는 단순히 `joint_map.py`의 sign만 바꿔서는 해결되지 않았고, 결국 “링크 외형이 붙는 frame 자체”를 runtime에서 실험할 필요가 생겼습니다.

그래서 현재 `RUNTIME_LINK_FRAME_PATCH_POSE` 테이블이 들어 있습니다.

현재 patch의 성격:

- 체크인된 원본 URDF/STL은 수정하지 않음
- runtime URDF에서만 `visual/collision origin xyz/rpy`를 덮어씀
- 양팔 `shoulder_1`, `shoulder_2`, `elbow`, `wrist`에 대해 x축 180도 회전 실험 중
- 최근에는 회전만으로는 어깨/손목이 떠 보이는 문제가 있어 `xyz` 보정까지 같이 넣은 상태

중요:

- 이 patch는 현재 “정답 확정본”이 아니라 실험용입니다.
- 숫자는 실기 대비 visual fidelity를 맞추기 위한 1차 값입니다.
- 외형이 어긋나면 이 테이블을 먼저 보세요.
- 관절 회전 방향이 어긋나면 `joint_map.py`를 먼저 보세요.

## 현재 디렉터리 구조와 역할

- [requirements.txt](/home/shy/robot_project/Drum_intheloop/requirements.txt)
  - Python 의존성. 현재 `pybullet`
- [sil/SilCommandPipeReader.py](/home/shy/robot_project/Drum_intheloop/sil/SilCommandPipeReader.py)
  - ingress, startup pose, tick 기반 frame 배치 처리, idle return, main loop
- [sil/command_types.py](/home/shy/robot_project/Drum_intheloop/sil/command_types.py)
  - C++ payload 대응 자료구조
- [sil/command_applier.py](/home/shy/robot_project/Drum_intheloop/sil/command_applier.py)
  - production command -> URDF joint target 변환
- [sil/joint_map.py](/home/shy/robot_project/Drum_intheloop/sil/joint_map.py)
  - 이름/각도 transform/limit 정의, 페달/드럼패드 spec
- [sil/colors.py](/home/shy/robot_project/Drum_intheloop/sil/colors.py)
  - PyBullet 시각 테마 색상 상수 (바닥, 로봇, 페달)
- [sil/robot_spec.py](/home/shy/robot_project/Drum_intheloop/sil/robot_spec.py)
  - motor catalog와 startup pose
- [sil/pybullet_backend.py](/home/shy/robot_project/Drum_intheloop/sil/pybullet_backend.py)
  - PyBullet backend, 페달/드럼패드 MultiBody 시각화
- [sil/urdf_tools.py](/home/shy/robot_project/Drum_intheloop/sil/urdf_tools.py)
  - runtime URDF 생성과 pose/limit patch
- [tests/pipe_demo_reader.py](/home/shy/robot_project/Drum_intheloop/tests/pipe_demo_reader.py)
  - named pipe 학습용 최소 reader 예제
- [urdf/drumrobot_RL_urdf](/home/shy/robot_project/Drum_intheloop/urdf/drumrobot_RL_urdf)
  - 원본 URDF/STL 자산. 실험은 원칙적으로 runtime patch로 먼저 수행

## 현재 알려진 한계와 주의점

### 1. command-level SIL이지 frame-accurate SIL이 아닙니다

현재 이 경로는:

- raw CAN frame을 재생하지 않고
- C++에서 이미 해석된 목표 각도를 복사해
- Python에서 tick 경계마다 frame_targets를 한꺼번에 joint state로 넣습니다

즉 실기 버스 지연, 제어 loop 주기, PID profile을 재현하지 않습니다. 1ms frame 경계 정렬은 tick 메시지로 맞추지만, 실기 수준의 CAN timing fidelity는 아닙니다.

### 2. DXL timing 문제는 아직 남아 있습니다

지금 backend는 즉시 적용형이라, 실기에서는 시간을 두고 움직일 DXL도 시뮬레이터에서는 “즉시 점프”에 가깝게 보입니다.

그래서 다음과 같은 현상이 있을 수 있습니다.

- 너무 즉각적인 머리 움직임
- 실기에서의 부드러운 프로파일과 다른 느낌

이건 현재 구조의 알려진 제한입니다.

### 3. planner의 right/left 의미 오류는 이 디렉터리 바깥일 수 있습니다

예를 들어:

- 사용자 발화는 “오른팔”
- planner 출력은 `move:L_arm1,...`

라면, 그건 `Drum_intheloop` 이전의 planner/resolver 층 문제일 가능성이 높습니다.

반대로:

- `move:L_arm2,...` 명령이 왔고
- simulator의 `left_shoulder_2`가 실제로 움직였다

면, 이 디렉터리에서는 라우팅보다 visual 또는 angle semantic을 먼저 봐야 합니다.

### 4. startup pose는 첫 화면을 보기 좋게 만드는 편의 계층입니다

reader 시작 직후 preset pose를 한 번 적용하므로, “원래 URDF 기본 자세가 어떻게 생겼는지”를 바로 보기는 어렵습니다.

초기 그림을 완전히 원본 기준으로 보고 싶다면:

- `build_startup_joint_targets()` 경로를 임시로 빼고
- runtime URDF만 로드했을 때의 기본 모습을 따로 확인해야 합니다.

### 5. runtime arm visual patch는 아직 진행 중입니다

현재 arm patch는 “실기와 simulator 그림이 왜 다르게 보이는지”를 빠르게 검증하기 위한 실험입니다.

따라서:

- 값이 더 조정될 수 있습니다.
- left/right가 완전히 대칭 정답이라고 가정하면 안 됩니다.
- 실기 사진/관찰과 비교하면서 숫자를 다시 맞춰갈 수 있습니다.

## 문제를 볼 때 어디부터 확인할지

### 1. 아예 안 움직인다

먼저 확인:

- reader가 먼저 떠 있는가
- `/tmp/drum_command.pipe`가 reader에 의해 생성되었는가
- `DrumRobot2`를 `sudo env DRUM_SIL_MODE=1 ./main.out`로 실행했는가
- C++ 로그에 SIL enabled가 찍혔는가

### 2. 머리만 이상하다

먼저 확인:

- [sil/command_applier.py](/home/shy/robot_project/Drum_intheloop/sil/command_applier.py)의 `head_tilt` `-90deg` 보정
- `head_pan`/`head_tilt`가 `LOOK_JOINTS`에 맞게 들어가는지

### 3. 팔이 반대 방향으로 돈다

먼저 확인:

- [sil/joint_map.py](/home/shy/robot_project/Drum_intheloop/sil/joint_map.py)의 `PRODUCTION_TO_URDF_CAN_TRANSFORM`

이건 “관절 의미” 문제일 가능성이 큽니다.

### 4. 팔이 도는 방향은 맞는데 외형이 어깨/손목에서 빠져 보인다

먼저 확인:

- [sil/urdf_tools.py](/home/shy/robot_project/Drum_intheloop/sil/urdf_tools.py)의 `RUNTIME_LINK_FRAME_PATCH_POSE`

이건 “visual frame/origin” 문제일 가능성이 큽니다.

### 5. planner는 오른팔이라고 하는데 왼팔 명령을 생성한다

먼저 확인:

- `phil_robot`
- planner prompt
- motion resolver
- validator

이건 `Drum_intheloop` 밖의 문제일 수 있습니다.

## 앞으로 이 디렉터리에서 바꾸기 좋은 것과 아닌 것

이 디렉터리에서 바꾸기 좋은 것:

- named pipe ingress
- Python command 자료구조
- production-to-URDF 각도 보정
- startup pose
- runtime URDF patch
- PyBullet backend 표시/로딩/적용 흐름

이 디렉터리에서 바로 바꾸지 않는 것이 좋은 것:

- 체크인된 URDF/STL 원본 자산
- planner의 자연어 의미 해석
- `DrumRobot2` 내부 state JSON 규약
- 실기 하드웨어 초기화 순서 자체

## 정리

현재 `Drum_intheloop`는 완성형 physics simulator가 아니라, `DrumRobot2`의 command-level 출력을 안전하게 받아 PyBullet에서 빠르게 시각 검증하는 레이어입니다.

새로 보는 사람이 기억해야 할 핵심은 네 가지입니다.

1. reader가 pipe를 만들고, C++ writer는 `DRUM_SIL_MODE=1`일 때만 씁니다.
2. 각도 의미 보정은 `command_applier.py`와 `joint_map.py`에 있습니다.
3. 외형/연결부 보정은 `urdf_tools.py`의 runtime patch에 있습니다.
4. 지금 arm visual patch는 실험 중이므로, “관절 semantic 문제”와 “자산/visual frame 문제”를 분리해서 봐야 합니다.
