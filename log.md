# Change Log

프로젝트에서 의미 있는 변경을 할 때마다 한국시간(KST, UTC+9) 기준의 날짜, 시간, 요약을 기록합니다.
최신 항목이 위로 오도록 추가합니다.

## 2026-03-26
- 15:25 KST (UTC+9) — 루트 `.gitignore`에 `phil_intheloop/` 전체를 legacy mock/simulator workspace로 추가해, 앞으로 이 디렉터리 아래의 새 미추적 파일이 기본적으로 버전 관리 대상에 올라오지 않도록 정리했습니다.
  - 수정 파일: `.gitignore`, `log.md`
  - 메모: `phil_intheloop`는 이미 다수의 파일이 Git에 추적 중이므로, 이번 변경은 “새 파일 무시” 성격이고 기존 tracked 파일을 자동으로 untrack하지는 않습니다.
- 15:22 KST (UTC+9) — `drum_intheloop/README.md`에 pipe reader의 실제 소비 단위와 `stepSimulation()` 타이밍 의미를 사람 친화적으로 다시 풀어 써, “한 줄 메시지 = 한 번 적용 + 한 번 step + optional sleep” 구조와 batch 부재를 타임라인으로 이해할 수 있게 정리했습니다.
  - 수정 파일: `drum_intheloop/README.md`, `log.md`
  - 메모: 기존의 “timing 없는 즉시 이동형” 표현은 너무 뭉뚱그려져 있었고, 이번에는 C++ write cadence, Python read/apply 순서, `--sleep 0.0001`의 정확한 의미, startup preset의 별도 1회 step까지 구분해서 문서화했습니다.
- 15:03 KST (UTC+9) — `drum_intheloop/TODO.md`에 simulator ON일 때만 드러나는 `current_angles`/planner/validator 흔들림 문제를 `Simulator-Triggered Integration Blockers` 섹션으로 다시 포함시켜, simulator 내부 작업과 end-to-end surfaced blocker를 분리해 함께 보게 정리했습니다.
  - 수정 파일: `drum_intheloop/TODO.md`, `log.md`
  - 메모: 책임 코드가 `DrumRobot2`나 `phil_robot` 쪽일 수 있어도, 실제로 문제가 표면화되는 조건이 `drum_intheloop` 시뮬레이터 실행인 만큼 TODO에 추적 항목으로 남겼습니다.
- 14:55 KST (UTC+9) — `drum_intheloop/TODO.md`에서 simulator 범위를 벗어난 `DrumRobot2`/`phil_robot` blocker 항목을 제거해, TODO가 `drum_intheloop` 내부 simulator 작업만 다루도록 다시 좁혔습니다.
  - 수정 파일: `drum_intheloop/TODO.md`, `log.md`
  - 메모: 이전 버전은 end-to-end 맥락을 함께 적으려다 바깥 계층의 state/owner 이슈가 섞였고, 이번 수정에서 `READY -> snare`, visual 보정, trace/replay, timing/future SIL 같은 simulator 항목만 남겼습니다.
- 14:51 KST (UTC+9) — `drum_intheloop`의 TODO를 현재 command-level SIL 역할과 실제 blocker 기준으로 전면 교체하고, 중복 복사본 `TODO copy.md`는 제거했습니다.
  - 수정 파일: `drum_intheloop/TODO.md`, `drum_intheloop/TODO copy.md`, `log.md`
  - 메모: 새 TODO는 `READY -> snare` 자세 문제, arm visual 보정값 정리, command-level seam에서 CAN-frame SIL로 넘어갈 migration checklist, 그리고 `current_angles` garbage-value 같은 외부 blocker를 분리해서 적도록 재구성했습니다.
- 13:19 KST (UTC+9) — `drum_intheloop`의 runtime URDF pose patch 주석을 고쳐, `RUNTIME_LINK_FRAME_PATCH_POSE`가 joint origin이 아니라 각 link의 `visual`과 `collision` origin을 둘 다 같은 pose로 덮어쓴다는 점이 바로 읽히도록 정리했습니다.
  - 수정 파일: `drum_intheloop/sil/urdf_tools.py`, `log.md`
  - 메모: 기존 설명은 `visual/collision`을 함께 적어 놓고도 실제로 둘 다 수정된다는 사실이 눈에 잘 안 들어와 오해를 부를 수 있었습니다. left/right arm 섹션 라벨도 `visual frame`에서 `link origin patch`로 바꿨습니다.
- 10:21 KST (UTC+9) — `drum_intheloop`의 현재 command-level SIL 구조, 실행 순서, 각도/visual 보정 계층, 알려진 한계와 디버깅 기준을 README와 하위 AGENTS에 최신 상태로 정리했습니다.
  - 수정 파일: `drum_intheloop/README.md`, `drum_intheloop/AGENTS.md`, `log.md`
  - 메모: 새 README는 reader/writer 경계, startup pose, `head_tilt -90` 보정, per-joint transform, runtime URDF pose patch, 현재 남은 이슈까지 처음 보는 사람이 따라갈 수 있도록 상세 설명으로 재구성했고, 하위 AGENTS는 같은 내용을 작업 원칙 관점으로 압축했습니다.

## 2026-03-25
- 18:05 KST (UTC+9) — arm runtime patch를 회전만 주는 방식에서 xyz+rpy pose 보정으로 확장해, x축 180도 회전 후 어깨/손목에서 mesh가 떠 보이던 현상을 STL bounds 중심 유지 기준으로 같이 비교할 수 있게 했습니다.
  - 수정 파일: `drum_intheloop/sil/urdf_tools.py`, `phil_intheloop/sil/urdf_tools.py`, `log.md`
  - 메모: `left/right_shoulder_1`, `left/right_shoulder_2`, `left/right_elbow`, `left/right_wrist`에 대해 runtime URDF에서만 `origin xyz`와 `origin rpy`를 함께 덮어쓰도록 바꿨고, xyz 값은 각 STL의 bounds center가 회전 전후로 유지되도록 계산한 1차 실험값입니다.
- 17:55 KST (UTC+9) — arm visual runtime patch를 어깨 1축과 손목 링크까지 확장해, 중간 링크만 뒤집혀 체인이 어깨/손목에서 분리돼 보이던 현상을 같은 실험 조건으로 연속 확인할 수 있게 했습니다.
  - 수정 파일: `drum_intheloop/sil/urdf_tools.py`, `phil_intheloop/sil/urdf_tools.py`, `log.md`
  - 메모: 체크인된 URDF/STL은 그대로 두고 runtime URDF에서만 `left/right_shoulder_1`, `left/right_shoulder_2`, `left/right_elbow`, `left/right_wrist`의 `visual/collision origin rpy`를 모두 x축 180도 회전시키도록 확장했습니다.
- 17:47 KST (UTC+9) — runtime URDF patch helper의 tuple 타입 힌트를 `typing.Tuple`로 바꿔, 실행 환경에서 import 시 `type object is not subscriptable` 에러가 나지 않도록 정리했습니다.
  - 수정 파일: `drum_intheloop/sil/urdf_tools.py`, `phil_intheloop/sil/urdf_tools.py`, `log.md`
  - 메모: 동작 변화는 없고, arm link runtime patch 코드를 실제 `python sil/SilCommandPipeReader.py` 실행 경로에서 바로 import할 수 있게 타입 annotation만 호환형으로 맞췄습니다.
- 17:45 KST (UTC+9) — `drum_intheloop`와 `phil_intheloop`의 runtime URDF patch에 상완/하완 링크 frame 보정 테이블을 추가해, 양팔 연결부가 바깥을 보는 현상을 x축 180도 회전으로 비교 확인할 수 있게 했습니다.
  - 수정 파일: `drum_intheloop/sil/urdf_tools.py`, `phil_intheloop/sil/urdf_tools.py`, `log.md`
  - 메모: 체크인된 URDF/STL은 그대로 두고, generated runtime URDF에서만 `left/right_shoulder_2`, `left/right_elbow`의 `visual/collision origin rpy`를 덮어씌우도록 했습니다. 수정 블록은 큰 구분 주석으로 분리해 이후 실험 시 숫자만 바로 바꿔볼 수 있게 정리했습니다.
- 17:10 KST (UTC+9) — `drum_intheloop`의 CAN-to-URDF 각도 변환을 joint별 `reference/sign/bias` 테이블로 일반화해, 어떤 관절이 단순 부호 반전인지 90도 기준 mirror인지 코드에서 바로 보이도록 정리했습니다.
  - 수정 파일: `drum_intheloop/sil/joint_map.py`, `drum_intheloop/sil/command_applier.py`, `log.md`
  - 메모: 현재 동작은 유지하면서 기존 `R_arm1` 특수처리도 동일한 수식으로 흡수했고, 사용자가 각 joint 보정을 눈으로 확인하고 바로 조정하기 쉽도록 큰 구분 주석을 함께 넣었습니다.
- 16:39 KST (UTC+9) — `drum_intheloop`의 `_place_robot_on_ground()`에 z축 보정과 AABB 계산 흐름을 이해하기 쉬운 설명 주석으로 덧붙였습니다.
  - 수정 파일: `drum_intheloop/sil/pybullet_backend.py`, `log.md`
  - 메모: 동작은 바꾸지 않고, base orientation은 유지한 채 바닥 clearance를 맞추기 위해 어떤 값을 계산하는지 바로 읽히도록 변수 의미를 주석으로 정리했습니다.

## 2026-03-24
- 18:06 KST (UTC+9) — SIL 시작 시 실기에서 손으로 맞춰두는 preset 자세를 다시 적용하고, `안녕 손 흔들어 봐` 같은 혼합 발화를 motion 경로로 승격하며, `/tmp/drum_command.pipe`가 있으면 `sudo` 실행에서도 SIL 모드가 자동으로 켜지도록 보강했습니다.
  - 수정 파일: `drum_intheloop/sil/robot_spec.py`, `drum_intheloop/sil/SilCommandPipeReader.py`, `phil_robot/pipeline/intent_classifier.py`, `phil_robot/pipeline/planner.py`, `DrumRobot2/src/CanManager.cpp`, `log.md`
  - 메모: startup pose 는 `DrumRobot.hpp`의 `initialJointAngles`와 같은 body preset(손목 90도) 기준으로 적용하고, 이후 실제 C++ HOME/READY pipe 명령이 들어오면 그 흐름으로 자연스럽게 덮이도록 했습니다. planner 쪽은 `chat + needs_motion` 혼합 케이스를 motion planning 으로 흘려 `wave_hi`가 비워지지 않게 했고, `DRUM_SIL_MODE` 환경변수가 없어도 reader가 만든 FIFO가 있으면 SIL 모드를 자동 활성화해 `sudo ./main.out` 경로에서도 body export가 살아나도록 했습니다.
- 17:52 KST (UTC+9) — `SilCommandPipeReader`가 시작 직후 HOME 자세를 강제로 적용하던 동작을 제거해, PyBullet이 실제 pipe 명령 흐름과 동기화되도록 되돌렸습니다.
  - 수정 파일: `drum_intheloop/sil/SilCommandPipeReader.py`, `drum_intheloop/sil/robot_spec.py`, `log.md`
  - 메모: 초기 pose는 더 이상 reader에서 주입하지 않고, 시뮬레이터는 실제로 수신한 `tmotor`/`maxon`/`dxl` 명령만 반영합니다. 함께 `robot_spec.py`에 임시로 추가했던 초기 pose 상수도 제거했습니다.
- 17:48 KST (UTC+9) — SIL 시작 자세 기준을 `initialJointAngles`가 아니라 실제 시작 흐름과 맞는 HOME 최종 자세로 정정해, 양 손목이 `75deg`로 내려온 상태에서 시작하도록 맞췄습니다.
  - 수정 파일: `drum_intheloop/sil/robot_spec.py`, `log.md`
  - 메모: DrumRobot2는 `initializePos("o")` 후 `HOME`으로 진입하므로, SIL도 `PathManager::setAddStanceAngle()`의 `homeAngle` 기준을 따르도록 바꿨습니다. head tilt는 이미 `90deg`로 일치하고 있어 손목 값만 `75deg`로 조정했습니다.
- 17:37 KST (UTC+9) — `SilCommandPipeReader` 시작 직후 DrumRobot2의 초기 자세와 DXL 기본 head 자세를 한 번 적용하도록 해, PyBullet이 URDF 기본 pose 대신 더 자연스러운 시작 자세로 뜨게 맞췄습니다.
  - 수정 파일: `drum_intheloop/sil/robot_spec.py`, `drum_intheloop/sil/SilCommandPipeReader.py`, `log.md`
  - 메모: body는 `DrumRobot.hpp`의 `initialJointAngles`, head는 `PathManager`의 기본 `readyAngle(12/13)`를 기준으로 사용했고, 변환은 기존 `CommandApplier`를 재사용해 sign/특수 매핑을 그대로 따르도록 했습니다.
- 17:31 KST (UTC+9) — `DRUM_SIL_MODE=1`일 때 미연결 CAN 모터를 `motors` 맵에서 지우지 않고, body 명령 export/queue 소비는 유지한 채 실제 CAN 송신과 TMotor send safety만 우회하도록 `CanManager`를 보강했습니다.
  - 수정 파일: `DrumRobot2/include/motors/Motor.hpp`, `DrumRobot2/include/managers/CanManager.hpp`, `DrumRobot2/src/CanManager.cpp`, `log.md`
  - 메모: 기본 실기 동작은 그대로 두고 SIL 모드에서만 body 모터를 살려 command-level exporter가 `tmotor/maxon`을 내보낼 수 있게 했습니다. disconnected 모터의 기본 socket 값도 `-1`로 명시해 안전하지 않은 초기 상태를 줄였습니다.
- 17:24 KST (UTC+9) — `pybullet_backend.py`에 바닥 plane 로드와 `_place_robot_on_ground()` 호출을 다시 붙이고, 초기 카메라를 더 높고 여유 있게 보이도록 조정했습니다.
  - 수정 파일: `drum_intheloop/sil/pybullet_backend.py`, `log.md`
  - 메모: `pybullet_data` 경로를 추가해 `plane.urdf`를 로드했고, 로봇을 clearance 기준으로 바닥 위에 맞춘 뒤 GUI 카메라는 더 높은 target과 완만한 pitch로 시작하게 조정했습니다.
- 17:19 KST (UTC+9) — `drum_intheloop`의 `pybullet_backend.py`에 GUI 디버그 패널 숨김과 초기 카메라 위치 설정을 다시 추가해, 창을 열었을 때 바로 로봇 자세를 보기 쉬운 상태로 시작하도록 정리했습니다.
  - 수정 파일: `drum_intheloop/sil/pybullet_backend.py`, `log.md`
  - 메모: `p.COV_ENABLE_GUI`를 꺼서 기본 패널을 숨기고, `resetDebugVisualizerCamera(...)`로 초기 시점을 고정했습니다. 요청에 맞춰 추가된 부분은 `#=====================` 블록으로 표시했습니다.
- 17:10 KST (UTC+9) — `SilCommandPipeWriter`가 실제 소비 지점에서 no-op으로 빠지지 않도록 `CanManager`와 DXL 소비 루프에서 명시적으로 `setEnabled(true)`를 호출하도록 켰습니다.
  - 수정 파일: `DrumRobot2/src/CanManager.cpp`, `DrumRobot2/src/DrumRobot.cpp`, `log.md`
  - 메모: 기존에는 writer 호출 자리는 있었지만 `enabled=false` 기본값 때문에 pipe로 아무 메시지도 쓰지 않았고, 이제 reader를 먼저 실행해두면 command-level SIL ingress가 실제로 데이터를 받기 시작합니다.
- 17:05 KST (UTC+9) — `drum_intheloop`의 옛 mock 엔트리포인트였던 `run_sil.py`를 제거하고, README 실행 문구를 현재 `sil/SilCommandPipeReader.py` 기준으로 정리했습니다.
  - 수정 파일: `drum_intheloop/run_sil.py`, `drum_intheloop/README.md`, `log.md`
  - 메모: `run_sil.py`는 이미 삭제된 `sil.server`를 가리키고 있었고, 현재 구조에서는 `drum_intheloop` 루트에서 `python sil/SilCommandPipeReader.py --mode gui`로 실행하는 경로만 남기면 충분합니다.
- 16:23 KST (UTC+9) — `motorTypeConversion(...)` 도입 후 `Motor` 변환 메서드의 비-const 시그니처에 맞게 `SilCommandPipeWriter`의 참조 타입을 조정하고 단독 컴파일을 다시 확인했습니다.
  - 수정 파일: `DrumRobot2/include/managers/SilCommandPipeWriter.hpp`, `DrumRobot2/src/SilCommandPipeWriter.cpp`, `log.md`
  - 메모: `TMotor::motorPositionToJointAngle()`와 `MaxonMotor::motorPositionToJointAngle()`가 `const` 메서드가 아니어서 writer helper 인자도 non-const 참조로 맞췄고, `SilCommandPipeWriter.cpp` 단독 컴파일이 통과했습니다.
- 16:17 KST (UTC+9) — `SilCommandPipeWriter`가 exporter 단계에서 `TMotor/Maxon`의 motor position과 DXL rad 값을 simulator용 joint degree로 변환해 내보내도록 `motorTypeConversion(...)` helper를 추가했습니다.
  - 수정 파일: `DrumRobot2/include/managers/SilCommandPipeWriter.hpp`, `DrumRobot2/src/SilCommandPipeWriter.cpp`, `DrumRobot2/src/CanManager.cpp`, `log.md`
  - 메모: pipe transport만 확인되는 수준을 넘어 Python `CommandApplier`가 바로 사용할 수 있도록, writer 호출부에서 실제 모터 객체를 함께 넘겨 joint-space degree로 변환한 뒤 NDJSON의 `position` 필드에 기록하게 했습니다.
- 16:05 KST (UTC+9) — `drum_intheloop`의 README를 현재 command-level SIL + named pipe 구조 기준으로 전면 정리하고, `joint_map`은 실제 매핑과 URDF limit patch에 필요한 최소 상수만 남기도록 다이어트했습니다.
  - 수정 파일: `drum_intheloop/README.md`, `drum_intheloop/sil/joint_map.py`, `drum_intheloop/sil/__init__.py`, `log.md`
  - 메모: 옛 `phil_intheloop`/TCP/server/protocol/state_model 설명을 걷어내고, `JOINT_LIMITS_DEG`, 초기/홈 포즈, gesture/clamp 상수는 제거했습니다. `URDF_JOINT_LIMITS_DEG`는 `urdf_tools.py`의 runtime limit patch 때문에 유지했습니다.
- 15:24 KST (UTC+9) — `drum_intheloop`에 C++ writer 기본 경로(`/tmp/drum_command.pipe`)를 그대로 읽는 최소 Python ingress인 `SilCommandPipeReader.py`를 추가했습니다.
  - 수정 파일: `drum_intheloop/sil/SilCommandPipeReader.py`, `log.md`
  - 메모: named pipe에서 NDJSON 한 줄을 읽어 `TMotorData`/`MaxonData`/DXL payload로 복원한 뒤 `CommandApplier -> PyBulletBackend`에 바로 넘기는 최소 러너를 마련했습니다.
- 14:48 KST (UTC+9) — `SilCommandPipeWriter`의 NDJSON 생성 코드를 학습 단계에 맞게 `std::ostringstream` 기반으로 정리하고, 이에 맞춰 `<sstream>` include와 불필요한 저수준 include를 정리했습니다.
  - 수정 파일: `DrumRobot2/src/SilCommandPipeWriter.cpp`, `log.md`
  - 메모: 문자열 버퍼 직접 조립보다 `TMotorData/MaxonData/DXL -> JSON 한 줄` 흐름이 바로 읽히는 구현을 우선했고, `charconv`/`cstdio`는 제거했습니다.
- 14:41 KST (UTC+9) — `SilCommandPipeWriter::buildTMotorLine()`의 TODO를 NDJSON 직렬화 구현으로 교체하고, 스트림 대신 직접 문자열 버퍼를 조립해 호출 비용을 줄였습니다.
  - 수정 파일: `DrumRobot2/src/SilCommandPipeWriter.cpp`, `log.md`
  - 메모: `motorName`, `position`, `velocityERPM`, `mode`, `useBrake`를 한 줄 JSON으로 만들고, 숫자 필드는 `snprintf`/`to_chars`로 바로 붙이도록 정리했습니다.
- 14:12 KST (UTC+9) — velog 초안을 사용자가 선호한 첫 번째 설명형 문체로 다시 정리해 `named pipe` 학습 배경과 SIL 맥락이 더 자연스럽게 드러나도록 수정했습니다.
  - 수정 파일: `named_pipe_velog_draft.txt`, `log.md`
  - 메모: 서두를 `오늘은 로봇 SIL 연결을 준비하면서...`로 되돌리고, `PathManager/CanManager` 문맥과 `command-level SIL` 동기를 더 직접적으로 설명하는 버전으로 맞췄습니다.
- 14:08 KST (UTC+9) — velog용 named pipe 정리 글 초안을 마크다운 문법이 살아 있는 `.txt` 파일로 루트에 추가했습니다.
  - 수정 파일: `named_pipe_velog_draft.txt`, `log.md`
  - 메모: velog에 바로 옮겨 붙일 수 있도록 제목, 본문, 코드펜스(````), 강조 문장 흐름을 포함한 초안 형태로 정리했습니다.
- 13:30 KST (UTC+9) — 루트 작업 규칙에 `DrumRobot2`는 가능하면 C++ 위주로 유지하고, Python 실행 코드는 별도 Python 영역에 둔다는 배치 원칙을 추가했습니다.
  - 수정 파일: `AGENTS.md`, `log.md`
  - 메모: 이후 `DrumRobot2/` 아래에는 가급적 새 Python 파일을 만들지 않고, `drum_intheloop/`나 `phil_intheloop/` 같은 별도 프로세스/폴더 쪽에 Python 진입점을 두는 방향을 기본으로 따릅니다.
- 12:37 KST (UTC+9) — `DrumRobot2` 안의 Python 실행 보조 파일을 정리하고, pipe reader 예제와 SIL 데모의 Python 진입점은 `drum_intheloop` 쪽에만 두도록 위치를 맞췄습니다.
  - 수정 파일: `drum_intheloop/tests/pipe_demo_reader.py`, `DrumRobot2/tests/pipe_demo_reader.py`, `DrumRobot2/tests/wave_nod_demo.py`, `log.md`
  - 메모: `DrumRobot2`에는 C++ writer 예제만 남기고, Python reader 및 SIL demo runner는 별도 Python 프로세스 영역인 `drum_intheloop/tests`로 정리했습니다.
- 12:33 KST (UTC+9) — named pipe 개념을 로봇 코드와 분리해서 이해할 수 있도록 `DrumRobot2/tests`에 C++ writer / Python reader 최소 예제를 추가했습니다.
  - 수정 파일: `DrumRobot2/tests/pipe_demo_writer.cpp`, `DrumRobot2/tests/pipe_demo_reader.py`, `log.md`
  - 메모: `DemoCommand` struct를 NDJSON 한 줄로 보내고 Python이 그대로 읽어 파싱하는 흐름만 남겨, `PathManager`나 `CanManager`를 빼고도 pipe 자체를 먼저 눈으로 확인할 수 있게 했습니다.
- 11:25 KST (UTC+9) — `SilCommandPipeWriter::writeLine()`이 문자열 복사 없이 NDJSON 라인과 개행을 `writev()`로 바로 기록하도록 구현했습니다.
  - 수정 파일: `DrumRobot2/src/SilCommandPipeWriter.cpp`, `log.md`
  - 메모: `EINTR`은 즉시 재시도하고, reader disconnect로 인한 `EPIPE`가 나면 fd를 닫아 다음 호출에서 다시 열 수 있게 했습니다.
- 11:18 KST (UTC+9) — `SilCommandPipeWriter::openPipe()`가 FIFO를 생성 또는 재사용한 뒤 non-blocking writer fd를 열도록 구현했습니다.
  - 수정 파일: `DrumRobot2/src/SilCommandPipeWriter.cpp`, `log.md`
  - 메모: `mkfifo()`에서 기존 파이프가 이미 있으면 그대로 사용하고, `O_WRONLY | O_NONBLOCK | O_CLOEXEC`로 열어 reader 부재 시 다음 호출에서 다시 시도할 수 있게 했습니다.

## 2026-03-23
- 11:18 KST (UTC+9) — `DrumRobot2`에 1차 command-level SIL용 named pipe exporter 골격(`SilCommandPipeWriter`)을 추가하고, 실제 소비 지점인 `CanManager::setCANFrame()` 및 DXL 소비 지점에서 호출 자리를 마련했습니다.
  - 수정 파일: `DrumRobot2/include/managers/SilCommandPipeWriter.hpp`, `DrumRobot2/src/SilCommandPipeWriter.cpp`, `DrumRobot2/src/CanManager.cpp`, `DrumRobot2/src/DrumRobot.cpp`, `log.md`
  - 메모: writer 기본값은 비활성(no-op)이라 기존 동작에는 영향이 없고, `TMotorData`/`MaxonData`/DXL command를 NDJSON으로 FIFO에 내보내는 TODO만 채우면 ingress 실험을 시작할 수 있는 상태입니다. `Makefile`의 `src/*.cpp` 와일드카드로 새 `.cpp`가 자동 빌드 대상에 포함되는 것도 dry-run으로 확인했습니다.
- 17:56 KST (UTC+9) — `command_applier.py`가 예전 stateful 버전으로 남아 있던 흔적을 정리하고, stateless command-to-joint-target 변환 버전으로 다시 맞췄습니다.
  - 수정 파일: `drum_intheloop/sil/command_applier.py`, `log.md`
  - 메모: `clear()`/`get_joint_targets()` 기반 내부 저장 구조를 제거하고, `build_default_can_command()`와 `apply_can_command(motor_name, command)` / `apply_dxl_command()`가 바로 URDF joint target dict를 반환하도록 복구했습니다. `DrumRobot2/tests/wave_nod_demo.py --mode direct` 재실행으로 동작도 다시 확인했습니다.
- 17:50 KST (UTC+9) — `drum_intheloop`에 C++ `TMotorData`/`MaxonData` 대응 Python command 자료구조를 추가하고, `CommandApplier`가 command 객체를 받아 바로 URDF joint target dict를 반환하도록 정리했습니다.
  - 수정 파일: `drum_intheloop/sil/command_types.py`, `drum_intheloop/sil/command_applier.py`, `drum_intheloop/tests/wave_nod_demo.py`, `drum_intheloop/sil/server.py`, `log.md`
  - 메모: `clear()`/`get_joint_targets()` 같은 내부 상태 저장을 제거하고, `apply_can_command()`가 `TMotorData` 또는 `MaxonData`를 받아 검증 후 매핑하도록 바꿨습니다. `wave_nod_demo.py`와 `server.py`도 새 구조로 맞췄고, `drum4` 환경에서 `DrumRobot2/tests/wave_nod_demo.py --mode direct` 재실행까지 확인했습니다.
- 17:05 KST (UTC+9) — `wave` 포즈가 의도와 다르게 보이던 원인으로 확인된 `R_arm1` 예외 각도 변환을 `command_applier`에 복원했습니다.
  - 수정 파일: `drum_intheloop/sil/command_applier.py`, `log.md`
  - 메모: backend 단순화 과정에서 빠졌던 `R_arm1`의 90도 기준 미러링 보정을 `command_applier`로 옮겨 매핑 책임 분리 구조 안에서 유지하도록 조정했습니다.
- 17:02 KST (UTC+9) — `DrumRobot2/tests`에서 바로 `python wave_nod_demo.py`로 실행할 수 있도록 `drum_intheloop` 데모 러너 래퍼를 추가하고, `drum4` 환경에서 `--mode direct` 실행까지 확인했습니다.
  - 수정 파일: `DrumRobot2/tests/wave_nod_demo.py`, `log.md`
  - 메모: 실제 구현은 `drum_intheloop/tests/wave_nod_demo.py`에 두고, `DrumRobot2/tests`에서는 경로를 신경쓰지 않고 동일 이름으로 실행할 수 있게 연결했습니다.
- 16:57 KST (UTC+9) — `wave`/`nod`를 하드코딩으로 재생해 `CommandApplier -> PyBulletBackend` 흐름을 확인할 수 있도록 `drum_intheloop/tests/wave_nod_demo.py` 러너를 추가했습니다.
  - 수정 파일: `drum_intheloop/tests/wave_nod_demo.py`, `log.md`
  - 메모: `joint_map.get_gesture_sequence()`를 읽어 `CommandApplier`가 URDF joint target dict를 만들고, `PyBulletBackend.apply_targets()`가 그 dict를 적용하는 최소 경로를 테스트할 수 있게 했습니다.
- 16:14 KST (UTC+9) — `drum_intheloop`에서 매핑 책임을 `command_applier`로 몰아주고, `pybullet_backend`는 URDF joint target 적용만 담당하도록 역할을 분리했습니다.
  - 수정 파일: `drum_intheloop/sil/command_applier.py`, `drum_intheloop/sil/pybullet_backend.py`, `drum_intheloop/sil/server.py`, `log.md`
  - 메모: backend에서 `joint_map` 의존을 제거했고, `command_applier`가 production name과 DXL logical joint를 URDF joint target dict로 변환하도록 정리했습니다. `server.py`도 새 경로에 맞게 `CommandApplier`를 통해 target을 구성하도록 맞췄습니다.
- 16:06 KST (UTC+9) — `drum_intheloop`의 `pybullet_backend.py`를 더 줄여 `start()`가 PyBullet 연결, 런타임 URDF 생성, 로봇 로드, joint index 조회만 수행하도록 단순화했습니다.
  - 수정 파일: `drum_intheloop/sil/pybullet_backend.py`, `log.md`
  - 메모: 바닥, 중력, debug visualizer, self-collision flag, 기본 모터 비활성화, ground placement를 초기 경로에서 제거해 smoke test와 학습용 읽기 난도를 낮췄습니다.
- 15:47 KST (UTC+9) — `drum_intheloop`의 `pybullet_backend.py`를 smoke test용 최소 구조로 정리해 시작/종료, URDF 런타임 패치, joint index 조회, target 적용, ground placement만 남겼습니다.
  - 수정 파일: `drum_intheloop/sil/pybullet_backend.py`, `log.md`
  - 메모: `wave`/`nod` 같은 하드코딩 동작을 먼저 확인할 수 있도록 시각 테마, 카메라, 스크린샷, 색상 처리 등 부수 기능을 제거해 읽기 쉽게 단순화했습니다.
- 11:33 KST (UTC+9) — DYNAMIXEL 단품 테스트 스크립트에 `ping` 재시도와 포트 버퍼 초기화를 추가해 불안정한 응답 상황에서 바로 실패하지 않도록 보강했습니다.
  - 수정 파일: `DrumRobot2/tests/dxl_move_test.py`, `log.md`
  - 메모: 단일 `ping`에서 `Incorrect status packet`이 발생해도 여러 번 재시도하도록 했고, 각 read/write 전 포트 버퍼를 비우도록 조정했습니다.
- 11:31 KST (UTC+9) — 저장소의 루트/하위 `AGENTS.md` 범위를 다시 확인하고, DYNAMIXEL 단품 테스트용 스크립트를 `tests/` 아래로 추가했습니다.
  - 수정 파일: `DrumRobot2/tests/dxl_move_test.py`, `log.md`
  - 메모: `XM430-W350-T` 테스트 모터는 `Protocol 2.0`, `57600 baud`, `ID 28`, `model 1020`으로 응답함을 확인했고, `U2D2`는 전원 공급기가 아니라 USB-DYNAMIXEL 통신 어댑터라는 점과 TTL/RS485 배선 구분을 확인했습니다.

## 2026-03-20
- 15:31 KST (UTC+9) — 작업 로그 시각을 한국시간 기준으로 통일하도록 루트 `AGENTS.md`와 `log.md`를 업데이트했습니다.
  - 수정 파일: `AGENTS.md`, `log.md`
  - 메모: 이후 로그는 별도 지시가 없으면 한국시간(KST, UTC+9/GMT+9)으로 기록합니다.
- 15:16 KST (UTC+9) — 저장소 전역 작업 규칙을 문서화하기 위해 루트 `AGENTS.md`와 `log.md`를 새로 추가했습니다.
  - 수정 파일: `AGENTS.md`, `log.md`
  - 메모: 이후 에이전트 작업 시 변경 내용을 같은 형식으로 누적 기록하도록 기준을 마련했습니다.
## 2026-03-25
- 14:22 KST (UTC+9) — 현재 SIL/TCP 통합 상태와 우회 사항을 문서로 정리해 future handoff용 컨텍스트를 남겼다.
  - 수정 파일: `AGENTS.md`, `drum_intheloop/README.md`
  - 메모: 명시적 SIL 모드, FIFO 수명, disconnected CAN/Maxon 우회, TCP/gate/CSV 착시 문제와 실행 순서를 README/AGENTS에 반영.

## 2026-03-25
- 11:19 KST (UTC+9) — SIL 모드에서 미연결 Maxon 초기화 하드웨어 왕복을 건너뛰도록 해 pre-TCP 단계의 CAN 노이즈를 줄였다.
  - 수정 파일: `DrumRobot2/src/DrumRobot.cpp`
  - 메모: `motorSettingCmd`, `maxonMotorEnable`, `setMaxonMotorMode`에서 disconnected Maxon을 우회해 TCP 서버가 뜨기 전부터 `sendAndRecv`/`txFrame` 오류가 반복되지 않도록 조정.

## 2026-03-25
- 11:04 KST (UTC+9) — SIL 활성화 경계를 명시적 모드로 좁히고 FIFO 수명을 reader 중심으로 정리했다.
  - 수정 파일: `DrumRobot2/src/CanManager.cpp`, `DrumRobot2/src/DrumRobot.cpp`, `DrumRobot2/include/managers/SilCommandPipeWriter.hpp`, `DrumRobot2/src/SilCommandPipeWriter.cpp`, `drum_intheloop/sil/SilCommandPipeReader.py`
  - 메모: `DRUM_SIL_MODE=1`일 때만 pipe writer를 켜고, reader가 `/tmp/drum_command.pipe`를 매 실행마다 삭제 후 재생성/종료 시 정리하도록 바꿔 stale FIFO로 인한 SIL 오인 검출을 막음.
