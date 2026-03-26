이 디렉터리는 `DrumRobot2`의 command-level 출력을 `PyBullet`에 적용하는 느슨한 SIL 디렉터리다.

이 범위에서 작업할 때는 아래 원칙을 기본으로 따른다.

## 이 디렉터리의 현재 역할

- `DrumRobot2` C++ pipe writer가 내보낸 NDJSON command를 읽는다.
- production motor 이름과 각도 의미를 URDF joint target으로 변환한다.
- 체크인된 URDF/STL을 직접 수정하지 않고 runtime URDF 복사본에 보정을 적용한다.
- `PyBullet`에서 빠르게 시각 검증 가능한 command-level simulator를 유지한다.

## 반드시 기억할 현재 사실

- `/tmp/drum_command.pipe`의 생성/삭제는 `sil/SilCommandPipeReader.py`가 담당한다.
- C++ export는 현재 `DRUM_SIL_MODE=1`일 때만 활성화된다고 가정한다.
- reader는 시작 직후 `sil/robot_spec.py`의 startup preset pose를 한 번 적용한다.
- `head_tilt`의 `-90deg` 보정은 `sil/command_applier.py`에 있다.
- CAN joint 의미 보정은 `sil/joint_map.py`의 `PRODUCTION_TO_URDF_CAN_TRANSFORM`이 담당한다.
- arm 외형 보정은 `sil/urdf_tools.py`의 runtime pose patch가 담당하며, 아직 실험 중이다.
- 현재 backend는 `resetJointState()` 기반이라 timing 없이 즉시 적용된다.

## 변경 우선순위

문제가 이 디렉터리에서 해결 가능한지 먼저 층을 구분한다.

1. ingress/pipe 문제인가
2. production 이름 -> URDF 이름 매핑 문제인가
3. production 각도 의미 -> URDF 각도 의미 변환 문제인가
4. runtime URDF visual/collision frame 문제인가
5. 이 디렉터리 바깥(planner, TCP, C++ state JSON, 하드웨어 초기화) 문제인가

이 중 2~4는 가능하면 `drum_intheloop/` 안에서 해결하는 것을 우선한다.

## 코드 수정 원칙

- 코드 변경 시 `drum_intheloop/` 코드를 우선 수정한다.
- `PyBulletBackend`는 가능한 한 단순한 backend로 유지하고, 의미 변환 로직을 불필요하게 넣지 않는다.
- production-to-URDF 각도 보정은 `sil/command_applier.py` 또는 `sil/joint_map.py`에 둔다.
- visual/collision origin 보정은 `sil/urdf_tools.py`의 runtime patch에서 먼저 시도한다.
- 체크인된 `urdf/` 아래 원본 URDF/STL 파일은 직접 수정하지 않는다.
- `package://` 경로 수정, limit 보강, visual/collision origin 보정은 runtime 임시 파일로 처리한다.
- 관절 범위의 기준이 필요하면 Python validator보다 C++ 쪽 범위를 우선 참고한다.

## 디버깅할 때의 판단 기준

- `move:L_arm2`가 실제로 `left_shoulder_2`에 적용되면 라우팅은 대체로 맞는 것이다.
- 팔이 반대 방향으로 돌면 먼저 `joint_map.py`의 transform을 본다.
- 팔이 맞는 방향으로 도는데 어깨/손목에서 빠져 보이면 먼저 `urdf_tools.py`의 runtime pose patch를 본다.
- planner가 오른팔이라고 말하면서 `L_arm*`를 생성하면 그건 대개 이 디렉터리 밖 문제다.
- `current_angles`가 비정상적으로 깨져 보이면 C++ state broadcast 경로를 의심하고, 이 디렉터리만의 버그로 단정하지 않는다.

## 문서와 설명 원칙

- 사용자가 헷갈리기 쉬운 층을 분리해서 설명한다.
  - 라우팅 문제
  - 각도 semantic 문제
  - visual/frame 문제
  - planner 의미 해석 문제
- README는 실제 코드 상태와 맞아야 하며, 실험 중인 내용은 “확정된 사실”과 섞어 쓰지 않는다.
- arm visual patch처럼 아직 조정 중인 값은 “실험 중”이라고 명시한다.

## 작업 로그

- 이 디렉터리에서 의미 있는 변경을 하면 루트 `log.md`에 반드시 기록한다.
- 기록 시 한국시간(KST, UTC+9)과 수정 파일, 변경 이유를 함께 남긴다.

## 추천 작업 순서

1. `sil/SilCommandPipeReader.py`로 ingress와 startup 흐름을 확인한다.
2. `sil/command_applier.py`와 `sil/joint_map.py`로 각도 의미 변환을 확인한다.
3. `sil/urdf_tools.py`로 runtime URDF patch를 확인한다.
4. `sil/pybullet_backend.py`는 마지막에 PyBullet 적용/표시 문제인지 확인하는 용도로 본다.
5. 이 범위를 벗어나면 `DrumRobot2`, `phil_robot`, `phil_intheloop` 중 어디 책임인지 먼저 분리한다.
