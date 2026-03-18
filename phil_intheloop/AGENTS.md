이 디렉터리에서는 `PyBullet` 기반 1차 시각화와 느슨한 `SIL`(Simulation in the Loop) 준비만 다룬다.

규칙:

- 기본 원칙은 `phil_intheloop/` 안에만 새 코드를 추가하는 것이다.
- `DrumRobot2/`와 `phil_robot/`는 명시적으로 승인받지 않은 한 수정하지 않는다.
- 기존 런타임 계약을 유지한다.
  - 호스트: `127.0.0.1`
  - 포트: `9999`
  - 입력: 줄바꿈(`\n`)으로 끝나는 명령 문자열
  - 출력: 줄바꿈(`\n`)으로 끝나는 JSON 상태 문자열
- 프로덕션 상태 키 이름과 명령 포맷은 유지한다.
  - 예: `state`, `is_fixed`, `last_action`, `current_angles`, `move:L_wrist,90`, `look:0,90`
- `urdf/` 아래 자산은 원본으로 취급한다.
  - `package://` 경로 수정이나 limit 보강이 필요하면 런타임 임시 파일로 처리한다.
  - 체크인된 URDF/STL 파일은 직접 수정하지 않는다.
- 관절 이름 매핑은 한 곳에만 둔다.
  - 프로덕션 이름(`R_arm1`, `L_wrist`)과 URDF 이름(`right_shoulder_1`, `left_wrist`) 간 매핑은 `sil/joint_map.py`에서만 관리한다.
- 1차 목표는 엄밀한 동역학이 아니라 "로봇을 띄우고, 명령을 받아, 화면과 상태를 보여주는 것"이다.
- 정확한 물리/충돌/복구 정책은 이후 단계에서 보강한다.
- 관절 범위가 필요하면 Python validator보다 C++의 범위를 우선 기준으로 삼는다.
