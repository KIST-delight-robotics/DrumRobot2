# drum_intheloop TODO

`drum_intheloop`는 `DrumRobot2`의 command-level 출력을 `named pipe`로 받아
`PyBullet`에 적용하는 SIL 디렉터리다.

이 보드는 두 층을 같이 적는다.

- `drum_intheloop` 안에서 직접 고치는 simulator 작업
- simulator를 켰을 때만 드러나는 통합 blocker

단, 통합 blocker를 적더라도 수정 책임이 이 디렉터리 밖이면 그 사실을 같이 적는다.

현재 방향:

- 현재 공식 경계는 `LLM 플래너 -> 로봇 제어기 -> command-level simulator`이다.
- 현재 ingress는 `/tmp/drum_command.pipe` NDJSON이다.
- 현재 backend는 `resetJointState()` 기반 즉시 적용형 viewer다.
- 장기 목표는 command-level seam을 유지한 채, 이후 `vcan` 또는 `struct can_frame`
  기반 CAN-frame SIL로 넘어가는 것이다.

운영 규칙:

- `Now`에는 최대 3개만 둔다.
- 원본 URDF/STL 수정 대신 runtime patch를 우선한다.
- 라우팅 문제, 각도 semantic 문제, visual/frame 문제, C++ state 문제를 섞어 쓰지 않는다.
- simulator를 켰을 때만 드러나는 외부 blocker는 따로 적되, simulator 내부 작업과 섞어 쓰지 않는다.

## Now

- [V] `READY -> snare` 자세 mismatch를 층별로 분리
  - 목표: READY/시작 자세가 스네어 방향으로 모이지 않는 원인이 startup preset인지,
    joint 의미 매핑인지, visual frame patch인지 분리한다.
  - 확인 포인트:
    - `sil/robot_spec.py` startup preset
    - `sil/joint_map.py` CAN -> URDF transform
    - `sil/urdf_tools.py` runtime pose patch
  - 완료 기준:
    - 원인 층이 문서로 명시된다.
    - 기준 스크린샷 또는 joint target 값이 함께 남는다.

- [ ] arm visual 보정값을 `눈대중`과 `확정값`으로 분리
  - 목표: 현재 xyz/rpy 값이 eyeballing 기반이라는 사실을 남기고, 추후 측정값으로
    교체하기 쉽게 정리한다.
  - 완료 기준:
    - 링크별 patch 값에 임시/확정 상태가 드러난다.
    - 값이 왜 필요한지 한 줄 설명이 붙는다.

- [ ] command-level seam에서 CAN-frame SIL로 넘어갈 migration checklist 작성
  - 목표: 지금 seam이 무엇을 보존하고 무엇을 잃는지 정리해, 다음 단계가
    `frame-accurate SIL` 쪽으로 이어지게 한다.
  - 포함 항목:
    - 현재 pipe payload가 보존하는 것
    - 현재 pipe payload가 잃는 것
    - `vcan` vs `struct can_frame` replay 후보
    - timing fidelity requirements

## Next

- [ ] named pipe trace capture/replay 러너 추가
  - command-level ingress를 파일로 저장하고 재생해 mapping/visual regression을 반복 확인한다.

- [ ] startup pose와 실제 첫 pipe command 사이의 덮어쓰기 흐름을 더 잘 보이게 만들기
  - startup preset이 잠깐 보이는 문제와 실제 HOME/READY 흐름을 눈으로 분리해서 확인한다.

- [ ] joint target / URDF joint name overlay 또는 로그 정리
  - `move:L_arm2`가 실제로 어떤 URDF joint에 몇 도로 적용됐는지 바로 확인할 수 있게 만든다.

## Simulator-Triggered Integration Blockers

- [ ] simulator ON일 때 `robot_state.current_angles`가 planner/validator를 흔드는 경로 정리
  - 현상:
    - simulator를 켜지 않았을 때는 `current_angles`가 단순 0도 스냅샷 수준으로 들어와도
      planner/validator 해석이 비교적 예측 가능하다.
    - simulator를 켜면 미연결 모터 유지, startup pose, state broadcast, validator 입력이
      함께 얽히면서 `garbage angle` 또는 `잘못 믿을 current angle` 문제가 드러난다.
  - 이 TODO에 적는 이유:
    - 책임 코드는 `DrumRobot2`/`phil_robot` 쪽일 수 있어도,
      문제가 실제로 surfaced 되는 트리거가 `drum_intheloop` 시뮬레이터 실행이기 때문이다.
  - 확인할 것:
    - simulator ON/OFF에서 `current_angles` 스냅샷 차이
    - startup preset이 state 해석에 주는 영향
    - planner resolved command와 validator reject reason의 차이
  - 목표:
    - `simulator 때문에 생긴 것`과 `원래부터 있던 state 문제`를 분리한다.

## Timing Facts To Preserve

- `DrumRobot2` send loop 주기는 `1ms`다.
- `TMotor` command 소비/송신은 `5ms` 주기다.
- `Maxon` command 송신은 `1ms` 주기다.
- `DXL`은 현재 `cycleCounter == 0` 경로라 `5ms` 주기다.
- CAN receive loop는 `100us` 주기다.
- 현재 `drum_intheloop` backend는 timing 없는 `resetJointState()` 기반이다.
- 발 두 개를 제외하고 현재 cadence를 그대로 보면, `5ms` 창에서 대략
  `TMotor 7회 + Maxon 10회 + DXL 2개 joint target = 19개 actuator update`
  수준을 의식해야 한다.

## Later

- [ ] PyBullet 쪽 timestamped replay 실험
  - 현재 즉시 적용형 backend에서 한 단계 나아가, 최소한의 시간축 재생을 시험한다.

- [ ] command-level pipe와 frame-level ingress를 공존시키는 구조 검토
  - 빠른 시각 검증 경로와 더 높은 fidelity 경로를 분리할지 검토한다.

- [ ] frame-accurate SIL에서 필요한 state/schema 목록 정리
  - 단순 joint target 외에 frame id, send order, timestamp, feedback validity가
    얼마나 필요한지 미리 정리한다.
