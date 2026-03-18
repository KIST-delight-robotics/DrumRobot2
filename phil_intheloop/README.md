# phil_intheloop

`phil_intheloop`는 실제 로봇을 바로 건드리지 않고 `PyBullet`에서 Phil 로봇을 띄워 보는 1차 시뮬레이션 디렉터리입니다.

현재 목표:

- URDF를 `PyBullet`에 띄운다.
- `127.0.0.1:9999`에서 기존 `phil_robot`와 비슷한 계약으로 통신한다.
- 완전한 SIL이 아니어도, 스크린샷과 데모 화면을 만들 수 있게 한다.

아직 하지 않는 것:

- 엄밀한 실시간 제어 재현
- 실제 CAN/모터 루프 재현
- 완전한 관절/제스처 매핑
- 충돌 기반 안전성 검증

## 설치

`drum4` 환경 기준:

```bash
/home/shy/miniforge3/envs/drum4/bin/python -m pip install -r phil_intheloop/requirements.txt
```

## 빠른 실행

GUI가 되는 환경이면:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_intheloop/run_sil.py --mode gui
```

헤드리스에서 바로 한 장 뽑아보려면:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_intheloop/run_sil.py \
  --mode direct \
  --demo wave \
  --frames 240 \
  --screenshot phil_intheloop/artifacts/wave.ppm
```

이렇게 실행하면:

- 로봇 URDF를 로드한다.
- `gesture:wave`에 해당하는 간단한 포즈 시퀀스를 적용한다.
- 마지막 프레임을 `PPM` 이미지로 저장한다.

## phil_robot와 붙여보기

실제 `DrumRobot2`를 띄우지 않고, 대신 이 시뮬레이터가 `9999` 포트를 잡도록 실행한다.

터미널 1:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_intheloop/run_sil.py --mode gui
```

터미널 2:

```bash
cd phil_robot
/home/shy/miniforge3/envs/drum4/bin/python phil_brain.py
```

이 경우 `phil_robot`는 기존처럼 `127.0.0.1:9999`에 붙지만, 실제 C++ 로봇 대신 `PyBullet` 시뮬레이터에 연결됩니다.

주의:

- 현재 구조상 음성 루프도 원리적으로는 연결된다.
- 즉, `run_sil.py`를 먼저 띄우고 그다음 `phil_brain.py`를 실행하면, `phil_robot`가 생성한 명령은 실제 C++ 대신 SIL로 들어간다.
- 다만 제가 실제로 끝까지 검증한 것은 소켓 명령 레벨이며, 마이크/STT/TTS를 포함한 end-to-end 음성 루프는 아직 별도 smoke test로 고정하지 않았다.

## 지원 범위

현재 구현은 다음 명령을 1차적으로 지원합니다.

- `r`
- `h`
- `s`
- `p:<song_code>`
- `look:<pan>,<tilt>`
- `gesture:<name>`
- `led:<emotion>`
- `move:<joint>,<angle>`

지원하지 않거나 느슨하게 처리하는 부분:

- 실제 드럼 연주 궤적
- `R_foot`, `L_foot` 물리 매핑
- 정교한 gesture 타이밍
- 실제 하드웨어와 동일한 상태 천이

## 파일 구조

- `run_sil.py`
  - 가장 단순한 실행 엔트리포인트
- `sil/server.py`
  - TCP 서버, 명령 처리, 상태 브로드캐스트
- `sil/pybullet_backend.py`
  - URDF 로드, `PyBullet` 제어, 스크린샷
- `sil/state_model.py`
  - 프로덕션 호환 상태 모델
- `sil/joint_map.py`
  - 프로덕션 이름, URDF 이름, 관절 범위, 간단한 포즈/제스처 정의
- `sil/protocol.py`
  - 줄바꿈 기반 프로토콜과 JSON 직렬화
