# phil_intheloop TODO

현재 기준:

- `PyBullet`에서 URDF를 띄울 수 있다.
- `127.0.0.1:9999`에서 `phil_robot`와 비슷한 계약으로 통신할 수 있다.
- 1차 데모, 스크린샷, 간단한 명령 재생은 가능하다.

## Now

- [x] URDF를 `PyBullet`에서 로드하기
- [x] `package://` 메쉬 경로를 런타임에서 보정하기
- [x] zero joint limit을 런타임 URDF 복사본에서 보강하기
- [x] 최소 TCP 서버 만들기
- [x] newline command / newline JSON state 계약 맞추기
- [x] `move`, `look`, `gesture`, `led`, `p:*`의 1차 처리 넣기
- [x] 헤드리스 스크린샷 생성 경로 만들기

## Next

- [ ] `phil_robot`와 붙인 end-to-end smoke test 스크립트 추가
- [ ] 텍스트 입력 기반 데모 런처 추가
- [ ] `wave`, `nod`, `shake`, `happy` gesture를 더 보기 좋게 다듬기
- [ ] `PPM` 말고 `PNG` 저장 옵션 추가
- [ ] 카메라 프리셋 몇 개 추가

## Controller-In-The-Loop

- [ ] `DrumRobot2`의 `AgentSocket` 경로를 그대로 살린 simulator mode 설계
- [ ] 하드웨어 초기화와 simulator backend를 분리할 seam 정의
- [ ] `AgentAction` / state machine을 유지한 채 backend만 교체하는 구조 설계
- [ ] C++가 생성한 state와 `PyBullet` state를 연결하는 adapter 설계

## Fidelity

- [ ] `R_foot`, `L_foot` placeholder 처리 방향 정하기
- [ ] head joint와 production state 표현 방식 정리
- [ ] play 상태 진행률을 더 자연스럽게 만들기
- [ ] `gesture:*`를 C++ 동작과 더 비슷하게 맞추기
- [ ] `look:*`와 body joint 동시 제어를 더 자연스럽게 만들기

## Safety

- [ ] self-collision 체크 추가
- [ ] joint limit 초과 시 warning/log 강화
- [ ] scripted scenario runner 추가
- [ ] failure/recovery 시나리오 추가

## Docs

- [ ] 데모 촬영용 권장 실행 절차 정리
- [ ] 실제 로봇 대신 SIL에 붙이는 방법 더 자세히 문서화
- [ ] 현재 가능한 것 / 아직 아닌 것 표로 정리
