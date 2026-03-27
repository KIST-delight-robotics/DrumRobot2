# 드럼 로봇 개발 TIL - `drum_intheloop`는 real SIL이 아니라 command-level viewer에 가깝다

이번 세션에서는 내가 만든 `drum_intheloop` 시뮬레이터를 다시 정의해 보고, 왜 `current_angles`가 시뮬레이터와 다르게 보이는지, 왜 `k` 입력 타이밍에 따라 상태 반영이 달라 보이는지까지 한 번에 정리했다.

결론부터 말하면 지금의 `drum_intheloop`는 `real SIL`이라기보다 `command-level simulator` 혹은 `viewer`에 더 가깝다. 그리고 지금 겪는 혼란의 핵심은 대부분 "같은 각도처럼 보이지만 사실 서로 다른 source of truth를 읽고 있다"는 점에서 나온다.

## 지금 구조를 한 문장으로 요약하면

현재 경계는 아래에 가깝다.

```text
LLM planner
-> DrumRobot2
-> command export
-> named pipe
-> drum_intheloop
-> PyBullet viewer
```

즉 `drum_intheloop`는 컨트롤러를 대신하는 가짜 하드웨어라기보다, `DrumRobot2`가 이미 계산해서 소비한 command를 밖으로 뽑아 `PyBullet`에 적용해 눈으로 확인하는 층이다.

이 말은 곧:

- 현재 simulator는 closed-loop SIL이 아니다.
- feedback을 다시 제어기에 넣어 주는 구조가 아니다.
- 지금은 command-level seam을 검증하는 단계다.
- 내부 timing fidelity도 actuator-level realtime과는 거리가 있다.

## 왜 real SIL이라고 부르기 애매한가

real SIL이라고 부르려면 보통 컨트롤러가 실제 모터 대신 시뮬레이터와 상호작용해야 한다. 즉 command를 보내면 시뮬레이터가 그 결과를 state, feedback, delay, limit, fault 같은 형태로 다시 돌려줘야 한다.

하지만 지금 구조는 다르다.

- C++ 제어기가 이미 command를 소비한다.
- 그 command를 pipe로 export한다.
- Python이 그 값을 바로 `PyBullet` joint에 적용한다.

그래서 현재 `drum_intheloop`는 "control loop 안의 participant"보다 "control output을 보는 observer"에 가깝다.

## `current_angles`와 시뮬레이터 화면이 왜 다를까

이 부분이 이번 세션에서 가장 중요한 포인트였다.

겉보기에는 둘 다 "현재 각도"처럼 보이지만, 실제로는 서로 다른 값을 읽고 있다.

### 1. C++ state JSON의 `current_angles`

`current_angles`는 `motor->jointAngle`를 읽어 만든다. 이 값은 CAN feedback를 받았을 때 갱신되는 실측 계열 값이다.

즉 의미는 대체로 이쪽이다.

- measured angle
- feedback 기반 state

### 2. `drum_intheloop`의 PyBullet 자세

시뮬레이터는 pipe로 받은 command를 읽어서 바로 `resetJointState()`에 적용한다.

즉 의미는 대체로 이쪽이다.

- consumed command
- simulator-applied pose

그래서 viewer는 움직였는데 `current_angles`는 0이거나 이상한 값으로 남아 있을 수 있다.

이건 복사 방식의 문제가 아니라 source of truth의 문제다.

## 왜 `L_arm2`, `L_arm3`만 쓰레기 값이 튀었나

`current_angles`에 거대한 숫자가 들어간 이유도 꽤 명확했다.

- `jointAngle`는 feedback가 와야 갱신된다.
- SIL 우회 상태에서는 disconnected motor가 feedback를 못 받을 수 있다.
- 그런데 초기 `jointAngle`가 안전한 값으로 초기화되지 않으면 미정값이 그대로 state JSON에 들어갈 수 있다.

그래서 `L_arm2`, `L_arm3`처럼 실제 feedback가 없던 관절에서만 큰 쓰레기 값이 튀는 현상이 설명된다.

반대로 손목은 viewer에서 움직였어도 state JSON 기준으로는 여전히 `0`일 수 있다. viewer는 command를 따라갔고, state는 measured feedback를 못 받았기 때문이다.

## "깊은 복사해서 가져오면 같은 값 아닌가?"가 헷갈렸던 이유

처음에는 나도 "어차피 시뮬레이터로 보내진 값을 복사하면 같은 값 아닌가?"라는 생각이 들었다.

그런데 실제로는 최소 3개의 값이 섞여 있었다.

- measured angle
- consumed command
- final target

이 셋은 같은 타이밍에 서로 다를 수 있다.

예를 들어 `move:L_wrist,75` 같은 명령이 들어오면:

- planner는 최종 목표 `75`를 정할 수 있다.
- send loop는 그중 현재 tick에 소비한 작은 step만 보낼 수 있다.
- viewer는 그 step을 적용한다.
- feedback가 없으면 measured angle은 여전히 이전값일 수 있다.

그래서 무엇을 복사하느냐에 따라 의미가 달라진다.

- `final target`을 복사하면 "지금 자세"가 아니라 "결국 가고 싶은 자세"가 된다.
- `consumed command`를 복사하면 현재 viewer와는 꽤 잘 맞는다.
- `measured angle`은 실측이지만 SIL 우회에서는 비거나 stale할 수 있다.

즉 여기서 필요한 건 deep copy가 아니라 state naming과 source priority 정리다.

## `vcan`을 붙이면 해결되나

답은 "절반은 맞고, 절반은 아니다"였다.

`vcan` 자체는 그냥 가상 CAN 버스다. 그것만으로는 state가 자동으로 살아나지 않는다. 중요한 건 그 위에서 누가 feedback frame을 써주느냐다.

즉:

- `vcan`만 있으면 부족하다.
- `vcan + motor emulator`가 있어야 한다.

왜냐하면 현재 C++는 command를 보냈다는 사실보다, receive frame이 들어왔다는 사실을 기준으로 `jointAngle`를 갱신하기 때문이다.

그래서 real SIL 쪽으로 가려면 결국 아래가 필요하다.

- 가상 CAN transport
- 모터별 응답 frame 생성기
- feedback로 `jointAngle` 갱신
- measured와 estimated state의 우선순위 정리

## head 보정은 어디에 있나

head 쪽은 body CAN transform 테이블과 경로가 다르다.

- DXL head 모터 정의는 `sil/robot_spec.py`
- head logical joint 이름 매핑은 `sil/joint_map.py`
- `head_tilt`의 `-90deg` 보정은 `sil/command_applier.py`

즉 head는 `joint_map.py`만 보면 되는 게 아니고, 실제 semantic 보정은 `command_applier.py`까지 같이 봐야 한다.

## 왜 `k` 입력은 어떤 때는 먹고 어떤 때는 안 먹어 보였나

이 부분도 생각보다 단순한 "validator bug"는 아니었다.

핵심은 `TCP connected`와 `첫 state 수신`이 같은 사건이 아니라는 점이다.

Python brain 쪽은 소켓 연결만 되면 연결 성공으로 보고 수신 스레드를 띄운다. 하지만 첫 JSON state를 받을 때까지 기다리지는 않는다.

반면 C++는:

- brain 연결을 기다리고
- `initializePos("o")`까지 마친 다음에
- 그제서야 state broadcast thread를 시작한다

그래서 Python 입장에서는 이미 연결돼 있어도 아직 state를 한 번도 못 받았을 수 있다.

이 경우 planner는 Python의 기본 상태를 쓴다. 실제로 실패 사례의 값은 C++ 값이 아니라 Python 기본값과 정확히 일치했다.

여기에 한 가지가 더 겹친다.

`phil_brain`은 턴이 시작되면 state를 한 번 snapshot으로 복사하고, classifier, planner, validator가 그 snapshot만 끝까지 쓴다.

즉:

- snapshot 전에 `k` 반영이 오면 그 턴은 성공 가능
- snapshot 뒤에 `k`를 눌러도 그 턴은 계속 옛 상태를 본다
- LLM이 오래 걸리면 stale snapshot 문제는 더 잘 드러난다

그래서 "왜 어떤 때는 되고 어떤 때는 안 되지?"가 아니라, 더 정확히는 "그 턴이 state snapshot을 언제 떴는가?"의 문제에 가깝다.

## 내가 이해한 현재 한계사항

이번 세션 기준으로 당장 문서화해 둘 만한 한계는 아래와 같다.

- 현재 `drum_intheloop`는 frame-accurate SIL이 아니라 command-level simulator다.
- simulator가 움직였다고 해서 `current_angles`가 자동으로 맞아지지 않는다.
- `current_angles`는 measured feedback 계열이라 SIL 우회 상태에서는 stale하거나 garbage일 수 있다.
- disconnected motor의 angle state는 안전한 초기화가 없으면 이상값이 튈 수 있다.
- Python brain은 TCP 연결 이후 첫 state 수신을 보장하지 않는다.
- 한 턴에서 state snapshot은 한 번만 찍기 때문에, `k`나 최신 상태가 도중에 들어와도 그 턴 validator는 모를 수 있다.
- `isLockKeyRemoved` 같은 값은 스레드 간 동기화 관점에서도 더 안전하게 다룰 여지가 있다.

## 지금 단계에서의 현실적인 해석

현재 구조는 실패한 게 아니라, "무엇을 검증하는 simulator인지"가 분명해진 단계라고 보는 편이 더 맞다.

지금 `drum_intheloop`는:

- 제어기 출력이 어떤 자세를 만들지 빠르게 보는 데는 유용하다
- planner -> controller -> command export 경계를 검증하는 데도 유용하다
- 하지만 아직 feedback까지 닫힌 real SIL이라고 부르기엔 이르다

오히려 지금 단계에서 중요한 건 성급하게 모든 걸 SIL이라고 부르기보다, 현재 seam을 정확히 문서화하고 다음 단계인 `vcan` 또는 frame-level feedback 경로를 차근차근 붙여 나가는 것이다.

## 오늘의 한 줄 정리

지금의 `drum_intheloop`는 "로봇이 실제로 어떻게 느끼는가"를 재현하는 SIL이라기보다, "제어기가 어떤 자세를 만들려고 했는가"를 보여주는 command-level viewer에 가깝다.
