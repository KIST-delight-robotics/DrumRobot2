import numpy as np
import matplotlib.pyplot as plt

# 전체 시간 설정
T = 0.6  # 예: 전체 시간 10초
num_points = 1000  # 그래프 해상도

# 시간 벡터
t = np.linspace(0, T, num_points)

# 구간 나누기 (예: T = 10일 때 0~3.3, 3.3~6.6, 6.6~10)
t_release = min(0.2 * T, 0.1)
t_stay = 0.45 * T
t_lift = max(0.6 * T, T - 0.2)
t_hit = T

# 각도 함수 정의
def angle_release(t):
    return 

def angle_stay(t):
    return 

def angle_lift(t):
    return 

def angle_hit(t):
    return 

# 구간별 각도 계산
theta = np.piecewise(
    t,
    [t < t_release, (t >= t1) & (t < t2), t >= t2],
    [angle_func1, angle_func2, angle_func3]
)

# 그래프 그리기
plt.figure(figsize=(10, 5))
plt.plot(t, theta, label='Angle over Time')
plt.axvline(t1, color='gray', linestyle='--', label='Section 1 End')
plt.axvline(t2, color='gray', linestyle='--', label='Section 2 End')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.title('Piecewise Angle Function over Time')
plt.grid(True)
plt.legend()
plt.show()
