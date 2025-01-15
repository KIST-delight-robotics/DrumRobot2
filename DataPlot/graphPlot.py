import numpy as np
import matplotlib.pyplot as plt

def make_wrist_angle(t1, t2, t, state, param, intensity):
    t_contact = param['wristContactTime']
    t_lift = param['wristLiftTime']
    t_stay = param['wristStayTime']
    t_release = param['wristReleaseTime']
    t_hit = t2 - t1

    intensity_factor = 0.4 * intensity + 0.2
    wrist_lift_angle = param['wristLiftAngle'] * intensity_factor

    wrist_q = 0.0

    if state == 0:
        wrist_q = param['wristStayAngle']

    elif state == 1:
        if t < t_contact:
            A = np.array([[1, 0, 0],
                          [1, t_contact, t_contact**2],
                          [0, 1, 2*t_contact]])
            b = np.array([0, param['wristContactAngle'], 0])
            sol = np.linalg.solve(A, b)
            wrist_q = sol[0] + sol[1]*t + sol[2]*t**2

        elif t <= t_release:
            A = np.array([[1, t_contact, t_contact**2, t_contact**3],
                          [1, t_release, t_release**2, t_release**3],
                          [0, 1, 2*t_contact, 3*t_contact**2],
                          [0, 1, 2*t_release, 3*t_release**2]])
            b = np.array([param['wristContactAngle'], param['wristStayAngle'], 0, 0])
            sol = np.linalg.solve(A, b)
            wrist_q = sol[0] + sol[1]*t + sol[2]*t**2 + sol[3]*t**3

        else:
            wrist_q = param['wristStayAngle']

    elif state == 2:
        if t < t_stay:
            wrist_q = param['wristStayAngle']

        elif t < t_lift:
            A = np.array([[1, t_stay, t_stay**2, t_stay**3],
                          [1, t_lift, t_lift**2, t_lift**3],
                          [0, 1, 2*t_stay, 3*t_stay**2],
                          [0, 1, 2*t_lift, 3*t_lift**2]])
            b = np.array([param['wristStayAngle'], wrist_lift_angle, 0, 0])
            sol = np.linalg.solve(A, b)
            wrist_q = sol[0] + sol[1]*t + sol[2]*t**2 + sol[3]*t**3

        elif t <= t_hit:
            A = np.array([[1, t_lift, t_lift**2],
                          [1, t_hit, t_hit**2],
                          [0, 1, 2*t_lift]])
            b = np.array([wrist_lift_angle, 0, 0])
            sol = np.linalg.solve(A, b)
            wrist_q = sol[0] + sol[1]*t + sol[2]*t**2

        else:
            wrist_q = 0.0

    elif state == 3:
        if t < t_contact:
            A = np.array([[1, 0, 0],
                          [1, t_contact, t_contact**2],
                          [0, 1, 2*t_contact]])
            b = np.array([0, param['wristContactAngle'], 0])
            sol = np.linalg.solve(A, b)
            wrist_q = sol[0] + sol[1]*t + sol[2]*t**2

        elif t < t_release:
            A = np.array([[1, t_contact, t_contact**2, t_contact**3],
                          [1, t_release, t_release**2, t_release**3],
                          [0, 1, 2*t_contact, 3*t_contact**2],
                          [0, 1, 2*t_release, 3*t_release**2]])
            b = np.array([param['wristContactAngle'], param['wristStayAngle'], 0, 0])
            sol = np.linalg.solve(A, b)
            wrist_q = sol[0] + sol[1]*t + sol[2]*t**2 + sol[3]*t**3

        elif t < t_stay:
            wrist_q = param['wristStayAngle']

        elif t < t_lift:
            A = np.array([[1, t_stay, t_stay**2, t_stay**3],
                            [1, t_lift, t_lift**2, t_lift**3],
                            [0, 1, 2*t_stay, 3*t_stay**2],
                            [0, 1, 2*t_lift, 3*t_lift**2]])
            b = np.array([param['wristStayAngle'], wrist_lift_angle, 0, 0])
            sol = np.linalg.solve(A, b)
            wrist_q = sol[0] + sol[1]*t + sol[2]*t**2 + sol[3]*t**3

        elif t <= t_hit:
            A = np.array([[1, t_lift, t_lift**2],
                          [1, t_hit, t_hit**2],
                          [0, 1, 2*t_lift]])
            b = np.array([wrist_lift_angle, 0, 0])
            sol = np.linalg.solve(A, b)
            wrist_q = sol[0] + sol[1]*t + sol[2]*t**2

        else:
            wrist_q = 0.0

    return wrist_q

# Parameters for testing
param = {
    'wristContactTime': 0.1,
    'wristLiftTime': 0.8,
    'wristStayTime': 0.6,
    'wristReleaseTime': 0.2,
    'wristContactAngle': -5,
    'wristStayAngle': 10,
    'wristLiftAngle': 25
}

# Generate the plot
t1 = 0.0
t2 = 1.0
t_values = np.linspace(t1, t2, 100)
state = 3
intensity = 2

angles = [make_wrist_angle(t1, t2, t, state, param, intensity) for t in t_values]

plt.plot(t_values, angles, label=f"State {state}, Intensity {intensity}")
plt.xlabel("Time (s)")
plt.ylabel("Wrist Angle (degrees)")
plt.title("Wrist Angle vs Time")
plt.legend()
plt.grid()
plt.show()
