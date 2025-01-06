import numpy as np
import matplotlib.pyplot as plt

# Define t_contact and t_lift as functions of T
def t_contact(T):
    return min(0.1 * T, 0.05)

def t_lift(T):
    return max(0.6 * T, T - 0.2)

def contact_angle(T):
    return -1.0 * min(T * 5 / 0.5, 5)

def stay_angle(T):
    return min(T * 10 / 0.5, 10)

def lift_angle(T):
    return np.where(T <= 0.5, -100 * (T - 0.5)**2 + 25, 25)

# Solve for wrist_q
def calculate_wrist_q(T, t):
    A = np.array([
        [1, t_contact(T), t_contact(T)**2, t_contact(T)**3],
        [1, t_lift(T), t_lift(T)**2, t_lift(T)**3],
        [0, 1, 2 * t_contact(T), 3 * t_contact(T)**2],
        [0, 1, 2 * t_lift(T), 3 * t_lift(T)**2]
    ])
    b = np.array([contact_angle(T), lift_angle(T), 0, 0])
    sol = np.linalg.solve(A, b)
    wrist_q = sol[0] + sol[1] * t + sol[2] * t**2 + sol[3] * t**3
    return wrist_q, sol

# Derivative of wrist_q
def wrist_q_derivative(sol, t):
    return sol[1] + 2 * sol[2] * t + 3 * sol[3] * t**2

# Find t and slope where wrist_q matches stay_angle
def find_t_and_slope(T):
    t_values = np.linspace(0, 1, 200)
    for t in t_values:
        if stop_flag == 1:
            break
        wrist_q, sol = calculate_wrist_q(T, t)
        stay_q = stay_angle(T)
        if np.isclose(wrist_q, stay_q, atol=0.05):
            stop_flag = 1
            slope = wrist_q_derivative(sol, t)
            return t, slope 
    return None, None

# Define cubic function with zero slope at boundaries
def cubic_function(T, T_start, T_end, y_start, y_end, slope_target):
    delta_T = T_end - T_start
    a = 2 * (y_start - y_end) / (delta_T**3)
    b = 3 * (y_end - y_start) / (delta_T**2)
    c = slope_target / delta_T
    return a * (T - T_start)**3 + b * (T - T_start)**2 + c * (T - T_start) + y_start

# Generate and plot the cubic function
def plot_cubic_function():
    T_values = np.linspace(0.1, 1, 200)  # Divide T into 200 parts
    t_matches = []
    slopes_at_t = []

    # Find t_match and slope for each T
    for T in T_values:
        t_match, slope_at_t = find_t_and_slope(T)
        if t_match is not None:
            t_matches.append(T)
            slopes_at_t.append(slope_at_t)

    # If no valid T found
    if not t_matches:
        print("No valid T values found where wrist_q matches stay_angle.")
        return

    # Define cubic function parameters
    T_start = t_matches[0]
    T_end = t_matches[-1]
    y_start = slopes_at_t[0]
    y_end = slopes_at_t[-1]
    slope_target = slopes_at_t[len(slopes_at_t) // 2]  # Example slope in the middle

    # Generate cubic values
    cubic_values = cubic_function(T_values, T_start, T_end, y_start, y_end, slope_target)

    # Plot results
    plt.figure(figsize=(10, 6))
    plt.plot(T_values, cubic_values, label="Cubic Function", color="green", linewidth=2)
    plt.scatter(t_matches, slopes_at_t, color="red", label="t_match points", zorder=5)
    plt.axvline(T_start, color="gray", linestyle="--", label=f"T_start = {T_start:.2f}")
    plt.axvline(T_end, color="red", linestyle="--", label=f"T_end = {T_end:.2f}")
    plt.axhline(y_start, color="blue", linestyle="--", label=f"y_start = {y_start:.2f}")
    plt.axhline(y_end, color="orange", linestyle="--", label=f"y_end = {y_end:.2f}")
    plt.xlabel("T")
    plt.ylabel("Angle (degrees)")
    plt.title("Cubic Function with Matching Slope")
    plt.legend()
    plt.grid(alpha=0.5)
    plt.show()

    # Return t_match values
    return t_matches

# Run the plotting function and return t_match values
t_matches = plot_cubic_function()
print("t_matches:", t_matches)
