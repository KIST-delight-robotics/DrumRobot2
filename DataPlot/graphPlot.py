import numpy as np
import matplotlib.pyplot as plt

# Define t_contact and t_lift as functions of T
def t_contact(T):
    return min(0.1 * T, 0.05)  

def t_lift(T):
    return max(0.6 * T, T - 0.2)

def contact_angle(T):
    return -1 * min(T * 5 / 0.5, 5) 

def lift_angle(T):
    return np.where(T <= 0.5, -100 * (T - 0.5)**2 + 25, 25)

# Solve for wrist_q = 10 and plot against T
def calculate_wrist_q(T, t):
    # Define A and b matrices based on t_contact and t_lift
    A = np.array([
        [1, t_contact(T), t_contact(T)**2, t_contact(T)**3],
        [1, t_lift(T), t_lift(T)**2, t_lift(T)**3],
        [0, 1, 2 * t_contact(T), 3 * t_contact(T)**2],
        [0, 1, 2 * t_lift(T), 3 * t_lift(T)**2]
    ])
    b = np.array([contact_angle(T), lift_angle(T), 0, 0])
    
    # Solve for coefficients (sol)
    sol = np.linalg.solve(A, b)
    
    # Calculate wrist_q
    wrist_q = sol[0] + sol[1] * t + sol[2] * t**2 + sol[3] * t**3
    return wrist_q


# Define range of T and t
T_values = np.linspace(0.1, 1, 200)  # Range of T
t_values = np.linspace(0, 1, 200)  # Range of t


# Initialize results
results_T = []
results_t = []

# Find T and t such that wrist_q = 10
for T in T_values:
    for t in t_values:
        # if t > T / 2:  # Exclude t values greater than T / 2
        #     continue
        target_wrist_q = min(T / 0.5 * 10, 10)
        wrist_q = calculate_wrist_q(T, t)
        if np.isclose(wrist_q, target_wrist_q, atol=0.05):  # Check if wrist_q is close to target
            results_T.append(T)
            results_t.append(t)
            print(wrist_q, target_wrist_q)
            break  # Break once a t is found for the given T

# Fit a 1st-degree polynomial (line) to the data
coefficients = np.polyfit(results_T, results_t, 1)  # 1차 함수 피팅
m, c = coefficients  # 기울기와 절편

# Print the equation of the line
print(f"Fitted line equation: y = {m:.2f}x + {c:.2f}")

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(results_T, results_t, 'o', label="Data points", color="blue")  # Data points
plt.plot(results_T, m * np.array(results_T) + c, label=f"Fitted line: y = {m:.2f}x + {c:.2f}", color="red")  # Fitted line
plt.xlabel("T")
plt.ylabel("t")
plt.title("Graph of T and t with Fitted Line")
plt.grid(alpha=0.5)
plt.legend()
plt.show()
