import matplotlib.pyplot as plt
import numpy as np

def plot_graph(T_values, y_function, y_label, title, highlight_T=None):
    """
    Plots a graph based on the given T values and a y_function.

    Parameters:
        T_values (numpy.ndarray): Array of T values.
        y_function (function): A function to calculate y values based on T.
        y_label (str): Label for the y-axis.
        title (str): Title of the graph.
        highlight_T (float or None): A specific T value to highlight on the graph (optional).
    """
    # Calculate y values using the provided function
    y_values = y_function(T_values)

    # Plot the graph
    plt.figure(figsize=(10, 6))
    plt.plot(T_values, y_values, label=f"{y_label} vs T", linewidth=2)
    
    # Highlight a specific T value if provided
    if highlight_T is not None:
        highlight_y = y_function(highlight_T)
        plt.scatter([highlight_T], [highlight_y], color='red', label=f"T = {highlight_T}, y = {highlight_y:.2f}", zorder=5)

    # Add labels and title
    plt.xlabel("T")
    plt.ylabel(y_label)
    plt.title(title)
    plt.axhline(0, color='gray', linewidth=0.8, linestyle='--')
    plt.axvline(0, color='gray', linewidth=0.8, linestyle='--')
    plt.legend()
    plt.grid(alpha=0.5)
    plt.show()

# Example of usage
if __name__ == "__main__":
    # Define a range of T values
    T = np.linspace(0, 1, 200)

    # Define a sample function for y
    def sample_function(T):
        return np.where(T <= 0.5, 0.7 * T, T - 0.15)


    # Call the plot_graph function
    plot_graph(T_values=T, 
               y_function=sample_function, 
               y_label="t_max", 
               title="Sample Graph", 
               highlight_T=0.5)
