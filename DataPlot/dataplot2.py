import pandas as pd
import matplotlib.pyplot as plt

def plot_from_txt(file1, file2=None):
    """
    Plots data from one or two CSV-like txt files.
    
    Parameters:
        file1 (str): Path to the first txt file.
        file2 (str, optional): Path to the second txt file (default: None).
    """
    # Function to load data from a txt file
    def load_data(file):
        # Read the data
        data = pd.read_csv(file, header=None, names=["Time", "CAN_ID", "Angle", "Extra"])
        return data

    # Load the first file
    data1 = load_data(file1)

    # Plot the first file
    plt.figure(figsize=(12, 6))
    plt.plot(data1["Time"], data1["Angle"], 'o', label=f"File 1: {file1}", linewidth=1.5)

    # If the second file is provided, load and plot it
    if file2:
        data2 = load_data(file2)
        plt.plot(data2["Time"], data2["Angle"], 'o', label=f"File 2: {file2}", linewidth=1.5)

    # Customize the plot
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (radians)")
    plt.title("Angle vs Time")
    plt.grid(alpha=0.5)
    plt.legend(loc="upper center", bbox_to_anchor=(0.5, 1.15), ncol=2)  # Move legend above the plot
    plt.tight_layout()
    plt.show()


# Example usage
if __name__ == "__main__":
    # Path to txt files
    file1_path = '../DrumRobot_data/wristAngleData.txt'  # Replace with your file path
    file2_path = '../DrumRobot_data/solveIK_q6.txt'  # Replace with your file path (or None for single plot)

    # Call the function with one or two files
    plot_from_txt(file1_path, None)
