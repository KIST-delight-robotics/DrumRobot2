import pandas as pd
import matplotlib.pyplot as plt

def load_txt(file_path):
    """
    데이터를 읽어서 시간, Desire Torque, Actual Torque, Actual Position을 가져오는 함수
    """
    df = pd.read_csv(file_path, header=None, names=['시간', 'Desire q[11]', 'Actual Torque', 'Actual Position'])
    return df

def plot_torque(df):
    """
    시간에 따른 Desire Torque와 Actual Torque를 색을 다르게 하여 플로팅하는 함수
    """
    plt.figure(figsize=(10, 6))
    plt.plot(df['시간'], df['Desire q[11]'], "o" ,label='Desire q[11]', color='red')
    #plt.plot(df['시간'], df['Actual Torque'], 'o', label='Actual Torque', color='blue')
    plt.xlabel('시간')
    plt.ylabel('Desire q[11]')
    #plt.title('Desire Torque vs Actual Torque Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    file_path = '../DrumRobot_data/maxonForTest_Q.txt'  # 로그 파일 경로
    df = load_txt(file_path)
    plot_torque(df)
    # df["Torque Error"] = df["Actual Torque"] - df["Desire Torque"]

    # print(f"평균 오차: {df['Torque Error'].mean():.6f} Nm")
    # print(f"표준 편차: {df['Torque Error'].std():.6f} Nm")



if __name__ == '__main__':
    main()
