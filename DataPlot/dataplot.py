import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def load_txt(file_path):
    """
    데이터를 읽어서 CAN ID가 1자리이면 Receive, 3자리이면 Send로 분류하는 함수
    """
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            split_line = line.strip().split(',')
            time, can_id, pos, current_or_error = float(split_line[0]), int(split_line[1]), float(split_line[2]), float(split_line[3])

            if can_id < 100:
                data.append([time, can_id, pos, current_or_error, 'Receive'])
            else:
                data.append([time, can_id - 100, pos, current_or_error, 'Send'])

    df = pd.DataFrame(data, columns=['시간', 'CAN ID', 'Pos', 'Current_or_Error', '타입'])
    df = df[~((df['타입'] == 'Receive') & (df['Pos'] == 99))]  # Pos 99 제거
    return df

def plot_pos_by_can_id(receive_df, send_df, can_id, ax):
    """
    CAN ID별 Pos 그래프 그리기 함수
    """
    ax.plot(receive_df['시간'], receive_df['Pos'], 
            label=f'Actual Pos (Receive) - ID {can_id}', 
            color='blue', marker='o', markersize=3, linestyle='None')
    ax.plot(send_df['시간'], send_df['Pos'], 
            label=f'Desire Pos (Send) - ID {can_id}', 
            color='red', marker='o', markersize=3, linestyle='None')

def plot_error_by_can_id(send_df, can_id, ax):
    """
    CAN ID별 Error 그래프 그리기 함수 (Send 데이터)
    """
    ax.plot(send_df['시간'], send_df['Current_or_Error'], 
            label=f'Error (Send) - ID {can_id}', 
            color='purple', marker='o', markersize=3, linestyle='None')

def plot_current_by_can_id(receive_df, can_id, ax):
    """
    CAN ID별 Current 그래프 그리기 함수 (Receive 데이터)
    """
    ax.plot(receive_df['시간'], receive_df['Current_or_Error'], 
            label=f'Current (Receive) - ID {can_id}', 
            color='green', marker='o', markersize=3, linestyle='None')

def main():
    file_path = '../DrumRobot_data/data25.txt'
    df = load_txt(file_path)

    print("Choose mode:")
    print("0: Plot all CAN IDs individually")
    print("1: Plot Actual Pos/Desire Pos and Current (with rolling trend lines using previous 5 points) together for each CAN ID")
    print("2: Plot specified CAN IDs together in separate rows")
    mode = int(input("Enter mode (0, 1, or 2): "))

    if mode == 0:
        can_ids = df['CAN ID'].unique()
        for can_id in can_ids:
            can_id_df = df[df['CAN ID'] == can_id]
            receive_df = can_id_df[can_id_df['타입'] == 'Receive']
            send_df = can_id_df[can_id_df['타입'] == 'Send']

            # Plot Pos
            fig, ax = plt.subplots(figsize=(10, 6))
            plot_pos_by_can_id(receive_df, send_df, can_id, ax)
            plt.xlabel('시간')
            plt.ylabel('Position 값')
            plt.title(f'CAN ID {can_id} - Actual Pos, Desire Pos')
            plt.legend()
            plt.grid(True)
            plt.show()

            # Plot Error (Send 데이터)
            fig, ax = plt.subplots(figsize=(10, 6))
            plot_error_by_can_id(send_df, can_id, ax)
            plt.xlabel('시간')
            plt.ylabel('Error 값')
            plt.title(f'CAN ID {can_id} - Error (Send)')
            plt.legend()
            plt.grid(True)
            plt.show()

            # Plot Current (Receive 데이터)
            fig, ax = plt.subplots(figsize=(10, 6))
            plot_current_by_can_id(receive_df, can_id, ax)
            plt.xlabel('시간')
            plt.ylabel('Current 값')
            plt.title(f'CAN ID {can_id} - Current (Receive)')
            plt.legend()
            plt.grid(True)
            plt.show()

    elif mode == 1:
        # mode 1: 각 CAN ID별로 Actual Pos/Desire Pos와 Current를 같은 시간축의 두 서브플롯으로 그림
        # 아래에서는 Current 그래프에 대해,
        # 1) 각 데이터 포인트에 대해 '바로 이전 5개의 점'으로 선형 회귀를 수행하여 그 추세선을 그림.
        # 2) 그리고 current 값이 40을 초과하는 (판단 기준은 현재 보는 점의 직전 점) 상황에서 회귀 기울기가 양수에서 음수로 바뀌는 구간(증가에서 감소로 전환)을 표시함.
        can_ids = df['CAN ID'].unique()
        for can_id in can_ids:
            can_id_df = df[df['CAN ID'] == can_id]
            receive_df = can_id_df[can_id_df['타입'] == 'Receive']
            send_df = can_id_df[can_id_df['타입'] == 'Send']

            # 하나의 figure에 2개의 서브플롯 생성 (상단: Pos, 하단: Current; x축 공유)
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
            
            # 상단: Actual Pos 및 Desire Pos
            plot_pos_by_can_id(receive_df, send_df, can_id, ax1)
            ax1.set_ylabel('Position 값')
            ax1.set_title(f'CAN ID {can_id} - Actual Pos & Desire Pos')
            ax1.legend()
            ax1.grid(True)

            # 하단: Current 데이터 표시
            plot_current_by_can_id(receive_df, can_id, ax2)
            
            # 시간순으로 정렬된 데이터를 이용하여 current에 대한 x, y 배열 생성
            sorted_df = receive_df.sort_values(by='시간')
            x = sorted_df['시간'].values
            y = sorted_df['Current_or_Error'].values

            window_size = 5  # 이전 5개 점 사용
            slopes = []  # 각 회귀의 기울기를 저장 (i: 인덱스, i >= window_size)
            # --- 1) 각 데이터 포인트에 대해, 바로 이전 5개의 점을 이용한 회귀 추세선 그리기 ---
            for i in range(window_size, len(x)):
                window_x = x[i-window_size:i]   # 바로 이전 5개 점
                window_y = y[i-window_size:i]
                coefficients = np.polyfit(window_x, window_y, 1)  # 1차 회귀
                m = coefficients[0]
                slopes.append(m)
                trend_poly = np.poly1d(coefficients)
                # 해당 window의 x 범위 내에서 부드러운 선 생성
                x_fit = np.linspace(window_x.min(), window_x.max(), 10)
                if i == window_size:
                    ax2.plot(x_fit, trend_poly(x_fit), "k-", label="Trend (prev 5 points)", alpha=0.7)
                else:
                    ax2.plot(x_fit, trend_poly(x_fit), "k-", alpha=0.7)
            
            # --- 2) current 값이 40을 초과하는지를 직전 점(y[i-1])로 판단하면서,
            #      회귀 기울기가 양수에서 음수로 바뀌는 구간 표시 ---
            marked = False  # 최초 한 번만 범례(label)을 추가하기 위한 변수
            for i in range(window_size+1, len(x)):
                # 이전 회귀의 기울기: slopes[i-1-window_size]
                # 현재 회귀의 기울기: slopes[i-window_size]
                if slopes[i-1-window_size] > 0 and slopes[i-window_size] < 0 and y[i-1] > 40:
                    region_start = (x[i-1] + x[i]) / 2
                    if i < len(x)-1:
                        region_end = (x[i] + x[i+1]) / 2
                    else:
                        region_end = x[i]
                    if not marked:
                        ax2.axvspan(region_start, region_end, color='orange', alpha=0.3, label="Peak Region")
                        marked = True
                    else:
                        ax2.axvspan(region_start, region_end, color='orange', alpha=0.3)
            
            ax2.set_xlabel('시간')
            ax2.set_ylabel('Current 값')
            ax2.set_title(f'CAN ID {can_id} - Current')
            ax2.legend()
            ax2.grid(True)

            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plt.suptitle(f'CAN ID {can_id} - Combined Plot', fontsize=16)
            plt.show()

    elif mode == 2:
        specified_ids = list(map(int, input("Enter CAN IDs to plot (comma-separated): ").split(',')))

        fig, axes = plt.subplots(len(specified_ids), 1, figsize=(12, 8 * len(specified_ids)), sharex=True)

        for idx, can_id in enumerate(specified_ids):
            can_id_df = df[df['CAN ID'] == can_id]
            receive_df = can_id_df[can_id_df['타입'] == 'Receive']
            send_df = can_id_df[can_id_df['타입'] == 'Send']

            # Plot Pos와 Current 데이터를 한 서브플롯에 같이 그림
            plot_pos_by_can_id(receive_df, send_df, can_id, axes[idx])
            plot_current_by_can_id(receive_df, can_id, axes[idx])  # Current 추가
            axes[idx].set_ylabel(f'CAN ID {can_id}\nPosition & Current 값')
            axes[idx].legend()
            axes[idx].grid(True)

        axes[-1].set_xlabel('시간')
        plt.suptitle('Comparison of Actual Pos, Desire Pos, and Current for Specified CAN IDs', fontsize=16)
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()

if __name__ == '__main__':
    main()
