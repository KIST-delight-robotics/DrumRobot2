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

def load_hit_txt(file_path):
    """
    hit_data.txt 파일 읽기: [시간, 타격 여부, 0, 0] 형식.
    타격 여부가 1이면 타격 감지.
    """
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            split_line = line.strip().split(',')
            time = float(split_line[0])
            hit = int(split_line[1])
            data.append([time, hit])
    return pd.DataFrame(data, columns=['시간', '타격'])

def plot_pos_by_can_id(receive_df, send_df, can_id, ax):
    """
    CAN ID별 Actual/Desire Position 그래프 그리기 함수
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
    file_path = '../DrumRobot_data/data5.txt'
    df = load_txt(file_path)

    print("Choose mode:")
    print("0: Plot all CAN IDs individually")
    print("1: Plot Actual Pos/Desire Pos and Current (with rolling trend lines) together for each CAN ID")
    print("2: Plot specified CAN IDs together in separate rows")
    print("3: Plot graphs like mode 0 but overlay hit events on the Position graph")
    mode = int(input("Enter mode (0, 1, 2, or 3): "))

    if mode == 0:
        # Mode 0: 각 CAN ID별로 Pos, Error, Current를 각각 개별 그래프로 그림.
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
        # Mode 1: 각 CAN ID별로 상단에 Pos, 하단에 Current와 rolling trend line을 그린다.
        can_ids = df['CAN ID'].unique()
        for can_id in can_ids:
            can_id_df = df[df['CAN ID'] == can_id]
            receive_df = can_id_df[can_id_df['타입'] == 'Receive']
            send_df = can_id_df[can_id_df['타입'] == 'Send']
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
            # 상단 subplot: Pos
            plot_pos_by_can_id(receive_df, send_df, can_id, ax1)
            ax1.set_ylabel('Position 값')
            ax1.set_title(f'CAN ID {can_id} - Actual Pos & Desire Pos')
            ax1.legend()
            ax1.grid(True)
            # 하단 subplot: Current + rolling trend line (이전 5개 점 사용)
            plot_current_by_can_id(receive_df, can_id, ax2)
            sorted_df = receive_df.sort_values(by='시간')
            x = sorted_df['시간'].values
            y = sorted_df['Current_or_Error'].values
            window_size = 5
            slopes = []
            for i in range(window_size, len(x)):
                window_x = x[i-window_size:i]
                window_y = y[i-window_size:i]
                coefficients = np.polyfit(window_x, window_y, 1)
                m = coefficients[0]
                slopes.append(m)
                trend_poly = np.poly1d(coefficients)
                x_fit = np.linspace(window_x.min(), window_x.max(), 10)
                if i == window_size:
                    ax2.plot(x_fit, trend_poly(x_fit), "k-", label="Trend (prev 5 points)", alpha=0.7)
                else:
                    ax2.plot(x_fit, trend_poly(x_fit), "k-", alpha=0.7)
            # 피크 구간 표시 (직전 점이 40 초과 + 기울기 양수→음수 전환)
            marked = False
            for i in range(window_size+1, len(x)):
                if slopes[i-1-window_size] > 0 and slopes[i-window_size] < 0 and y[i-1] > 40:
                    region_start = (x[i-1] + x[i]) / 2
                    region_end = (x[i] + x[i+1]) / 2 if i < len(x)-1 else x[i]
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
        # Mode 2: 지정한 CAN ID들만 따로 그래프로 그림 (mode 0 스타일)
        specified_ids = list(map(int, input("Enter CAN IDs to plot (comma-separated): ").split(',')))
        fig, axes = plt.subplots(len(specified_ids), 1, figsize=(12, 8 * len(specified_ids)), sharex=True)
        for idx, can_id in enumerate(specified_ids):
            can_id_df = df[df['CAN ID'] == can_id]
            receive_df = can_id_df[can_id_df['타입'] == 'Receive']
            send_df = can_id_df[can_id_df['타입'] == 'Send']
            plot_pos_by_can_id(receive_df, send_df, can_id, axes[idx])
            plot_current_by_can_id(receive_df, can_id, axes[idx])
            axes[idx].set_ylabel(f'CAN ID {can_id}\nPosition & Current 값')
            axes[idx].legend()
            axes[idx].grid(True)
        axes[-1].set_xlabel('시간')
        plt.suptitle('Comparison of Actual Pos, Desire Pos, and Current for Specified CAN IDs', fontsize=16)
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()
        
    elif mode == 3:
        # Mode 3: Mode 0 스타일로 개별 그래프를 그리되,
        # Desired/Actual Position 그래프에 추가로 hit_data.txt 파일을 읽어 타격 이벤트(타격 여부==1)를 표시한다.
        hit_file_path = '../DrumRobot_data/hittingDetect.txt'
        hit_df = load_hit_txt(hit_file_path)
        can_ids = df['CAN ID'].unique()
        for can_id in can_ids:
            can_id_df = df[df['CAN ID'] == can_id]
            receive_df = can_id_df[can_id_df['타입'] == 'Receive']
            send_df = can_id_df[can_id_df['타입'] == 'Send']
            
            # Plot Pos (Desired/Actual Position) with hit events overlay
            fig, ax = plt.subplots(figsize=(10, 6))
            plot_pos_by_can_id(receive_df, send_df, can_id, ax)
            # x 범위: receive_df의 시간 값으로 결정
            sorted_df = receive_df.sort_values(by='시간')
            if not sorted_df.empty:
                x_vals = sorted_df['시간'].values
                x_min, x_max = x_vals[0], x_vals[-1]
            else:
                x_min, x_max = 0, 0
            hit_label_added = False
            for row in hit_df.itertuples():
                if row.타격 == 1:
                    hit_time = row.시간
                    if hit_time >= x_min and hit_time <= x_max:
                        if not hit_label_added:
                            ax.axvline(x=hit_time, color='magenta', linestyle='--', linewidth=2, label="Hit")
                            hit_label_added = True
                        else:
                            ax.axvline(x=hit_time, color='magenta', linestyle='--', linewidth=2)
            ax.set_xlabel('시간')
            ax.set_ylabel('Position 값')
            ax.set_title(f'CAN ID {can_id} - Actual Pos, Desire Pos with Hit Events')
            ax.legend()
            ax.grid(True)
            plt.show()
            
            # Plot Error (Send 데이터) - Mode 0 스타일
            fig, ax = plt.subplots(figsize=(10, 6))
            plot_error_by_can_id(send_df, can_id, ax)
            plt.xlabel('시간')
            plt.ylabel('Error 값')
            plt.title(f'CAN ID {can_id} - Error (Send)')
            plt.legend()
            plt.grid(True)
            plt.show()
            
            # Plot Current (Receive 데이터) - Mode 0 스타일
            fig, ax = plt.subplots(figsize=(10, 6))
            plot_current_by_can_id(receive_df, can_id, ax)
            plt.xlabel('시간')
            plt.ylabel('Current 값')
            plt.title(f'CAN ID {can_id} - Current (Receive)')
            plt.legend()
            plt.grid(True)
            plt.show()

if __name__ == '__main__':
    main()
