import pandas as pd
import matplotlib.pyplot as plt

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

def on_click(event, data):
    """
    클릭 이벤트 처리 함수
    """
    if event.inaxes:
        x_click = event.xdata
        y_click = event.ydata
        distances = ((data['시간'] - x_click) ** 2 + (data['Pos'] - y_click) ** 2)
        min_index = distances.idxmin()
        x_val = data['시간'].iloc[min_index]
        y_val = data['Pos'].iloc[min_index]
        print(f"Clicked on: x={x_val}, y={y_val}")

def plot_pos_by_can_id(receive_df, send_df, can_id, ax):
    """
    CAN ID별 Pos 그래프 그리기 함수
    """
    ax.plot(receive_df['시간'], receive_df['Pos'], label=f'Actual Pos (Receive) - ID {can_id}', color='blue', marker='o', markersize=3, linestyle='None')
    ax.plot(send_df['시간'], send_df['Pos'], label=f'Desire Pos (Send) - ID {can_id}', color='red', marker='o', markersize=3, linestyle='None')

def plot_error_by_can_id(send_df, can_id, ax):
    """
    CAN ID별 Error 그래프 그리기 함수
    """
    ax.plot(send_df['시간'], send_df['Current_or_Error'], label=f'Error (Send) - ID {can_id}', color='purple', marker='o', markersize=3, linestyle='None')

def main():
    file_path = '../DrumRobot_data/data5.txt'
    df = load_txt(file_path)

    print("Choose mode:")
    print("0: Plot all CAN IDs individually")
    print("2: Plot specified CAN IDs together in separate rows")
    mode = int(input("Enter mode (0 or 2): "))

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

            # Plot Error
            fig, ax = plt.subplots(figsize=(10, 6))
            plot_error_by_can_id(send_df, can_id, ax)
            plt.xlabel('시간')
            plt.ylabel('Error 값')
            plt.title(f'CAN ID {can_id} - Error (Send)')
            plt.legend()
            plt.grid(True)
            plt.show()

    elif mode == 2:
        specified_ids = list(map(int, input("Enter CAN IDs to plot (comma-separated): ").split(',')))

        fig, axes = plt.subplots(len(specified_ids), 1, figsize=(12, 8 * len(specified_ids)), sharex=True)

        for idx, can_id in enumerate(specified_ids):
            can_id_df = df[df['CAN ID'] == can_id]
            receive_df = can_id_df[can_id_df['타입'] == 'Receive']
            send_df = can_id_df[can_id_df['타입'] == 'Send']

            # Plot Pos for each specified CAN ID in a separate row
            plot_pos_by_can_id(receive_df, send_df, can_id, axes[idx])
            axes[idx].set_ylabel(f'CAN ID {can_id}\nPosition 값')
            axes[idx].legend()
            axes[idx].grid(True)

        axes[-1].set_xlabel('시간')
        plt.suptitle('Comparison of Actual Pos and Desire Pos for Specified CAN IDs', fontsize=16)
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()

if __name__ == '__main__':
    main()
