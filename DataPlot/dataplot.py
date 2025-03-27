import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

class DraggableLine:
    def __init__(self, axes, x_init):
        self.axes = axes  # 모든 축을 리스트로 저장
        self.fig = axes[0].figure
        self.lines = [ax.axvline(x=x_init, color='magenta', linestyle='--', linewidth=2, label="Draggable Line") for ax in axes]
        
        self.press = None
        self.cid_press = self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.cid_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_motion = self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
    
    def on_press(self, event):
        if event.inaxes not in self.axes:
            return
        for line in self.lines:
            contains, _ = line.contains(event)
            if contains:
                self.press = event.xdata
                return

    def on_release(self, event):
        self.press = None
        self.fig.canvas.draw()

    def on_motion(self, event):
        if self.press is None or event.xdata is None:
            return
        dx = event.xdata - self.press
        new_x = self.lines[0].get_xdata()[0] + dx
        for line in self.lines:
            line.set_xdata([new_x, new_x])
        self.press = event.xdata
        self.fig.canvas.draw()

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
    if can_id == 4:
        receive_df = receive_df.copy()
        send_df = send_df.copy()
        receive_df['Pos'] *= -1
        send_df['Pos'] *= -1

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
    
def mode_2_plot(df):
    specified_ids = list(map(int, input("Enter CAN IDs to plot (comma-separated): ").split(',')))
    fig, axes = plt.subplots(len(specified_ids), 1, figsize=(12, 6 * len(specified_ids)), sharex=True)
    draggable_lines = []
    
    for idx, can_id in enumerate(specified_ids):
        can_id_df = df[df['CAN ID'] == can_id]
        receive_df = can_id_df[can_id_df['타입'] == 'Receive']
        send_df = can_id_df[can_id_df['타입'] == 'Send']

        ax = axes[idx]

        plot_pos_by_can_id(receive_df, send_df, can_id, ax)
        # plot_current_by_can_id(receive_df, can_id, ax)
        
        ax.set_ylabel(f'CAN ID {can_id}\nPosition')
        ax.legend()
        ax.grid(True)
        
        draggable_lines.append(DraggableLine(axes, x_init=receive_df['시간'].median()))
    
    axes[-1].set_xlabel('시간')
    
    plt.suptitle('Comparison of Actual and Desired Position for Specified CAN IDs', fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()


def main():
    file_path = '../DrumRobot_data/data1.txt'
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
            
            plot_pos_by_can_id(receive_df, send_df, can_id, ax1)
            ax1.set_ylabel('Position 값')
            ax1.set_title(f'CAN ID {can_id} - Actual Pos & Desire Pos')
            ax1.legend()
            ax1.grid(True)
            
            plot_current_by_can_id(receive_df, can_id, ax2)
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
        mode_2_plot(df)
        
    if mode == 3:
        can_ids = df['CAN ID'].unique()
        for can_id in can_ids:
            can_id_df = df[df['CAN ID'] == can_id]
            receive_df = can_id_df[can_id_df['타입'] == 'Receive']
            send_df = can_id_df[can_id_df['타입'] == 'Send']

            # Select the appropriate hit detection file based on CAN ID
            if can_id == 7:
                hit_file_path = '../DrumRobot_data/hittingDetectR.txt'
            elif can_id == 8:
                hit_file_path = '../DrumRobot_data/hittingDetectL.txt'
            else:
                continue  # Skip CAN IDs that are not 7 or 8
            
            hit_df = load_hit_txt(hit_file_path)
            
            # Plot Pos (Desired/Actual Position) with hit events overlay
            fig, ax = plt.subplots(figsize=(10, 6))
            plot_pos_by_can_id(receive_df, send_df, can_id, ax)
            
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
                    if x_min <= hit_time <= x_max:
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


if __name__ == '__main__':
    main()
