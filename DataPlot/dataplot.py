import pandas as pd
import matplotlib.pyplot as plt

# 데이터를 읽어서 CAN ID가 1자리이면 Receive, 3자리이면 Send로 분류하는 함수
def load_txt(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            # 줄을 쉼표로 구분하여 데이터를 리스트로 변환
            split_line = line.strip().split(',')
            # CAN ID는 두 번째 열에 있음
            time, can_id, pos, current_or_error = float(split_line[0]), int(split_line[1]), float(split_line[2]), float(split_line[3])

            # CAN ID가 1자리이면 Receive, 3자리
            # 이면 Send 신호로 분류
            if can_id < 100:
                data.append([time, can_id, pos, current_or_error, 'Receive'])
            else:
                data.append([time, can_id - 100, pos, current_or_error, 'Send'])  # CAN ID를 맞추기 위해 100을 뺌
    
    # 데이터를 DataFrame으로 변환
    df = pd.DataFrame(data, columns=['시간', 'CAN ID', 'Pos', 'Current_or_Error', '타입'])
    
    # Receive 데이터에서 'Pos' 값이 99인 행은 제외
    df = df[~((df['타입'] == 'Receive') & (df['Pos'] == 99))]

    return df

# 클릭 이벤트 처리 함수
def on_click(event, data):
    if event.inaxes:
        x_click = event.xdata
        y_click = event.ydata
        # 클릭한 좌표와 가장 가까운 점 찾기
        distances = ((data['시간'] - x_click) ** 2 + (data['Pos'] - y_click) ** 2)
        min_index = distances.idxmin()
        x_val = data['시간'].iloc[min_index]
        y_val = data['Pos'].iloc[min_index]
        print(f"Clicked on: x={x_val}, y={y_val}")

# CAN ID별 Pos 그래프 그리기 함수 (Actual Pos, Desire Pos)
def plot_pos_by_can_id(receive_df, send_df, can_id):
    fig, ax = plt.subplots(figsize=(10, 6))

    # Actual Pos (Receive, 파란색) - 점만 표시
    ax.plot(receive_df['시간'], receive_df['Pos'], label='Actual Pos (Receive)', color='blue', marker='o', markersize=3, linestyle='None')

    # Desire Pos (Send, 빨간색) - 점만 표시
    ax.plot(send_df['시간'], send_df['Pos'], label='Desire Pos (Send)', color='red', marker='o', markersize=3, linestyle='None')

    plt.xlabel('시간')
    plt.ylabel('Position 값')
    plt.title(f'CAN ID {can_id} - Actual Pos, Desire Pos')
    plt.legend()
    plt.grid(True)

    # 클릭 이벤트 연결
    fig.canvas.mpl_connect('button_press_event', lambda event: on_click(event, receive_df.append(send_df).reset_index()))

    plt.show()

# CAN ID별 Error 그래프 그리기 함수 (점만 표시)
def plot_error_by_can_id(send_df, can_id):
    fig, ax = plt.subplots(figsize=(10, 6))

    # Error (Send, 보라색) - 점만 표시
    ax.plot(send_df['시간'], send_df['Current_or_Error'], label='Error (Send)', color='purple', marker='o', markersize=3, linestyle='None')

    plt.xlabel('시간')
    plt.ylabel('Error 값')
    plt.title(f'CAN ID {can_id} - Error (Send)')
    plt.legend()
    plt.grid(True)

    # 클릭 이벤트 연결
    fig.canvas.mpl_connect('button_press_event', lambda event: on_click(event, send_df.reset_index()))

    plt.show()

# 메인 함수
def main():
    # txt 파일 경로
    file_path = '../DrumRobot_data/data1.txt'  # txt 파일 경로

    # 데이터 로드
    df = load_txt(file_path)

    # CAN ID별로 그래프 그리기
    for can_id in df['CAN ID'].unique():
        # CAN ID별로 데이터 분리
        can_id_df = df[df['CAN ID'] == can_id]
        receive_df = can_id_df[can_id_df['타입'] == 'Receive']
        send_df = can_id_df[can_id_df['타입'] == 'Send']

        # Pos 그래프 (Actual Pos, Desire Pos) - 점만 표시
        plot_pos_by_can_id(receive_df, send_df, can_id)

        # Error 그래프 (Send의 Error만 그리기) - 점만 표시
        plot_error_by_can_id(send_df, can_id)

if __name__ == '__main__':
    main()
