import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import scipy.io.wavfile as wav
import json
from datetime import datetime, timedelta
from scipy.signal import argrelextrema

# === 파일 경로 ===
txt_file = "../DrumRobot_data/wristTime.txt"
wav_file = "/home/shy/DrumRobot/DataPlot/recorded.wav"
json_file = "/home/shy/DrumRobot/DataPlot/recorded_info.json"

# === 1. Load txt data ===
df = pd.read_csv(txt_file, header=None, names=["Time", "CAN_ID", "Target", "Reference"])
df["Time"] = pd.to_datetime(df["Time"], format="%H:%M:%S.%f")

# === 2. Load wav + json metadata ===
sample_rate, audio = wav.read(wav_file)
audio = audio.flatten()

with open(json_file, "r") as f:
    info = json.load(f)

# 수정: 시간만 있는 경우
wav_start = datetime.strptime(info["start_time"], "%H:%M:%S.%f")
seconds_per_sample = 1.0 / sample_rate
wav_time_axis = [wav_start + timedelta(seconds=i * seconds_per_sample) for i in range(len(audio))]

# === Plot function for CAN ID and waveform ===
def plot_canid_and_waveform(can_id, color):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    data = df[df["CAN_ID"] == can_id].reset_index(drop=True)
    targets = data["Target"].values
    ax1.plot(data["Time"], targets, '-o', label=f"CAN ID {can_id}", color=color)

    # Local minima
    local_min_idx = argrelextrema(targets, np.less, order=3)[0]
    for idx in local_min_idx:
        t = data.loc[idx, "Time"]
        y = data.loc[idx, "Target"]
        ax1.axvline(x=t, color='gray', linestyle='--', alpha=0.3)
        ax1.annotate(t.strftime("%H:%M:%S.%f")[:-3], xy=(t, y), xytext=(5, 10), textcoords='offset points',
                     arrowprops=dict(arrowstyle='->', lw=0.5), fontsize=8, color='blue')

    # === 추가 기능: 특정 값 기준선 + 오차범위 내 지점 표시 ===
    threshold_value = 2.00713
    tolerance = 0.005

    ax1.axhline(y=threshold_value, color='red', linestyle='--', linewidth=1.2, label=f"Target = {threshold_value} ± {tolerance}")

    close_indices = np.where(np.abs(targets - threshold_value) <= tolerance)[0]

    for idx in close_indices:
        t = data.loc[idx, "Time"]
        y = data.loc[idx, "Target"]
        label_time = t.strftime("%H:%M:%S.%f")[:-3]
        ax1.annotate(label_time, xy=(t, y), xytext=(10, -20), textcoords='offset points',
                    arrowprops=dict(arrowstyle='->', color='red', lw=0.8), fontsize=8, color='red')


    ax1.set_ylabel("Target Position")
    ax1.set_title(f"Target Position (CAN ID {can_id})")
    ax1.legend()
    ax1.grid(True)

    # --- plot lower: waveform ---
    ax2.plot(wav_time_axis, audio, label="Waveform", color='black')
    ax2.set_ylabel("Amplitude")
    ax2.set_title("Recorded Audio Waveform")
    ax2.grid(True)

    # === Interactive vertical line ===
    vline = ax2.axvline(wav_time_axis[0], color='red', linestyle='--', lw=1)
    text = ax2.text(0.01, 0.95, '', transform=ax2.transAxes, fontsize=10,
                    verticalalignment='top', bbox=dict(boxstyle="round", facecolor='wheat', alpha=0.5))

    def format_time(x):
        dt = mdates.num2date(x)
        return dt.strftime("%H:%M:%S.%f")[:-3]

    def on_click(event):
        if event.inaxes == ax2:
            vline.set_xdata(event.xdata)
            text.set_text(f"Time: {format_time(event.xdata)}")
            fig.canvas.draw_idle()

    def on_drag(event):
        if event.inaxes == ax2 and event.button == 1:
            vline.set_xdata(event.xdata)
            text.set_text(f"Time: {format_time(event.xdata)}")
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('motion_notify_event', on_drag)

    ax2.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S.%f"))
    fig.autofmt_xdate()

    plt.tight_layout()
    plt.show()

# === 각각의 CAN ID + 오디오 파형 시각화 ===
plot_canid_and_waveform(can_id=7, color='tab:blue')
plot_canid_and_waveform(can_id=8, color='tab:orange')
