import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, LSTM

# 1. 데이터 로드
# txt 파일 읽기
file_path = "./include/codes/codeMOY_easy0.txt"  # 드럼 악보 txt 파일 경로
columns = ['Measure', 'Time', 'RightHandPos', 'LeftHandPos', 'RightHandForce', 'LeftHandForce', 'BaseHit', 'HiHat']
drum_data = pd.read_csv(file_path, sep="\t", header=None, names=columns)

# 필요한 데이터 선택
data = drum_data[['Time', 'RightHandPos', 'LeftHandPos', 'RightHandForce', 'LeftHandForce', 'BaseHit', 'HiHat']].to_numpy()

# 2. 입력 시퀀스 생성 함수
def seq2dataset(seq, window, horizon):
    X, Y = [], []
    for i in range(len(seq) - (window + horizon) + 1):
        x = seq[i : i + window]
        y = seq[i + window + horizon - 1]
        X.append(x)
        Y.append(y)
    return np.array(X), np.array(Y)

# 3. 윈도우 크기와 수평선 계수 설정
w = 8  # 윈도우 크기 (8개의 이전 데이터를 사용)
h = 1  # 수평선 계수 (1개 데이터 예측)

X, Y = seq2dataset(data, w, h)
print(f"X shape: {X.shape}, Y shape: {Y.shape}")
print(f"첫 번째 입력 샘플: \n{X[0]}\n첫 번째 출력 샘플: \n{Y[0]}")

# 4. 데이터 분할 (훈련: 70%, 테스트: 30%)
split = int(len(X) * 0.7)
x_train, y_train = X[:split], Y[:split]
x_test, y_test = X[split:], Y[split:]

# 5. LSTM 모델 설계
model = Sequential()
model.add(LSTM(units=128, activation='relu', input_shape=(x_train.shape[1], x_train.shape[2])))
model.add(Dense(Y.shape[1]))  # 출력 크기는 Y의 피처 수 (7개)
model.compile(loss='mae', optimizer='adam', metrics=['mae'])

# 6. 모델 학습
hist = model.fit(x_train, y_train, epochs=200, batch_size=32, validation_data=(x_test, y_test), verbose=2)

# 7. 평가 및 예측
ev = model.evaluate(x_test, y_test, verbose=0)
print(f"손실함수: {ev[0]}, MAE: {ev[1]}")

pred = model.predict(x_test)
mape = np.mean(np.abs((y_test - pred) / y_test)) * 100
print(f"LSTM (MAPE): {mape:.2f}%")

# 8. 학습 결과 시각화
plt.plot(hist.history['mae'], label='Train MAE')
plt.plot(hist.history['val_mae'], label='Validation MAE')
plt.title('Model MAE')
plt.ylabel('Mean Absolute Error')
plt.xlabel('Epoch')
plt.legend(loc='best')
plt.grid()
plt.show()
