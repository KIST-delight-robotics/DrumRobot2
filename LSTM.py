import numpy as np
import pandas as pd
import matplotlib

f = open("BTC_USD_2019-02-28_2020-02-27-CoinDesk.csv", "r")
coindesk_data = pd.read_csv(f, header = 0)
seq = coindesk_data[['Closing Prise (USD)',  '24h Open (USD)', '24 High (USD)', '24h Low (USD)']].to_numpy()

def seq2dataset(seq, window.horizon):
    X=[]; Y=[]
    for i in range(len(seq) - (window + horizon)+ 1):
        x = seq[i : (i + window)]
        y = (seq[i + window + horizon -1])
        X.append(x); Y.append(y)
        return np.array(X), np.array(Y)
    
w = 7 # 윈도우 크기
h = 1 # 수평선 계수

X, Y = seq2dataset(seq, w, h)
print(X.shape, Y.shape)
print(X[0], Y[0]) # 0번 샘플 확인

from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, LSTM, Dropout

split = int(len(X)*0.7)
x_train = X[0:split]; y_train = Y[0:split]
x_test = X[split:]; y_test = Y[split:]

model = Sequential()
model.add(LSTM(units = 128, activation = 'relu', input_shape = x_train[0].shape))
model.add(Dense(4))
model.compile(loss = 'mae', optimizer = 'adam', metrics = ['mae'])
hist = model.fit(x.train, y_train, epochs = 200, batch_size = 1, validation_data = (x_test, y_test), verbose = 2)

ev = model.evaluate(x_test, y_test, verbose = 0)
print("손실함수: ", ev[0], "MAE: ", ev[1])

pred = model.predict(x_test)
print("LSTM (MAPE): ", sum(abs(y_test-pred)/y_test)/len(x_test))

plt.plot(hist.history['mae'])
plt.plot(hist.history['val_mae'])
plt.title('Model mae')
plt.ylabel('mae')
plt.xlabel('Epoch')
plt.ylim([100, 600])
plt.legend(['Train', 'Vadlidation', loc = 'best'])
plt.grid()
plt.show()

