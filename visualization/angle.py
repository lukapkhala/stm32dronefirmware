import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from collections import deque
import numpy as np

# --- Serial setup ---
SERIAL_PORT = "COM11"   # your port
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# --- Buffers ---
N = 500
roll_data, pitch_data = deque([0.0]*N, maxlen=N), deque([0.0]*N, maxlen=N)

# --- PyQtGraph setup ---
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="IMU Attitude (Complementary Filter)")
win.resize(800, 400)

# Roll & Pitch plot
p1 = win.addPlot(title="Roll & Pitch (degrees)")
curve_roll = p1.plot(pen='r', name="Roll")
curve_pitch = p1.plot(pen='b', name="Pitch")

# Fix Y-axis range between -190 and 190
p1.setYRange(-190, 190)

# --- Update loop ---
counter = 0
def update():
    global counter
    for _ in range(5):  # read a few lines per update
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("Roll:"):
            try:
                # Example: "Roll: 12.34 Pitch: -7.89"
                parts = line.replace("Roll:", "").replace("Pitch:", "").split()
                roll, pitch = float(parts[0]), float(parts[1])
                roll_data.append(roll)
                pitch_data.append(pitch)
            except:
                continue

    # Only update plot every 3rd call
    counter += 1
    if counter % 3 == 0:
        curve_roll.setData(np.fromiter(roll_data, dtype=float))
        curve_pitch.setData(np.fromiter(pitch_data, dtype=float))

# Timer (calls update continuously)
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(5)  # run update ~200 Hz

# --- Start Qt event loop ---
if __name__ == "__main__":
    QtWidgets.QApplication.instance().exec_()
