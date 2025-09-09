import sys
import serial
import threading
import queue
import re
import os
from PyQt5.QtWidgets import QApplication, QMainWindow, QLineEdit, QPushButton, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
import pyqtgraph as pg

# ----------------- Serial Thread -----------------
class SerialThread(threading.Thread):
    def __init__(self, data_queue, port='/dev/ttyACM0', baud=115200):
        super().__init__()
        self.daemon = True
        self.data_queue = data_queue
        self.active = True
        self.ser = serial.Serial(port, baud, timeout=1)

        # Regex matches Arduino v3 log: t, target, current, torque, pwm
        self.pattern = re.compile(
            r"(\d+),([-\d.]+),([-\d.]+),([-\d.]+),(\d+)"
        )

    def run(self):
        while self.active:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                match = self.pattern.match(line)
                if match:
                    t = float(match.group(1))
                    target = float(match.group(2))
                    current = float(match.group(3))
                    torque = float(match.group(4))
                    pwm = int(match.group(5))
                    self.data_queue.put((t, target, current, torque, pwm))
            except Exception as e:
                print("Serial read error:", e)

    def stop(self):
        self.active = False
        if self.ser.is_open:
            self.ser.close()

# ----------------- Main Plotting Window -----------------
class Plotter(QMainWindow):
    def __init__(self, data_queue):
        super().__init__()
        self.data_queue = data_queue
        self.setWindowTitle("Arduino PID Logger")
        self.resize(1000, 800)

        # Setup central widget
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        layout = QVBoxLayout(self.central_widget)

        # PyQtGraph widget
        self.graphWidget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphWidget)

        # Controls
        self.filenameEdit = QLineEdit("data_log", self)
        self.saveButton = QPushButton("Save CSV")
        self.saveButton.clicked.connect(self.save_data)
        self.statusLabel = QLabel("", self)

        layout.addWidget(self.filenameEdit)
        layout.addWidget(self.saveButton)
        layout.addWidget(self.statusLabel)

        # Data buffers
        self.time = []
        self.target = []
        self.current = []
        self.torque = []
        self.pwm = []

        # Setup plots
        self.setup_plots()

        # Start serial thread
        self.serial_thread = SerialThread(self.data_queue)
        self.serial_thread.start()

        # Timer to update plots every 50ms
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)

    def setup_plots(self):
        # Plot 1: Target vs Current
        self.positionPlot = self.graphWidget.addPlot(row=0, col=0, title="Target vs Current")
        self.positionPlot.setLabel('left', "Angle")
        self.positionPlot.setLabel('bottom', "Time (s)")
        self.plot_target = self.positionPlot.plot(pen='g', name='Target')
        self.plot_current = self.positionPlot.plot(pen='r', name='Current')

        # Plot 2: Torque
        self.torquePlot = self.graphWidget.addPlot(row=1, col=0, title="Torque")
        self.torquePlot.setLabel('left', "Torque")
        self.torquePlot.setLabel('bottom', "Time (s)")
        self.plot_torque = self.torquePlot.plot(pen='b', name='Torque')

        # Plot 3: PWM
        self.pwmPlot = self.graphWidget.addPlot(row=2, col=0, title="PWM")
        self.pwmPlot.setLabel('left', "PWM")
        self.pwmPlot.setLabel('bottom', "Time (s)")
        self.plot_pwm = self.pwmPlot.plot(pen='y', name='PWM')

    def update_plots(self):
        # Read all new data from queue
        while not self.data_queue.empty():
            t, target, current, torque, pwm = self.data_queue.get()
            self.time.append(t / 1000.0)  # Convert ms to s
            self.target.append(target)
            self.current.append(current)
            self.torque.append(torque)
            self.pwm.append(pwm)

        # Update plots
        if self.time:
            self.plot_target.setData(self.time, self.target)
            self.plot_current.setData(self.time, self.current)
            self.plot_torque.setData(self.time, self.torque)
            self.plot_pwm.setData(self.time, self.pwm)

    def save_data(self):
        filename = self.filenameEdit.text()
        desktop = os.path.join(os.path.expanduser("~"), "Desktop")
        folder = os.path.join(desktop, "Exo_Data")
        os.makedirs(folder, exist_ok=True)
        filepath = os.path.join(folder, filename + ".csv")

        # Avoid overwriting
        counter = 1
        while os.path.exists(filepath):
            filepath = os.path.join(folder, f"{filename}_{counter}.csv")
            counter += 1

        # Write CSV
        with open(filepath, 'w') as f:
            f.write("Time(s),Target,Current,Torque,PWM\n")
            for row in zip(self.time, self.target, self.current, self.torque, self.pwm):
                f.write(",".join(map(str, row)) + "\n")
        self.statusLabel.setText(f"Saved to {filepath}")

    def closeEvent(self, event):
        self.serial_thread.stop()
        self.serial_thread.join()
        event.accept()


# ----------------- Main -----------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    data_queue = queue.Queue()
    window = Plotter(data_queue)
    window.show()
    sys.exit(app.exec_())
