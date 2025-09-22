import sys
import serial
import threading
import queue
import re
from PyQt5.QtWidgets import QApplication, QMainWindow, QLineEdit, QPushButton, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import os
from pyqtgraph import LegendItem

class SerialThread(threading.Thread):
    def __init__(self, data_queue):
        super().__init__()
        self.daemon = True
        self.data_queue = data_queue
        # Change this COM port for Windows (e.g. "COM3") or Linux ("/dev/ttyACM0")
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.active = True

    def run(self):
        pattern = re.compile(
            r"Time: (\d+)ms, Target Current: ([\d\.-]+), Current: ([\d\.-]+), "
            r"Error: ([\d\.-]+), PWM: ([\d\.-]+), Current Angle: ([\d\.-]+), sampling Frequency: ([\d\.-]+)"
        )
        while self.active:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                match = pattern.search(line)
                if match:
                    time_val = float(match.group(1)) / 1000.0  # convert ms → seconds
                    target_cur = float(match.group(2))
                    current = float(match.group(3))
                    error = float(match.group(4))
                    pwm = float(match.group(5))
                    angle = float(match.group(6))
                    freq = float(match.group(7))
                    self.data_queue.put((time_val, target_cur, current, error, pwm, angle, freq))
            except Exception as e:
                print("Error:", e)

    def stop(self):
        self.active = False
        self.ser.close()


class SimplePlotter(QMainWindow):
    def __init__(self, data_queue):
        super().__init__()
        self.data_queue = data_queue
        self.initUI()
        self.serial_thread = SerialThread(data_queue)
        self.serial_thread.start()

    def initUI(self):
        self.setWindowTitle("Serial Plotter")
        self.resize(1200, 900)
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        layout = QVBoxLayout(self.centralWidget)

        self.graphWidget = pg.GraphicsLayoutWidget(show=True)
        layout.addWidget(self.graphWidget)

        self.filenameEdit = QLineEdit("Enter filename here", self)
        self.saveButton = QPushButton("Save", self)
        self.saveButton.clicked.connect(self.save_data)
        self.statusLabel = QLabel("", self)

        layout.addWidget(self.filenameEdit)
        layout.addWidget(self.saveButton)
        layout.addWidget(self.statusLabel)

        self.setup_plots()

        # Timer setup for updating plots
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)

    def setup_plots(self):
        # --- Plot 1: Target Current vs Current ---
        self.plot1 = self.graphWidget.addPlot(0, 0, title="Target vs Current")
        self.plot1.addLegend()
        self.x_time, self.y_target, self.y_current = [], [], []
        self.curve_target = self.plot1.plot([], [], pen="g", name="Target Current")
        self.curve_current = self.plot1.plot([], [], pen="r", name="Current")

        # --- Plot 2: Error vs Time ---
        self.plot2 = self.graphWidget.addPlot(1, 0, title="Error vs Time")
        self.plot2.addLegend()
        self.x_error, self.y_error = [], []
        self.curve_error = self.plot2.plot([], [], pen="b", name="Error")

        # --- Plot 3: PWM vs Time ---
        self.plot3 = self.graphWidget.addPlot(2, 0, title="PWM vs Time")
        self.plot3.addLegend()
        self.x_pwm, self.y_pwm = [], []
        self.curve_pwm = self.plot3.plot([], [], pen="m", name="PWM")

        # --- Plot 4: Current Angle vs Time ---
        self.plot4 = self.graphWidget.addPlot(3, 0, title="Current Angle vs Time")
        self.plot4.addLegend()
        self.x_angle, self.y_angle = [], []
        self.curve_angle = self.plot4.plot([], [], pen="c", name="Angle")

        # Storage for saving
        self.data_log = []

    def update_plots(self):
        while not self.data_queue.empty():
            data = self.data_queue.get()
            (time_sec, target_cur, current, error, pwm, angle, freq) = data

            # Append data
            self.x_time.append(time_sec)
            self.y_target.append(target_cur)
            self.y_current.append(current)
            self.x_error.append(time_sec)
            self.y_error.append(error)
            self.x_pwm.append(time_sec)
            self.y_pwm.append(pwm)
            self.x_angle.append(time_sec)
            self.y_angle.append(angle)

            self.data_log.append(data)

        # Update curves
        self.curve_target.setData(self.x_time, self.y_target)
        self.curve_current.setData(self.x_time, self.y_current)
        self.curve_error.setData(self.x_error, self.y_error)
        self.curve_pwm.setData(self.x_pwm, self.y_pwm)
        self.curve_angle.setData(self.x_angle, self.y_angle)

    def save_data(self):
        filename = self.filenameEdit.text()
        if not filename:
            filename = "log"

        desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
        folder_path = os.path.join(desktop_path, "Exo_Data")
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        full_filename = os.path.join(folder_path, f"{filename}.csv")
        counter = 1
        while os.path.exists(full_filename):
            full_filename = os.path.join(folder_path, f"{filename}_{counter}.csv")
            counter += 1

        header = "Time(s), Target Current, Current, Error, PWM, Current Angle, Frequency"

        with open(full_filename, "w") as f:
            f.write(header + "\n")
            for row in self.data_log:
                f.write(", ".join(map(str, row)) + "\n")

        self.statusLabel.setText(f"Data saved to {full_filename}")

    def closeEvent(self, event):
        self.serial_thread.stop()
        self.serial_thread.join()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    data_queue = queue.Queue()
    window = SimplePlotter(data_queue)
    window.show()
    sys.exit(app.exec_())
