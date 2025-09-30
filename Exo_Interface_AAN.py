import sys
import serial
import threading
import queue
import re
from PyQt5.QtWidgets import QApplication, QMainWindow, QLineEdit, QPushButton, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import os


class SerialThread(threading.Thread):
    def __init__(self, data_queue):
        super().__init__()
        self.daemon = True
        self.data_queue = data_queue
        # ⚠️ Change COM port for your system
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.active = True

    def run(self):
        pattern = re.compile(
            r"Time: (\d+)ms, Target Angle: ([\d\.-]+), Current Angle: ([\d\.-]+), "
            r"T_FF: ([\d\.-]+), T_FB: ([\d\.-]+), Torque: ([\d\.-]+), "
            r"Target Current: ([\d\.-]+), Current: ([\d\.-]+), "
            r"PWM: ([\d\.-]+), sampling Frequency: ([\d\.-]+)"
        )
        while self.active:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                match = pattern.search(line)
                if match:
                    time_val = float(match.group(1)) / 1000.0  # ms → s
                    target_angle = float(match.group(2))
                    current_angle = float(match.group(3))
                    t_ff = float(match.group(4))
                    t_fb = float(match.group(5))
                    torque = float(match.group(6))
                    target_cur = float(match.group(7))
                    actual_cur = float(match.group(8))
                    pwm = float(match.group(9))
                    freq = float(match.group(10))
                    self.data_queue.put((time_val, target_angle, current_angle,
                                         t_ff, t_fb, torque,
                                         target_cur, actual_cur,
                                         pwm, freq))
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
        self.resize(1200, 1000)
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

        # Timer setup
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)

    def setup_plots(self):
        # --- Plot 1: Angles ---
        self.plot1 = self.graphWidget.addPlot(0, 0, title="Angles")
        self.plot1.addLegend()
        self.x_time, self.y_target_angle, self.y_current_angle = [], [], []
        self.curve_target_angle = self.plot1.plot([], [], pen="g", name="Target Angle")
        self.curve_current_angle = self.plot1.plot([], [], pen="r", name="Current Angle")

        # --- Plot 2: Currents ---
        self.plot2 = self.graphWidget.addPlot(1, 0, title="Currents")
        self.plot2.addLegend()
        self.x_current, self.y_target_cur, self.y_actual_cur = [], [], []
        self.curve_target_cur = self.plot2.plot([], [], pen="b", name="Target Current")
        self.curve_actual_cur = self.plot2.plot([], [], pen="m", name="Actual Current")

        # --- Plot 3: Torque, T_FF, T_FB ---
        self.plot3 = self.graphWidget.addPlot(2, 0, title="Torque Components")
        self.plot3.addLegend()
        self.x_torque, self.y_torque, self.y_tff, self.y_tfb = [], [], [], []
        self.curve_torque = self.plot3.plot([], [], pen="y", name="Torque")
        self.curve_tff = self.plot3.plot([], [], pen="c", name="T_FF")
        self.curve_tfb = self.plot3.plot([], [], pen="r", name="T_FB")

        # --- Plot 4: PWM ---
        self.plot4 = self.graphWidget.addPlot(3, 0, title="PWM")
        self.x_pwm, self.y_pwm = [], []
        self.curve_pwm = self.plot4.plot([], [], pen="g", name="PWM")

        # --- Plot 5: Frequency ---
        self.plot5 = self.graphWidget.addPlot(4, 0, title="Sampling Frequency")
        self.x_freq, self.y_freq = [], []
        self.curve_freq = self.plot5.plot([], [], pen="b", name="Frequency")

        # Storage for saving
        self.data_log = []

    def update_plots(self):
        while not self.data_queue.empty():
            data = self.data_queue.get()
            (time_sec, target_angle, current_angle,
             t_ff, t_fb, torque,
             target_cur, actual_cur,
             pwm, freq) = data

            # Append data
            self.x_time.append(time_sec)
            self.y_target_angle.append(target_angle)
            self.y_current_angle.append(current_angle)

            self.x_current.append(time_sec)
            self.y_target_cur.append(target_cur)
            self.y_actual_cur.append(actual_cur)

            self.x_torque.append(time_sec)
            self.y_torque.append(torque)
            self.y_tff.append(t_ff)
            self.y_tfb.append(t_fb)

            self.x_pwm.append(time_sec)
            self.y_pwm.append(pwm)

            self.x_freq.append(time_sec)
            self.y_freq.append(freq)

            self.data_log.append(data)

        # Update curves
        self.curve_target_angle.setData(self.x_time, self.y_target_angle)
        self.curve_current_angle.setData(self.x_time, self.y_current_angle)

        self.curve_target_cur.setData(self.x_current, self.y_target_cur)
        self.curve_actual_cur.setData(self.x_current, self.y_actual_cur)

        self.curve_torque.setData(self.x_torque, self.y_torque)
        self.curve_tff.setData(self.x_torque, self.y_tff)
        self.curve_tfb.setData(self.x_torque, self.y_tfb)

        self.curve_pwm.setData(self.x_pwm, self.y_pwm)
        self.curve_freq.setData(self.x_freq, self.y_freq)

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

        header = "Time(s), Target Angle, Current Angle, T_FF, T_FB, Torque, Target Current, Actual Current, PWM, Frequency"

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
