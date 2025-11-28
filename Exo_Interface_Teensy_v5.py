import sys, os, serial, struct, threading, queue, time, math
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QFileDialog
)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg

# ---------------- Serial Thread -----------------
import threading
import struct
import serial
import time
import queue

class SerialThread(threading.Thread):
    def __init__(self, data_queue, cmd_queue, port='COM9', baud=1_000_000):
        super().__init__(daemon=True)
        self.data_queue = data_queue
        self.cmd_queue = cmd_queue
        self.active = True

        # Serial port
        self.ser = serial.Serial(port, baud, timeout=0.01)
        time.sleep(0.1)  # Allow Teensy to initialize

        # Frame info
        self.HEADER_TEENSY_TO_PC = b'\x55\xaa'
        self.HEADER_PC_TO_TEENSY = 0x55AA
        self.FRAME_SIZE = 16
        self.STRUCT_FMT = "<HfffH"  # header + currentA + actualI + tau + padding

        self.buffer = bytearray()
        self.value_to_send = 0.0

    def run(self):
        while self.active:
            # -------------------
            # 1️⃣ Send commands
            # -------------------
            try:
                cmd = self.cmd_queue.get_nowait()
                if isinstance(cmd, str):
                    self.ser.write((cmd + "\n").encode())
                elif isinstance(cmd, float):
                    pkt = struct.pack("<Hf", self.HEADER_PC_TO_TEENSY, cmd)
                    self.ser.write(pkt)
            except queue.Empty:
                # No command to send, send test value
                pkt = struct.pack("<Hf", self.HEADER_PC_TO_TEENSY, self.value_to_send)
                self.ser.write(pkt)
                self.value_to_send += 0.2
                if self.value_to_send > 5.0:
                    self.value_to_send = 0.0

            # -------------------
            # 2️⃣ Read bytes
            # -------------------
            try:
                bytes_available = self.ser.in_waiting
                if bytes_available:
                    data = self.ser.read(bytes_available)
                    self.buffer.extend(data)
            except Exception:
                pass

            # -------------------
            # 3️⃣ Parse full frames
            # -------------------
            ptr = 0
            while ptr + self.FRAME_SIZE <= len(self.buffer):
                if self.buffer[ptr:ptr+2] == self.HEADER_TEENSY_TO_PC:
                    frame_bytes = self.buffer[ptr:ptr+self.FRAME_SIZE]
                    try:
                        header, currentA, actualI, tau, padding = struct.unpack(self.STRUCT_FMT, frame_bytes)
                        if padding != 0xBB66:
                            print("Warning: frame padding mismatch!")
                        # Put the data into the queue
                        self.data_queue.put((currentA, actualI, tau))
                        print(f"Recv: currentA={currentA:.3f}, actualI={actualI:.3f}, tau={tau:.3f}")
                    except struct.error as e:
                        print("Struct unpack error:", e)
                    ptr += self.FRAME_SIZE
                else:
                    ptr += 1

            # Remove processed bytes
            self.buffer = self.buffer[ptr:]
            time.sleep(0.01)  # 100 Hz loop rate

    def stop(self):
        self.active = False
        try:
            self.ser.close()
        except Exception:
            pass


# ---------------- AAN Controller -----------------
class AANController:
    def __init__(self, alpha=0.1, beta=0.05, amplitude=35.0, freq=0.1, offset=35.0):
        self.alpha = alpha
        self.beta = beta
        self.amplitude_deg = amplitude
        self.freq = freq
        self.offset_deg = offset
        self.offset_rad = math.radians(offset)
        self.amplitude_rad = math.radians(amplitude)

        self.prev_error = 0.0
        self.prev_tau_ff = 0.0
        self.current_tau_ff = 0.0

        self.start_time = None
        self.data_log = []

        self.trial_index = 0
        self.total_iterations = 5  # hardcoded for now

    def generate_target(self, t):
        # Sinusoidal target in radians
        return self.offset_rad + self.amplitude_rad * math.sin(2 * math.pi * self.freq * t-math.pi/2)

    def update(self, current_angle_rad, iteration_done=False):
        if self.start_time is None:
            self.start_time = time.time()
        t = time.time() - self.start_time

        target = self.generate_target(t)
        error = target - current_angle_rad
        tau_fb = self.beta * error
        tau_ff = self.prev_tau_ff + self.alpha * self.prev_error if self.trial_index > 0 else 0.0
        tau_total = tau_ff + tau_fb

        self.prev_error = error
        self.current_tau_ff = tau_ff

        return target, tau_fb, tau_ff, tau_total, t

    def log_data(self, t, target, current_angle, tau_ff, tau_fb, tau_total, actualI):
        self.data_log.append([t, target, current_angle, tau_ff, tau_fb, tau_total, actualI])

    def save_iteration(self, folder, prefix):
        self.trial_index += 1
        filename = f"{prefix}_Iteration_{self.trial_index}.csv"
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, filename)
        with open(path, "w") as f:
            f.write("time,target_angle,current_angle,tau_ff,tau_fb,tau_total,actual_current\n")
            for row in self.data_log:
                f.write(",".join(map(str, row)) + "\n")
        self.data_log.clear()
        self.prev_tau_ff = self.current_tau_ff
        self.prev_error = 0.0
        return path

# ---------------- GUI -----------------
class ExoController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Exo AAN Controller")
        self.resize(1300, 800)

        self.data_queue = queue.Queue()
        self.cmd_queue = queue.Queue()
        self.serial_thread = SerialThread(self.data_queue, self.cmd_queue)
        self.serial_thread.start()

        self.aan = AANController()
        self.running = False
        self.save_folder = os.path.join(os.path.expanduser("~"), "Desktop", "Exo_Data")
        self.file_prefix = "Trial"

        self.initUI()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(2)  # 500 Hz -> 2 ms

    def initUI(self):
        cw = QWidget()
        self.setCentralWidget(cw)
        layout = QVBoxLayout(cw)

        # Controls
        ctrl_layout = QHBoxLayout()
        self.startBtn = QPushButton("START")
        self.stopBtn = QPushButton("STOP")
        self.saveFolderBtn = QPushButton("Set Folder")
        self.setParamsBtn = QPushButton("Set Parameters")
        self.fileEdit = QLineEdit("Trial")
        self.alphaEdit = QLineEdit(str(self.aan.alpha))
        self.betaEdit = QLineEdit(str(self.aan.beta))
        self.ampEdit = QLineEdit(str(self.aan.amplitude_deg))
        self.freqEdit = QLineEdit(str(self.aan.freq))
        self.offsetEdit = QLineEdit(str(self.aan.offset_deg))
        self.statusLabel = QLabel("Idle")

        for w in [self.startBtn, self.stopBtn, self.saveFolderBtn, self.setParamsBtn]:
            w.setFixedWidth(120)

        for lbl, edit in zip(["File Prefix:", "α:", "β:", "Amplitude [deg]:", "Freq [Hz]:", "Offset [deg]:"],
                             [self.fileEdit, self.alphaEdit, self.betaEdit, self.ampEdit, self.freqEdit, self.offsetEdit]):
            ctrl_layout.addWidget(QLabel(lbl))
            ctrl_layout.addWidget(edit)

        ctrl_layout.addWidget(self.setParamsBtn)
        ctrl_layout.addWidget(self.startBtn)
        ctrl_layout.addWidget(self.stopBtn)
        ctrl_layout.addWidget(self.saveFolderBtn)
        ctrl_layout.addWidget(self.statusLabel)

        layout.addLayout(ctrl_layout)

        # Plots
        self.graphWidget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphWidget)

        self.plot_angle = self.graphWidget.addPlot(title="Angle [rad]")
        self.plot_angle.addLegend()
        self.curve_target = self.plot_angle.plot([], [], pen='g', name='Target')
        self.curve_current = self.plot_angle.plot([], [], pen='r', name='Current')
        self.x_angle, self.y_target, self.y_current = [], [], []

        self.plot_tau = self.graphWidget.addPlot(title="Torque")
        self.plot_tau.addLegend()
        self.curve_tau_total = self.plot_tau.plot([], [], pen='y', name='Tau Total')
        self.curve_tau_ff = self.plot_tau.plot([], [], pen='c', name='Tau FF')
        self.curve_tau_fb = self.plot_tau.plot([], [], pen='r', name='Tau FB')
        self.x_tau, self.y_total, self.y_ff, self.y_fb = [], [], [], []

        self.plot_current = self.graphWidget.addPlot(title="Current [A]")
        self.plot_current.addLegend()
        self.curve_currentI = self.plot_current.plot([], [], pen='m', name='Actual Current')
        self.x_cur, self.y_cur = [], []

        # Connections
        self.startBtn.clicked.connect(self.start_test)
        self.stopBtn.clicked.connect(self.stop_test)
        self.saveFolderBtn.clicked.connect(self.select_folder)
        self.setParamsBtn.clicked.connect(self.set_parameters)

    def set_parameters(self):
        try:
            self.aan.alpha = float(self.alphaEdit.text())
            self.aan.beta = float(self.betaEdit.text())
            self.aan.amplitude_deg = float(self.ampEdit.text())
            self.aan.amplitude_rad = math.radians(self.aan.amplitude_deg)
            self.aan.freq = float(self.freqEdit.text())
            self.aan.offset_deg = float(self.offsetEdit.text())
            self.aan.offset_rad = math.radians(self.aan.offset_deg)
            self.file_prefix = self.fileEdit.text()
            self.statusLabel.setText("Parameters updated")
        except ValueError:
            self.statusLabel.setText("Invalid parameter(s)")

    def select_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Folder", self.save_folder)
        if folder:
            self.save_folder = folder
            self.statusLabel.setText(f"Save folder: {folder}")

    def start_test(self):
        if self.running:
            return
        self.running = True
        self.aan.trial_index = 0
        self.aan.start_time = None
        self.statusLabel.setText("Running")
        
        self.cmd_queue.put("START")
        
        self.start_next_iteration()
        

    def start_next_iteration(self):
        if self.aan.trial_index >= self.aan.total_iterations:
            self.running = False
            self.statusLabel.setText("All iterations done")
            return
        self.statusLabel.setText(f"Iteration {self.aan.trial_index + 1} running")
        # Clear plot data
        self.x_angle.clear()
        self.y_target.clear()
        self.y_current.clear()
        self.x_tau.clear()
        self.y_total.clear()
        self.y_ff.clear()
        self.y_fb.clear()
        self.x_cur.clear()
        self.y_cur.clear()
        self.aan.data_log.clear()
        self.aan.start_time = time.time()

    def stop_test(self):
        if self.running:
            self.running = False
            self.cmd_queue.put("STOP")
            self.statusLabel.setText("Stopped")

    def update_loop(self):
        if not self.data_queue.empty():
            current_angle, actualI, receivedTau = self.data_queue.get()
            print("GUI got:", current_angle)
            current_rad = math.radians(current_angle)

            if self.running:
                target, tau_fb, tau_ff, tau_total, t = self.aan.update(current_rad)
                self.cmd_queue.put(tau_total)
                self.aan.log_data(t, target, current_rad, tau_ff, tau_fb, tau_total, actualI)

                # Update plots
                self.x_angle.append(t)
                self.y_target.append(target)
                self.y_current.append(current_rad)
                self.curve_target.setData(self.x_angle, self.y_target)
                self.curve_current.setData(self.x_angle, self.y_current)

                self.x_tau.append(t)
                self.y_total.append(tau_total)
                self.y_ff.append(tau_ff)
                self.y_fb.append(tau_fb)
                self.curve_tau_total.setData(self.x_tau, self.y_total)
                self.curve_tau_ff.setData(self.x_tau, self.y_ff)
                self.curve_tau_fb.setData(self.x_tau, self.y_fb)

                self.x_cur.append(t)
                self.y_cur.append(actualI)
                self.curve_currentI.setData(self.x_cur, self.y_cur)

                # Check if iteration complete (full sin period) ~ 1/freq * 5 periods
                if t >= 5.0 / self.aan.freq:
                    path = self.aan.save_iteration(self.save_folder, self.file_prefix)
                    self.statusLabel.setText(f"Saved iteration {self.aan.trial_index} → {path}")
                    self.start_next_iteration()

    def closeEvent(self, event):
        self.stop_test()
        self.serial_thread.stop()
        event.accept()


# ---------------- Main -----------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ExoController()
    window.show()
    sys.exit(app.exec_())
