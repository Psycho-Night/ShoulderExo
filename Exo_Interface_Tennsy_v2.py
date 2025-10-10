import sys, serial, struct, threading, queue, os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLineEdit, QPushButton,
    QLabel, QVBoxLayout, QWidget, QHBoxLayout
)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg


# ---------------- Serial Thread -----------------
class SerialThread(threading.Thread):
    def __init__(self, data_queue, cmd_queue):
        super().__init__(daemon=True)
        self.data_queue = data_queue
        self.cmd_queue = cmd_queue
        self.active = True

        # Adjust COM port for your Teensy
        self.ser = serial.Serial('/dev/ttyACM0', 1_000_000, timeout=1)

        # Frame structure constants
        self.FRAME_SIZE = 38  # bytes
        self.HEADER = b'\x55\xaa'
        self.PAYLOAD_SIZE = self.FRAME_SIZE - len(self.HEADER)
        self.struct_fmt = "<Iffffffff"
        self.struct_len = struct.calcsize(self.struct_fmt)

    def run(self):
        print("Waiting for Teensy...")

        while self.active:
            # Process outgoing commands
            if not self.cmd_queue.empty():
                cmd = self.cmd_queue.get()
                self.ser.write((cmd + "\n").encode())

            # Find header
            byte = self.ser.read(1)
            if byte == self.HEADER[:1]:
                next_byte = self.ser.read(1)
                if next_byte == self.HEADER[1:]:
                    payload = self.ser.read(self.PAYLOAD_SIZE)
                    if len(payload) == self.PAYLOAD_SIZE:
                        try:
                            unpacked = struct.unpack(self.struct_fmt, payload)
                            self.data_queue.put(unpacked)
                        except struct.error:
                            continue  # skip corrupted frame

    def stop(self):
        self.active = False
        try:
            self.ser.close()
        except:
            pass


# ---------------- GUI -----------------
class ExoController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.data_queue = queue.Queue()
        self.cmd_queue = queue.Queue()
        self.serial_thread = SerialThread(self.data_queue, self.cmd_queue)
        self.serial_thread.start()

        self.initUI()
        self.data_log = []
        self.running = False

        # Timer for GUI updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)

    # ---------------- UI ----------------
    def initUI(self):
        self.setWindowTitle("Exo Control + Tuning (Synced)")
        self.resize(1450, 900)

        cw = QWidget(self)
        self.setCentralWidget(cw)
        layout = QVBoxLayout(cw)

        # Control layout
        ctrl_layout = QHBoxLayout()

        self.fileEdit = QLineEdit("Test1")
        self.freqEdit = QLineEdit("0.05")
        self.ampEdit = QLineEdit("35.0")
        self.offEdit = QLineEdit("35.0")
        self.phaseEdit = QLineEdit("-1.57")
        self.aEdit = QLineEdit("1.5")
        self.alphaEdit = QLineEdit("0.006")

        self.startBtn = QPushButton("START")
        self.stopBtn = QPushButton("STOP")
        self.saveBtn = QPushButton("Save")
        self.tuneBtn = QPushButton("Send Tuning")
        self.statusLabel = QLabel("Idle")

        for w in [self.startBtn, self.stopBtn, self.saveBtn, self.tuneBtn]:
            w.setFixedWidth(100)

        ctrl_layout.addWidget(QLabel("File:"))
        ctrl_layout.addWidget(self.fileEdit)
        ctrl_layout.addWidget(QLabel("Freq [Hz]:"))
        ctrl_layout.addWidget(self.freqEdit)
        ctrl_layout.addWidget(QLabel("Amp [°]:"))
        ctrl_layout.addWidget(self.ampEdit)
        ctrl_layout.addWidget(QLabel("Offset [°]:"))
        ctrl_layout.addWidget(self.offEdit)
        ctrl_layout.addWidget(QLabel("Phase [rad]:"))
        ctrl_layout.addWidget(self.phaseEdit)
        ctrl_layout.addWidget(QLabel("a:"))
        ctrl_layout.addWidget(self.aEdit)
        ctrl_layout.addWidget(QLabel("α:"))
        ctrl_layout.addWidget(self.alphaEdit)
        ctrl_layout.addWidget(self.tuneBtn)
        ctrl_layout.addWidget(self.startBtn)
        ctrl_layout.addWidget(self.stopBtn)
        ctrl_layout.addWidget(self.saveBtn)
        ctrl_layout.addWidget(self.statusLabel)
        layout.addLayout(ctrl_layout)

        # Plot setup
        self.graphWidget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphWidget)
        self.setup_plots()

        # Connections
        self.startBtn.clicked.connect(self.start_test)
        self.stopBtn.clicked.connect(self.stop_test)
        self.saveBtn.clicked.connect(self.save_data)
        self.tuneBtn.clicked.connect(self.send_tuning)

    # ---------------- Plot setup ----------------
    def setup_plots(self):
        self.plot1 = self.graphWidget.addPlot(title="Angles")
        self.plot1.addLegend()
        self.x_time, self.y_target, self.y_current = [], [], []
        self.curve_target = self.plot1.plot([], [], pen="g", name="Target")
        self.curve_current = self.plot1.plot([], [], pen="r", name="Current")

        self.plot2 = self.graphWidget.addPlot(title="Torque")
        self.plot2.addLegend()
        self.x_torque, self.y_torque, self.y_tff, self.y_tfb = [], [], [], []
        self.curve_torque = self.plot2.plot([], [], pen="y", name="Torque")
        self.curve_tff = self.plot2.plot([], [], pen="c", name="T_FF")
        self.curve_tfb = self.plot2.plot([], [], pen="r", name="T_FB")

        self.plot3 = self.graphWidget.addPlot(title="Currents")
        self.plot3.addLegend()
        self.x_cur, self.y_targetI, self.y_actualI = [], [], []
        self.curve_targetI = self.plot3.plot([], [], pen="b", name="Target I")
        self.curve_actualI = self.plot3.plot([], [], pen="m", name="Actual I")

        self.plot4 = self.graphWidget.addPlot(title="Sampling Frequency")
        self.x_freq, self.y_freq = [], []
        self.curve_freq = self.plot4.plot([], [], pen="w", name="Freq")

    # ---------------- Loop update ----------------
    def update_plots(self):
        while not self.data_queue.empty():
            (t_us, target, current, torque, T_ff, T_fb,
             targetI, actualI, freq) = self.data_queue.get()

            t_s = t_us / 1e6
            self.x_time.append(t_s)
            self.y_target.append(target)
            self.y_current.append(current)
            self.x_torque.append(t_s)
            self.y_torque.append(torque)
            self.y_tff.append(T_ff)
            self.y_tfb.append(T_fb)
            self.x_cur.append(t_s)
            self.y_targetI.append(targetI)
            self.y_actualI.append(actualI)
            self.x_freq.append(t_s)
            self.y_freq.append(freq)

            self.data_log.append([t_s, target, current, torque, T_ff, T_fb,
                                  targetI, actualI, freq])

        # Update plots
        self.curve_target.setData(self.x_time, self.y_target)
        self.curve_current.setData(self.x_time, self.y_current)
        self.curve_torque.setData(self.x_torque, self.y_torque)
        self.curve_tff.setData(self.x_torque, self.y_tff)
        self.curve_tfb.setData(self.x_torque, self.y_tfb)
        self.curve_targetI.setData(self.x_cur, self.y_targetI)
        self.curve_actualI.setData(self.x_cur, self.y_actualI)
        self.curve_freq.setData(self.x_freq, self.y_freq)

    # ---------------- Controls ----------------
    def lock_inputs(self, state):
        for w in [self.fileEdit, self.freqEdit, self.ampEdit, self.offEdit,
                  self.phaseEdit, self.aEdit, self.alphaEdit, self.tuneBtn]:
            w.setEnabled(state)

    def start_test(self):
        freq = self.freqEdit.text()
        amp = self.ampEdit.text()
        off = self.offEdit.text()
        phase = self.phaseEdit.text()
        self.cmd_queue.put(f"SET {freq} {amp} {off} {phase}")
        self.cmd_queue.put("START")
        self.running = True
        self.lock_inputs(False)
        self.statusLabel.setText("RUNNING")

    def stop_test(self):
        self.cmd_queue.put("STOP")
        self.running = False
        self.lock_inputs(True)
        self.statusLabel.setText("STOPPED")

    def send_tuning(self):
        if self.running:
            self.statusLabel.setText("Can't tune while running!")
            return
        a = self.aEdit.text()
        alpha = self.alphaEdit.text()
        self.cmd_queue.put(f"TUNE {a} {alpha}")
        self.statusLabel.setText("Tuning params sent")

    def save_data(self):
        freq = self.freqEdit.text().replace('.', '_')
        amp = self.ampEdit.text().replace('.', '_')
        off = self.offEdit.text().replace('.', '_')
        name = f"{self.fileEdit.text()}_{freq}Hz{amp}Amp_{off}Off.csv"

        desktop = os.path.join(os.path.expanduser("~"), "Desktop", "Exo_Data")
        os.makedirs(desktop, exist_ok=True)
        path = os.path.join(desktop, name)

        with open(path, "w") as f:
            f.write("Time,Target,Current,Torque,T_ff,T_fb,TargetI,ActualI,Freq\n")
            for row in self.data_log:
                f.write(",".join(map(str, row)) + "\n")

        self.statusLabel.setText(f"Saved → {path}")

    def closeEvent(self, event):
        self.serial_thread.stop()
        event.accept()


# ---------------- Main -----------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ExoController()
    window.show()
    sys.exit(app.exec_())
