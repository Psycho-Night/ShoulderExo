import sys, serial, struct, threading, queue, os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLineEdit, QPushButton,
    QLabel, QVBoxLayout, QWidget, QHBoxLayout
)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg

# ========================= Serial Thread =============================
class SerialThread(threading.Thread):
    def __init__(self, data_queue, cmd_queue):
        super().__init__(daemon=True)
        self.data_queue = data_queue
        self.cmd_queue = cmd_queue
        self.active = True
        
        # Change COM port as needed
        self.ser = serial.Serial('COM9', 1_000_000, timeout=1)

        # ---- NEW FRAME FORMAT ----
        # header = 0xAA55 (little endian → 0x55 0xAA)
        # t = uint32
        # 8 floats
        # padding = 0xBB66
        self.HEADER = b'\x55\xAA'
        self.PADDING = 0xBB66

        self.struct_fmt = "<H I f f f f f f H"
        self.frame_size = struct.calcsize(self.struct_fmt)   # = 40 bytes

        self.buffer = bytearray()

    def run(self):
        print("Serial reader running...")

        while self.active:
            # Process outgoing commands
            if not self.cmd_queue.empty():
                cmd = self.cmd_queue.get()
                self.ser.write((cmd + "\n").encode())

            try:
                chunk = self.ser.read(128)
            except:
                continue

            if len(chunk) == 0:
                continue

            self.buffer.extend(chunk)

            # ------ Parse frames ------
            while True:
                idx = self.buffer.find(self.HEADER)
                if idx < 0:
                    # No header found, keep last few bytes only
                    if len(self.buffer) > 128:
                        self.buffer = self.buffer[-4:]
                    break

                # Discard preceding garbage
                if idx > 0:
                    self.buffer = self.buffer[idx:]

                # Not enough bytes yet
                if len(self.buffer) < self.frame_size:
                    break

                # Slice candidate frame
                frame = self.buffer[:self.frame_size]

                try:
                    parsed = struct.unpack(self.struct_fmt, frame)
                except struct.error:
                    # Bad alignment, discard header byte and retry
                    self.buffer = self.buffer[1:]
                    continue

                header = parsed[0]
                padding = parsed[-1]

                # Validate header & padding
                if header != 0xAA55 or padding != self.PADDING:
                    self.buffer = self.buffer[1:]
                    continue

                # Valid frame → extract
                (_, t, targetA, currentA, t_ff, t_fb,
                 actualI, freq, _) = parsed

                self.data_queue.put(
                    (t, targetA, currentA, t_ff, t_fb, actualI, freq)
                )

                # Remove frame from buffer
                self.buffer = self.buffer[self.frame_size:]

    def stop(self):
        self.active = False
        try:
            self.ser.close()
        except:
            pass


# =============================== GUI ====================================
class ExoController(QMainWindow):
    MaxPoints = 2000
    def __init__(self):
        super().__init__()
        self.data_queue = queue.Queue()
        self.cmd_queue = queue.Queue()
        self.serial_thread = SerialThread(self.data_queue, self.cmd_queue)
        self.serial_thread.start()

        self.initUI()
        self.data_log = []
        self.running = False

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)
    def trim_list(self, lst):
        if len(lst) > self.MaxPoints:
            del lst[:-self.MaxPoints]

    # ---------------- UI ----------------
    def initUI(self):
        self.setWindowTitle("Exo Control")
        self.resize(1450, 900)

        cw = QWidget(self)
        self.setCentralWidget(cw)
        layout = QVBoxLayout(cw)

        # ---------- Control panel ----------
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
        self.saveBtn = QPushButton("SAVE")
        self.tuneBtn = QPushButton("SET")
        self.closeBtn = QPushButton("CLOSE")
        self.statusLabel = QLabel("Idle")

        for w in [self.startBtn, self.stopBtn, self.saveBtn, self.tuneBtn, self.closeBtn]:
            w.setFixedWidth(100)

        for label, widget in [
            ("File:", self.fileEdit),
            ("Freq [Hz]:", self.freqEdit),
            ("Amp [°]:", self.ampEdit),
            ("Offset [°]:", self.offEdit),
            ("Phase [rad]:", self.phaseEdit),
            ("a:", self.aEdit),
            ("α:", self.alphaEdit),
        ]:
            ctrl_layout.addWidget(QLabel(label))
            ctrl_layout.addWidget(widget)

        ctrl_layout.addWidget(self.tuneBtn)
        ctrl_layout.addWidget(self.startBtn)
        ctrl_layout.addWidget(self.stopBtn)
        ctrl_layout.addWidget(self.saveBtn)
        ctrl_layout.addWidget(self.closeBtn)
        ctrl_layout.addWidget(self.statusLabel)

        layout.addLayout(ctrl_layout)

        # ---------- Plots ----------
        self.graphWidget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphWidget)
        self.setup_plots()

        # ---------- Buttons ----------
        self.startBtn.clicked.connect(self.start_test)
        self.stopBtn.clicked.connect(self.stop_test)
        self.saveBtn.clicked.connect(self.save_data)
        self.tuneBtn.clicked.connect(self.send_tuning)
        self.closeBtn.clicked.connect(self.close_window)

        # Shortcuts
        self.startBtn.setShortcut(Qt.Key_Return)
        self.stopBtn.setShortcut(Qt.Key_Space)
        self.closeBtn.setShortcut(Qt.Key_C)

        self.update_button_states(False)

    # ---------------- Plots ----------------
    def setup_plots(self):
        self.plot1 = self.graphWidget.addPlot(0, 0, title="Angles")
        self.plot1.addLegend()
        self.x_time, self.y_target, self.y_current = [], [], []
        self.curve_target = self.plot1.plot([], [], pen="g", name="Target")
        self.curve_current = self.plot1.plot([], [], pen="r", name="Current")

        self.plot2 = self.graphWidget.addPlot(1, 0, title="Torque")
        self.plot2.addLegend()
        self.x_torque, self.y_tfb, self.y_tff, self.y_torque = [], [], [], []
        self.curve_torque = self.plot2.plot([], [], pen="y", name="Torque")
        self.curve_tff = self.plot2.plot([], [], pen="c", name="T_FF")
        self.curve_tfb = self.plot2.plot([], [], pen="r", name="T_FB")

        self.plot3 = self.graphWidget.addPlot(2, 0, title="Currents")
        self.plot3.addLegend()
        self.x_cur, self.y_actualI = [], []
        self.curve_targetI = self.plot3.plot([], [], pen="b", name="Target I")
        self.curve_actualI = self.plot3.plot([], [], pen="m", name="Actual I")

        self.plot4 = self.graphWidget.addPlot(3, 0, title="Sampling Frequency")
        self.x_freq, self.y_freq = [], []
        self.curve_freq = self.plot4.plot([], [], pen="w", name="Freq")

    # ---------------- Update Loop ----------------
    def update_plots(self):
        while not self.data_queue.empty():
            (t_us, target, current, T_ff, T_fb,
             actualI, freq) = self.data_queue.get()

            if not hasattr(self, "start_time") or self.start_time is None:
                self.start_time = t_us

            t_s = (t_us - self.start_time) / 1e6
            torque = T_ff+T_fb


            self.x_time.append(t_s)
            self.y_target.append(target)
            self.y_current.append(current)

            self.trim_list(self.x_time)
            self.trim_list(self.y_target)
            self.trim_list(self.y_current)

            self.x_torque.append(t_s)
            self.y_torque.append(torque)
            self.y_tff.append(T_ff)
            self.y_tfb.append(T_fb)

            self.trim_list(self.x_torque)
            self.trim_list(self.y_torque)
            self.trim_list(self.y_tff)
            self.trim_list(self.y_tfb)

            self.x_cur.append(t_s)

            self.trim_list(self.x_cur)
            
            self.y_actualI.append(actualI)
            self.trim_list(self.y_actualI)

            self.x_freq.append(t_s)
            self.y_freq.append(freq)
            self.trim_list(self.x_freq)
            self.trim_list(self.y_freq)

            self.data_log.append([t_s, target, current, torque, T_ff, T_fb,
                                   actualI, freq])

        # Update graphs
        self.curve_target.setData(self.x_time, self.y_target)
        self.curve_current.setData(self.x_time, self.y_current)
        self.curve_torque.setData(self.x_torque, self.y_torque)
        self.curve_tff.setData(self.x_torque, self.y_tff)
        self.curve_tfb.setData(self.x_torque, self.y_tfb)
        # self.curve_targetI.setData(self.x_cur, self.y_targetI)
        self.curve_actualI.setData(self.x_cur, self.y_actualI)
        self.curve_freq.setData(self.x_freq, self.y_freq)

    # ---------------- Controls ----------------
    def lock_inputs(self, state):
        for w in [self.freqEdit, self.ampEdit, self.offEdit,
                  self.phaseEdit, self.aEdit, self.alphaEdit, self.tuneBtn]:
            w.setEnabled(state)

    def start_test(self):
        if self.running:
            return
        self.cmd_queue.put("START")
        self.running = True
        self.lock_inputs(False)
        self.update_button_states(True)
        self.statusLabel.setText("RUNNING")

        self.data_log.clear()
        self.start_time = None

        self.x_time.clear()
        self.y_target.clear()
        self.y_current.clear()

        self.x_torque.clear()
        self.y_torque.clear()
        self.y_tff.clear()
        self.y_tfb.clear()

        self.x_cur.clear()
        # self.y_targetI.clear()
        self.y_actualI.clear()

        self.x_freq.clear()
        self.y_freq.clear()

    def stop_test(self):
        if not self.running:
            return
        self.cmd_queue.put("STOP")
        self.running = False
        self.lock_inputs(True)
        self.update_button_states(False)
        self.statusLabel.setText("STOPPED")

    def send_tuning(self):
        if self.running:
            self.statusLabel.setText("Can't send while running!")
            return
        freq = self.freqEdit.text()
        amp = self.ampEdit.text()
        off = self.offEdit.text()
        phase = self.phaseEdit.text()
        a = self.aEdit.text()
        alpha = self.alphaEdit.text()
        cmd = f"SET {freq} {amp} {off} {phase} {alpha} {a}"
        self.cmd_queue.put(cmd)
        self.statusLabel.setText("Params sent")

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

    def close_window(self):
        self.stop_test()
        self.serial_thread.stop()
        self.close()
    
    def update_button_states(self, running):
        self.startBtn.setEnabled(not running)
        self.stopBtn.setEnabled(running)

    def closeEvent(self, event):
        self.close_window()
        self.serial_thread.stop()
        event.accept()


# ========================= Main =========================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ExoController()
    window.show()
    sys.exit(app.exec_())
