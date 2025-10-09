import sys, serial, struct, threading, queue, os, time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLineEdit, QPushButton,
    QLabel, QVBoxLayout, QWidget, QHBoxLayout
)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg

# ---------------- Serial Thread -----------------
class SerialThread(threading.Thread):
    def __init__(self, data_queue, msg_queue, cmd_queue, port='/dev/ttyACM0', baud=1_000_000):
        super().__init__(daemon=True)
        self.data_queue = data_queue      # numerical binary frames
        self.msg_queue = msg_queue        # ASCII messages (READY, START OK, STOP OK, etc.)
        self.cmd_queue = cmd_queue        # outgoing commands
        self.active = True
        self.frame_size = struct.calcsize("<Iffffffff")  # 36 bytes
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except Exception as e:
            raise RuntimeError(f"Could not open serial port {port}: {e}")

    def run(self):
        # Wait for incoming data; handle ASCII lines first, then binary frames
        while self.active:
            # Send outgoing commands (non-blocking)
            try:
                if not self.cmd_queue.empty():
                    cmd = self.cmd_queue.get_nowait()
                    # ensure newline
                    self.ser.write((cmd + "\n").encode())
            except queue.Empty:
                pass
            except Exception as e:
                print("Serial write error:", e)

            # Read ASCII line(s) first (if any)
            try:
                line = self.ser.readline()
            except Exception as e:
                print("Serial read error (line):", e)
                line = b''

            if line:
                try:
                    text = line.decode("utf-8", errors="ignore").strip()
                except:
                    text = None
                if text:
                    # forward human-readable messages to GUI
                    self.msg_queue.put(text)
                    # continue loop to allow binary read afterwards
                    continue

            # If no ASCII line, attempt to read a binary frame
            try:
                data = self.ser.read(self.frame_size)
            except Exception as e:
                print("Serial read error (frame):", e)
                data = b''

            if len(data) == self.frame_size:
                try:
                    unpacked = struct.unpack("<Iffffffff", data)
                    self.data_queue.put(unpacked)
                except struct.error as e:
                    # If unpack fails, drop and continue
                    print("Struct unpack error:", e)
                    continue

            # small sleep to avoid pegging CPU
            time.sleep(0.001)

    def stop(self):
        self.active = False
        try:
            self.ser.close()
        except:
            pass


# ---------------- GUI -----------------
class ExoController(QMainWindow):
    def __init__(self, serial_port='/dev/ttyACM0'):
        super().__init__()
        self.data_queue = queue.Queue()
        self.msg_queue = queue.Queue()
        self.cmd_queue = queue.Queue()
        try:
            self.serial_thread = SerialThread(self.data_queue, self.msg_queue, self.cmd_queue, port=serial_port)
        except RuntimeError as e:
            raise

        self.serial_thread.start()

        self.initUI()
        self.data_log = []

        # Timer for GUI updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)  # 20 Hz UI update

        self.running = False

    def initUI(self):
        self.setWindowTitle("Exo Control + Tuning")
        self.resize(1450, 900)
    
        # --- Central widget setup ---
        cw = QWidget(self)
        self.setCentralWidget(cw)
        main_layout = QVBoxLayout(cw)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(6)
    
        # ---------- ROW 1: Input parameters ----------
        row1 = QHBoxLayout()
        row1.setSpacing(6)
    
        self.fileEdit = QLineEdit("Test1")
        self.freqEdit = QLineEdit("0.05")
        self.ampEdit = QLineEdit("35.0")
        self.offEdit = QLineEdit("35.0")
        self.phaseEdit = QLineEdit("-1.57")
        self.aEdit = QLineEdit("1.5")
        self.alphaEdit = QLineEdit("0.006")
    
        edits = [
            ("File:", self.fileEdit),
            ("Freq [Hz]:", self.freqEdit),
            ("Amp [°]:", self.ampEdit),
            ("Offset [°]:", self.offEdit),
            ("Phase [rad]:", self.phaseEdit),
            ("a:", self.aEdit),
            ("α:", self.alphaEdit),
        ]
    
        for label, field in edits:
            lbl = QLabel(label)
            lbl.setFixedWidth(70)
            field.setFixedWidth(80)
            row1.addWidget(lbl)
            row1.addWidget(field)
    
        row1.addStretch()
    
        # ---------- ROW 2: Control buttons + status ----------
        row2 = QHBoxLayout()
        row2.setSpacing(10)
    
        self.tuneBtn = QPushButton("Send Tuning")
        self.startBtn = QPushButton("START")
        self.stopBtn = QPushButton("STOP")
        self.saveBtn = QPushButton("Save")
        self.statusLabel = QLabel("Idle")
        self.statusLabel.setFixedWidth(150)
        self.statusLabel.setAlignment(Qt.AlignCenter)
    
        for w in [self.tuneBtn, self.startBtn, self.stopBtn, self.saveBtn]:
            w.setFixedSize(120, 32)
    
        row2.addStretch()
        row2.addWidget(self.tuneBtn)
        row2.addWidget(self.startBtn)
        row2.addWidget(self.stopBtn)
        row2.addWidget(self.saveBtn)
        row2.addSpacing(30)
        row2.addWidget(self.statusLabel)
        row2.addStretch()
    
        # Add both control rows in a fixed container
        ctrl_container = QWidget()
        ctrl_vbox = QVBoxLayout(ctrl_container)
        ctrl_vbox.setContentsMargins(0, 0, 0, 0)
        ctrl_vbox.setSpacing(4)
        ctrl_vbox.addLayout(row1)
        ctrl_vbox.addLayout(row2)
    
        main_layout.addWidget(ctrl_container, stretch=0)
    
        # ---------- GRAPH AREA ----------
        self.graphWidget = pg.GraphicsLayoutWidget()
        self.graphWidget.setSizePolicy(
            pg.QtWidgets.QSizePolicy.Expanding,
            pg.QtWidgets.QSizePolicy.Expanding
        )
        main_layout.addWidget(self.graphWidget, stretch=1)
        self.setup_plots()
    
        # ---------- Connections ----------
        self.startBtn.clicked.connect(self.start_test)
        self.stopBtn.clicked.connect(self.stop_test)
        self.saveBtn.clicked.connect(self.save_data)
        self.tuneBtn.clicked.connect(self.send_tuning)
    
        # Keyboard shortcuts
        self.startBtn.setShortcut(Qt.Key_Space)
        self.stopBtn.setShortcut(Qt.Key_Escape)



    # ---------------- Plot setup ----------------
    def setup_plots(self):
        self.plot1 = self.graphWidget.addPlot(0, 0, title="Angles")
        self.plot1.addLegend()
        self.x_time, self.y_target, self.y_current = [], [], []
        self.curve_target = self.plot1.plot([], [], pen="g", name="Target")
        self.curve_current = self.plot1.plot([], [], pen="r", name="Current")

        self.plot2 = self.graphWidget.addPlot(1, 0, title="Torque")
        self.plot2.addLegend()
        self.x_torque, self.y_torque, self.y_tff, self.y_tfb = [], [], [], []
        self.curve_torque = self.plot2.plot([], [], pen="y", name="Torque")
        self.curve_tff = self.plot2.plot([], [], pen="c", name="T_FF")
        self.curve_tfb = self.plot2.plot([], [], pen="r", name="T_FB")

        self.plot3 = self.graphWidget.addPlot(2, 0, title="Currents")
        self.plot3.addLegend()
        self.x_cur, self.y_targetI, self.y_actualI = [], [], []
        self.curve_targetI = self.plot3.plot([], [], pen="b", name="Target I")
        self.curve_actualI = self.plot3.plot([], [], pen="m", name="Actual I")

        self.plot4 = self.graphWidget.addPlot(3, 0, title="Sampling Frequency")
        self.x_freq, self.y_freq = [], []
        self.curve_freq = self.plot4.plot([], [], pen="w", name="Freq")

    # ---------------- Helpers ----------------
    def clear_data(self):
        # clear stored arrays + the plotted curves
        self.x_time = []
        self.y_target = []
        self.y_current = []
        self.x_torque = []
        self.y_torque = []
        self.y_tff = []
        self.y_tfb = []
        self.x_cur = []
        self.y_targetI = []
        self.y_actualI = []
        self.x_freq = []
        self.y_freq = []
        self.data_log = []
        # clear plot visuals
        self.curve_target.setData([], [])
        self.curve_current.setData([], [])
        self.curve_torque.setData([], [])
        self.curve_tff.setData([], [])
        self.curve_tfb.setData([], [])
        self.curve_targetI.setData([], [])
        self.curve_actualI.setData([], [])
        self.curve_freq.setData([], [])

    def lock_inputs(self, enable):
        """enable=False disables editing while motor runs; enable=True enables editing"""
        for w in [self.fileEdit, self.freqEdit, self.ampEdit, self.offEdit,
                  self.phaseEdit, self.aEdit, self.alphaEdit, self.tuneBtn]:
            w.setEnabled(enable)

    # ---------------- Loop update ----------------
    def update_plots(self):
        # First, handle ASCII messages from Teensy
        while not self.msg_queue.empty():
            msg = self.msg_queue.get()
            # debug
            # print("MSG:", msg)
            if "READY" in msg:
                self.statusLabel.setText("READY")
            elif "START OK" in msg:
                # Teensy confirmed start — clear old data and lock inputs
                self.clear_data()
                self.running = True
                self.lock_inputs(False)
                self.statusLabel.setText("RUNNING")
            elif "STOP OK" in msg or "SAFETY STOP" in msg:
                self.running = False
                self.lock_inputs(True)
                self.statusLabel.setText("STOPPED")
            elif "PARAMS UPDATED" in msg or "TUNING UPDATED" in msg:
                self.statusLabel.setText(msg)
            else:
                # unknown text line -> show briefly
                self.statusLabel.setText(msg)

        # Then handle binary frames
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

        # Update plots (fast)
        self.curve_target.setData(self.x_time, self.y_target)
        self.curve_current.setData(self.x_time, self.y_current)
        self.curve_torque.setData(self.x_torque, self.y_torque)
        self.curve_tff.setData(self.x_torque, self.y_tff)
        self.curve_tfb.setData(self.x_torque, self.y_tfb)
        self.curve_targetI.setData(self.x_cur, self.y_targetI)
        self.curve_actualI.setData(self.x_cur, self.y_actualI)
        self.curve_freq.setData(self.x_freq, self.y_freq)

    # ---------------- Controls ----------------
    def start_test(self):
        # clear plots immediately when user hits START (ensures visually fresh run)
        self.clear_data()

        # send SET then START
        freq = self.freqEdit.text()
        amp = self.ampEdit.text()
        off = self.offEdit.text()
        phase = self.phaseEdit.text()

        cmd = f"SET {freq} {amp} {off} {phase}"
        self.cmd_queue.put(cmd)
        # small pause to let params be processed first (not strictly necessary)
        time.sleep(0.02)
        self.cmd_queue.put("START")

        # update UI immediately; will also be confirmed by Teensy's "START OK"
        self.running = True
        self.lock_inputs(False)
        self.statusLabel.setText("SENT START...")

    def stop_test(self):
        self.cmd_queue.put("STOP")
        # update UI: disable motor-editing until stop confirmed by Teensy
        self.statusLabel.setText("SENT STOP...")
        # the serial thread will push "STOP OK" which will finalize state

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
    # optionally, let user pass port via command-line
    port = '/dev/ttyACM0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    try:
        app = QApplication(sys.argv)
        window = ExoController(serial_port=port)
        window.show()
        sys.exit(app.exec_())
    except RuntimeError as e:
        print(e)
        sys.exit(1)
