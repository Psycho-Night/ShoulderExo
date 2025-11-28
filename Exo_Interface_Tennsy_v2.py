# exo_pc_controller.py
import sys
import os
import time
import struct
import threading
import queue
import csv
import math
import serial
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLineEdit, QPushButton,
    QLabel, QVBoxLayout, QWidget, QHBoxLayout
)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg

# ---------------- Settings ----------------
SERIAL_PORT = '/dev/ttyACM0'    # Change to your port, e.g. 'COM4' on Windows
BAUDRATE = 1_000_000
DT = 1/500.0                    # Controller sample time (500 Hz)
ITERATIONS = 9                  # Number of trial runs
RAMP_SECONDS = 0.5              # Smoothing ramp in/out seconds
# PID defaults
Kp_default = 0.8
Ki_default = 0.0
Kd_default = 0.0

# Binary packet formats
# Packet we SEND to Teensy: header uint16 0x55AA + float torque -> '<Hf'
SEND_FMT = '<Hf'
SEND_HEADER = 0x55AA

# Frame we RECEIVE from Teensy: header uint16 0xAA55 + 3 floats (currentA, actualI, receivedTau) -> '<Hfff'
RECV_FMT = '<Hfff'
RECV_HEADER = 0xAA55
RECV_SIZE = struct.calcsize(RECV_FMT)


# ---------------- Helper: read exact bytes with timeout ----------------
def read_exact(ser, n, timeout=0.01):
    """Read exactly n bytes from serial or return None."""
    buf = bytearray()
    deadline = time.perf_counter() + timeout
    while len(buf) < n and time.perf_counter() < deadline:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    if len(buf) == n:
        return bytes(buf)
    return None


# ---------------- Controller worker (runs in background thread) ----------------
class ControllerWorker(threading.Thread):
    def __init__(self, ser, data_queue, stop_event,
                 freq_hz, amp_deg, offset_deg, phase_rad,
                 Kp, Ki, Kd, trials=ITERATIONS):
        super().__init__(daemon=True)
        self.ser = ser
        self.data_queue = data_queue
        self.stop_event = stop_event
        self.freq = float(freq_hz)
        self.amp = float(amp_deg)
        self.offset = float(offset_deg)
        self.phase = float(phase_rad)
        self.Kp = float(Kp)
        self.Ki = float(Ki)
        self.Kd = float(Kd)
        self.trials = int(trials)
        self.dt = DT
        self.ramp = RAMP_SECONDS

        # physical constants for current calc / filename
        self.gearRatio = 70
        self.Kt = 21.3/1000.0

    def build_folder(self, base_name):
        freq_safe = str(self.freq).replace('.', '_')
        amp_safe = str(self.amp).replace('.', '_')
        off_safe = str(self.offset).replace('.', '_')
        folder = os.path.join(os.path.expanduser('~'), 'Desktop', 'Exo_Data',
                              f"{base_name}_{freq_safe}Hz{amp_safe}Amp_{off_safe}Off")
        os.makedirs(folder, exist_ok=True)
        return folder

    def precompute_traj(self):
        # total duration for 5.5 periods
        periods = 5.5
        dur = periods / self.freq
        nsteps = int(round(dur / self.dt))
        t = [i * self.dt for i in range(nsteps)]
        # sine centered at offset, with phase and amplitude
        traj = []
        for ti in t:
            traj.append(self.offset + self.amp * math.sin(2*math.pi*self.freq*ti + self.phase))
        # apply ramp smoothing (cosine taper for first/last ramp seconds)
        ramp_steps = int(round(self.ramp / self.dt))
        if ramp_steps > 0:
            for i in range(min(ramp_steps, nsteps)):
                s = 0.5 * (1 - math.cos(math.pi * (i / ramp_steps)))  # 0..1
                traj[i] = (1 - s) * traj[0] + s * traj[i]
            for i in range(min(ramp_steps, nsteps)):
                k = nsteps - 1 - i
                s = 0.5 * (1 - math.cos(math.pi * (i / ramp_steps)))
                traj[k] = (1 - s) * traj[-1] + s * traj[k]
        return t, traj

    def load_prev_iteration(self, folder, it_num, expected_len):
        """Try to load previous iteration's T_ff and error arrays"""
        prev_n = it_num - 1
        if prev_n < 1:
            return None, None
        prev_path = os.path.join(folder, f"Iteration_{prev_n}.csv")
        if not os.path.exists(prev_path):
            return None, None
        t_ff = []
        err = []
        with open(prev_path, 'r') as f:
            rdr = csv.DictReader(f)
            for row in rdr:
                # expect columns 'T_ff' and 'error' (if missing, abort)
                try:
                    t_ff.append(float(row.get('T_ff', '0')))
                    err.append(float(row.get('error', '0')))
                except:
                    return None, None
        if len(t_ff) != expected_len:
            # lengths mismatch -> ignore
            return None, None
        return t_ff, err

    def send_torque_packet(self, torque):
        pkt = struct.pack(SEND_FMT, SEND_HEADER, float(torque))
        self.ser.write(pkt)

    def recv_frame(self, timeout=0.02):
        """Receive one binary frame with header RECV_HEADER. Non-blocking-ish with timeout."""
        # we'll scan stream until we find header bytes in little-endian order (0xAA55)
        start_deadline = time.perf_counter() + timeout
        buf = b''
        while time.perf_counter() < start_deadline:
            b = self.ser.read(1)
            if not b:
                continue
            buf += b
            # keep last 2 bytes only
            if len(buf) >= 2:
                if buf[-2:] == struct.pack('<H', RECV_HEADER):
                    # read remaining floats: 3 * 4 = 12 bytes
                    rest = read_exact(self.ser, RECV_SIZE - 2, timeout=timeout)
                    if rest is None:
                        return None
                    frame_bytes = struct.pack('<H', RECV_HEADER) + rest
                    try:
                        unpacked = struct.unpack(RECV_FMT, frame_bytes)
                        # unpacked => (header, currentA, actualI, receivedTau)
                        return unpacked
                    except struct.error:
                        return None
                else:
                    # trim buffer to last byte for continued scanning
                    buf = buf[-1:]
        return None

    def run(self):
        # build folder name from settings (use GUI file name passed via queue)
        # The GUI will put the folder_name string into data_queue as special control message before starting
        # but if not, create a default folder name
        try:
            # Wait for GUI to push run base_name in queue (timeout small)
            base_name = None
            try:
                base_name = self.data_queue.get(timeout=0.5)
                # if it's a tuple (control message) we expect ('RUNBASE', base_name)
                if isinstance(base_name, tuple) and base_name[0] == 'RUNBASE':
                    base_name = base_name[1]
            except queue.Empty:
                base_name = "Test"
        except Exception:
            base_name = "Test"

        folder = self.build_folder(base_name)

        # precompute trajectory
        t_arr, traj = self.precompute_traj()
        nsteps = len(t_arr)
        print(f"[Controller] trajectory len {nsteps}, duration {nsteps*self.dt:.3f}s")

        # run iterations
        for it in range(1, self.trials + 1):
            if self.stop_event.is_set():
                break

            print(f"[Controller] Starting iteration {it}/{self.trials}")
            # try to load T_ff and error from previous iteration if present
            prev_tff, prev_err = self.load_prev_iteration(folder, it, nsteps)
            # if present, use them as feedforward sequence; else zeros
            if prev_tff is None:
                ff_seq = [0.0]*nsteps
                prev_err_seq = [0.0]*nsteps
            else:
                ff_seq = prev_tff
                prev_err_seq = prev_err

            # per-iteration log buffer (list of dicts)
            rows = []

            # PID integrator
            integ = 0.0
            prev_error = 0.0

            start_time = time.perf_counter()

            for k in range(nsteps):
                if self.stop_event.is_set():
                    break
                t_target = t_arr[k]
                target_angle = traj[k]

                # If previous ff present, use it, else 0
                T_ff = ff_seq[k] if ff_seq else 0.0

                # receive latest frame from Teensy (if available) before computing
                # attempt to read one frame (small timeout)
                frame = self.recv_frame(timeout=self.dt*0.5)
                if frame is None:
                    # no fresh frame — use previous measurement if available
                    currentA = rows[-1]['currentA'] if rows else 0.0
                    actualI = rows[-1]['actualI'] if rows else 0.0
                    recvTau = rows[-1]['sentTau'] if rows else 0.0
                else:
                    # unpacked frame: (header, currentA, actualI, receivedTau)
                    _, currentA, actualI, recvTau = frame

                # compute error & PID (simple)
                error = (target_angle - currentA) * (math.pi/180.0)  # convert deg->rad for control if you prefer; but keep units consistent
                integ += error * self.dt
                deriv = (error - prev_error) / self.dt if self.dt > 0 else 0.0
                T_fb = self.Kp * error + self.Ki * integ + self.Kd * deriv

                torque_cmd = T_ff + T_fb

                # clamp torque to reasonable bounds (user can change)
                # (optional) we don't know motor limits here; keep them loose
                # send torque packet to Teensy
                self.send_torque_packet(torque_cmd)

                # compute desired current (for logging & comparison)
                desiredI = abs(torque_cmd) / (self.gearRatio * self.Kt)

                t_now = time.perf_counter() - start_time

                row = {
                    'time': t_now,
                    'target': target_angle,
                    'currentA': currentA,
                    'sentTau': torque_cmd,
                    'T_ff': T_ff,
                    'T_fb': T_fb,
                    'desiredI': desiredI,
                    'actualI': actualI,
                    'error': error
                }
                rows.append(row)

                # push to GUI plotting queue (as tuple)
                self.data_queue.put(('DATA', row))

                prev_error = error

                # maintain sample time
                next_step = start_time + (k+1)*self.dt
                while (not self.stop_event.is_set()) and time.perf_counter() < next_step:
                    time.sleep(0.0005)

            # end of single iteration
            # save CSV for this iteration
            csv_path = os.path.join(folder, f"Iteration_{it}.csv")
            with open(csv_path, 'w', newline='') as f:
                fieldnames = ['time','target','currentA','sentTau','T_ff','T_fb','desiredI','actualI','error']
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for r in rows:
                    writer.writerow(r)
            print(f"[Controller] Saved iteration {it} -> {csv_path}")

            # small pause between iterations
            time.sleep(0.2)

        # done with all iterations or stopped
        print("[Controller] finished")
        self.data_queue.put(('DONE', None))
