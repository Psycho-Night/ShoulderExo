import struct,serial, struct, threading, queue, os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLineEdit, QPushButton,
    QLabel, QVBoxLayout, QWidget, QHBoxLayout
)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg


# ----- Settings ---------------------
SERIAL_PORT = 'COM9'
BAUD_RATE = 1_000_000

SEND_FMT = '<Hf'
SEND_HEADER = 0x55AA

RECV_FMT = '<Hfff'
RECV_HEADER = b'\x55\xaa'
FRAME_SIZE = 14

# ----- Serial Thread -----------------
class SerialThread(threading.Thread):
    def __init__(self, data_queue, cmd_queue, switch):
        super().__init__(daemon=False)
        self.data_queue = data_queue
        self.cmd_queue = cmd_queue
        self.switch = switch
        self.active = True

        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


        self.frame_size = FRAME_SIZE
        self.recv_header = RECV_HEADER
        self.recv_struct_fmt = RECV_FMT

        self.send_header = SEND_HEADER
        self.send_struct_fmt = SEND_FMT


    def run(self):
        print("Waiting for Teensy READY...")
        while self.active:
            try:
                switch_cmd = self.switch.get_nowait()   # non-blocking
            except queue.Empty:
                switch_cmd = None

            if switch_cmd is not None and self.ser:
                try:
                    # ensure string -> bytes
                    self.ser.write((str(switch_cmd) + "\n").encode())
                except Exception as e:
                    print("Error writing switch command:", e)
            send_value = None
            try:
                # block briefly so we don't busy-wait
                send_value = self.cmd_queue.get(timeout=0.001)
            except queue.Empty:
                send_value = None

            if send_value is not None and self.ser:
                try:
                    tx_frame = struct.pack(self.send_struct_fmt, self.send_header, float(send_value))
                    self.ser.write(tx_frame)
                except Exception as e:
                    print("Error packing/sending frame:", e)

            if self.ser:
                try:
                    # read one byte and check header
                    byte = self.ser.read(1)
                    if byte == self.recv_header[:1]:
                        next_byte = self.ser.read(1)
                        if next_byte == self.recv_header[1:]:
                            data = self.ser.read(self.frame_size)
                            if len(data) == self.frame_size:
                                try:
                                    header, a, i, tau = struct.unpack(self.recv_struct_fmt, data)
                                    # push to data queue or print
                                    self.data_queue.put((a, i, tau))
                                    print(f"Recv: currentA={a:.3f}, actualI={i:.3f}, tau={tau:.3f}")
                                except struct.error:
                                    # malformed, skip
                                    continue
                except Exception as e:
                    # catch exceptions but don't crash the thread
                    print("Serial read error:", e)


    def stop(self):
        self.active = False
        try:
            self.ser.close()
        except:
            pass

class Controller():
    def __init__(self):
        super().__init__()
        self.data_queue = queue.Queue()
        self.cmd_queue = queue.Queue()
        self.switch = queue.Queue()
        self.serial_thread = SerialThread(self.data_queue, self.cmd_queue,self.switch)
        self.serial_thread.start()

    def InitUI(self):
        value2send = 0.0
        try:
            self.cmd_queue.put(value2send)
            value2send +=0.5
            if value2send >= 10.0:
                value2send = 0.0
        except:
            pass




            
        
 
# ---------------- Main -----------------
if __name__ == "__main__":
    Controller()