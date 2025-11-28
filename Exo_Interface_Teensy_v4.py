import struct
import serial
import threading
import queue
import sys
import time

# ----- Settings ---------------------
SERIAL_PORT = 'COM9'
BAUD_RATE = 1_000_000

SEND_FMT = '<Hf'           # header (unsigned short) + float
SEND_HEADER = 0x55AA

RECV_FMT = '<Hfff'
RECV_HEADER = b'\x55\xaa'
FRAME_SIZE = 14

TARGET_HZ = 500
PERIOD = 1.0 / TARGET_HZ  # 0.002 seconds

# ----- Serial Thread -----------------
class SerialThread(threading.Thread):
    def __init__(self, data_queue, cmd_queue, switch_queue):
        # do NOT set daemon=True so we can shutdown cleanly
        super().__init__(daemon=False)
        self.data_queue = data_queue
        self.cmd_queue = cmd_queue
        self.switch = switch_queue
        self.active = True

        # open serial in constructor (you may wrap in try/except)
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        except Exception as e:
            print("Failed to open serial port:", e)
            self.ser = None

        self.frame_size = FRAME_SIZE
        self.recv_header = RECV_HEADER
        self.recv_struct_fmt = RECV_FMT
        self.send_header = SEND_HEADER
        self.send_struct_fmt = SEND_FMT

    def run(self):
        print("SerialThread started. Waiting for Teensy READY...")
        while self.active:
            # 1) process textual switch commands (if any)
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

            # 2) process numeric command to send (float)
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

            # 3) read incoming frames (non-blocking-ish)
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

            # short sleep so loop isn't tight
            time.sleep(0.01)

        # cleanup when leaving loop
        print("SerialThread stopping, closing serial if open.")
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def stop(self):
        """Signal the thread to stop (call from main thread)."""
        self.active = False


class Controller:
    def __init__(self):
        self.data_queue = queue.Queue()
        self.cmd_queue = queue.Queue()
        self.switch = queue.Queue()
        self.serial_thread = SerialThread(self.data_queue, self.cmd_queue, self.switch)
        self.serial_thread.start()

    def init_send_cycle(self):
        """Example function that pushes values to cmd_queue periodically."""
        value2send = 0.0
        try:
            # put a value and update the variable (example)
            self.cmd_queue.put(value2send)
            value2send += 0.5
            if value2send >= 10.0:
                value2send = 0.0
        except Exception as e:
            print("Error queueing value:", e)

    def stop(self):
        # stop the serial thread and wait for it to finish
        self.serial_thread.stop()
        self.serial_thread.join(timeout=2)
        print("Controller stopped.")


# ---------------- Main -----------------
if __name__ == "__main__":
    ctrl = Controller()
    try:
        # example main loop: periodically send commands and read incoming data
        counter = 0.0
        nex_time = time.perf_counter()
        while True:
            # push a test value every 0.5 s
            ctrl.cmd_queue.put(counter)
            counter += 0.5
            if counter >= 10.0:
                counter = 0.0

            # read any received data without blocking
            try:
                while True:
                    a, i, tau = ctrl.data_queue.get_nowait()
                    # do something with the data
                    # (in GUI app you'd update the UI via signals)
            except queue.Empty:
                pass
            nex_time += PERIOD
            sleep_time = nex_time-time.perf_counter()

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                nex_time = time.perf_counter()

            # time.sleep(0.5)

    except KeyboardInterrupt:
        print("KeyboardInterrupt received - shutting down.")
    finally:
        # IMPORTANT: stop thread and join before exiting to avoid shutdown race
        ctrl.stop()
