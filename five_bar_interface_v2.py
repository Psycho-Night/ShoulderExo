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
        self.ser = serial.Serial('COM7', 115200, timeout=1)
        self.active = True

    def run(self):
        while self.active:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                # Adjust the regex pattern if necessary
                match = re.search(r"Time: (\d+) ms, Magnetic Encoder 1: ([\d\.-]+), Magnetic Encoder 2: ([\d\.-]+), Encoder1RAW: (\d+), Encoder2RAW: (\d+), Motor 1 Position: ([\d\.-]+), Motor 2 Position: ([\d\.-]+), Deflection 1: ([\d\.-]+), Deflection 2: ([\d\.-]+), Reference 1: ([\d\.-]+), Reference 2: ([\d\.-]+), Error 1: ([\d\.-]+), Error 2: ([\d\.-]+), Integral Error 1: ([\d\.-]+), Integral Error 2: ([\d\.-]+)", line)
                if match:
                    time_ms = float(match.group(1))
                    encoder1_value = float(match.group(2))
                    encoder2_value = float(match.group(3))
                    encoder1_raw = int(match.group(4))
                    encoder2_raw = int(match.group(5))
                    angular_position1 = float(match.group(6))
                    angular_position2 = float(match.group(7))
                    control_1 = float(match.group(8))
                    control_2 = float(match.group(9))
                    reference1 = float(match.group(10))
                    reference2 = float(match.group(11))
                    error1 = float(match.group(12))
                    error2 = float(match.group(13))
                    integral_error1 = float(match.group(14))
                    integral_error2 = float(match.group(15))
                    self.data_queue.put((time_ms, encoder1_value, encoder2_value, encoder1_raw, encoder2_raw, angular_position1, angular_position2, control_1, control_2, reference1, reference2, error1, error2, integral_error1, integral_error2))
            except UnicodeDecodeError:
                pass
            except Exception as e:
                print("Error:", e)

    def stop(self):
        self.active = False
        self.ser.close()


class SerialChartApp(QMainWindow):
    def __init__(self, data_queue):
        super().__init__()
        self.data_queue = data_queue
        self.initUI()
        self.serial_thread = SerialThread(data_queue)
        self.serial_thread.start()

    def initUI(self):
        self.setWindowTitle("GUI for Motor Control")
        self.resize(1000, 800)
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

        # Setup plots in a 2x2 grid
        self.setup_plots()

        # Timer setup for updating plots
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def setup_plots(self):
        # Configure the combined plots for Motor 1
        self.motor1Plot = self.graphWidget.addPlot(0, 0, title="Motor 1: Encoder & Hall Sensor Data")
        self.x_encoder1, self.y_encoder1 = [], []
        self.x_position1, self.y_position1 = [], []
        self.x_reference1, self.y_reference1 = [], []
        self.plot_encoder1 = self.motor1Plot.plot(self.x_encoder1, self.y_encoder1, pen='g', name='Magnetic Encoder')
        self.plot_position1 = self.motor1Plot.plot(self.x_position1, self.y_position1, pen='r', name='Hall Sensor')
        self.plot_reference1 = self.motor1Plot.plot(self.x_reference1, self.y_reference1, pen='y', name='Reference Signal')

        self.motor1Plot.setLabel('left', "Values")
        self.motor1Plot.setLabel('bottom', "Time", units='s')

        # Add custom legend
        legend = LegendItem(offset=(30, 30))  # offset from top-left corner in pixels
        legend.setParentItem(self.motor1Plot.graphicsItem())  # set legend to be part of the plot
        legend.addItem(self.plot_encoder1, 'Magnetic Encoder')
        legend.addItem(self.plot_position1, 'Hall Sensor')
        legend.addItem(self.plot_reference1, 'Reference Signal')

        # Motor 2 plots
        self.motor2Plot = self.graphWidget.addPlot(0, 1, title="Motor 2: Encoder & Hall Sensor Data")
        self.x_encoder2, self.y_encoder2 = [], []
        self.x_position2, self.y_position2 = [], []
        self.x_reference2, self.y_reference2 = [], []
        self.plot_encoder2 = self.motor2Plot.plot(self.x_encoder2, self.y_encoder2, pen='g', name='Magnetic Encoder')
        self.plot_position2 = self.motor2Plot.plot(self.x_position2, self.y_position2, pen='r', name='Hall Sensor')
        self.plot_reference2 = self.motor2Plot.plot(self.x_reference2, self.y_reference2, pen='y', name='Reference Signal')

        self.motor2Plot.setLabel('left', "Values")
        self.motor2Plot.setLabel('bottom', "Time", units='s')

        # Add custom legend
        legend2 = LegendItem(offset=(30, 30))  # offset from top-left corner in pixels
        legend2.setParentItem(self.motor2Plot.graphicsItem())  # set legend to be part of the plot
        legend2.addItem(self.plot_encoder2, 'Magnetic Encoder')
        legend2.addItem(self.plot_position2, 'Hall Sensor')
        legend2.addItem(self.plot_reference2, 'Reference Signal')

        # Encoder raw plots
        self.encoderrawPlot1 = self.graphWidget.addPlot(1, 0, title="Raw readings of Motor encoder 1")
        self.x_encoder1raw, self.y_encoder1raw = [], []
        self.plot_encoder1raw = self.encoderrawPlot1.plot(self.x_encoder1raw, self.y_encoder1raw, pen='g')
        self.encoderrawPlot1.setLabel('left', "Reading")
        self.encoderrawPlot1.setLabel('bottom', "Time", units='s')

        self.encoderrawPlot2 = self.graphWidget.addPlot(1, 1, title="Raw readings of Motor encoder 2")
        self.x_encoder2raw, self.y_encoder2raw = [], []
        self.plot_encoder2raw = self.encoderrawPlot2.plot(self.x_encoder2raw, self.y_encoder2raw, pen='g')
        self.encoderrawPlot2.setLabel('left', "Reading")
        self.encoderrawPlot2.setLabel('bottom', "Time", units='s')
 
        # Control signal plots
        self.controlPlot1 = self.graphWidget.addPlot(2, 0, title="Deflection Motor 1")
        self.x_control1, self.y_control1 = [], []
        self.plot_control1 = self.controlPlot1.plot(self.x_control1, self.y_control1, pen='g')
        self.controlPlot1.setLabel('left', "Control Value")
        self.controlPlot1.setLabel('bottom', "Time", units='s')

        self.controlPlot2 = self.graphWidget.addPlot(2, 1, title="Deflection Motor 2")
        self.x_control2, self.y_control2 = [], []
        self.plot_control2 = self.controlPlot2.plot(self.x_control2, self.y_control2, pen='g')
        self.controlPlot2.setLabel('left', "Control Value")
        self.controlPlot2.setLabel('bottom', "Time", units='s')

    def update_plot(self):
        while not self.data_queue.empty():
            data = self.data_queue.get()
            if len(data) < 15:
                print("Warning: Incomplete data tuple found and skipped.")
                continue
            time_sec = data[0] / 1000.0
            self.update_data_plot(time_sec, data[1:],
                                  [self.x_encoder1, self.y_encoder1, self.plot_encoder1],
                                  [self.x_encoder2, self.y_encoder2, self.plot_encoder2],
                                  [self.x_encoder1raw, self.y_encoder1raw, self.plot_encoder1raw],
                                  [self.x_encoder2raw, self.y_encoder2raw, self.plot_encoder2raw],
                                  [self.x_position1, self.y_position1, self.plot_position1],
                                  [self.x_position2, self.y_position2, self.plot_position2],
                                  [self.x_control1, self.y_control1, self.plot_control1],
                                  [self.x_control2, self.y_control2, self.plot_control2],
                                  [self.x_reference1, self.y_reference1, self.plot_reference1],
                                  [self.x_reference2, self.y_reference2, self.plot_reference2])

    def update_data_plot(self, time_sec, values, *args):
        for (x, y, plot), value in zip(args, values):
            x.append(time_sec)
            y.append(value)
            plot.setData(x, y)

    def save_data(self):
        filename = self.filenameEdit.text()
        original_filename = filename
        file_extension = '.csv'
        counter = 1

        # Define the path to the new folder on the desktop
        desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
        folder_path = os.path.join(desktop_path, "Fivebar_experiments")
        
        # Create the folder if it doesn't exist
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        # Check if the file already exists and find a unique filename by appending a number
        full_filename = os.path.join(folder_path, f"{filename}{file_extension}")
        while os.path.exists(full_filename):
            filename = f"{original_filename}_{counter}"
            full_filename = os.path.join(folder_path, f"{filename}{file_extension}")
            counter += 1

        # Prepare header
        header = "Time(s), Magnetic Encoder 1, Magnetic Encoder 2, Encoder1RAW, Encoder2RAW, Motor 1 Position (Degrees), Motor 2 Position (Degrees), Deflection Motor 1, Deflection Motor 2, Reference 1, Reference 2, Error 1, Error 2, Integral Error 1, Integral Error 2"

        # Ensure all lists have the same length by padding shorter lists with None
        max_len = max(len(self.x_encoder1), len(self.y_encoder1), len(self.y_encoder2), len(self.y_encoder1raw), len(self.y_encoder2raw), len(self.y_position1), len(self.y_position2), len(self.y_control1), len(self.y_control2), len(self.y_reference1), len(self.y_reference2))
        padded_lists = [lst + [None] * (max_len - len(lst)) for lst in [self.x_encoder1, self.y_encoder1, self.y_encoder2, self.y_encoder1raw, self.y_encoder2raw, self.y_position1, self.y_position2, self.y_control1, self.y_control2, self.y_reference1, self.y_reference2]]

        # Save the data to the unique filename
        with open(full_filename, 'w') as file:
            file.write(header + "\n")  # Write the header to the file
            for data in zip(*padded_lists):
                file.write(", ".join([str(item) if item is not None else '' for item in data]) + "\n")
        
        self.statusLabel.setText(f"Data saved to {full_filename}")


    def closeEvent(self, event):
        self.serial_thread.stop()
        self.serial_thread.join()  # Ensure the thread has finished execution
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    data_queue = queue.Queue()
    window = SerialChartApp(data_queue)
    window.show()
    sys.exit(app.exec_())
