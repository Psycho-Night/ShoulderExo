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
        ############## Change it to COM port #######################
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1) 
        self.active = True
    
    def run(self):
        while self.active:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                match = re.search(r"Time: (\d+)ms, Target Angle: ([\d\.-]+), Current Angle: ([\d\.-]+), Torque: (\d+)", line)
                if match:
                    time_val = float(match.group(1))
                    target = float(match.group(2))
                    current = float(match.group(3))
                    tau = float(match.group(4))
                    self.data_queue.put((time_val, target, current, tau))
            except UnboundLocalError:
                pass
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
        self.data_log = []

    def initUI(self):
        self.setWindowTitle("Serial Plotter")
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

        # Setup plots in 2x1 grid
        self.setup_plots()
        
        # Timer setup for updating plots
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)
        
        
    def setup_plots(self):            
        # Plot 1: Target vs Current Position
        self.positionPlot = self.graphWidget.addPlot(0, 0, title="Target vs Current Position")
        self.positionPlot.addLegend()
        self.x_target, self.y_target = [], []
        self.x_current, self.y_current = [], []
        self.plot_target = self.positionPlot.plot(self.x_target, self.y_target, pen='g', name='Target')
        self.plot_current = self.positionPlot.plot(self.x_current, self.y_target, pen='r', name='Current')

        self.positionPlot.setLabel('left', "Values")
        self.positionPlot.setLabel('bottom', "Time", units='s')
        # Add custom legend
        legend = LegendItem(offset=(30, 30))  # offset from top-left corner in pixels
        legend.setParentItem(self.positionPlot.graphicsItem())  # set legend to be part of the plot
        legend.addItem(self.plot_target, 'Target')
        legend.addItem(self.plot_current, 'Current')



        # Plot 2: Torque vs Time
        self.torquePlot = self.graphWidget.addPlot(1, 0, title="Torque ")
        self.torquePlot.addLegend()
        self.x_torque, self.y_torque = [], []
        self.plot_tourqe = self.torquePlot.plot(self.x_torque, self.y_torque, pen='g', name='Torque')

        self.torquePlot.setLabel('left', "Values")
        self.torquePlot.setLabel('bottom', "Time", units='s')
        # Add custom legend
        legend = LegendItem(offset=(30, 30))  # offset from top-left corner in pixels
        legend.setParentItem(self.torquePlot.graphicsItem())  # set legend to be part of the plot
        legend.addItem(self.plot_tourqe, 'Torque')


    def update_plots(self):
        while not self.data_queue.empty():
            data = self.data_queue.get()
            if len(data) < 3:
                print("Warning: Incomplete data tuple found and skipped.")
                continue
            time_sec = data[0]/1000.0
            self.update_data_plot(time_sec, data[1:],
                                  [self.x_target, self.y_target, self.plot_target],
                                  [self.x_current, self.y_current, self.plot_current],
                                  [self.x_torque, self.y_torque, self.plot_tourqe])

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
        folder_path = os.path.join(desktop_path, "Exo_Data")

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
        header = "Time(s), Reference, Current Position, Torque"

        # Ensure all lists have the same length by padding shorter lists with None
        max_len = max(len(self.x_target), len(self.y_target), len(self.y_torque))
        padded_lists = [lst + [None] * (max_len - len(lst)) for lst in [self.x_target, self.y_target, self.y_current, self.y_torque]]



        # Save the data to the unique filename
        with open(full_filename, 'w') as file:
            file.write(header + "\n")  # Write the header to the file
            for data in zip(*padded_lists):
                file.write(", ".join([str(item) if item is not None else '' for item in data]) + "\n")
        
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
