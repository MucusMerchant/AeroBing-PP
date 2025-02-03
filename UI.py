from PyQt6 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
from lib.packet_stream_serial import *#import PacketStream, PACKET_SPEC
from lib.ekf import EkfWrapper
from lib.calibrate import *
from serial.tools import list_ports
import time

PLOT_BACKGROUND = "#141729"
CALIB_DATAPOINTS = 2000
CALIB_FREQUENCY = 200

current_time = time.localtime()
formatted_time = time.strftime("%Y-%m-%d_%H-%M-%S", current_time)

class PacketReader(QtCore.QThread):
    sensorPacketReceived = QtCore.pyqtSignal(list)
    gpsPacketReceived    = QtCore.pyqtSignal(list)
    def __init__(self, com_port="COM8"):
        super().__init__()
        self.radio_serial = PacketStream(com_port, 230400, "data/" + formatted_time + ".poop")
        self.radio_serial.open_port()
        self.radio_serial.start()
        self.paused = False

    def flush(self):
        self.radio_serial.serial_bus.reset_input_buffer()

    def run(self):
        while True:
            if self.paused:
                time.sleep(5)
                continue
            packet_type, packet = self.radio_serial.read_packet()
            if packet_type == b'\x0b':
                self.sensorPacketReceived.emit(packet)
            if packet_type == b'\xca':
                self.gpsPacketReceived.emit(packet)
    
    def pause(self):
        self.paused = True

    def unpause(self):
        self.paused = False

class SensorPlot(pg.PlotWidget):
    def __init__(self, parent, title):
        super().__init__(parent)
        self.setMouseEnabled(x=False, y=False)
        self.getAxis('bottom').setVisible(False)
        self.setMinimumSize(400,100)
        self.setTitle(title)
        self.setBackground(PLOT_BACKGROUND)
        self.time = np.arange(250)
        self.data = np.zeros(250)
        self.color = 'r'
        self.line = self.plot(self.time, self.data, pen=self.color)
    
    def update_plot(self, points):
        self.data[-1] = self.data[1:]
        self.data[-1] = points
        self.line.setData(self.time, self.data, _callSync='off')

class SensorPlot3Axes(pg.PlotWidget):
    def __init__(self, parent, title):
        super().__init__(parent)
        self.setMouseEnabled(x=False, y=False)
        self.getAxis('bottom').setVisible(False)
        self.addLegend()
        self.setMinimumSize(400,100)
        self.setTitle(title)
        self.setBackground(PLOT_BACKGROUND)
        self.time = np.arange(250)
        self.data = np.zeros((3, 250))
        self.colors = ['r', 'g', '#845ae6']
        self.names = ['X', 'Y', 'Z']
        self.lines = [self.plot(self.time, self.data[i], pen=self.colors[i], name=self.names[i]) for i in range(3)]
    
    def update_plot(self, points):
        for i in range(3):
            self.data[i, :-1] = self.data[i, 1:]
            self.data[i, -1] = points[i]
            self.lines[i].setData(self.time, self.data[i], _callSync='off')

class PosPlot(pg.PlotWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.setMouseEnabled(x=False, y=False)
        self.addLegend()
        self.setMinimumSize(200,200)
        self.setAspectLocked()
        #self.alpha = np.linspace(0,255,200)
        #self.colors = [pg.mkColor([255,255,255,self.alpha[i]]) for i in range(200)]
        self.hor_data = np.zeros(200)
        self.ver_data = np.zeros(200)
        self.line = self.plot(self.hor_data, self.ver_data)
    
    def update_plot(self, hor, ver):
        for _ in range(3):
            self.hor_data[:-1] = self.hor_data[1:]
            self.ver_data[:-1] = self.ver_data[1:]
            self.hor_data[-1] = hor
            self.ver_data[-1] = ver
            self.line.setData(self.hor_data, self.ver_data, _callSync='off')

class ShartWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        #self._init_estimation()
        self._init_ui()
        self.sensor_packet = None
        self.gps_packet    = None
        self.count = 0
        self.calibrated = False
        self.calib_packets_read = 0
        self.calib_data_arr = np.empty((CALIB_DATAPOINTS, 6))
        self.calib_time_arr = np.empty(CALIB_DATAPOINTS)
        self.calib_params = [
            np.array([[1.0, 0.0, 0.0], 
                      [0.0, 1.0, 0.0], 
                      [0.0, 0.0, 1.0]]),
            np.array([0.0, 0.0, 0.0]),
            np.array([[1.0, 0.0, 0.0], 
                      [0.0, 1.0, 0.0], 
                      [0.0, 0.0, 1.0]]),
            np.array([0.0, 0.0, 0.0]),
            np.eye(3)
        ]

    def _init_ui(self):
        # setup header and main hlayout
        self.centralWidget = QtWidgets.QWidget(self)
        self.screen = QtWidgets.QVBoxLayout(self.centralWidget)
        self.header = QtWidgets.QHBoxLayout()
        self.main_layout = QtWidgets.QHBoxLayout()
        self.side_bar = QtWidgets.QVBoxLayout()
        self.mdi_area = QtWidgets.QMdiArea()
        self.mdi_area.tileSubWindows()
        self.mdi_area.setBackground(0x0c0e19)
        self.screen.addLayout(self.header)
        self.screen.addLayout(self.main_layout)
        self.main_layout.addWidget(self.mdi_area)
        self.main_layout.addLayout(self.side_bar)

        self.orient_window = QtWidgets.QMdiSubWindow()
        self.orient_window.setWidget(QtWidgets.QWidget())
        self.mdi_area.addSubWindow(self.orient_window)
        self.orient_window.show()

        self.altitude_window = QtWidgets.QMdiSubWindow()
        self.altitude_window.setWidget(QtWidgets.QWidget())
        self.mdi_area.addSubWindow(self.altitude_window)
        self.altitude_window.show()

        self.imu_window = QtWidgets.QMdiSubWindow()
        self.imu_window.setWidget(QtWidgets.QWidget())
        #self.imu_window.setWindowFlags(self.imu_window.windowFlags() & ~QtCore.Qt.WindowType.WindowCloseButtonHint)
        self.mdi_area.addSubWindow(self.imu_window)
        self.imu_window.show()  

        self.showSensorData = True
        self.showOrientation = True
        self.showPosition = False

        self._setup_header()
        self._setup_side_bar()
        self._setup_3d_views()
        self._setup_sensor_plots()
        self._setup_altitude()
        #self._setup_pos_plots()
        

        self.setCentralWidget(self.centralWidget)

        # QTimer controlling update rate of all the plots (for now they all share on rate)
        self.ui_update_timer = QtCore.QTimer()
        self.ui_update_timer.timeout.connect(self._update_ui)
        self.ui_update_timer.setInterval(50)
        self.ui_update_timer.start()

    def _toggle_all(self):
        self.showSensorData = not self.showSensorData

    def _reset_ekf(self):
        self.kalman = None # this triggers reinitialization in the process_packet() function

    def _setup_header(self):
        header_label = QtWidgets.QLabel("SHART Telemetry Visualizer", self)
        header_label.setStyleSheet("font-size: 20px; font-weight: bold; padding: 20px; text-align: center;")

        self.com_port_combo = QtWidgets.QComboBox()
        self.com_port_combo.addItems(self._get_available_com_ports())
        self.com_port_combo.currentTextChanged.connect(self._on_com_port_selected)
        
        
        self.header.addWidget(header_label, stretch = 1)
        self.header.addWidget(QtWidgets.QLabel("Select COM Port:"), stretch=0)
        self.header.addWidget(self.com_port_combo)

    def _setup_side_bar(self):
        self.status_labels = []
        
        for i in range(8):
            status = (1 >> (7 - i)) & 1
            status_label = QtWidgets.QLabel(f"Sensor {i + 1} Status:")
            if status == 1:
                status_label.setStyleSheet("color: green;")
            else:
                status_label.setStyleSheet("color: red;")
            self.side_bar.addWidget(status_label)
            self.status_labels.append(status_label)

    def _setup_sensor_plots(self):
        layout = QtWidgets.QVBoxLayout()
        sensor_names = ["Accelerometer", "Gyroscope", "Magnetometer"]
        self.imu_window.widget().setLayout(layout)
        self.sensor_plot_widgets = [SensorPlot3Axes(self, sensor_names[i]) for i in range(3)]
        for plot_widget in self.sensor_plot_widgets:
            layout.addWidget(plot_widget)

    def _setup_pos_plots(self):
        self.layout = QtWidgets.QVBoxLayout()
        self.pos_plot_widgets = [PosPlot(self) for _ in range(3)]
        for plot_widget in self.pos_plot_widgets:
            self.layout.addWidget(plot_widget)
        #self.screen.addLayout(self.layout)
        
    def _setup_3d_views(self):
        self.RotView = gl.GLViewWidget()
        self.RotView.setBackgroundColor(PLOT_BACKGROUND)

        self.xgrid = gl.GLGridItem()
        self.xgrid.setSpacing(5,5,5)
        self.xgrid.scale(0.3,0.3,0.3)
        self.rotAxes = gl.GLAxisItem(parentItem=self.xgrid, glOptions='additive')
        self.rotAxes.setSize(10,10,10)
        self.RotView.addItem(self.xgrid)

        
        self.RotView.setMinimumSize(300, 300)
        
        self.last_quat = QtGui.QQuaternion(0,0,1,0) 

        self.angle_label = QtWidgets.QLabel(parent=self.RotView)
        self.angle_label.setStyleSheet("font-family: 'Consolas';font-size: 14px")
        self.angle_label.setGeometry(30, 30, 100, 88)

        self.orient_window.setWidget(self.RotView)

    def _reset_orientation(self):
        self.xgrid.resetTransform()
        self.xgrid.scale(0.3,0.3,0.3)

    def _setup_altitude(self):
        self.altitude_window.setWidget(SensorPlot(self, "Altitude"))
    
    def _setup_status(self):
        self.layout = QtWidgets.QVBoxLayout()
        self.pos_plot_widgets = [PosPlot(self) for _ in range(3)]
        for plot_widget in self.pos_plot_widgets:
            self.side_bar.addWidget(plot_widget)
        #self.screen.addLayout(self.layout)

    def _get_available_com_ports(self):
        """Returns a list of available COM ports"""
        ports = serial.tools.list_ports.comports()
        return ["None"] + [port.device for port in ports]

    def _on_com_port_selected(self, com_port):
        """Called when a COM port is selected by the user."""
        if hasattr(self, 'packet_reader') and self.packet_reader.isRunning():
            self.packet_reader.terminate()
            self.packet_reader.radio_serial.close_port()
        self._reset_ekf()
        if (com_port == "None"):
            return
        
        msg = QtWidgets.QMessageBox()
        msg.setIconPixmap(QtGui.QPixmap("lala.png"))
        msg.setWindowTitle("IMU Calibration Options")
        msg.setText("Proceed with calibration or load from a file?")
        msg.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No)
        msg.setDefaultButton(QtWidgets.QMessageBox.StandardButton.Yes)
        yes_button = msg.button(QtWidgets.QMessageBox.StandardButton.Yes)
        no_button = msg.button(QtWidgets.QMessageBox.StandardButton.No)

        yes_button.setText("Start Calibration")
        no_button.setText("Load from File")
        response = msg.exec()
        if response == QtWidgets.QMessageBox.StandardButton.Yes:
            self.calibrated = False
            
        elif response == QtWidgets.QMessageBox.StandardButton.No:
            
            file_dialog = QtWidgets.QFileDialog(self)
            file_dialog.setFileMode(QtWidgets.QFileDialog.FileMode.ExistingFile)  
            file_dialog.setNameFilter("Calibration Files (*.npz)")
            file_dialog.setViewMode(QtWidgets.QFileDialog.ViewMode.List)

            if file_dialog.exec():
                file_path = file_dialog.selectedFiles()[0]
                calib_data = np.load(file_path)
                self.calib_params[0] = calib_data['aC']
                self.calib_params[1] = calib_data['ab']
                self.calib_params[2] = calib_data['gC']
                self.calib_params[3] = calib_data['gb']
                self.calib_params[4] = calib_data['Rm']
            
            self.calibrated = True
            
        self.packet_reader = PacketReader(com_port)
        self.packet_reader.sensorPacketReceived.connect(self.process_sensor_packet)
        self.packet_reader.gpsPacketReceived.connect(self.process_gps_packet)
        self.packet_reader.start()

    @QtCore.pyqtSlot(list)
    def process_sensor_packet(self, packet):
        self.sensor_packet = list(packet) #make packet available to UI for plotting
        self.sensor_packet[0] += self.packet_reader.radio_serial.overflows * 4294967295
        self.sensor_packet[1:4] = calibrate_accelerometer_single(convertRawAcc(*packet[1:4]), self.calib_params[0], self.calib_params[1])
        self.sensor_packet[4:7] = calibrate_gyroscope_single(convertRawGyr(*packet[4:7]), self.calib_params[2], self.calib_params[3], self.calib_params[4])
        #self.sensor_packet[12:15] = packet[4:7] * 0.48069 # raw adxl to m/s^2
        # Handle calibration in its entirety right here
        # Yes this is ugly, too much unrelated code in this function, but no overhead for regular execution (just an extra branch instr in theory)
        if not self.calibrated:
            if self.calib_packets_read < CALIB_DATAPOINTS:
                self.calib_time_arr[self.calib_packets_read] = self.sensor_packet[0]
                self.calib_data_arr[self.calib_packets_read] = self.sensor_packet[1:4] + self.sensor_packet[4:7]
                self.calib_packets_read += 1
                return
            
            self.packet_reader.pause()
            # self.packet_reader.setPriority(QtCore.QThread.Priority.IdlePriority)
            
            reg_intervals = np.arange(self.calib_time_arr[0], self.calib_time_arr[-1], 1e6 / CALIB_FREQUENCY)
            y_interpolated = np.zeros((len(reg_intervals), self.calib_data_arr.shape[1]))

            # for each column, interpolate the data based on our specified intervals
            for i in range(self.calib_data_arr.shape[1]):
                y_interpolated[:, i] = np.interp(reg_intervals, self.calib_time_arr, self.calib_data_arr[:, i])
            print(self.packet_reader.radio_serial.overflows)
            self.calib_params = list(all_calib_params(y_interpolated, CALIB_FREQUENCY))
            print(self.calib_params)
            np.savez("calib/" + formatted_time, aC=self.calib_params[0], ab=self.calib_params[1], gC=self.calib_params[2], gb=self.calib_params[3], Rm=self.calib_params[4])
            self.calibrated = True
            self.packet_reader.flush()
            self.packet_reader.unpause()
            # self.packet_reader.setPriority(QtCore.QThread.Priority.NormalPriority)
            return

        if not self.kalman:
            self.kalman = EkfWrapper()
            self.kalman.begin(packet[0])
        # Push data and update kalman filter
        self.kalman.setIMU(packet[0], np.array(self.sensor_packet[4:7], dtype=np.float32)[:,np.newaxis], np.array(self.sensor_packet[1:4], dtype=np.float32)[:,np.newaxis])
        self.kalman.setMag(packet[0], np.array(packet[7:10], dtype=np.float32)[:,np.newaxis] / 100)
        self.kalman.setBaro(packet[0], packet[11]) # pass raw pressure data in hPa here
        self.kalman.update() # update the filter on the IMU cycle, as in the PX4-EKF tests

    @QtCore.pyqtSlot(list)
    def process_gps_packet(self, packet):
            #vel = np.array(self.kalman.getVelocity()).squeeze()
        self.gps_packet = list(packet)
        self.gps_packet[0] += self.packet_reader.radio_serial.overflows * 4294967295
        if self.kalman:
            self.kalman.setGPS(*self.gps_packet)
            #self.kalman.setGPS(packet[0], 407000000,-740000000, 30000,0,0,0,0,0,0,0,0,16,3,0,1)

    def _update_ui(self):
        if not self.sensor_packet:
            return
          
        if self.showSensorData:
            self.sensor_plot_widgets[0].update_plot(self.sensor_packet[1:4])
            self.sensor_plot_widgets[1].update_plot(self.sensor_packet[4:7])
            self.sensor_plot_widgets[2].update_plot(self.sensor_packet[7:10])
        if self.showPosition and self.kalman:
            pos = np.array(self.kalman.getPosition()).squeeze()
            self.pos_plot_widgets[0].update_plot(pos[1],  pos[0])
            self.pos_plot_widgets[1].update_plot(pos[0], -pos[2]) # note we negate the Down component for intuitive plots
            self.pos_plot_widgets[2].update_plot(pos[1], -pos[2])
        if self.showOrientation and self.kalman:
            quat = np.array(self.kalman.getQuaternion()).squeeze()
            curr = QtGui.QQuaternion(*quat)
            self.xgrid.transform().rotate((self.last_quat.inverted()*curr)) # get the delta quaternion (only Transform3D object takes quaternion rotation)
            self.xgrid.update()
            self.last_quat = curr
            euler = self.last_quat.toEulerAngles()
            pitch = np.degrees(euler.x()) % 360
            roll =  np.degrees(euler.y()) % 360
            yaw =   np.degrees(euler.z()) % 360

            # Update label text
            self.angle_label.setText(f"Roll:  {roll:>.2f}°\nPitch: {pitch:>.2f}°\nYaw:   {yaw:>.2f}°")
        
        for i in range(8): 
            status = (self.sensor_packet[15] >> (7 - i)) & 1 
            if status == 1:
                self.status_labels[i].setStyleSheet("color: green;")
            else:
                self.status_labels[i].setStyleSheet("color: red;")
        ## CHECK IN-air/is vehicle at rest flags!
        #print(np.array(self.kalman.ekf.get_innovation_test_status())) #IMPORTANT!
        #print(np.array(self.kalman.ekf.getOutputTrackingError()))
        #print(np.array(self.kalman.ekf.getAccelBias()))
        #print(np.array(self.kalman.ekf.velocity_covariances()))
    #TODO: make a GUI panel for all of this status stuff
        #print(self.kalman.ekf.get_mag_decl_deg())
        #print(self.kalman.ekf.global_position_is_valid())
        #print(self.kalman.ekf.control_status().gps)
        #print(self.kalman.ekf.warning_event_status().value)

if __name__ == "__main__":
    
    
    app = QtWidgets.QApplication([])
    main = ShartWindow()
    main.setStyleSheet("""
        QMainWindow {
            background-color: #10121f;
            color: #f0f0f0;
        }
        QWidget {
            background-color: #10121f;
            color: #dcdcdc;
        }
        QLabel {
            background-color: transparent;    
            color: #aaaaaa     
        }
        QMdiArea {
            background-color: #10121f;
            color: #dcdcdc;
        }
        QPushButton {
            background-color: #10121f;
            color: white;
            padding: 10px;
            border-radius: 5px;
        }
        
        QComboBox {
            background-color: #10121f;
            color: #ffffff;
            border: 1px solid #4db8ff;
        }
    """)
    main.show()
    app.exec()

#pip install qt-material to make this look better