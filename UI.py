from PyQt6 import QtWidgets
from PyQt6.QtGui import QVector3D, QQuaternion, QPixmap
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot, QTimer, Qt
import pyqtgraph as pg
from pyqtgraph import functions as fn
import pyqtgraph.opengl as gl
from OpenGL.GL import glEnable, glBlendFunc, glHint, glBegin, glEnd, glVertex3f, glColor4f, GL_LINES
import numpy as np
from lib.packet_stream_serial import *
from lib.ekf import EkfWrapper
from lib.calibrate import *
from serial.tools import list_ports
import time
import math

PLOT_BACKGROUND = "#141729"
CALIB_DATAPOINTS = 2000
CALIB_FREQUENCY = 200

current_time = time.localtime()
formatted_time = time.strftime("%Y-%m-%d_%H-%M-%S", current_time)

class PacketReader(QThread):
    sensorPacketReceived = pyqtSignal(list)
    gpsPacketReceived    = pyqtSignal(list)
    disconnected         = pyqtSignal()
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
            try:
                packet_type, packet = self.radio_serial.read_packet()
            except:
                self.disconnected.emit()
                return
            if packet_type == b'\x0b':
                self.sensorPacketReceived.emit(packet)
            if packet_type == b'\xca':
                self.gpsPacketReceived.emit(packet)
    
    def stop(self):
        self.radio_serial.stop()

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
    
    def update_plot(self, point):
        self.data[:-1] = self.data[1:]
        self.data[-1] = point
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

class GLMeshItem(gl.GLGraphicsItem.GLGraphicsItem):
    """
    **Bases:** :class:`GLGraphicsItem <pyqtgraph.opengl.GLGraphicsItem.GLGraphicsItem>`
    
    Displays a wire-frame grid. 
    """
    
    def __init__(self, mesh, size=None, color=(255, 255, 255, 76.5), antialias=True, glOptions='translucent', parentItem=None):
        super().__init__(parentItem=parentItem)
        self.setGLOptions(glOptions)
        self.antialias = antialias
        self.mesh = mesh
        if size is None:
            size = QVector3D(20,20,1)
        self.setSize(size=size)
        self.setColor(color)
    
    def setSize(self, x=None, y=None, z=None, size=None):
        """
        Set the size of the axes (in its local coordinate system; this does not affect the transform)
        Arguments can be x,y,z or size=QVector3D().
        """
        if size is not None:
            x = size.x()
            y = size.y()
            z = size.z()
        self.__size = [x,y,z]
        self.update()
        
    def size(self):
        return self.__size[:]

    def setColor(self, color):
        """Set the color of the grid. Arguments are the same as those accepted by functions.mkColor()"""
        self.__color = fn.mkColor(color)
        self.update()

    def color(self):
        return self.__color

    def paint(self):
        self.setupGLState()
        
        # if self.antialias:
        #     glEnable(GL_LINE_SMOOTH)
        #     glEnable(GL_BLEND)
        #     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        #     glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
            
        glBegin( GL_LINES )
        
        x,y,z = self.size()
        glColor4f(*self.color().getRgbF())
        counter = 0
        for x in self.mesh:
            if counter % 3 == 0:
                glVertex3f(x[0], x[1], x[2])
            counter += 1
        
        glEnd()

def load_obj_to_numpy(file_path):
    vertices = []

    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('v '):
                parts = line.split()
                x, y, z = map(float, parts[1:4])
                vertices.append([x, y, z])
    vertices_array = np.array(vertices)

    return vertices_array

class ShartWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        #self._init_estimation()
        self._init_ui()
        self.kalman = EkfWrapper()
        self._kalman_uninitialized = True
        self._sensor_packet = None
        self._gps_packet    = None
        self._baseline_pressure = 101325
        self._last_timestamp = 0
        self._packets_missed = 0
        self._calibrated = False
        self._calib_packets_read = 0
        self._calib_data_arr = np.empty((CALIB_DATAPOINTS, 6))
        self._calib_time_arr = np.empty(CALIB_DATAPOINTS)
        self._calib_params = [
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

        # self.orient_window = QtWidgets.QWidget()
        # self.mdi_area.addSubWindow(self.orient_window)
        # self.orient_window.show()

        # self.altitude_window = QtWidgets.QWidget()
        # self.mdi_area.addSubWindow(self.altitude_window)
        # self.altitude_window.show()

        # self.imu_window = QtWidgets.QWidget()#QtWidgets.QMdiSubWindow()
        # #self.imu_window.setWidget(QtWidgets.QWidget())
        # self.mdi_area.addSubWindow(self.imu_window, QtCore.Qt.WindowType.WindowMinMaxButtonsHint)
        # self.imu_window.show()  

        self.showSensorData = True
        self.showOrientation = True
        self.showPosition = False

        self._setup_header()
        self._setup_side_bar()
        self._setup_3d_views()
        self._setup_altitude()
        self._setup_sensor_plots()
       
        #self._setup_pos_plots()
        

        self.setCentralWidget(self.centralWidget)

        # QTimer controlling update rate of all the plots (for now they all share on rate)
        self.ui_update_timer = QTimer()
        self.ui_update_timer.timeout.connect(self._update_ui)
        self.ui_update_timer.setInterval(50)
        self.ui_update_timer.start()

    def _toggle_all(self):
        self.showSensorData = not self.showSensorData

    def _reset_ekf(self):
        self._kalman_uninitialized = True # this triggers reinitialization in the process_packet() function

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
        self.timer_widget = QtWidgets.QLabel("00:00:00.000")
        self.timer_widget.setStyleSheet("font-size: 25px; font-weight: bold; padding: 5px; text-align: center;")
        self.side_bar.addWidget(self.timer_widget)
        section_header_style = "font-size: 15px; font-weight: bold; padding: 5px; text-align: center;"
        sensor_status_view = QtWidgets.QVBoxLayout()
        sensor_status_view.setContentsMargins(10, 10, 10, 10) 
        status_section_label = QtWidgets.QLabel("Component Checks")
        status_section_label.setStyleSheet(section_header_style)
        sensor_status_view.addWidget(status_section_label)

        status_names = ["ICM20948 (Mag)", "BMP390 (Baro)", "ADXL375 (Accel)",  "LSM6DSO32 (IMU)", "SD Card","Black Powder"]
        self.status_labels = []
        
        for i in range(6):
            status_label = QtWidgets.QLabel(status_names[i])
            status_label.setStyleSheet("color: red;")
            sensor_status_view.addWidget(status_label)
            self.status_labels.append(status_label)
        self.side_bar.addLayout(sensor_status_view, stretch=1)
        kill_button = QtWidgets.QPushButton("MURDER")
        kill_button.clicked.connect(self._stop)
        self.side_bar.addWidget(kill_button)

        gps_view = QtWidgets.QVBoxLayout()
        gps_view_label = QtWidgets.QLabel("GPS Data")
        gps_view_label.setStyleSheet(section_header_style)
        self.coords = QtWidgets.QLabel()
        gps_view.addWidget(gps_view_label)
        gps_view.addWidget(self.coords)
        self.side_bar.addLayout(gps_view, stretch=1)

        kalman_view = QtWidgets.QVBoxLayout()
        kalman_view_label = QtWidgets.QLabel("Kalman Filter Data")
        kalman_view_label.setStyleSheet(section_header_style)
        self.blah = QtWidgets.QLabel()
        kalman_view.addWidget(kalman_view_label)
        kalman_view.addWidget(self.blah)
        self.side_bar.addLayout(kalman_view, stretch=1)

    def _setup_sensor_plots(self):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        widget.setLayout(layout)
        sensor_names = ["Accelerometer (m/s^2)", "Gyroscope (rad/s)", "Magnetometer (uT)"]
        #self.imu_window.setLayout(layout)
        self.sensor_plot_widgets = [SensorPlot3Axes(self, sensor_names[i]) for i in range(3)]
        for plot_widget in self.sensor_plot_widgets:
            layout.addWidget(plot_widget)
        self.mdi_area.addSubWindow(widget, Qt.WindowType.WindowMinMaxButtonsHint)

    def _setup_pos_plots(self):
        layout = QtWidgets.QVBoxLayout()
        self.pos_plot_widgets = [PosPlot(self) for _ in range(3)]
        for plot_widget in self.pos_plot_widgets:
            layout.addWidget(plot_widget)
        #self.mdi_area.addSubWindow(layout)
        #self.screen.addLayout(self.layout)
        
    def _setup_3d_views(self):
        self.RotView = gl.GLViewWidget()
        self.RotView.setBackgroundColor(PLOT_BACKGROUND)


        self.xgrid = gl.GLGridItem()
        self.xgrid = GLMeshItem(load_obj_to_numpy("assets/bunny.obj"))
        self.xgrid.scale(0.2,0.2,0.2)
        self.rotAxes = gl.GLAxisItem(parentItem=self.xgrid, glOptions='additive')
        self.rotAxes.setSize(10,10,10)
        self.RotView.addItem(self.xgrid)

        
        self.RotView.setMinimumSize(300, 300)
        
        self.last_quat = QQuaternion(0,0,1,0) 

        self.angle_label = QtWidgets.QLabel(parent=self.RotView)
        self.angle_label.setStyleSheet("font-family: 'Consolas';font-size: 14px")
        self.angle_label.setGeometry(30, 30, 100, 88)

        reset_orientation_button = QtWidgets.QPushButton("Reset Orientation", parent=self.RotView)
        reset_orientation_button.clicked.connect(self._reset_orientation)
        #reset_orientation_button.setGeometry()

        self.mdi_area.addSubWindow(self.RotView, Qt.WindowType.WindowMinMaxButtonsHint)

    def _reset_orientation(self):
        self.xgrid.resetTransform()
        self.xgrid.scale(0.2,0.2,0.2)

    def _setup_altitude(self):
        self.altitude_plot = SensorPlot(self, "Altitude (m)")
        self.mdi_area.addSubWindow(self.altitude_plot, Qt.WindowType.WindowMinMaxButtonsHint)
    
    def _setup_status(self):
        self.layout = QtWidgets.QVBoxLayout()
        self.pos_plot_widgets = [PosPlot(self) for _ in range(3)]
        for plot_widget in self.pos_plot_widgets:
            self.side_bar.addWidget(plot_widget)

    def _get_available_com_ports(self):
        ports = list_ports.comports()
        return ["None"] + [port.device for port in ports]

    def _on_com_port_selected(self, com_port):
        if hasattr(self, 'packet_reader') and self.packet_reader.isRunning():
            self.packet_reader.terminate()
            self.packet_reader.wait()
            self.packet_reader.radio_serial.close_port()
            
        self._reset_ekf()
        if (com_port == "None"):
            return

        if not self._calibrated:
            msg = QtWidgets.QMessageBox()
            msg.setIconPixmap(QPixmap("assets/lala.png").scaled(50, 50, transformMode=Qt.TransformationMode.SmoothTransformation))
            msg.setWindowTitle("IMU Calibration Options")
            msg.setText("\"Proceed with calibration or load from a file?\"")
            msg.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No)
            msg.setDefaultButton(QtWidgets.QMessageBox.StandardButton.Yes)
            yes_button = msg.button(QtWidgets.QMessageBox.StandardButton.Yes)
            no_button = msg.button(QtWidgets.QMessageBox.StandardButton.No)
            
            yes_button.setText("Start Calibration")
            no_button.setText("Load from File")
            response = msg.exec()
            if response == QtWidgets.QMessageBox.StandardButton.Yes:
                self._calibrated = False
                
            elif response == QtWidgets.QMessageBox.StandardButton.No:
                
                file_dialog = QtWidgets.QFileDialog(self)
                file_dialog.setFileMode(QtWidgets.QFileDialog.FileMode.ExistingFile)  
                file_dialog.setNameFilter("Calibration Files (*.npz)")
                file_dialog.setViewMode(QtWidgets.QFileDialog.ViewMode.List)

                if file_dialog.exec():
                    file_path = file_dialog.selectedFiles()[0]
                    calib_data = np.load(file_path)
                    self._calib_params[0] = calib_data['aC']
                    self._calib_params[1] = calib_data['ab']
                    self._calib_params[2] = calib_data['gC']
                    self._calib_params[3] = calib_data['gb']
                    self._calib_params[4] = calib_data['Rm']
                
                self._calibrated = True
            
        self.packet_reader = PacketReader(com_port)
        self.packet_reader.sensorPacketReceived.connect(self.process_sensor_packet)
        self.packet_reader.gpsPacketReceived.connect(self.process_gps_packet)
        self.packet_reader.disconnected.connect(self.handle_disconnect)
        self.packet_reader.start()

    def _stop(self):
        if hasattr(self, "packet_reader") and self.packet_reader:
            self.packet_reader.stop()

    @pyqtSlot()
    def handle_disconnect(self):
        self.com_port_combo.setCurrentText("None")

    @pyqtSlot(list)
    def process_sensor_packet(self, packet):
        self._sensor_packet = list(packet) #make packet available to UI for plotting
        self._sensor_packet[0] += self.packet_reader.radio_serial.overflows * 4294967295
        self._sensor_packet[1:4] = calibrate_accelerometer_single(convertRawAcc(*packet[1:4]), self._calib_params[0], self._calib_params[1])
        self._sensor_packet[4:7] = calibrate_gyroscope_single(convertRawGyr(*packet[4:7]), self._calib_params[2], self._calib_params[3], self._calib_params[4])
        # approximate missed packets using timestamp differences
        self._packets_missed += max(0, int((self._sensor_packet[0] - self._last_timestamp) / 4800) - 1)
        self._last_timestamp = self._sensor_packet[0]
        print(self._packets_missed)
        #self.sensor_packet[12:15] = packet[4:7] * 0.48069 # raw adxl to m/s^2
        # Handle calibration in its entirety right here
        # Yes this is ugly, too much unrelated code in this function, but no overhead for regular execution (just an extra branch instr in theory)
        if not self._calibrated:
            if self._calib_packets_read < CALIB_DATAPOINTS:
                self._calib_time_arr[self._calib_packets_read] = self._sensor_packet[0]
                self.calib_data_arr[self._calib_packets_read] = self._sensor_packet[1:4] + self._sensor_packet[4:7]
                self._calib_packets_read += 1
                return
            
            self.packet_reader.pause()
            # self.packet_reader.setPriority(QtCore.QThread.Priority.IdlePriority)
            
            reg_intervals = np.arange(self._calib_time_arr[0], self._calib_time_arr[-1], 1e6 / CALIB_FREQUENCY)
            y_interpolated = np.zeros((len(reg_intervals), self._calib_data_arr.shape[1]))

            # for each column, interpolate the data based on our specified intervals
            for i in range(self._calib_data_arr.shape[1]):
                y_interpolated[:, i] = np.interp(reg_intervals, self._calib_time_arr, self._calib_data_arr[:, i])
            #print(self.packet_reader.radio_serial.overflows)
            self._calib_params = list(all_calib_params(y_interpolated, CALIB_FREQUENCY))
            #print(self.calib_params)
            np.savez("calib/" + formatted_time, aC=self._calib_params[0], ab=self._calib_params[1], gC=self._calib_params[2], gb=self._calib_params[3], Rm=self._calib_params[4])
            self._calibrated = True
            self.packet_reader.flush()
            self.packet_reader.unpause()
            # self.packet_reader.setPriority(QtCore.QThread.Priority.NormalPriority)
            return

        if self._kalman_uninitialized:
            self.kalman.begin(packet[0])
            self._kalman_uninitialized = False
        # Push data and update kalman filter
        self.kalman.setIMU(packet[0], np.array(self._sensor_packet[4:7], dtype=np.float32)[:,np.newaxis], np.array(self._sensor_packet[1:4], dtype=np.float32)[:,np.newaxis])
        self.kalman.setMag(packet[0], np.array(packet[7:10], dtype=np.float32)[:,np.newaxis] / 100) # divide by 100 to convert from uT to Gauss
        self.kalman.setBaro(packet[0], packet[11]) # pass raw pressure data in hPa here
        self.kalman.update() # update the filter on the IMU cycle, as in the PX4-EKF tests

    @pyqtSlot(list)
    def process_gps_packet(self, packet):
            #vel = np.array(self.kalman.getVelocity()).squeeze()
        self._gps_packet = list(packet)
        self._gps_packet[0] += self.packet_reader.radio_serial.overflows * 4294967295
        if not self._kalman_uninitialized:
            self.kalman.setGPS(*self._gps_packet)
            #self.kalman.setGPS(packet[0], 407000000,-740000000, 30000,0,0,0,0,0,0,0,0,16,3,0,1)

    def _update_ui(self):
        if not self._sensor_packet:
            return
        sec_from_micro = self._sensor_packet[0]/1e6
        hours, rem = divmod(sec_from_micro, 3600)
        minutes, seconds = divmod(rem, 60)
        self.timer_widget.setText("{:0>2}:{:0>2}:{:0>6.3f}".format(int(hours),int(minutes),seconds))

        if self.showSensorData:
            self.sensor_plot_widgets[0].update_plot(self._sensor_packet[1:4])
            self.sensor_plot_widgets[1].update_plot(self._sensor_packet[4:7])
            self.sensor_plot_widgets[2].update_plot(self._sensor_packet[7:10])
        if self.showPosition:
            pos = np.array(self.kalman.getPosition()).squeeze()
            self.pos_plot_widgets[0].update_plot(pos[1],  pos[0])
            self.pos_plot_widgets[1].update_plot(pos[0], -pos[2]) # note we negate the Down component for intuitive plots
            self.pos_plot_widgets[2].update_plot(pos[1], -pos[2])
        if self.showOrientation:
            quat = np.array(self.kalman.getQuaternion()).squeeze()
            curr = QQuaternion(*quat)
            self.xgrid.transform().rotate((self.last_quat.inverted()*curr)) # get the delta quaternion (only Transform3D object takes quaternion rotation)
            self.xgrid.update()
            self.last_quat = curr
            euler = self.last_quat.toEulerAngles()
            pitch = np.degrees(euler.x()) % 360
            roll =  np.degrees(euler.y()) % 360
            yaw =   np.degrees(euler.z()) % 360

            # Update label text
            self.angle_label.setText(f"Roll:  {roll:>.2f}°\nPitch: {pitch:>.2f}°\nYaw:   {yaw:>.2f}°")

        self.altitude_plot.update_plot(self._press_to_alt(self._sensor_packet[11]))
        
        for i in range(6): 
            status = self._sensor_packet[15] & (1 << i)
            if status:
                self.status_labels[i].setStyleSheet("color: green;")
            else:
                self.status_labels[i].setStyleSheet("color: red;")

        if not self._gps_packet:
            return
        
        self.coords.setText(f"lat: {self._gps_packet[1] / 1e7: .6f}\nlon: {self._gps_packet[2] / 1e7: .6f}")
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

    def _press_to_alt(self, press: float):
        return 44330 * (1.0 - math.pow(press /
                                        self._baseline_pressure, 0.1903))

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
            background-color: #22222f;
            color: #aaaaaa;
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