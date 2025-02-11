from PyQt6 import QtWidgets
from PyQt6.QtGui import QVector3D, QQuaternion, QPixmap, QIcon
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot, QTimer, Qt
import pyqtgraph as pg
from pyqtgraph import functions as fn
import pyqtgraph.opengl as gl
from OpenGL.GL import glBegin, glPointSize, glEnd, glVertex3f, glColor4f, glLineWidth, GL_LINES, GL_POINTS
import numpy as np
from lib.packet_stream_serial import *
from lib.ekf import EkfWrapper
from lib.calibrate import *
from serial.tools import list_ports
import time
import math


RADIO_BAUD_RATE = 115200
PLOT_BACKGROUND = "#141729"
CALIB_DATAPOINTS = 10000
CALIB_FREQUENCY = 200
FIX_TYPES = ["No fix", "Dead-reckoning only", "2D Fix", "3D Fix", "Epic", "Time only", "other (bad)"]
ADXL_32G = 65536 # or 2^16
BASELINE_PRESSURE = 101325

current_time = time.localtime()
formatted_time = time.strftime("%Y-%m-%d_%H-%M-%S", current_time)

class PacketReaderSerial(QThread):
    sensorPacketReceived = pyqtSignal(list)
    gpsPacketReceived    = pyqtSignal(list)
    disconnected         = pyqtSignal()
    def __init__(self, com_port="COM8"):
        super().__init__()
        self.radio_serial = PacketStream(com_port, RADIO_BAUD_RATE, "data/" + formatted_time + ".poop")
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
        glPointSize(3.5)
        glBegin( GL_POINTS )
        
        glColor4f(*self.color().getRgbF())
        
        counter = 0
        for x in self.mesh:
            if counter % 4 == 0:
                glVertex3f(x[0], x[1], x[2])
            counter += 1
        
        glEnd()

class GLBetterAxes(gl.GLAxisItem):
    """
    **Bases:** :class:`GLGraphicsItem <pyqtgraph.opengl.GLGraphicsItem.GLGraphicsItem>`
    
    Displays three lines indicating origin and orientation of local coordinate system. 
    
    """
    
    def __init__(self, size=None, antialias=False, glOptions='opaque', parentItem=None):
        super().__init__(size, antialias, glOptions, parentItem)
    
    def paint(self):

        #glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        #glEnable( GL_BLEND )
        #glEnable( GL_ALPHA_TEST )
        self.setupGLState()
        
        # if self.antialias:
        #     glEnable(GL_LINE_SMOOTH)
        #     glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        glLineWidth(6)
        glBegin( GL_LINES )
        
        x,y,z = self.size()
        glColor4f(0, 1, 0, .6)  # z is green
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, z)

        glColor4f(1, 1, 0, .6)  # y is yellow
        glVertex3f(0, 0, 0)
        glVertex3f(0, y, 0)

        glColor4f(1, 0, 0, .6)  # x is blue
        glVertex3f(0, 0, 0)
        glVertex3f(x, 0, 0)
        glEnd()

class GLViewWidgetNoMouse(gl.GLViewWidget):
    def __init__(self):
        super().__init__()
    def mousePressEvent(self, event):
        pass
    def mouseMoveEvent(self, event):
        pass
    def mouseReleaseEvent(self, event):
        pass

class QLabelPair(QtWidgets.QWidget):
    def __init__(self, static_text, initial_dynamic_text, parent = None):
        super().__init__(parent)

        self.static_label = QtWidgets.QLabel(static_text, self)
        self.dynamic_label = QtWidgets.QLabel(initial_dynamic_text, self)
    
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.static_label)
        layout.addWidget(self.dynamic_label)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.setLayout(layout)

    def setText(self, new_text):
        self.dynamic_label.setText(new_text)

    def setStyleSheet(self, stylesheet):
        self.static_label.setStyleSheet(stylesheet)
        self.dynamic_label.setStyleSheet(stylesheet)

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
        self._init_ui()
        self.kalman = EkfWrapper()
        self._kalman_uninitialized: bool = True
        self._sensor_packet = None
        self._gps_packet    = None
        self._using_adxl: bool = False
        self._baseline_pressure: int = BASELINE_PRESSURE
        self._last_timestamp: int = 0
        self._packets_missed: int = 0
        self._calibrated: bool = False
        self._calib_packets_read: int = 0
        self._calib_data_arr = np.empty((CALIB_DATAPOINTS, 6))
        self._calib_time_arr = np.empty(CALIB_DATAPOINTS)
        self._calib_points: int = CALIB_DATAPOINTS
        self._calib_save: bool = True
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

        self.showSensorData = True
        self.showOrientation = True
        self.showPosition = False

        self._setup_header()
        self._setup_side_bar()
        self._setup_3d_views()
        self._setup_altitude()
        self._setup_sensor_plots()

        self.setCentralWidget(self.centralWidget)

        # QTimer controlling update rate of all the plots (for now they all share on rate)
        self.ui_update_timer = QTimer()
        self.ui_update_timer.timeout.connect(self._update_ui)
        self.ui_update_timer.setInterval(100)
        self.ui_update_timer.start()

        self.rot_update_timer = QTimer()
        self.rot_update_timer.timeout.connect(self._update_3d)
        self.rot_update_timer.setInterval(200)
        self.rot_update_timer.start()

    def _toggle_all(self):
        self.showSensorData = not self.showSensorData

    def _reset_ekf(self):
        self._kalman_uninitialized = True # this triggers reinitialization in the process_packet() function

    def _setup_header(self):
        logo = QtWidgets.QLabel()
        logo.setStyleSheet("padding-bottom: 20px")
        logo.setPixmap(QPixmap("assets/aerologo.png").scaled(200, 60, transformMode=Qt.TransformationMode.SmoothTransformation))
        header_label = QtWidgets.QLabel("SHART Telemetry Visualizer", self)
        header_label.setStyleSheet("font-size: 38px; padding: 20px; text-align: center;")

        self.com_port_combo = QtWidgets.QComboBox()
        self.com_port_combo.addItems(self._get_available_com_ports())
        self.com_port_combo.currentTextChanged.connect(self._on_com_port_selected)
        
        self.header.addWidget(logo)
        self.header.addWidget(header_label, stretch = 1)
        self.header.addWidget(QtWidgets.QLabel("Select COM Port:"), stretch=0)
        self.header.addWidget(self.com_port_combo)

    def _setup_side_bar(self):
        self.timer_widget = QtWidgets.QLabel("00:00:00.000")
        self.timer_widget.setStyleSheet("font-size: 25px; font-weight: bold; padding: 5px; text-align: center;")
        self.side_bar.addWidget(self.timer_widget)

        self.calib_progress_bar = QtWidgets.QProgressBar()
        self.calib_progress_bar.setTextVisible(False)
        self.calib_progress_bar.setFixedWidth(170)
        self.calib_progress_bar.setValue(0)
        self.side_bar.addWidget(self.calib_progress_bar)

        missed_layout = QtWidgets.QHBoxLayout()
        self.missed_packets_num = QLabelPair("Lost packets: ", "0")
        self.reload_missed = QtWidgets.QPushButton()
        self.reload_missed.setStyleSheet("padding: 0; margin: 0; background-color: transparent")
        self.reload_missed.setFixedSize(17,17)
        icon = QIcon("assets/reload.png")
        self.reload_missed.setIcon(icon)
        self.reload_missed.clicked.connect(self._reset_missed)
        missed_layout.addWidget(self.missed_packets_num)
        missed_layout.addWidget(self.reload_missed, stretch = 0)
        self.side_bar.addLayout(missed_layout)

        section_header_style = "font-size: 15px; font-weight: bold; padding: 5px; text-align: center;"
        sensor_status_view = QtWidgets.QVBoxLayout()
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
        

        gps_view = QtWidgets.QVBoxLayout()
        gps_view_label = QtWidgets.QLabel("GPS Data")
        gps_view_label.setStyleSheet(section_header_style)
        self.lat = QLabelPair("Latitude: ", "0")
        self.lon = QLabelPair("Longitude: ", "0")
        self.sats = QLabelPair("Satellites: ", "0")
        self.alt = QLabelPair("Altitude (m): ", "0")
        self.fix = QLabelPair("Fix type: ", "No Fix")
        gps_view.addWidget(gps_view_label)
        gps_view.addWidget(self.lat)
        gps_view.addWidget(self.lon)
        gps_view.addWidget(self.sats)
        gps_view.addWidget(self.alt)
        gps_view.addWidget(self.fix)
    
        kalman_view = QtWidgets.QVBoxLayout()
        kalman_view_label = QtWidgets.QLabel("Kalman Filter Data")
        kalman_view_label.setStyleSheet(section_header_style)
        self.blah = QLabelPair("Global position valid: ", "False")
        self.kalman_attitude_valid = QLabelPair("Attitude valid: ", "False")
        self.kalman_pos_x = QLabelPair("Position N: ", "0")
        self.kalman_pos_y = QLabelPair("Position E: ", "0")
        self.kalman_pos_z = QLabelPair("Position D: ", "0")
        kalman_view.addWidget(kalman_view_label)
        kalman_view.addWidget(self.blah)
        kalman_view.addWidget(self.kalman_attitude_valid)
        kalman_view.addWidget(self.kalman_pos_x)
        kalman_view.addWidget(self.kalman_pos_y)
        kalman_view.addWidget(self.kalman_pos_z)

        missed_layout.setContentsMargins(10, 0, 10, 0) 
        sensor_status_view.setContentsMargins(10, 10, 10, 10) 
        gps_view.setContentsMargins(10, 10, 10, 10) 
        kalman_view.setContentsMargins(10, 10, 10, 10) 

        self.side_bar.addLayout(sensor_status_view, stretch=1)
        self.side_bar.addLayout(gps_view, stretch=1)
        self.side_bar.addLayout(kalman_view, stretch=1)

        kill_button = QtWidgets.QPushButton("End Current File")
        kill_button.clicked.connect(self._stop)
        ekf_reset_button = QtWidgets.QPushButton("Reset EKF")
        ekf_reset_button.clicked.connect(self._reset_ekf)
        self.side_bar.addWidget(kill_button)
        self.side_bar.addWidget(ekf_reset_button)

    def _setup_sensor_plots(self):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        widget.setLayout(layout)
        sensor_names = ["Accelerometer (m/s^2)", "Gyroscope (rad/s)", "Magnetometer (uT)"]
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
        self.RotView = GLViewWidgetNoMouse()
        self.RotView.setBackgroundColor(PLOT_BACKGROUND)


        self.xgrid = gl.GLGridItem()
        self.xgrid = GLMeshItem(load_obj_to_numpy("assets/bunny.obj"))
        self.xgrid.scale(0.16,0.16,0.16)
        self.rotAxes = GLBetterAxes(parentItem=self.xgrid, glOptions='additive')
        self.rotAxes.setSize(20,20,20)
        self.rotAxes.rotate(180,1,0,0)
        self.RotView.addItem(self.xgrid)
        
        self.RotView.setMinimumSize(300, 300)
        self.last_quat = QQuaternion(0,0,1,0) 
        self.roll_label = QLabelPair("Roll ", "0", parent=self.RotView)
        self.pitch_label = QLabelPair("Pitch ", "0", parent=self.RotView)
        self.yaw_label = QLabelPair("Yaw ", "0", parent=self.RotView)
        self.roll_label.setStyleSheet("font-family: 'Consolas';font-size: 14px")
        self.pitch_label.setStyleSheet("font-family: 'Consolas';font-size: 14px")
        self.yaw_label.setStyleSheet("font-family: 'Consolas';font-size: 14px")
        self.roll_label.setGeometry(35, 30, 100, 58)
        self.pitch_label.setGeometry(35, 30, 100, 88)
        self.yaw_label.setGeometry(35, 30, 100, 118)

        reset_orientation_button = QtWidgets.QPushButton("Reset Orientation", parent=self.RotView)
        reset_orientation_button.clicked.connect(self._reset_orientation)
        #reset_orientation_button.setGeometry()

        self.mdi_area.addSubWindow(self.RotView, Qt.WindowType.WindowMinMaxButtonsHint)

    def _reset_orientation(self):
        self.xgrid.resetTransform()
        self.xgrid.rotate(90,0,1,0)
        self.xgrid.scale(0.16,0.16,0.16)

    def _reset_missed(self):
        self._packets_missed = 0

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
        return ["None"] + [port.device for port in ports] # + [".poop file"]

    def _on_com_port_selected(self, com_port):
        if hasattr(self, 'packet_reader') and self.packet_reader.isRunning():
            self.packet_reader.terminate()
            self.packet_reader.wait()
            self.packet_reader.radio_serial.close_port()
            # self._sensor_packet = None
            # self._gps_packet = None
            
        self._reset_ekf()

        if (com_port == "None"):
            return
        
        # if (com_port == ".poop file"):

        #     self._open_calib_file_dialog()
        #     # todo
        #     return

        
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

            # Show the input dialog
            response = msg.exec()

            # If the message box 'Yes' is selected, proceed with calibration or file loading
            if response == QtWidgets.QMessageBox.StandardButton.Yes:
                
                input_dialog = QtWidgets.QDialog(msg)
                input_dialog.setWindowTitle("Calibration Options")
                input_layout = QtWidgets.QVBoxLayout(input_dialog)
                input_field = QtWidgets.QLineEdit(input_dialog)
                input_field.setText(str(10000))
                def validate_and_accept():
                    text_value = input_field.text()
                    try:
                        numerical_value = int(text_value)
                        if (numerical_value >= 5000 and numerical_value <= 20000):
                            input_dialog.accept()
                        else:
                            QtWidgets.QMessageBox.warning(input_dialog, "Out of Range", "Please enter an integer between 5000 and 20000")
                    except ValueError:
                        QtWidgets.QMessageBox.warning(input_dialog, "Idiot", "Please enter an integer")

                combo_box = QtWidgets.QComboBox(input_dialog)
                combo_box.addItem("Yes", 1)
                combo_box.addItem("No", 0)

                input_layout.addWidget(QtWidgets.QLabel("Calibration Datapoints"))
                input_layout.addWidget(input_field)
                input_layout.addWidget(QtWidgets.QLabel("Save calibration file?"))
                input_layout.addWidget(combo_box)

                button_layout = QtWidgets.QHBoxLayout()
                ok_button = QtWidgets.QPushButton("OK", input_dialog)
                cancel_button = QtWidgets.QPushButton("Cancel", input_dialog)
                ok_button.clicked.connect(validate_and_accept)
                cancel_button.clicked.connect(input_dialog.reject)
                button_layout.addWidget(ok_button)
                button_layout.addWidget(cancel_button)

                input_layout.addLayout(button_layout)

                input_result = input_dialog.exec()

                if input_result == QtWidgets.QDialog.DialogCode.Accepted:
                    self._calib_points = int(input_field.text())
                    self.calib_progress_bar.setRange(0, self._calib_points)
                    self._calib_save = combo_box.currentData()
                    self._calib_data_arr = np.empty((self._calib_points, 6))
                    self._calib_time_arr = np.empty(self._calib_points)

                    self._calibrated = False

            elif response == QtWidgets.QMessageBox.StandardButton.No:
                # Open the file dialog for loading calibration data
                self._open_calib_file_dialog()
                
            
        self.packet_reader = PacketReaderSerial(com_port)
        self.packet_reader.sensorPacketReceived.connect(self.process_sensor_packet)
        self.packet_reader.gpsPacketReceived.connect(self.process_gps_packet)
        self.packet_reader.disconnected.connect(self.handle_disconnect)
        self.packet_reader.start()

    def _open_calib_file_dialog(self):
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
        self.calib_progress_bar.hide()
        self._calibrated = True

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
        # if True:#np.linalg.norm(packet[12:15]) > ADXL_32G:
        #     self._sensor_packet[1:4] = packet[12:15] # uncalibrated, later fix this
        #     self._using_adxl = True
        # else:
        #     self._sensor_packet[1:4] = calibrate_accelerometer_single(convertRawAcc(*packet[1:4]), self._calib_params[0], self._calib_params[1])
        #     self._using_adxl = False
        self._sensor_packet[1:4] = calibrate_accelerometer_single(convertRawAcc(*packet[1:4]), self._calib_params[0], self._calib_params[1])
        self._sensor_packet[4:7] = calibrate_gyroscope_single(convertRawGyr(*packet[4:7]), self._calib_params[2], self._calib_params[3], self._calib_params[4])
        # approximate missed packets using timestamp differences
        # if self._last_timestamp == 0:
        #     self._last_timestamp = self._sensor_packet[0]
        self._packets_missed += max(0, int((self._sensor_packet[0] - self._last_timestamp) / 4900) - 1)
        self._last_timestamp = self._sensor_packet[0]
        # print(self._packets_missed)
        #self.sensor_packet[12:15] = packet[12:15] * 0.48069 # raw adxl to m/s^2
        # Handle calibration in its entirety right here
        # Yes this is ugly, too much unrelated code in this function, but no overhead for regular execution (just an extra branch instr in theory)
        if not self._calibrated:
            if self._calib_packets_read < self._calib_points:
                self._calib_time_arr[self._calib_packets_read] = self._sensor_packet[0]
                self._calib_data_arr[self._calib_packets_read] = self._sensor_packet[1:4] + self._sensor_packet[4:7]
                self._calib_packets_read += 1
                return
            
            self.packet_reader.pause()
            # self.packet_reader.setPriority(QtCore.QThread.Priority.IdlePriority)
            
            reg_intervals  = np.arange(self._calib_time_arr[0], self._calib_time_arr[-1], 1e6 / CALIB_FREQUENCY)
            y_interpolated = np.zeros((len(reg_intervals), self._calib_data_arr.shape[1]))

            # for each column, interpolate the data based on our specified intervals
            for i in range(self._calib_data_arr.shape[1]):
                y_interpolated[:, i] = np.interp(reg_intervals, self._calib_time_arr, self._calib_data_arr[:, i])
            #print(self.packet_reader.radio_serial.overflows)
            #self._calib_params 
            self._calib_params[0], self._calib_params[1], self._calib_params[2], self._calib_params[3], self._calib_params[4], gyro_residuals = list(all_calib_params(y_interpolated, CALIB_FREQUENCY))
            # print(self._calib_params)
            if (self._calib_save):
                np.savez("calib/" + formatted_time + "-" + f"{gyro_residuals:.5e}", aC=self._calib_params[0], ab=self._calib_params[1], gC=self._calib_params[2], gb=self._calib_params[3], Rm=self._calib_params[4])
            self._calibrated = True
            self.calib_progress_bar.hide()
            msg = QtWidgets.QMessageBox()
            msg.setIcon(QtWidgets.QMessageBox.Icon.Information)
            msg.setWindowTitle("Calibration Results")
            msg.setText(f"Gyro residuals: {gyro_residuals:.5e}") 
            msg.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Ok)
            _ = msg.exec()
            self.packet_reader.flush()
            self.packet_reader.unpause()
            return

        if self._kalman_uninitialized:
            self.kalman.begin(packet[0])
            self._kalman_uninitialized = False
        # Push data and update kalman filter
        self.kalman.setIMU(self._sensor_packet[0], self._sensor_packet[4:7], self._sensor_packet[1:4])
        self.kalman.setMag(self._sensor_packet[0], packet[7:10]) # divide by 100 to convert from uT to Gauss
        # if (np.linalg.norm(np.array(packet[7:10])) > 100):
        #     print(packet)
        self.kalman.setBaro(self._sensor_packet[0], packet[11]) # pass raw pressure data in hPa here
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
        self.missed_packets_num.setText(f"{self._packets_missed}")
        if not self._calibrated:
            self.calib_progress_bar.setValue(self._calib_packets_read)
        if self.showSensorData:
            self.sensor_plot_widgets[0].update_plot(self._sensor_packet[1:4])
            self.sensor_plot_widgets[1].update_plot(self._sensor_packet[4:7])
            self.sensor_plot_widgets[2].update_plot(self._sensor_packet[7:10])
        # if self.showPosition:
        #     pos = self.kalman.getPosition().squeeze()
        #     self.pos_plot_widgets[0].update_plot(pos[1],  pos[0])
        #     self.pos_plot_widgets[1].update_plot(pos[0], -pos[2]) # note we negate the Down component for intuitive plots
        #     self.pos_plot_widgets[2].update_plot(pos[1], -pos[2])
        
        self.altitude_plot.update_plot(self._press_to_alt(self._sensor_packet[11]))
        
        for i in range(6): 
            status = self._sensor_packet[15] & (1 << i)
            if status:
                self.status_labels[i].setStyleSheet("color: green;")
            else:
                self.status_labels[i].setStyleSheet("color: red;")

        if self._gps_packet: 
            self.lat.setText(f"{self._gps_packet[1] / 1e7:.6f}")
            self.lon.setText(f"{self._gps_packet[2] / 1e7:.6f}")
            self.sats.setText(f"{self._gps_packet[12]}")
            self.alt.setText(f"{self._gps_packet[3] / 1e3:.3f}")
            self.fix.setText(f"{FIX_TYPES[min(self._gps_packet[13], 6)]}")

        self.blah.setText(f"{self.kalman.ekf.global_position_is_valid()}")
        self.kalman_attitude_valid.setText(f"{self.kalman.ekf.attitude_valid()}")
        pos = self.kalman.getPosition().squeeze()
        self.kalman_pos_x.setText(f"{pos[0]}")
        self.kalman_pos_y.setText(f"{pos[1]}")
        self.kalman_pos_z.setText(f"{pos[2]}")
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
    def _update_3d(self):
        if self.showOrientation:
            quat = self.kalman.getQuaternion().squeeze()
            curr = QQuaternion(*quat)
            self.xgrid.transform().rotate((self.last_quat.inverted()*curr)) # get the delta quaternion (only Transform3D object takes quaternion rotation)
            self.xgrid.update()
            self.last_quat = curr

            rotation_matrix = self.xgrid.transform().normalMatrix()
            quaternion = QQuaternion.fromRotationMatrix(rotation_matrix)
            euler = quaternion.toEulerAngles()
            pitch = np.degrees(euler.x()) % 360
            roll =  np.degrees(euler.y()) % 360
            yaw =   np.degrees(euler.z()) % 360

            self.roll_label.setText(f"{roll:>.2f}°")
            self.pitch_label.setText(f"{pitch:>.2f}°")
            self.yaw_label.setText(f"{yaw:>.2f}°")                                    

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
            font-size: 16px;          
            background-color: #22222f;
            color: #aaaaaa;
            padding: 10px;
            border-radius: 5px;
        }
        QPushButton:pressed {
                background-color: #10121f;
                border: 2px solid #10121f;
        }
        QComboBox {
            background-color: #10121f;
            color: #ffffff;
            border: 1px solid #4db8ff;
        }
    """)
    main.showMaximized()
    app.exec()