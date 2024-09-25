from PyQt6 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
from lib.packet_stream_serial import PacketStream, PACKET_SPEC
from lib.ekf import EkfWrapper
import time

# Reset the Teensy before running!!
class PacketReader(QtCore.QThread):
    sensorPacketReceived = QtCore.pyqtSignal(list)
    gpsPacketReceived    = QtCore.pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.radio_serial = PacketStream("COM8", 230400)
        self.radio_serial.begin()
        #self.radio_serial.send(b'\xAA') # send a byte to initialize shart

    def run(self):
        while True:
            packet_type, packet = self.radio_serial.read_packet(PACKET_SPEC)
            print(str(packet))
            if packet_type == b'\x0b':
                self.sensorPacketReceived.emit(packet)
            if packet_type == b'\xca':
                self.gpsPacketReceived.emit(packet)

class SensorPlot(pg.PlotWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.setMouseEnabled(x=False, y=False)
        self.getAxis('bottom').setVisible(False)
        self.addLegend()
        self.setMinimumSize(400,100)
        self.time = np.arange(25)
        self.data = np.zeros((3, 25))  # Initialize 3 lines
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
        self.ver_data = np.zeros(200)  # Initialize 3 lines
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
        self._init_estimation()
        self._init_ui()
        self.count = 0

    def _init_ui(self):
        self.centralWidget = QtWidgets.QWidget(self)
        self.hlayout = QtWidgets.QHBoxLayout(self.centralWidget)

        self.showSensorData = True
        self.showOrientation = True
        self.showPosition = True

        self._setup_3d_views()
        self._setup_sensor_plots()
        self._setup_pos_plots()
        self._setup_controls()

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
        

    def _init_estimation(self):
        self.sensor_packet = None
        self.gps_packet    = None
        self.packet_reader = PacketReader()
        self.packet_reader.sensorPacketReceived.connect(self.process_sensor_packet)
        self.packet_reader.gpsPacketReceived.connect(self.process_gps_packet)
        self.packet_reader.start()
        
        self.kalman = None

    def _setup_sensor_plots(self):
        #self.layout = QtWidgets.QVBoxLayout()
        self.sensor_plot_widgets = [SensorPlot(self) for _ in range(3)]
        for plot_widget in self.sensor_plot_widgets:
            self.vlayout.addWidget(plot_widget)
        #self.hlayout.addLayout(self.layout)

    def _setup_pos_plots(self):
        self.layout = QtWidgets.QVBoxLayout()
        self.pos_plot_widgets = [PosPlot(self) for _ in range(3)]
        for plot_widget in self.pos_plot_widgets:
            self.layout.addWidget(plot_widget)
        self.hlayout.addLayout(self.layout)
        
    def _setup_3d_views(self):
        self.RotView = gl.GLViewWidget()

        self.xgrid = gl.GLGridItem()
        self.xgrid.setSpacing(5,5,5)
        self.xgrid.scale(0.3,0.3,0.3)
        self.rotAxes = gl.GLAxisItem(parentItem=self.xgrid, glOptions='additive')
        self.rotAxes.setSize(10,10,10)
        self.RotView.addItem(self.xgrid)

        self.vlayout = QtWidgets.QVBoxLayout()
        self.RotView.setMinimumSize(300, 300)
        
        self.vlayout.addWidget(self.RotView)
        self.hlayout.addLayout(self.vlayout)
        self.last_quat = QtGui.QQuaternion(0,0,1,0) 

    def _setup_controls(self):
        layout = QtWidgets.QVBoxLayout()
        countBtn = QtWidgets.QPushButton("Click me!")
        countBtn.clicked.connect(self._toggle_all)
        reset = QtWidgets.QPushButton("Reset")
        reset.clicked.connect(self._reset_ekf)
        
        layout.addWidget(reset)
        layout.addWidget(countBtn)

        self.hlayout.addLayout(layout)

    @QtCore.pyqtSlot(list)
    def process_sensor_packet(self, packet):
        if not self.kalman:
            self.kalman = ekf.EkfWrapper()
            self.kalman.begin(packet[0])
        self.sensor_packet = packet #make packet available to UI for plotting
        # Push data and update kalman filter
        self.kalman.setIMU(packet[0], np.array(packet[7:10], dtype=np.float32)[:,np.newaxis] * np.pi/180, np.array(packet[4:7], dtype=np.float32)[:,np.newaxis] * 9.81)
        self.kalman.setMag(packet[0], np.array(packet[10:13], dtype=np.float32)[:,np.newaxis] / 100)
        self.kalman.setBaro(packet[0], packet[14]) # pass raw pressure data in hPa here
        self.kalman.update() # update the filter on the IMU cycle, as in the PX4-EKF tests

    @QtCore.pyqtSlot(list)
    def process_gps_packet(self, packet):
        if self.kalman:
            #vel = np.array(self.kalman.getVelocity()).squeeze()
            self.gps_packet = packet
            self.kalman.setGPS(*packet)
            #self.kalman.setGPS(packet[0], 407000000,-740000000, 30000,0,0,0,0,0,0,0,0,16,3,0,1)

    def _update_ui(self):
        if not self.sensor_packet:
            return
          
        if self.showSensorData:
            self.sensor_plot_widgets[0].update_plot(self.sensor_packet[4:7])
            self.sensor_plot_widgets[1].update_plot(self.sensor_packet[7:10])
            self.sensor_plot_widgets[2].update_plot(self.sensor_packet[10:13])
        if self.showPosition:
            pos = np.array(self.kalman.getPosition()).squeeze()
            self.pos_plot_widgets[0].update_plot(pos[1], pos[0])
            self.pos_plot_widgets[1].update_plot(pos[0], -pos[2]) # note we negate the Down component for intuitive plots
            self.pos_plot_widgets[2].update_plot(pos[1], -pos[2])
        if self.showOrientation:
            quat = np.array(self.kalman.getQuaternion()).squeeze()
            curr = QtGui.QQuaternion(*quat)
            self.xgrid.transform().rotate((self.last_quat.inverted()*curr)) # get the delta quaternion (only Transform3D object takes quaternion rotation)
            self.xgrid.update()
            self.last_quat = curr
        
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
    main.show()
    app.exec()

#pip install qt-material to make this look better