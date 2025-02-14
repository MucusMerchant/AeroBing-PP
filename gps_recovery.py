from PyQt6 import QtWidgets
from PyQt6.QtGui import QVector3D, QQuaternion, QPixmap, QIcon
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot, QTimer, Qt
import pyqtgraph as pg
from pyqtgraph import functions as fn
import pyqtgraph.opengl as gl
import numpy as np
from lib.packet_stream_file import *
import time
import math
import os

IGNORE_BAD_DATAPOINTS = True
PLOT_BACKGROUND = "#141729"
FIX_TYPES = ["No fix", "Dead-reckoning only", "2D Fix", "3D Fix", "Epic", "Time only", "other (bad)"]
ADXL_32G = 65536

current_time = time.localtime()
formatted_time = time.strftime("%Y-%m-%d_%H-%M-%S", current_time)

class PacketReader(QThread):
    gpsPacketReceived    = pyqtSignal(list)
    disconnected         = pyqtSignal()
    def __init__(self, file, speed):
        super().__init__()
        self.packet_reader = PacketStream(file)
        self.packet_reader.begin()
        self.speed = speed
        self.paused = False

    def run(self):
        while True:
            if self.paused:
                time.sleep(0.1)
                continue
            try:
                packet_type, packet = self.packet_reader.read_packet()
            except:
                self.disconnected.emit()
                return
            if packet_type == b'\xca':
                time.sleep(self.speed * 0.09)
                self.gpsPacketReceived.emit(packet)
    
    def stop(self):
        self.packet_reader.stop()

    def setSpeed(self, speed):
        self.speed = speed

    def pause(self):
        self.paused = True

    def unpause(self):
        self.paused = False

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

class ReplayWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Emergency GPS Recovery")
        self.setFixedSize(300,260)
        self._init_ui()
        self.ignore_bad_datapoints = IGNORE_BAD_DATAPOINTS
        self.packet_reader = None
        self.file = None
        self.speed = 1
        #self._paused = False

    def _init_ui(self):
        # setup header and main hlayout
        self.centralWidget = QtWidgets.QWidget(self)
        self.screen = QtWidgets.QVBoxLayout(self.centralWidget)
        self.header = QtWidgets.QHBoxLayout()
        self.main_layout = QtWidgets.QVBoxLayout()
        self.setCentralWidget(self.centralWidget)

        file_button = QtWidgets.QPushButton("Select .poop file")
        file_button.clicked.connect(self._open_file_dialog)
        self.main_layout.addWidget(file_button)

        timer_layout = QtWidgets.QHBoxLayout()
        self.timer_widget = QtWidgets.QLabel("00:00:00.000")
        self.timer_widget.setStyleSheet("font-size: 25px; font-weight: bold; padding: 5px; text-align: center;")
        
        self.pause_button = QtWidgets.QPushButton()
        self.pause_button.setStyleSheet("padding: 0; margin: 0; background-color: transparent")
        self.pause_button.setFixedSize(25,25)
        self.pause_button.setIcon(QIcon("assets/play.png"))
        self.pause_button.clicked.connect(self._pause_toggle)
        self.reset_button = QtWidgets.QPushButton()
        self.reset_button.setStyleSheet("padding: 0; margin: 0; background-color: transparent")
        self.reset_button.setFixedSize(25,25)
        self.reset_button.setIcon(QIcon("assets/reload.png"))
        self.reset_button.clicked.connect(self._reset)
        self.speed_button = QtWidgets.QPushButton()
        self.speed_button.setStyleSheet("padding: 0; margin: 0; background-color: transparent")
        self.speed_button.setFixedSize(25,25)
        self.speed_button.setIcon(QIcon("assets/speed.png"))
        self.speed_button.pressed.connect(self._speed_toggle)
        self.speed_button.released.connect(self._speed_toggle)
        timer_layout.addWidget(self.timer_widget, stretch = 2)
        timer_layout.addWidget(self.pause_button, stretch = 1)
        timer_layout.addWidget(self.speed_button, stretch = 1)
        timer_layout.addWidget(self.reset_button, stretch = 1)
        self.main_layout.addLayout(timer_layout)
        
        self.latitude_label = QLabelPair("Latitude: ", "0")
        self.longitude_label = QLabelPair("Longitude: ", "0")
        self.altitude_label = QLabelPair("Altitude: ", "0")
        self.fix_label = QLabelPair("Fix Type: ", "0")
        self.longitude_label.setStyleSheet("font-size: 16px; background-color: transparent; color: #aaaaaa")
        self.latitude_label.setStyleSheet("font-size: 16px; background-color: transparent; color: #aaaaaa")
        self.altitude_label.setStyleSheet("font-size: 16px; background-color: transparent; color: #aaaaaa")
        self.fix_label.setStyleSheet("font-size: 16px; background-color: transparent; color: #aaaaaa")
        self.main_layout.addWidget(self.latitude_label)
        self.main_layout.addWidget(self.longitude_label)
        self.main_layout.addWidget(self.altitude_label)
        self.main_layout.addWidget(self.fix_label)
        self.main_layout.setContentsMargins(10, 20, 10, 20) 

        self.screen.addLayout(self.header)
        self.screen.addLayout(self.main_layout)
    
    def _pause_toggle(self):
        if not self.packet_reader:
            return
        if self.packet_reader.paused:
            self.packet_reader.unpause()
            self.pause_button.setIcon(QIcon("assets/pause.png"))
        else:
            self.packet_reader.pause()
            self.pause_button.setIcon(QIcon("assets/play.png"))
    
    def _reset(self):
        if not self.file:
            return
        if self.packet_reader:
            self.packet_reader.stop()
        self.packet_reader = PacketReader(self.file, self.speed)
        self.packet_reader.gpsPacketReceived.connect(self.process_gps_packet)
        self.packet_reader.start()
        if not self.packet_reader.paused:
            self._pause_toggle()

    def _speed_toggle(self):
        self.speed = (self.speed + 1) % 2
        self.packet_reader.speed = self.speed

    def _open_file_dialog(self):
        file_dialog = QtWidgets.QFileDialog(self)
        file_dialog.setFileMode(QtWidgets.QFileDialog.FileMode.ExistingFile)  
        file_dialog.setNameFilter("Poop Files (*.poop)")
        file_dialog.setDirectory(os.path.join(os.path.dirname(__file__), "data"))
        file_dialog.setViewMode(QtWidgets.QFileDialog.ViewMode.List)

        if file_dialog.exec():
            self.file = file_dialog.selectedFiles()[0]
        
        self._reset()

    @pyqtSlot(list)
    def process_gps_packet(self, packet):
        if not packet or (not self.ignore_bad_datapoints and (packet[13] not in [2,3,4])):
            return
        sec_from_micro = packet[0]/1e6
        hours, rem = divmod(sec_from_micro, 3600)
        minutes, seconds = divmod(rem, 60)
        self.timer_widget.setText("{:0>2}:{:0>2}:{:0>6.3f}".format(int(hours),int(minutes),seconds))
        self.latitude_label.setText(f"{packet[1] / 1e7: .6f}")
        self.longitude_label.setText(f"{packet[2] / 1e7: .6f}")
        self.altitude_label.setText(f"{packet[3] / 1e3: 0.3f}")
        self.fix_label.setText(f"{FIX_TYPES[min(packet[13], 6)]}")
        
if __name__ == "__main__":
    
    
    app = QtWidgets.QApplication([])
    main = ReplayWindow()
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
    main.show()
    app.exec()