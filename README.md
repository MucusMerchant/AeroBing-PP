## AeroBing Post-Processing
### Overview
This is a repository for processing/filtering/visualizing data that is sent by the flight computer. The order of post-processing is generally as follows:
- [1] read raw binary packets from the flight computer and parse them into meaningful arrays of data
- [2] filter data from noisy sensors to mitigate the accumulation of error due to vibrations
- [3] send data from multiple sensors into a (properly tuned) Kalman filter to get orientation/position estimates
- [4] display useful information on a GUI for easy debugging/filter tuning

### TODO (a lot)
- An IMU calibration protocol that can handle the timing of a calibration routine, generate calibration values, and apply them to all subsequent datapoints
- A magnetometer calibration protocol
- Kalman filter tuning - which parameters give us the best data?
- Noise filtering for IMU data, intense vibration can ruin our data
- (worst case) if the PX4 kalman filter turns to be unsuitable, we may have to develop our own
  
### Files
`UI.py` contains the code for the GUI (using pyqt6) and uses classes from both `packet_stream_serial.py` and `ekf.py`. It contains code that passes parsed packets to the Kalman filter. There is a 3D orientation visualizer, live graphs for IMU measurements and position estimates.
- `ekf.py` is a wrapper class for the [ecl library](https://github.com/MucusMerchant/Py-PX4-ECL) (py-PX4-ECL). It abstracts the weird Vector API for us, we can just pass numbers to the filter without worrying about the minutia of PX4's linear algebra library. The ecl library must be installed before this class can be used.
- `packet_stream_serial.py` provides a class that parses packets from the flight computer into Python tuples. 
