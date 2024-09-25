
- `ekf.py` is a wrapper class for the [ecl library](https://github.com/MucusMerchant/Py-PX4-ECL) (py-PX4-ECL). The ecl library must be installed before this class can be used.
- `packet_stream_serial.py` provides a class that parses packets from the flight computer into Python tuples. 
- `UI.py` contains the code for the GUI (using pyqt6) and uses classes from both `packet_stream_serial.py` and `ekf.py`.