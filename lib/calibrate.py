import numpy as np
import argparse
import time

from lib.imu_calib.helpers import *
from lib.imu_calib.cost_functions import *
from lib.imu_calib.utilities import *
from lib.packet_stream_serial import *

# TODO: encapsulate all of this so it can be easily transplanted into UI code
# TODO (general PP): handle cases where time overflows
np.set_printoptions(edgeitems=30, linewidth=1000, formatter={'float': '{: 0.4f}'.format})

def get_imu_data(datapoints: int, target_frequency: int, stream: PacketStream):
    # preallocate a large np array - we will always fill this completely, no trimming required
    packets_read: int = 0
    time_arr = np.empty(datapoints)
    data_arr = np.empty((datapoints, 6))
    # Fill allocated array with packets, preprocess the imu data
    # this fist part seems to be necessary - first few gyro measurements corrupt the entire calibration process
    while packets_read < 1000:
        packet_type, packet = stream.read_packet()
        if stream.error_state == 0 and packet_type == TYPE_SENSOR:
            
            packets_read += 1
    packets_read = 0

    print("Reading packets")

    while packets_read < datapoints:
        packet_type, packet = stream.read_packet()
        if stream.error_state == 0 and packet_type == TYPE_SENSOR:
            
            data_arr[packets_read] = convertRawAcc(*packet[1:4]) + convertRawGyr(*packet[4:7])
            time_arr[packets_read] = packet[0]
            packets_read += 1

    print("Finished taking measurements")
    # logic: set up the target time intervals we want, then preallocate array for interpolated data, shape is known
    reg_intervals = np.arange(time_arr[0], time_arr[-1], 1.0 / target_frequency * 1e6)
    y_interpolated = np.zeros((len(reg_intervals), data_arr.shape[1]))

    # for each column, interpolate the data based on our specified intervals
    for i in range(data_arr.shape[1]):
        y_interpolated[:, i] = np.interp(reg_intervals, time_arr, data_arr[:, i])

    return y_interpolated
"""
if __name__ == '__main__':
    # parser = argparse.ArgumentParser(description = 'Run calibration on real data from IMU.')
    # parser.add_argument('--sampling_frequency', help = 'Sampling frequency for logfile.', 
    #     required = True, type = int)
    # parser.add_argument('--file', help = 'Path to file with data from IMU.',
    #     required = True, type = str)
    # args = parser.parse_args()

    # dt = 1 / args.sampling_frequency
    # datafile = args.file

    # # read file with ax, ay, az, wx, wy, wz measurements from IMU
    # imu_data = np.genfromtxt(datafile, delimiter=' ')
    sampling_frequency = 200
    dt = 1.0 / sampling_frequency
    radio_serial = PacketStream(SERIAL_PORT, SERIAL_BAUD)
    radio_serial.open_port()
    radio_serial.start()
    # Wait for acknowledgement
    print("Awaiting acknowledgement of start command")
    while True:
        packet_type, packet = radio_serial.read_packet()
        if (radio_serial.error_state == 0):
            break
    if (packet_type != TYPE_COMMAND or packet[0] != START_COMMAND):
        print("Acknowledgement not recognized, exiting")
        exit()

    imu_data = get_imu_data(20000, sampling_frequency, radio_serial)
    standstill = generate_standstill_flags(imu_data)

    plot_imu_data_and_standstill(imu_data, standstill)

    accs, angs = imu_data[:,0:3], imu_data[:,3:6]

    # find accelerometer calibration parameters and calibrate accel measurements
    theta_found_acc = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time_start = time.time()
    theta_found_acc = find_calib_params_acc(True, residual_acc, theta_found_acc, accs, standstill > 0)
    time_end = time.time()

    print("ACC calibration done in: ", time_end - time_start, "seconds")
    print("[ S_X     S_Y     S_Z     NO_X    NO_Y    NO_Z    B_X     B_Y     B_Z   ]")
    print(theta_found_acc)
    accs_calibrated = calibrate_accelerometer(accs, theta_found_acc)
    plot_accelerations_before_and_after(accs, accs_calibrated)


    # find gyroscope calibration parameters
    theta_found_gyr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    theta_found_gyr[-6:-3] = np.mean(angs[0:100,:], axis=0)

    residualSum = lambda: np.sum(np.rad2deg(residual_gyr(theta_found_gyr, 
             angs, 
             accs_calibrated,
             standstill, dt))**2)

    print("Gyroscope residuals before calibration: ", residualSum())
    time_start = time.time()
    theta_found_gyr = find_calib_params_gyr(True, residual_gyr, theta_found_gyr, 
        angs, accs_calibrated, standstill, dt)
    time_end = time.time()
    print("GYR calibration done in: ", time_end - time_start, "seconds")
    print("[ S_X     S_Y     S_Z     NO_X    NO_Y    NO_Z    B_X     B_Y     B_Z     E_X     E_Y     E_Z  ]")
    print(theta_found_gyr)
    print("Gyroscope residuals after calibration: ", residualSum())
    angs_calibrated = calibrate_gyroscope(angs, theta_found_gyr[0:9], theta_found_gyr[9:12])
    plot_rotations_before_and_after(angs, angs_calibrated)
"""
def all_calib_params(imu_data, frequency):
    standstill = generate_standstill_flags(imu_data)
    accs, angs = imu_data[:,0:3], imu_data[:,3:6]

    # find accelerometer calibration parameters and calibrate accel measurements
    theta_found_acc = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    theta_found_acc = find_calib_params_acc(True, residual_acc, theta_found_acc, accs, standstill > 0)
    accs_calibrated = calibrate_accelerometer(accs, theta_found_acc)

    # find gyroscope calibration parameters
    theta_found_gyr        = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    theta_found_gyr[-6:-3] = np.mean(angs[0:100,:], axis=0) # just bias here

    # Cost function that tells us how well the data fit the model
    residualSum = lambda: np.sum(np.rad2deg(residual_gyr(theta_found_gyr, 
             angs, 
             accs_calibrated,
             standstill, 1/frequency))**2)
    print("Accelerometer Calibrated")
    theta_found_gyr = find_calib_params_gyr(True, residual_gyr, theta_found_gyr, 
        angs, accs_calibrated, standstill, 1/frequency)
    print("Gyroscope residuals after calibration: ", residualSum())
    return acceleration_equation_components(theta_found_acc) + gyroscope_equation_components(theta_found_gyr[0:9], theta_found_gyr[9:12]) + (residualSum(),)