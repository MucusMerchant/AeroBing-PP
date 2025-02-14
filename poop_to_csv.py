import csv
from lib.packet_stream_file import *

POOP_TIMESTAMP = "2025-02-09_15-52-32"
FILE_NAME = "data/" + POOP_TIMESTAMP + ".poop"

# Define the headers for sensor and GPS data
SENSOR_HEADERS = ['time', 'acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z', 'mag_x', 'mag_y', 'mag_z', 'temp', 'pressure', 'acc_x_adxl', 'acc_y_adxl', 'acc_y_adxl', 'status', 'sd_file']  # Adjust as per your packet structure
GPS_HEADERS = ['time', 'latitude', 'longitude', 'altitude', 'vel_n', 'vel_e', 'vel_d', 'eph', 'epv', 'sacc', 'gspeed', 'pdop', 'nsats', 'fix_type', 'valid', 'flags']  # Adjust as per your packet structure

if __name__ == "__main__":
    packet_reader = PacketStream(FILE_NAME)
    packet_reader.begin()
    packets = 0

    with open('csv/' + POOP_TIMESTAMP + '_sensor.csv', mode='w', newline='') as sensor_file, open('csv/' + POOP_TIMESTAMP + '_gps.csv', mode='w', newline='') as gps_file:
        sensor_writer = csv.writer(sensor_file)
        gps_writer = csv.writer(gps_file)
        sensor_writer.writerow(SENSOR_HEADERS)
        gps_writer.writerow(GPS_HEADERS)
        while packet_reader.error_state != 3:
            packet_type, packet = packet_reader.read_packet()   
            if packet_type == TYPE_SENSOR:
                sensor_writer.writerow(packet)
                
            elif packet_type == TYPE_GPS:
                gps_writer.writerow(packet)
            
            else:
                continue
            packets += 1

    print("Done! " + str(packets) + " packets read. meow")
