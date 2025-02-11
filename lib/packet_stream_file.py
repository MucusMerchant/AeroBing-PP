# simple python class for reading binary shart packets from SD card file
# this can be a base for other functionality, i.e. storing sensor packets in a csv, graphing, etc.
# to use: put binary shart file in this folder. thats it

import struct # this library is very useful, handles structs for us
import os

# this will work if u got the file in the 'python' folder and your working directory is Aerobing-Firmware
#os.chdir(os.getcwd()+"/data")

FILE_NAME = "data/2025-02-10_11-25-42.poop"
SYNC_BYTE    : bytes = b'\xaa'
TYPE_SENSOR  : bytes = b'\x0b'
TYPE_GPS     : bytes = b'\xca'
TYPE_COMMAND : bytes = b'\xa5'

# struct specifications following documentation at https://docs.python.org/3/library/struct.html
# defined in shart comms.h
PACKET_SPEC = {
    TYPE_SENSOR  : (44, '<I6h5f3h2B'), 
    TYPE_GPS     : (52, '<I6i3Iif4B'),
    TYPE_COMMAND : (4,  '<i'),
}

#NUM_PACKETS_TO_READ = 1000 # set very high or infinity if u dont want a limit
NUM_PACKETS_TO_READ = float('inf')

LSM_ACC_CONVERSION_CONSTANT = 0.0095712904 # 0.976 * 9.80665 / 1000.0
LSM_GYR_CONVERSION_CONSTANT = 0.00122173051 # 70 * 0.017453293 / 1000.0
ADXL_CONVERSION_CONSTANT = 0.0047155689 # 0.480690  / 1000 * 9.81

# Raw IMU processing taken from adafruit library (i.e. from LSM datasheet)
# note that this is specific to out lsm configuration, must be adjusted if this changes
def convertRawAcc(ax: int, ay: int, az: int) -> tuple[float]:

    c_ax = ax * LSM_ACC_CONVERSION_CONSTANT
    c_ay = ay * LSM_ACC_CONVERSION_CONSTANT
    c_az = az * LSM_ACC_CONVERSION_CONSTANT
    
    return c_ax, c_ay, c_az

def convertRawGyr(gx: int, gy: int, gz: int) -> tuple[float]:

    c_gx = gx * LSM_GYR_CONVERSION_CONSTANT
    c_gy = gy * LSM_GYR_CONVERSION_CONSTANT
    c_gz = gz * LSM_GYR_CONVERSION_CONSTANT

    return c_gx, c_gy, c_gz

def convertRawAdxl(ax: int, ay: int, az: int) -> tuple[float]:
    c_ax = ax * ADXL_CONVERSION_CONSTANT
    c_ay = ay * ADXL_CONVERSION_CONSTANT
    c_az = az * ADXL_CONVERSION_CONSTANT
    
    return c_ax, c_ay, c_az

class PacketStream:
    def __init__(self, filename):
        self.filename = filename
        self.file = None
        self.last_time_stamp = 0
        self.overflows = 0
        self.error_state = 0

    def begin(self):
        self.file = open(self.filename, mode='rb')

    # Function to calculate the checksum
    def calculate_checksum(self, data: bytes) -> bytes:
        checksum_a = 0
        checksum_b = 0
        for byte in data:
            checksum_a += byte
            checksum_b += checksum_a
        return bytes([checksum_a & 0xFF, checksum_b & 0xFF])

    # Function to read data from serial and process packets
    def read_packet(self) -> tuple[int, tuple]:
        sync = self.file.read(1)
        if sync == bytes([]):
            self.error_state = 3
        elif (sync == SYNC_BYTE):
                # Found sync byte, read packet type
                packet_type_byte = self.file.read(1)
                if packet_type_byte in PACKET_SPEC:
                    received_checksum_a, received_checksum_b = struct.unpack('<BB', self.file.read(2))
                    packet_info = PACKET_SPEC[packet_type_byte]
                    packet_size = packet_info[0]
                    packet_data = self.file.read(packet_size)
                        
                    calculated_checksum_a, calculated_checksum_b = self.calculate_checksum(packet_data)

                    if (received_checksum_a, received_checksum_b) == (calculated_checksum_a, calculated_checksum_b):
                        packet_format = packet_info[1]
                        packet = struct.unpack(packet_format, packet_data)
                        if packet_type_byte == TYPE_SENSOR:
                            if (packet[0] < self.last_time_stamp):
                                self.overflows += 1 # originally not working bc of the gps
                            self.last_time_stamp = packet[0]
                        return packet_type_byte, packet
                    else:
                        print("Checksum failed!")
                        self.error_state = 1
                else:
                    print("Invalid packet type byte:", packet_type_byte)
                    self.error_state = 2
        return None, None
    
    def stop(self):
        self.file.close()
    
# note to Julie: barometer data is stored in the last 2 spots of the sensor tuple (temp in C and pressure in Pa)
# if __name__ == "__main__":
#     packet_reader = PacketStream(FILE_NAME)
#     packet_reader.begin()
#     packets = 0
#     # here you can filter by error state. right now, we stop only when we reach eof
#     while packet_reader.error_state != 3:
#         packet_type, packet = packet_reader.read_packet()
#         if packet_type == TYPE_SENSOR:
#             print("[SENSOR] " + str(packet))
#         elif packet_type == TYPE_GPS:
#             print("[GPS] " + str(packet))
#         else:
#             continue
    
#         packets += 1
#     print("Done! " + str(packets) + " packets read. meow")

