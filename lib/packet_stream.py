# simple python class for reading binary shart packets from SD card file
# this can be a base for other functionality, i.e. storing sensor packets in a csv, graphing, etc.
# to use: put binary shart file in this folder. thats it

import struct # this library is very useful, handles structs for us
import serial
import time
import os

# this will work if u got the file in the 'python' folder and your working directory is Aerobing-Firmware
#os.chdir(os.getcwd()+"/data")

SERIAL_BAUD  : int   = 230400
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

# shart-defined command codes
START_COMMAND : int = 0x6D656F77
STOP_COMMAND  : int = 0x6D696175

#NUM_PACKETS_TO_READ = 1000 # set very high or infinity if u dont want a limit
NUM_PACKETS_TO_READ = float('inf')

LSM_ACC_CONVERSION_CONSTANT = 0.0095712904 # 0.976 * 9.80665 / 1000.0
LSM_GYR_CONVERSION_CONSTANT = 0.00122173051 # 70 * 0.017453293 / 1000.0
ADXL_CONVERSION_CONSTANT = 0.48052585#0.0047155689 # 0.480690  / 1000 * 9.81

CRC_TABLE = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
]

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
    def __init__(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def _calculate_checksum(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc = (crc << 8) ^ CRC_TABLE[(crc >> 8) ^ byte]
            crc &= 0xFFFF
        return crc
    
    def read_packet(self):
        pass

class PacketStreamFile(PacketStream):
    def __init__(self, filename):
        super().__init__()
        self.filename = filename
        self.file = None
        self.last_time_stamp = 0
        self.overflows = 0
        self.error_state = 0

    def start(self):
        self.file = open(self.filename, mode='rb')

    # Function to read data from serial and process packets
    def read_packet(self) -> tuple[int, tuple]:
        sync = self.file.read(1)
        if sync == bytes([]):
            self.error_state = 3
        elif (sync == SYNC_BYTE):
                # Found sync byte, read packet type
                packet_type_byte = self.file.read(1)
                if packet_type_byte in PACKET_SPEC:
                    
                    received_checksum = struct.unpack('<H', self.file.read(2))[0]
                    packet_info = PACKET_SPEC[packet_type_byte]
                    packet_size = packet_info[0]
                    packet_data = self.file.read(packet_size)
                        
                    calculated_checksum  = self._calculate_checksum(packet_data)

                    if received_checksum == calculated_checksum:
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

class PacketStreamSerial(PacketStream):

    def __init__(self, port: int, baudrate: int, filename: str) -> None:
        super().__init__()
        self.serial_bus = serial.Serial(None, baudrate)
        self.serial_bus.port = port
        self.last_time_stamp = 0
        self.overflows = 0
        self.error_state = 0
        self.packets_since_last_flush = 0
        self.buffer_max_packets = 128
        self.file = open(filename, 'wb')

    def _open_port(self) -> None:
        print(f"Opening port {self.serial_bus.port}...", end="", flush=True)
        while not self.serial_bus.is_open: 
            print(".", end="", flush=True)
            try:
                self.serial_bus.open()
            except serial.SerialException as error:
                if str(error).startswith("could not open port"):
                    print(str(error))
                    time.sleep(1)
                else:
                    raise error from None
            else:
                break
        print(" Done!", flush=True)

    def _close_port(self) -> None:
        self.serial_bus.close()

    # Function to read data from serial and process packets
    def read_packet(self) -> tuple[int, tuple]:
        self.error_state = 0
        # print in_waiting to see if data coming in too fast for python to handle
        if self.serial_bus.in_waiting > 100:
            if self.serial_bus.read(1) == SYNC_BYTE:
                # Found sync byte, read packet type
                packet_type_byte = self.serial_bus.read(1)
                if packet_type_byte in PACKET_SPEC:
                    received_checksum = struct.unpack('<H', self.serial_bus.read(2))[0]
                    packet_info = PACKET_SPEC[packet_type_byte]
                    packet_size = packet_info[0]
                    packet_data = self.serial_bus.read(packet_size)

                    self.__write_packet(packet_type_byte, packet_data, 'file')
                    # or os.fsync(self.file.fileno())
                    calculated_checksums = self._calculate_checksum(packet_data)
                    if received_checksum == calculated_checksums:
                        packet_format = packet_info[1]
                        packet = struct.unpack(packet_format, packet_data)
                        if packet_type_byte == TYPE_SENSOR:
                            if (packet[0] < self.last_time_stamp):
                                self.overflows += 1 # originally not working bc of the gps
                            self.last_time_stamp = packet[0]
                        #packet[0] += self.overflows * 4294967295 # add uint32 max if overflow occurred
                        return packet_type_byte, packet#struct.unpack(packet_format, packet_data)
                    else:
                        # CHECKSUM FAILED
                        self.error_state = 1
                else:
                    # PACKET TYPE UNRECOGNIZED
                    self.error_state = 2

        # NO BYTES AVAILABLE TO READ
        else:
            self.error_state = 3
        return None, None
    
    def __write_packet(self, packet_type: bytes, data: bytes, target: str) -> None:
        checksum_bytes = self._calculate_checksum(data).to_bytes(2, byteorder='little')
        if target == 'serial':
            self.serial_bus.write(SYNC_BYTE + packet_type + checksum_bytes + data)
        elif target == 'file':
            self.packets_since_last_flush += 1
            self.file.write(SYNC_BYTE + packet_type + checksum_bytes + data)
            if (self.packets_since_last_flush > self.buffer_max_packets):
                self.file.flush()
                self.packets_since_last_flush = 0
        else:
            pass
    
    def start(self) -> None:
        self._open_port()
        self.__write_packet(TYPE_COMMAND, START_COMMAND.to_bytes(4, 'little'), 'serial')

    def stop(self) -> None:
        self.new_file()
        self._close_port()
    
    def new_file(self) -> None:
        self.__write_packet(TYPE_COMMAND, STOP_COMMAND.to_bytes(4, 'little'), 'serial')
    
# note to Julie: barometer data is stored in the last 2 spots of the sensor tuple (temp in C and pressure in Pa)
# if __name__ == "__main__":
#     packet_reader = PacketStreamFile(FILE_NAME)
#     packet_reader.start()
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

