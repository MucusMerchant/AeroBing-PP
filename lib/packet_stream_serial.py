# simple python class for reading binary shart packets

import serial
import struct # this library is very useful, handles structs for us
import time

SERIAL_PORT         = 'COM5' # will need to be changed for Mac or Linux, on windows enter 'mode' in cmd to find active port name
SERIAL_BAUD  : int   = 9600#230400 # need to change when switching from radio to usb serial mode
SYNC_BYTE    : bytes = b'\xaa'
TYPE_SENSOR  : bytes = b'\x0b'
TYPE_GPS     : bytes = b'\xca'
TYPE_COMMAND : bytes = b'\xa5'

# shart-defined command codes
START_COMMAND : int = 0x6D656F77
STOP_COMMAND  : int = 0x6D696175


FILENAME = 'python/out.poop'

# struct specifications following documentation at https://docs.python.org/3/library/struct.html
# defined in shart comms.h
PACKET_SPEC = {
    TYPE_SENSOR  : (44, '<I6h5f3h2B'), 
    TYPE_GPS     : (52, '<I6i3Iif4B'),
    TYPE_COMMAND : (4,  '<i'),
}

#NUM_PACKETS_TO_READ = 1000 # set very high or infinity if u dont want a limit
NUM_PACKETS_TO_READ = float('inf')

# Raw IMU processing taken from adafruit library (i.e. from LSM datasheet)
def convertRawIMU(ax: int, ay: int, az: int, gx: int, gy: int, gz: int) -> tuple[float]:

    c_ax = ax * 0.976 * 9.80665 / 1000.0
    c_ay = ay * 0.976 * 9.80665 / 1000.0
    c_az = az * 0.976 * 9.80665 / 1000.0

    c_gx = gx * 70 * 0.017453293 / 1000.0
    c_gy = gy * 70 * 0.017453293 / 1000.0
    c_gz = gz * 70 * 0.017453293 / 1000.0

    return c_ax, c_ay, c_az, c_gx, c_gy, c_gz

# main class for handling packets though serial and files
class PacketStream:

    def __init__(self, port: int, baudrate: int) -> None:
        self.serial_bus = serial.Serial(None, baudrate)
        self.serial_bus.port = port
        self.error_state = 0
        self.file = open(FILENAME, 'wb')

    def open_port(self) -> None:
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

    # Function to calculate the checksum
    def __calculate_checksum(self, data: bytes) -> bytes:
        checksum_a = 0
        checksum_b = 0
        for byte in data:
            checksum_a += byte
            checksum_b += checksum_a
        return bytes([checksum_a & 0xFF, checksum_b & 0xFF])

    # Function to read data from serial and process packets
    def read_packet(self) -> tuple[int, tuple]:
        self.error_state = 0
        # print in_waiting to see if data coming in too fast for python to handle
        if self.serial_bus.in_waiting > 100:
            if self.serial_bus.read(1) == SYNC_BYTE:
                # Found sync byte, read packet type
                packet_type_byte = self.serial_bus.read(1)
                if packet_type_byte in PACKET_SPEC:
                    received_checksums = bytes(struct.unpack('<BB', self.serial_bus.read(2)))
                    packet_info = PACKET_SPEC[packet_type_byte]
                    packet_size = packet_info[0]
                    packet_data = self.serial_bus.read(packet_size)

                    self.__write_packet(packet_type_byte, packet_data, 'file')
                    # or os.fsync(self.file.fileno())
                    calculated_checksums = self.__calculate_checksum(packet_data)

                    if (received_checksums) == (calculated_checksums):
                        packet_format = packet_info[1]
                        return packet_type_byte, struct.unpack(packet_format, packet_data)
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
        checksum_bytes = self.__calculate_checksum(data)
        if target == 'serial':
            self.serial_bus.write(SYNC_BYTE + packet_type + checksum_bytes + data)
        elif target == 'file':
            self.file.write(SYNC_BYTE + packet_type + checksum_bytes + data)
            self.file.flush()
        else:
            pass
    
    def start(self) -> None:
        self.__write_packet(TYPE_COMMAND, START_COMMAND.to_bytes(4, 'little'), 'serial')
    
    def stop(self) -> None:
        self.__write_packet(TYPE_COMMAND, STOP_COMMAND.to_bytes(4, 'little'), 'serial')
"""
if __name__ == "__main__":
    radio_serial = PacketStream(SERIAL_PORT, SERIAL_BAUD)
    radio_serial.open_port()
    packets = 0
    fails = 0
    start = time.time()

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
    
    while packets < NUM_PACKETS_TO_READ:
        packet_type, packet = radio_serial.read_packet()
        if radio_serial.error_state == 1 or radio_serial.error_state == 2:
            fails += 1
        if radio_serial.error_state == 0:
            packets += 1
        
        if packet_type == TYPE_SENSOR:
            print("[SENSOR] " + str(packet))
            #print(convertRawIMU(*packet[1:7]))
        elif packet_type == TYPE_GPS:
            print("[GPS] " + str(packet))
        else:
            continue
        
    radio_serial.stop()
    end = time.time()
    print(str(end-start) + " elapsed. " + str(NUM_PACKETS_TO_READ) + " packets read. " + str(fails) + " failures.")
"""
