# simple python class for reading binary shart packets

import serial
import struct # this library is very useful, handles structs for us
import time

SERIAL_PORT = 'COM7' # will need to be changed for Mac or Linux
SERIAL_BAUD = 9600 # need to change when switching from radio to usb serial mode
SYNC_BYTE   = b'\xaa'
TYPE_SENSOR = b'\x0b'
TYPE_GPS    = b'\xca'

FILENAME = 'out.poop'

# struct specifications following documentation at https://docs.python.org/3/library/struct.html
PACKET_SPEC = {
    TYPE_SENSOR : (60, '<I3i11f'), 
    TYPE_GPS    : (52, '<I6i3Iif4B'),
}

#NUM_PACKETS_TO_READ = 1000 # set very high or infinity if u dont want a limit
NUM_PACKETS_TO_READ = float('inf')

class PacketStream:
    def __init__(self, port, baudrate):
        self.device = serial.Serial(None, baudrate)
        self.device.port = port
        self.error_state = 0
        self.file = open(FILENAME, 'ab')

    def begin(self):
        print(f"Opening port {self.device.port}...", end="", flush=True)
        while not self.device.is_open: 
            print(".", end="", flush=True)
            try:
                self.device.open()
            except serial.SerialException as error:
                if str(error).startswith("could not open port"):
                    time.sleep(1)
                else:
                    raise error from None
            else:
                break
        print(" Done!", flush=True)

    # Function to calculate the checksum
    def calculate_checksum(self, data):
        checksum_a = 0
        checksum_b = 0
        for byte in data:
            checksum_a += byte
            checksum_b += checksum_a
        return checksum_a & 0xFF, checksum_b & 0xFF

    # Function to read data from serial and process packets
    def read_packet(self, packet_types):
        self.error_state = 0
        # print in_waiting to see if data coming in too fast for python to handle
        if self.device.in_waiting > 100:
            sync_byte = self.device.read(1)
            if sync_byte == SYNC_BYTE:
                # Found sync byte, read packet type
                packet_type_byte = self.device.read(1)
                if packet_type_byte in packet_types:
                    received_checksum_a, received_checksum_b = struct.unpack('<BB', self.device.read(2))
                    packet_info = packet_types[packet_type_byte]
                    packet_size = packet_info[0]
                    packet_data = self.device.read(packet_size)

                    self.file.write(sync_byte + packet_type_byte + bytes([received_checksum_a, received_checksum_b]))
                    self.file.write(packet_data)
                    self.file.flush()
                    # or os.fsync(self.file.fileno())
                    calculated_checksum_a, calculated_checksum_b = self.calculate_checksum(packet_data)

                    if (received_checksum_a, received_checksum_b) == (calculated_checksum_a, calculated_checksum_b):
                        packet_format = packet_info[1]
                        return packet_type_byte, struct.unpack(packet_format, packet_data)
                    else:
                        # CHECKSUM FAILED
                        self.error_state = 1
                else:
                    # PACKET TYPE UNRECOGNIZED
                    self.error_state = 2

        # NO BYTES AVAILABLE TO READ
        self.error_state = 3
        return None, None
    
    def send(self, data):
        self.device.write(data)

    def flush(self):
        self.device.flush()

# add code here to write to a .poop file  
if __name__ == "__main__":
    radio_serial = PacketStream(SERIAL_PORT, SERIAL_BAUD)
    radio_serial.begin()
    packets = 0
    fails = 0
    start = time.time()
    while packets < NUM_PACKETS_TO_READ:
        packet_type, packet = radio_serial.read_packet(PACKET_SPEC)
        if radio_serial.error_state == 1 or radio_serial.error_state == 2:
            fails += 1
        if radio_serial.error_state == 0:
            packets += 1
        #"""
        if packet_type == TYPE_SENSOR:
            print("[SENSOR] " + str(packet))
        elif packet_type == TYPE_GPS:
            print("[GPS] " + str(packet))
        else:
            continue
        #"""
    end = time.time()
    print(str(end-start) + " elapsed. " + str(NUM_PACKETS_TO_READ) + " packets read. " + str(fails) + " failures.")
