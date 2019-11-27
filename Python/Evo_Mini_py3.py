#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import crcmod.predefined
import serial.tools.list_ports
import threading


class Evo_Mini(object):
    TEXT_MODE = b"\x00\x11\x01\x45"
    BINARY_MODE = b"\x00\x11\x02\x4C"
    SINGLE_PIXEL_MODE = b"\x00\x21\x01\xBC"
    TWO_BY_TWO_PIXEL_MODE = b"\x00\x21\x02\xB5"
    TWO_PIXEL_MODE = b"\x00\x21\x03\xB2"
    SHORT_RANGE_MODE = b"\x00\x61\x01\xE7"
    LONG_RANGE_MODE = b"\x00\x61\x03\xE9"

    def __init__(self, portname=None):
        if portname is None:
            ports = list(serial.tools.list_ports.comports())
            for p in ports:
                if ":5740" in p[2]:
                    print("Evo Eco found on port {}".format(p[0]))
                    portname = p[0]
            if portname is None:
                print("Sensor not found. Please Check connections.")
                exit()
        self.portname = portname
        self.baudrate = 115200

        # Configure the serial connections
        self.port = serial.Serial(
            port=self.portname,
            baudrate=self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.port.isOpen()
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8')
        self.serial_lock = threading.Lock()

    def get_ranges(self):
        # Read one byte
        ranges = []
        data = self.port.read(1)
        if data == b'T':
            # After T read 3 bytes
            frame = data + self.port.read(3)  # Try a single-range frame
            if frame[-1] != self.crc8(frame[:-1]):
                frame = frame + self.port.read(2)  # Try a two-range frame
                if frame[-1] != self.crc8(frame[:-1]):
                    frame = frame + self.port.read(4) # Try a two-by-two-range frame
                elif frame[-1] != self.crc8(frame[:-1]):
                    return "CRC mismatch. Check connection or make sure only one progam accesses the sensor port."

            # Convert binary frame to decimal in shifting by 8 the frame
            for i in range(int((len(frame) - 2) / 2)):
                rng = frame[2 * i + 1] << 8
                rng = rng | (frame[2 * i + 2] & 0xFF)
                ranges.append(rng)
        else:
            return "Wating for frame header"
        # Check special cases (limit values)
        self.check_ranges(ranges)

        return ranges

    def check_ranges(self, range_list):
        for i in range(len(range_list)):
            # Checking error codes
            if range_list[i] == 65535:  # Sensor measuring above its maximum limit
                range_list[i] = float('inf')
            elif range_list[i] == 1:  # Sensor not able to measure
                range_list[i] = float('nan')
            elif range_list[i] == 0:  # Sensor detecting object below minimum range
                range_list[i] = -float('inf')
            else:
                # Convert frame in meters
                range_list[i] /= 1000.0

        return range_list

    def send_command(self, command):
        with self.serial_lock:  # This avoid concurrent writes/reads of serial
            self.port.write(command)
            ack = self.port.read(1)
            # This loop discards buffered frames until an ACK header is reached
            while ack != b"\x12":
                ack = self.port.read(1)
            else:
                ack += self.port.read(3)

            # Check ACK crc8
            crc8 = self.crc8(ack[:3])
            if crc8 == ack[3]:
                # Check if ACK or NACK
                if ack[2] == 0:
                    return True
                else:
                    print("Command not acknowledged")
                    return False
            else:
                print("Error in ACK checksum")
                return False

    def set_binary_mode(self):
        if self.send_command(Evo_Mini.BINARY_MODE):
            print("Sensor succesfully switched to binary mode")

    def set_two_by_two_pixel_mode(self):
        if self.send_command(Evo_Mini.TWO_BY_TWO_PIXEL_MODE):
            print("Sensor succesfully switched to 2 by 2 ranges measurement")

    def set_two_pixel_mode(self):
        if self.send_command(Evo_Mini.TWO_PIXEL_MODE):
            print("Sensor succesfully switched to two ranges measurement")

    def set_single_pixel_mode(self):
        if self.send_command(Evo_Mini.SINGLE_PIXEL_MODE):
            print("Sensor succesfully switched to single range measurement")

    def set_short_range_mode(self):
        if self.send_command(Evo_Mini.SHORT_RANGE_MODE):
            print("Sensor succesfully switched to short range measurement")

    def set_long_range_mode(self):
        if self.send_command(Evo_Mini.LONG_RANGE_MODE):
            print("Sensor succesfully switched to long range measurement")

    def run(self):
        self.port.flushInput()
        self.set_binary_mode()  # Set binary output as it is required for this sample

        # Set ranging mode
        self.set_long_range_mode()
        #self.set_short_range_mode()

        # Set pixel mode
        self.set_single_pixel_mode()
        #self.set_two_pixel_mode()
        #self.set_two_by_two_pixel_mode()

        ranges = []
        while ranges is not None:
            ranges = self.get_ranges()
            print(ranges)
        else:
            print("No data from sensor")


if __name__ == '__main__':
    sensor = Evo_Mini()
    sensor.run()
