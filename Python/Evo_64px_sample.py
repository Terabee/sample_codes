#!/usr/bin/python3
# -*- coding: utf-8 -*-
import argparse
import numpy as np
import serial
import crcmod.predefined
import threading


class Evo_64px(object):

    def __init__(self, portname):
        self.portname = portname # To be adapted if using UART backboard
        self.baudrate = 115200  # 3000000 for UART backboard

        # Configure the serial connections (the parameters differs on the device you are connecting to)
        self.port = serial.Serial(
            port=self.portname,
            baudrate=self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.port.isOpen()
        self.crc32 = crcmod.predefined.mkPredefinedCrcFun('crc-32-mpeg')
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8')
        self.serial_lock = threading.Lock()

    def get_depth_array(self):
        '''
        This function reads the data from the serial port and returns it as
        an array of 12 bit values with the shape 8x8
        '''
        got_frame = False
        while not got_frame:
            with self.serial_lock:
                frame = self.port.readline()
            if len(frame) == 269:
                if ord(frame[0]) == 0x11 and self.crc_check(frame):  # Check for range frame header and crc
                    dec_out = []
                    for i in range(1, 65):
                        rng = ord(frame[2 * i - 1]) << 7
                        rng = rng | (ord(frame[2 * i]) & 0x7F)
                        dec_out.append(rng & 0x3FFF)
                    depth_array = [dec_out[i:i + 8] for i in range(0, len(dec_out), 8)]
                    depth_array = np.array(depth_array)
                    got_frame = True
            else:
                print("Invalid frame length: {}".format(len(frame)))

        depth_array.astype(np.uint16)
        return depth_array

    def crc_check(self, frame):
        index = len(frame) - 9  # Start of CRC
        crc_value = (ord(frame[index]) & 0x0F) << 28
        crc_value |= (ord(frame[index + 1]) & 0x0F) << 24
        crc_value |= (ord(frame[index + 2]) & 0x0F) << 20
        crc_value |= (ord(frame[index + 3]) & 0x0F) << 16
        crc_value |= (ord(frame[index + 4]) & 0x0F) << 12
        crc_value |= (ord(frame[index + 5]) & 0x0F) << 8
        crc_value |= (ord(frame[index + 6]) & 0x0F) << 4
        crc_value |= (ord(frame[index + 7]) & 0x0F)
        crc_value = crc_value & 0xFFFFFFFF
        crc32 = self.crc32(frame[:index])

        if crc32 == crc_value:
            return True
        else:
            print("Discarding current buffer because of bad checksum")
            return False

    def send_command(self, command):
        with self.serial_lock:# This avoid concurrent writes/reads of serial
            self.port.write(command)
            ack = self.port.read(1)
            # This loop discards buffered frames until an ACK header is reached
            while ord(ack) != 20:
                self.port.readline()
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

    def start_sensor(self):
        if self.send_command("\x00\x52\x02\x01\xDF".encode()):
            print("Sensor started successfully")

    def stop_sensor(self):
        if self.send_command("\x00\x52\x02\x00\xD8".encode()):
            print("Sensor stopped successfully")

    def run(self):
        self.port.flushInput()
        if self.baudrate == 115200: # Sending VCP start when connected via USB
            self.start_sensor()

        depth_array = []
        while depth_array is not None:
            depth_array = self.get_depth_array()
            print(depth_array)
        else:
            if self.baudrate == 115200:
                self.stop_sensor()  # Sending VCP stop when connected via USB


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Read data from Evo 64px")
    parser.add_argument(
        "--portname",
        default="/dev/ttyACM0",
        help="portname, find using ls /dev/tty*",
    )
    args = parser.parse_args()
    evo_64px = Evo_64px(portname=args.portname)
    evo_64px.run()
