#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import crcmod.predefined
import serial
from struct import unpack
import serial.tools.list_ports
import threading

class EvoThermal():
    def __init__(self):
        ### Search for Evo Thermal port and open it ###
        ports = list(serial.tools.list_ports.comports())
        portname = None
        for p in ports:
            if ":5740" in p[2]:
                print("EvoThermal found on port " + p[0])
                portname = p[0]
        if portname is None:
            print("Sensor not found. Please Check connections.")
            exit()
        ser = serial.Serial(
                            port=portname,  # To be adapted if using UART backboard
                            baudrate=115200, # 1500000 for UART backboard
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS
                            )
        self.port = ser
        self.serial_lock = threading.Lock()
        ### CRC functions ###
        self.crc32 = crcmod.predefined.mkPredefinedCrcFun('crc-32-mpeg')
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8')
        ### Activate sensor USB output ###
        self.activate_command   = (0x00, 0x52, 0x02, 0x01, 0xDF)
        self.deactivate_command = (0x00, 0x52, 0x02, 0x00, 0xD8)
        self.send_command(self.activate_command)

    def get_thermals(self):
        got_frame = False
        while not got_frame:
            with self.serial_lock:
                ### Polls for header ###
                header = self.port.read(2)
                header = unpack('H', str(header))
                if header[0] == 13:
                    ### Header received, now read rest of frame ###
                    data = self.port.read(2068)
                    ### Calculate CRC for frame (except CRC value and header) ###
                    calculatedCRC = self.crc32(data[:2064])
                    data = unpack("H" * 1034, str(data))
                    receivedCRC = (data[1032] & 0xFFFF ) << 16
                    receivedCRC |= data[1033] & 0xFFFF
                    TA = data[1024]
                    data = data[:1024]
                    data = np.reshape(data, (32, 32))
                    ### Compare calculated CRC to received CRC ###
                    if calculatedCRC == receivedCRC:
                        got_frame = True
                    else:
                        print("Bad CRC. Dropping frame")
        self.port.flushInput()
        ### Data is sent in dK, this converts it to celsius ###
        data = (data/10.0) - 273.15
        TA = (TA/10.0) - 273.15

        return data

    def send_command(self, command):
        ### This avoid concurrent writes/reads of serial ###
        with self.serial_lock:
            self.port.write(command)
            ack = self.port.read(1)
            ### This loop discards buffered frames until an ACK header is reached ###
            while ord(ack) != 20:
                ack = self.port.read(1)
            else:
                ack += self.port.read(3)
            ### Check ACK crc8 ###
            crc8 = self.crc8(ack[:3])
            if crc8 == ord(ack[3]):
                ### Check if ACK or NACK ###
                if ord(ack[2]) == 0:
                    print("Command acknowledged")
                    return True
                else:
                    print("Command not acknowledged")
                    return False
            else:
                print("Error in ACK checksum")
                return False

    def run(self):
        ### Get frame and print it ###
        frame = self.get_thermals()
        print(frame)

    def stop(self):
        ### Deactivate USB VCP output and close port ###
        self.send_command(self.deactivate_command)
        self.port.close()


if __name__ == "__main__":
    evo = EvoThermal()
    try:
        while True:
            evo.run()
    except KeyboardInterrupt:
        evo.stop()
