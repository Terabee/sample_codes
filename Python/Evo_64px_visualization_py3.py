#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import serial
import serial.tools.list_ports
import crcmod.predefined
import threading
import time
from PIL import Image, ImageTk
import tkinter as Tk

class Evo_64px(object):

    def __init__(self, portname=None):
        if portname is None:
            ports = list(serial.tools.list_ports.comports())
            for p in ports:
                if ":5740" in p[2]:
                    print("Evo 64px found on port {}".format(p[0]))
                    portname = p[0]
            if portname is None:
                print("Sensor not found. Please Check connections.")
                exit()
        self.portname = portname  # To be adapted if using UART backboard
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

        self.got_frame = False

        #  NEW WINDOW FOR MATRIX DATA VISUALIZATION #
        self.activate_visualization = True
        self.window = Tk.Tk()
        self.window.wm_geometry("640x720")
        self.canvas_width = 600
        self.canvas_height = 600
        self.canvas2 = Tk.Canvas(self.window, width=self.canvas_width, height=self.canvas_height)
        self.canvas2.pack(side=Tk.TOP)
        self.photo = ImageTk.PhotoImage("P")
        self.img = self.canvas2.create_image(300, 300, image=self.photo)
        self.text2 = Tk.Label(self.window)
        self.text2.config(height=10, width=20, text='', font=("Helvetica", 25))
        self.text2.pack(side=Tk.BOTTOM)
        self.text2.config(text="Evo_64px depth array")

        self.last_frame_timestamp = time.time()

        self.label_list = []
        for i in range(8):
            for j in range(8):
                label = self.canvas2.create_text(i * self.canvas_width/8 + (self.canvas_width/8)/2,
                                                 j * self.canvas_height/8 + (self.canvas_height/8)/2,
                                                 fill="#f2d500", font="Helvetica 14 bold", text="0000")
                self.label_list.append(label)

    def update_GUI(self):
        self.canvas2.itemconfig(self.img, image=self.photo)
        self.window.update()

    def array_2_image(self):
        '''
        This function is creating an Image from numpy array
        '''
        depth = self.rounded_array / 64.0
        im = Image.fromarray(np.uint8(depth), mode="P")
        im = im.resize(size=(self.canvas_width, self.canvas_height), resample=Image.NEAREST)
        return im

    def update_label(self, frame):
        for i in range(8):
            for j in range(8):
                self.canvas2.itemconfig(self.label_list[8 * j + i], text=str(frame[i][j]))

    def sample(self):
        frame = self.rounded_array
        self.photo = ImageTk.PhotoImage(self.array_2_image())
        self.update_label(frame)
        # print("updating gui")
        self.update_GUI()

    def get_depth_array(self):
        '''
        This function reads the data from the serial port and returns it as
        an array of 12 bit values with the shape 8x8
        '''
        got_frame = False
        while not got_frame:
            with self.serial_lock:
                #print(self.port.in_waiting)
                frame = self.port.readline()
            if len(frame) == 269:
                if frame[0] == 0x11 and self.crc_check(frame):  # Check for range frame header and crc
                    dec_out = []
                    for i in range(1, 65):
                        rng = frame[2 * i - 1] << 7
                        rng = rng | (frame[2 * i] & 0x7F)
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
        crc_value = (frame[index] & 0x0F) << 28
        crc_value |= (frame[index + 1] & 0x0F) << 24
        crc_value |= (frame[index + 2] & 0x0F) << 20
        crc_value |= (frame[index + 3] & 0x0F) << 16
        crc_value |= (frame[index + 4] & 0x0F) << 12
        crc_value |= (frame[index + 5] & 0x0F) << 8
        crc_value |= (frame[index + 6] & 0x0F) << 4
        crc_value |= (frame[index + 7] & 0x0F)
        crc_value = crc_value & 0xFFFFFFFF
        crc32 = self.crc32(frame[:index])

        if crc32 == crc_value:
            return True
        else:
            print("Discarding current buffer because of bad checksum")
            return False

    def send_command(self, command):
        with self.serial_lock:  # This avoid concurrent writes/reads of serial
            self.port.write(command)
            ack = self.port.read(1)
            # This loop discards buffered frames until an ACK header is reached
            while ack != b"\x14":
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
        if self.send_command(b"\x00\x52\x02\x01\xDF"):
            print("Sensor started successfully")

    def stop_sensor(self):
        if self.send_command(b"\x00\x52\x02\x00\xD8"):
            print("Sensor stopped successfully")

    def run(self):
        self.port.flushInput()
        if self.baudrate == 115200: # Sending VCP start when connected via USB
            self.start_sensor()

        depth_array = []
        while depth_array is not None:
            depth_array = self.get_depth_array()
            self.rounded_array = np.round(depth_array, 0)
            if self.activate_visualization:
                self.sample()
        else:
            if self.baudrate == 115200:
                self.stop_sensor()  # Sending VCP stop when connected via USB


if __name__ == '__main__':
    evo_64px = Evo_64px()
    evo_64px.run()
