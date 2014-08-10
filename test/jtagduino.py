
#   This file is part of JTAGduino project.
#
#   Copyright 2011 Francesco Balducci <balau@users.sourceforge.net>
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU Lesser General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.

import serial
import time

class jtagduino_cmd:
    CMD_IF_VER = 0x1
    CMD_FW_VER = 0x2
    CMD_SET_SERIAL_SPEED = 0x3
    CMD_SET_PIN = 0x10
    CMD_CLEAR_PIN = 0x11
    CMD_GET_PIN = 0x12
    CMD_PULSE_HIGH = 0x13
    CMD_PULSE_LOW = 0x14
    CMD_ASSIGN_PIN = 0x15  
    CMD_SET_JTAG_SPEED = 0x20
    CMD_JTAG_CLOCK = 0x21
    CMD_JTAG_SEQUENCE = 0x22

class jtagduino_rsp:
    RSP_OK = 0
    RSP_ERROR_BAD_CMD = 1
    RSP_ERROR_UNKNOWN = 2
    RSP_ERROR_BAD_PIN = 3
    RSP_ERROR_BAD_SPEED = 4
    RSP_BAD_SEQUENCE_LEN = 5
    RSP_BAD_BAUD = 6

class jtag_pins:
    TCK = 0
    TMS = 1
    TDI = 2
    TDO = 3
    TRST = 4
    N_JTAG_PINS = 5

class jtagduino:
    MAX_JTAG_SPEED_KHZ = 500
    BAUD_RATES = [300, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200]
    DEFAULT_BAUD_RATE = 9600
    def __init__(self, port='/dev/ttyACM0', baudrate=DEFAULT_BAUD_RATE):
        self.ser = serial.Serial(
            port = port,
            baudrate = baudrate,
        )
        time.sleep(1.5)
        self.ser.flushInput()
        self.ser.flushOutput()
    def if_ver(self):
        self.ser.write(chr(jtagduino_cmd.CMD_IF_VER))
        rsp = ord(self.ser.read(1))
        if (rsp == jtagduino_rsp.RSP_OK):
            if_ver_minor = ord(self.ser.read(1))
            if_ver_minor = if_ver_minor + (ord(self.ser.read(1)) * 256)
            if_ver_major = ord(self.ser.read(1))
            if_ver_major = if_ver_major + (ord(self.ser.read(1)) * 256)
            return (if_ver_major, if_ver_minor)
        else:
            return None
    def fw_ver(self):
        self.ser.write(chr(jtagduino_cmd.CMD_FW_VER))
        rsp = ord(self.ser.read(1))
        if (rsp == jtagduino_rsp.RSP_OK):
            fw_ver_minor = ord(self.ser.read(1))
            fw_ver_minor = fw_ver_minor + (ord(self.ser.read(1)) * 256)
            fw_ver_major = ord(self.ser.read(1))
            fw_ver_major = fw_ver_major + (ord(self.ser.read(1)) * 256)
            return (fw_ver_major, fw_ver_minor)
        else:
            return None
    def set_serial_speed(self, baud):
        self.ser.write(chr(jtagduino_cmd.CMD_SET_SERIAL_SPEED))
        self.ser.write(chr((baud >>  0) & 0xFF))
        self.ser.write(chr((baud >>  8) & 0xFF))
        self.ser.write(chr((baud >> 16) & 0xFF))
        self.ser.write(chr((baud >> 24) & 0xFF))
        rsp = ord(self.ser.read(1))
        if (rsp == jtagduino_rsp.RSP_OK):
            time.sleep(0.027) #time to switch baud rate for device, worst case.
            self.ser.baudrate = baud
        return rsp
    def set_pin(self, pin):
        self.ser.write(chr(jtagduino_cmd.CMD_SET_PIN))
        self.ser.write(chr(pin))
        rsp = ord(self.ser.read(1))
        return rsp
    def clear_pin(self, pin):
        self.ser.write(chr(jtagduino_cmd.CMD_CLEAR_PIN))
        self.ser.write(chr(pin))
        rsp = ord(self.ser.read(1))
        return rsp
    def get_pin(self, pin):
        self.ser.write(chr(jtagduino_cmd.CMD_GET_PIN))
        self.ser.write(chr(pin))
        rsp = ord(self.ser.read(1))
        if (rsp == jtagduino_rsp.RSP_OK):
            val = ord(self.ser.read(1))
        else:
            val = 0
        return (rsp, val)
    def pulse_high(self, pin, micros):
        self.ser.write(chr(jtagduino_cmd.CMD_PULSE_HIGH))
        self.ser.write(chr(pin))
        self.ser.write(chr((micros >> 0) & 0xFF))
        self.ser.write(chr((micros >> 8) & 0xFF))
        rsp = ord(self.ser.read(1))
        return rsp
    def pulse_low(self, pin, micros):
        self.ser.write(chr(jtagduino_cmd.CMD_PULSE_LOW))
        self.ser.write(chr(pin))
        self.ser.write(chr((micros >> 0) & 0xFF))
        self.ser.write(chr((micros >> 8) & 0xFF))
        rsp = ord(self.ser.read(1))
        return rsp
    def assign_pin(self, jtag_pin, arduino_pin):
        self.ser.write(chr(jtagduino_cmd.CMD_ASSIGN_PIN))
        self.ser.write(chr(jtag_pin))
        self.ser.write(chr(arduino_pin))
        rsp = ord(self.ser.read(1))
        return rsp
    def set_jtag_speed(self, khz):
        self.ser.write(chr(jtagduino_cmd.CMD_SET_JTAG_SPEED))
        self.ser.write(chr((khz >> 0) & 0xFF))
        self.ser.write(chr((khz >> 8) & 0xFF))
        rsp = ord(self.ser.read(1))
        return rsp
    def jtag_clock(self, tms, tdi):
        self.ser.write(chr(jtagduino_cmd.CMD_JTAG_CLOCK))
        jtag_in = 0
        if tms:
            jtag_in = jtag_in | 1
        if tdi:
            jtag_in = jtag_in | 2
        self.ser.write(chr(jtag_in))
        rsp = ord(self.ser.read(1))
        if (rsp == jtagduino_rsp.RSP_OK):
            tdo = ord(self.ser.read(1))
        else:
            tdo = 0
        return (rsp, tdo)
    def jtag_sequence(self, tms_seq, tdi_seq):
        if len(tms_seq) != len(tdi_seq):
            return (jtagduino_rsp.RSP_BAD_SEQUENCE_LEN, None)
        n_bytes =  (len(tms_seq) + 7) >> 3 # ceil(len(tdi_seq)/8)
        i_seq = 0
        tms_bytes = bytearray(n_bytes)
        tdi_bytes = bytearray(n_bytes)
        while i_seq < len(tms_seq):
            i_byte = i_seq >> 3
            bit = 1<<(i_seq & 0x7)
            if tms_seq[i_seq]:
                tms_bytes[i_byte] = tms_bytes[i_byte] | bit
            if tdi_seq[i_seq]:
                tdi_bytes[i_byte] = tdi_bytes[i_byte] | bit
            i_seq += 1
        self.ser.write(chr(jtagduino_cmd.CMD_JTAG_SEQUENCE))
        self.ser.write(chr(len(tms_seq)))
        self.ser.write(tms_bytes)
        self.ser.write(tdi_bytes)
        tdo_seq = []
        rsp = ord(self.ser.read(1))
        if (rsp == jtagduino_rsp.RSP_OK):
            tdo_bytes = bytearray(n_bytes)
            for i_byte in range(n_bytes):
                tdo_bytes[i_byte] = ord(self.ser.read(1))
            i_seq = 0
            while i_seq < len(tms_seq):
                i_byte = i_seq >> 3
                bit = 1<<(i_seq & 0x7)
                if (tdo_bytes[i_byte] & bit) != 0:
                    tdo_seq.append(1)
                else:
                    tdo_seq.append(0)
                i_seq += 1
        return (rsp, tdo_seq)
        

