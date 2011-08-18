
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
    def __init__(self, port='/dev/ttyACM0'):
        self.ser = serial.Serial(
            port = port,
            #baudrate = 115200,
            baudrate = 9600,
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
    def set_pin(self, pin):
        self.ser.write(chr(jtagduino_cmd.CMD_SET_PIN))
        self.ser.write(chr(pin))
        rsp = ord(self.ser.read(1))
        return rsp;
    def clear_pin(self, pin):
        self.ser.write(chr(jtagduino_cmd.CMD_CLEAR_PIN))
        self.ser.write(chr(pin))
        rsp = ord(self.ser.read(1))
        return rsp;
    def get_pin(self, pin):
        self.ser.write(chr(jtagduino_cmd.CMD_GET_PIN))
        self.ser.write(chr(pin))
        rsp = ord(self.ser.read(1))
        if (rsp == jtagduino_rsp.RSP_OK):
            val = ord(self.ser.read(1))
        else:
            val = 0
        return (rsp, val)

        

