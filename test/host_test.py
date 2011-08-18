#!/usr/bin/python

# This test needs python-serial installed and permission to access serial ports.
# Your user needs to be in group "dialout" (usually)
# In Debian-based distribution, you need to install python-serial package.
# This script has been tested with python version 2.6.7

import sys
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

jtagduino_port = '/dev/ttyACM0'
if (len(sys.argv) > 1):
    jtagduino_port = sys.argv[1]

ser = serial.Serial(
        port = jtagduino_port,
        #baudrate = 115200,
        baudrate = 9600,
        )
time.sleep(1.5)
ser.flushInput()
ser.flushOutput()

print 'IF_VER'
ser.write(chr(jtagduino_cmd.CMD_IF_VER))
rsp = ord(ser.read(1))
print 'rsp = %d' % rsp
if (rsp == jtagduino_rsp.RSP_OK):
    if_ver_minor = ord(ser.read(1))
    if_ver_minor = if_ver_minor + (ord(ser.read(1)) * 256)
    if_ver_major = ord(ser.read(1))
    if_ver_major = if_ver_major + (ord(ser.read(1)) * 256)
    print 'IF_VER_MINOR = %d' % if_ver_minor
    print 'IF_VER_MAJOR = %d' % if_ver_major
    print 'IF_VER OK'
else:
    print 'IF_VER KO'
print

print 'FW_VER'
ser.write(chr(jtagduino_cmd.CMD_FW_VER))
rsp = ord(ser.read(1))
print 'rsp = %d' % rsp
if (rsp == jtagduino_rsp.RSP_OK):
    fw_ver_minor = ord(ser.read(1))
    fw_ver_minor = fw_ver_minor + (ord(ser.read(1)) * 256)
    fw_ver_major = ord(ser.read(1))
    fw_ver_major = fw_ver_major + (ord(ser.read(1)) * 256)
    print 'FW_VER_MINOR = %d' % fw_ver_minor
    print 'FW_VER_MAJOR = %d' % fw_ver_major
    print 'FW_VER OK'
else:
    print 'FW_VER KO'
print

#TODO: other commands

