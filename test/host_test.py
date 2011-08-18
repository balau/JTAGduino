#!/usr/bin/python

# This test needs python-serial installed and permission to access serial ports.
# Your user needs to be in group "dialout" (usually)
# In Debian-based distribution, you need to install python-serial package.
# This script has been tested with python version 2.6.7

import sys
from jtagduino import *

jtagduino_port = '/dev/ttyACM0'
if (len(sys.argv) > 1):
    jtagduino_port = sys.argv[1]

device = jtagduino(jtagduino_port)
print 'if_ver_major = %d if_ver_minor = %d' % device.if_ver()
print 'fw_ver_major = %d fw_ver_minor = %d' % device.fw_ver()
print 'clear pin TDI rsp = %d' % device.clear_pin(jtag_pins.TDI)
print 'set pin TDI rsp = %d' % device.set_pin(jtag_pins.TDI)
print 'get pin TDO rsp = %d val = %d' % device.get_pin(jtag_pins.TDO)

sys.exit(0);

#TODO: other commands

