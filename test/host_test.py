#!/usr/bin/python

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

print 'if_ver_major = %d; if_ver_minor = %d' % device.if_ver()
print 'fw_ver_major = %d; fw_ver_minor = %d' % device.fw_ver()
print 'set_serial_speed(115200) = %d' % device.set_serial_speed(115200)
print 'clear pin TDI rsp = %d' % device.clear_pin(jtag_pins.TDI)
print 'set pin TDI rsp = %d' % device.set_pin(jtag_pins.TDI)
print 'get pin TDO rsp = %d; val = %d' % device.get_pin(jtag_pins.TDO)
print 'jtag_clock(1,1): rsp = %d; tdo = %d' % device.jtag_clock(1,1)
print 'jtag_clock(1,0): rsp = %d; tdo = %d' % device.jtag_clock(1,0)
(rsp, tdo_seq) = device.jtag_sequence([1,1,1,1], [1, 1, 0, 1])
print ('jtag_sequence([1,1,1,1], [1, 1, 0, 1]), rsp = %d;' % rsp) + ' tdo_seq = ' + str(tdo_seq)
(rsp, tdo_seq) = device.jtag_sequence([1] * 255, [1] * 255)
print ('jtag_sequence([1] * 255, [1] * 255), rsp = %d;' % rsp) + ' tdo_seq = ' + str(tdo_seq)
print 'device.set_jtag_speed(1) = %d' % device.set_jtag_speed(1)
(rsp, tdo_seq) = device.jtag_sequence([1] * 255, [1] * 255)
print ('jtag_sequence([1] * 255, [1] * 255), rsp = %d;' % rsp) + ' tdo_seq = ' + str(tdo_seq)
sys.exit(0);

#TODO: other commands

