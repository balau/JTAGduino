#!/usr/bin/env python

#   This file is part of JTAGduino project.
#
#   Copyright 2014 Francesco Balducci <balau@users.sourceforge.net>
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
# This script has been tested with python version 2.7.8

import sys
from jtagduino import *

def dev_id_reg_from_seq(seq):
    dev_id_reg = 0
    for i_bit in range(0, len(seq)):
            dev_id_reg = dev_id_reg + (seq[i_bit] << i_bit)
    return dev_id_reg

def jtag_tap_state_move(device, tms):
    tdi = [0] * len(tms)
    (rsp, tdo) = device.jtag_sequence(tms, tdi)
    if rsp != 0:
        raise Exception('error: jtag_sequence returned %d' % rsp)

def read_idcodes(device):
    dev_id_reg_len = 32

    tms_seq_idle = [1, 1, 1, 1, 1, 0]
    tms_seq_idle_to_shiftdr = [1, 0, 0]
    tms_seq_readid = [0] * (dev_id_reg_len - 1)
    tms_seq_readid.append(1)
    tdi_seq_readid = [0] * dev_id_reg_len
    tms_seq_exitdr_to_shiftdr = [0, 1, 0]

    jtag_tap_state_move(device, tms_seq_idle)
    jtag_tap_state_move(device, tms_seq_idle_to_shiftdr)

    chain_end = False
    taps = []
    while not chain_end:
        (rsp, tdo_seq) = device.jtag_sequence(tms_seq_readid, tdi_seq_readid)
        if rsp != 0:
            raise Exception('error: jtag_sequence returned %d' % rsp)
        dev_id_reg = dev_id_reg_from_seq(tdo_seq)
        if dev_id_reg == 0x00000000:
            chain_end = True
            jtag_tap_state_move(device, tms_seq_idle)
        else:
            taps.append(dev_id_reg)
            jtag_tap_state_move(device, tms_seq_exitdr_to_shiftdr)

    return taps

if __name__ == '__main__':
    jtagduino_port = '/dev/ttyACM0'
    jtagduino_serial_baud = max(jtagduino.BAUD_RATES)
    jtagduino_jtag_khz = jtagduino.MAX_JTAG_SPEED_KHZ

    if (len(sys.argv) > 1):
        jtagduino_port = sys.argv[1]

    device = jtagduino(jtagduino_port)
    rsp = device.set_serial_speed(jtagduino_serial_baud)
    if rsp != 0: raise Exception('error: set_serial_speed returned %d' % rsp)
    rsp = device.set_jtag_speed(jtagduino_jtag_khz)
    if rsp != 0: raise Exception('error: set_jtag_speed returned %d' % rsp)

    taps = read_idcodes(device)

    print '#Taps = %d' % len(taps)
    print 'Device identifier registers (beginning from closest to TDO):'
    for tap in taps:
        print '0x%08X' % tap

