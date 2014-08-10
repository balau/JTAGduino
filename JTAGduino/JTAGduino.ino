/*   This file is part of JTAGduino project.
 *
 *   Copyright 2011 Francesco Balducci <balau@users.sourceforge.net>
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

void set_array_bit(int i_bit, byte *data, int value) {
  int i_byte;
  byte mask;
  
  i_byte = i_bit >> 3; // floor(i_bit/8)
  mask = 1<<(i_bit&0x7);
  if(value == 0) {
    data[i_byte] &= ~mask;
  } else {
    data[i_byte] |= mask;
  }
}

int get_array_bit(int i_bit, const byte *data) {
  int i_byte;
  byte mask;
  i_byte = i_bit >> 3; // floor(i_bit/8)
  mask = 1<<(i_bit&0x7);
  return ((data[i_byte]&mask)==0)?0:1;
}

/* JTAG and digital pins functionality */

enum jtag_pins {
  TCK = 0,
  TMS = 1,
  TDI = 2,
  TDO = 3,
  TRST = 4,
  
  N_JTAG_PINS = 5,
};

enum jtag_errors {
  JTAG_NO_ERROR = 0,
  JTAG_ERROR_BAD_PIN = -1,
  JTAG_ERROR_BAD_SPEED = -2,
  JTAG_ERROR_BAD_SEQUENCE_LEN = -3,
};

enum jtag_constants {
  JTAG_MAX_SPEED_KHZ = 500,
  JTAG_MAX_SEQUENCE_LEN = 256,
  JTAG_MAX_SEQUENCE_LEN_BYTES = JTAG_MAX_SEQUENCE_LEN/8, //32
};

int jtag_pin_map[N_JTAG_PINS] = { 2, 3, 4, 5, 6 };
const int jtag_pin_dir[N_JTAG_PINS] = { OUTPUT, OUTPUT, OUTPUT, INPUT, OUTPUT };
unsigned long jtag_last_tck_micros;
unsigned long jtag_min_tck_micros;

void jtag_setup() {
  int i_pin;
  for(i_pin = 0; i_pin < N_JTAG_PINS; i_pin++) {
    pinMode(jtag_pin_map[i_pin], jtag_pin_dir[i_pin]);
  }
  jtag_last_tck_micros = micros();
  jtag_min_tck_micros = 1;
}

int jtag_write_pin(int pin, int value) {
  if(pin >= N_JTAG_PINS || pin < 0) {
    return JTAG_ERROR_BAD_PIN;
  }
  if(jtag_pin_dir[pin] != OUTPUT) {
    return JTAG_ERROR_BAD_PIN;
  }
  digitalWrite(jtag_pin_map[pin], value);
  return JTAG_NO_ERROR;
}

int jtag_set_pin(int pin) {
  return jtag_write_pin(pin, HIGH);
}

int jtag_clear_pin(int pin) {
  return jtag_write_pin(pin, LOW);
}

int jtag_get_pin(int pin) {
  if(pin >= N_JTAG_PINS || pin < 0) {
    return JTAG_ERROR_BAD_PIN;
  }
  if(jtag_pin_dir[pin] != INPUT) {
    return JTAG_ERROR_BAD_PIN;
  }
  if(digitalRead(jtag_pin_map[pin]) == HIGH)
    return 1;
  else
    return 0;  
}

int jtag_pulse_high(int pin, unsigned int us) {
  int jtag_error;
  jtag_error = jtag_set_pin(pin);
  if(jtag_error != JTAG_NO_ERROR) {
    return jtag_error;
  }
  delayMicroseconds(us);
  jtag_error = jtag_clear_pin(pin);
  if(jtag_error != JTAG_NO_ERROR) {
    return jtag_error;
  }
  return JTAG_NO_ERROR;
}

int jtag_pulse_low(int pin, unsigned int us) {
  int jtag_error;
  jtag_error = jtag_clear_pin(pin);
  if(jtag_error != JTAG_NO_ERROR) {
    return jtag_error;
  }
  delayMicroseconds(us);
  jtag_error = jtag_set_pin(pin);
  if(jtag_error != JTAG_NO_ERROR) {
    return jtag_error;
  }
  return JTAG_NO_ERROR;
}

int jtag_assign_pin(int jtag_pin, int digital_pin) {
  int i_pin;
  if(jtag_pin >= N_JTAG_PINS || jtag_pin < 0) {
    return JTAG_ERROR_BAD_PIN;
  }
  if(digital_pin > 13 || digital_pin < 0) {
    return JTAG_ERROR_BAD_PIN;
  }
  for(i_pin = 0; i_pin < N_JTAG_PINS; i_pin++) {
    if(i_pin != jtag_pin && digital_pin == jtag_pin_map[i_pin]) {
      return JTAG_ERROR_BAD_PIN;
    }
  }
  jtag_pin_map[jtag_pin] = digital_pin;
  pinMode(digital_pin, jtag_pin_dir[jtag_pin]);
  return JTAG_NO_ERROR;
}

int jtag_set_speed(unsigned int khz) {
  if(khz == 0 || khz > JTAG_MAX_SPEED_KHZ) {
    return JTAG_ERROR_BAD_SPEED;
  }
  // Mininum time for TCK to be stable is half the clock period.
  // For 100kHz of TCK frequency the period is 10us so jtag_min_tck_micros is 5us.
  jtag_min_tck_micros = (500U + khz - 1)/khz; //ceil
  
  return JTAG_NO_ERROR;
}

int jtag_clock(int tms, int tdi) {
  unsigned long cur_micros;
  int tdo;
  
  // Setting TDI and TMS before rising edge of TCK.
  jtag_write_pin(TDI, tdi);
  jtag_write_pin(TMS, tms);
  // Waiting until TCK has been stable for at least jtag_min_tck_micros.
  cur_micros = micros();
  if(cur_micros < jtag_last_tck_micros + jtag_min_tck_micros) {
    delayMicroseconds(jtag_last_tck_micros + jtag_min_tck_micros - cur_micros);
  }
  tdo = jtag_get_pin(TDO); // TDO changes on falling edge of TCK, we are reading value changed during last jtag_clock.
  jtag_set_pin(TCK); // Rising edge of TCK. TDI and TMS are sampled in.
  delayMicroseconds(jtag_min_tck_micros);
  jtag_clear_pin(TCK); // Falling edge of TCK. TDO is changing.
  jtag_last_tck_micros = micros(); // Saving timestamp of last TCK change.
  
  return tdo;
}

int jtag_sequence(unsigned int n, const byte *tms, const byte *tdi, byte *tdo) {
  unsigned int i_seq;
  if(n > JTAG_MAX_SEQUENCE_LEN) {
    return JTAG_ERROR_BAD_SEQUENCE_LEN;
  }
  for(i_seq = 0; i_seq < n; i_seq++) {
    set_array_bit(i_seq, tdo,
      jtag_clock(
        get_array_bit(i_seq, tms),
        get_array_bit(i_seq, tdi)
       )
     );
  }
  return JTAG_NO_ERROR;
}

/* JTAGduino command protocol */

enum jtagduino_cmd {
  CMD_IF_VER = 0x1,
  CMD_FW_VER = 0x2,
  CMD_SET_SERIAL_SPEED = 0x3,
  
  CMD_SET_PIN = 0x10,
  CMD_CLEAR_PIN = 0x11,
  CMD_GET_PIN = 0x12,
  CMD_PULSE_HIGH = 0x13,
  CMD_PULSE_LOW = 0x14,
  CMD_ASSIGN_PIN = 0x15,
  
  CMD_SET_JTAG_SPEED = 0x20,
  CMD_JTAG_CLOCK = 0x21,
  CMD_JTAG_SEQUENCE = 0x22,
};

enum jtagduino_rsp {
  RSP_OK = 0,
  RSP_ERROR_BAD_CMD = 1,
  RSP_ERROR_UNKNOWN = 2,
  RSP_ERROR_BAD_PIN = 3,
  RSP_ERROR_BAD_SPEED = 4,
  RSP_BAD_SEQUENCE_LEN = 5,
  RSP_BAD_BAUD = 6,
};

enum jtagduino_constants {
  MAX_RSP_LEN = 1 + JTAG_MAX_SEQUENCE_LEN_BYTES,
  MAX_CMD_LEN = 1 + 1 + JTAG_MAX_SEQUENCE_LEN_BYTES + JTAG_MAX_SEQUENCE_LEN_BYTES,
  IF_VER_MAJOR = 0,
  IF_VER_MINOR = 1,
  FW_VER_MAJOR = 0,
  FW_VER_MINOR = 1,
  N_BAUD_RATES = 11,
};

enum jtagduino_parse_states {
  PARSE_STATE_IDLE = 0,
  PARSE_STATE_CMD = 1,
};

const long baud_rates[N_BAUD_RATES] = {
  300, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
  };
const long DEFAULT_BAUD_RATE = 9600;
byte cmd[MAX_CMD_LEN];
byte rsp[MAX_RSP_LEN];
long new_baud;
int parse_state;
int n_rx_bytes;

void jtagduino_setup() {
  new_baud = 0;
  parse_state = PARSE_STATE_IDLE;
  n_rx_bytes = 0;
}

int jtagduino_parse(byte c) {
  int rsp_len = 0;  
  switch(parse_state) {
    case PARSE_STATE_IDLE:
      switch(c) {
        case CMD_IF_VER:
          rsp[0] = RSP_OK;
          rsp[1] = IF_VER_MINOR & 0xFF;
          rsp[2] = (IF_VER_MINOR >> 8) & 0xFF;
          rsp[3] = IF_VER_MAJOR & 0xFF;
          rsp[4] = (IF_VER_MAJOR >> 8) & 0xFF;
          rsp_len = 5;
          break;
        case CMD_FW_VER:
          rsp[0] = RSP_OK;
          rsp[1] = FW_VER_MINOR & 0xFF;
          rsp[2] = (FW_VER_MINOR >> 8) & 0xFF;
          rsp[3] = FW_VER_MAJOR & 0xFF;
          rsp[4] = (FW_VER_MAJOR >> 8) & 0xFF;
          rsp_len = 5;
          break;
        case CMD_SET_SERIAL_SPEED:
        
        case CMD_SET_PIN:
        case CMD_CLEAR_PIN:
        case CMD_GET_PIN:
        case CMD_PULSE_HIGH:
        case CMD_PULSE_LOW:
        case CMD_ASSIGN_PIN:
        case CMD_SET_JTAG_SPEED:
        case CMD_JTAG_CLOCK:
        case CMD_JTAG_SEQUENCE:
          cmd[0] = c;
          n_rx_bytes = 1;
          parse_state = PARSE_STATE_CMD;
          break;
        default:
          rsp[0] = RSP_ERROR_BAD_CMD;
          rsp_len = 1;
          break;
      }
      break;
    case PARSE_STATE_CMD:
      cmd[n_rx_bytes] = c;
      n_rx_bytes++;
      switch(cmd[0]) {
        case CMD_SET_SERIAL_SPEED:
          if(n_rx_bytes == 5) {
            long baud = 0;
            int i_baud;
            baud += cmd[4];
            baud <<= 8;
            baud += cmd[3];
            baud <<= 8;
            baud += cmd[2];
            baud <<= 8;
            baud += cmd[1];
            rsp[0] = RSP_BAD_BAUD;
            rsp_len = 1;
            for(i_baud = 0; i_baud < N_BAUD_RATES; i_baud++) {
              if(baud_rates[i_baud] == baud) {
                new_baud = baud;
                rsp[0] = RSP_OK;
                break;      
              }              
            }
            parse_state = PARSE_STATE_IDLE;
            n_rx_bytes = 0;

          }
          break;
        case CMD_SET_PIN:
          if(n_rx_bytes == 2) {
            int jtag_err;
            jtag_err = jtag_set_pin(cmd[1]);
            if(jtag_err == JTAG_NO_ERROR) {
              rsp[0] = RSP_OK;
            } else {
              rsp[0] = RSP_ERROR_BAD_PIN;
            }            
            parse_state = PARSE_STATE_IDLE;
            n_rx_bytes = 0;
            rsp_len = 1;
          }
          break;
        case CMD_CLEAR_PIN:
          if(n_rx_bytes == 2) {
            int jtag_err;
            jtag_err = jtag_clear_pin(cmd[1]);
            if(jtag_err == JTAG_NO_ERROR) {
              rsp[0] = RSP_OK;
            } else {
              rsp[0] = RSP_ERROR_BAD_PIN;
            }            
            parse_state = PARSE_STATE_IDLE;
            n_rx_bytes = 0;
            rsp_len = 1;
          }
          break;
        case CMD_GET_PIN:
          if(n_rx_bytes == 2) {
            int val;
            val = jtag_get_pin(cmd[1]);
            if(val == JTAG_ERROR_BAD_PIN) {
              rsp[0] = RSP_ERROR_BAD_PIN;
              rsp_len = 1;
            } else {
              rsp[0] = RSP_OK;
              rsp[1] = val;
              rsp_len = 2;
            }            
            parse_state = PARSE_STATE_IDLE;
            n_rx_bytes = 0;
          }
          break;
        case CMD_PULSE_HIGH:
          if(n_rx_bytes == 4) {
            int jtag_err;
            unsigned int us;
            us = cmd[2] + (cmd[3] << 8);
            jtag_err = jtag_pulse_high(cmd[1], us);
            if(jtag_err == JTAG_NO_ERROR) {
              rsp[0] = RSP_OK;
            } else {
              rsp[0] = RSP_ERROR_BAD_PIN;
            }            
            parse_state = PARSE_STATE_IDLE;
            n_rx_bytes = 0;
            rsp_len = 1;
          }
          break;
        case CMD_PULSE_LOW:
          if(n_rx_bytes == 4) {
            int jtag_err;
            unsigned int us;
            us = cmd[2] + (cmd[3] << 8);
            jtag_err = jtag_pulse_low(cmd[1], us);
            if(jtag_err == JTAG_NO_ERROR) {
              rsp[0] = RSP_OK;
            } else {
              rsp[0] = RSP_ERROR_BAD_PIN;
            }            
            parse_state = PARSE_STATE_IDLE;
            n_rx_bytes = 0;
            rsp_len = 1;
          }
          break;
        case CMD_ASSIGN_PIN:
          if(n_rx_bytes == 3) {
            int jtag_err;
            jtag_err = jtag_assign_pin(cmd[1], cmd[2]);
            if(jtag_err == JTAG_NO_ERROR) {
              rsp[0] = RSP_OK;
            } else {
              rsp[0] = RSP_ERROR_BAD_PIN;
            }
            parse_state = PARSE_STATE_IDLE;
            n_rx_bytes = 0;
            rsp_len = 1;
          }
          break;
        case CMD_SET_JTAG_SPEED:
          if(n_rx_bytes == 3) {
            int jtag_err;
            unsigned int khz;
            khz = cmd[1] + (cmd[2] << 8);
            jtag_err = jtag_set_speed(khz);
            if(jtag_err == JTAG_NO_ERROR) {
              rsp[0] = RSP_OK;
            } else {
              rsp[0] = RSP_ERROR_BAD_SPEED;
            }            
            parse_state = PARSE_STATE_IDLE;
            n_rx_bytes = 0;
            rsp_len = 1;
          }
          break;
        case CMD_JTAG_CLOCK:
          if(n_rx_bytes == 2) {
            int tdi;
            int tms;
            int tdo;
            tms = cmd[1] & 1;
            tdi = (cmd[1] & 2) >> 1;
            tdo = jtag_clock(tms, tdi);
            rsp[0] = RSP_OK;
            rsp[1] = tdo;
            parse_state = PARSE_STATE_IDLE;
            n_rx_bytes = 0;
            rsp_len = 2;
          }
          break;
        case CMD_JTAG_SEQUENCE:
          do {
            int n_bytes;
            n_bytes = (cmd[1]+7)>>3; // ceil(cmd[1]/8)
            if(n_rx_bytes == 1 + 1 + n_bytes*2) {
              int jtag_err;
              jtag_err = jtag_sequence(cmd[1], &cmd[2], &cmd[2+n_bytes], &rsp[1]);
              if(jtag_err == JTAG_NO_ERROR) {
                rsp[0] = RSP_OK;
                rsp_len = 1 + n_bytes;
              } else {
                rsp[0] = RSP_BAD_SEQUENCE_LEN;
                rsp_len = 1;
              }
              parse_state = PARSE_STATE_IDLE;
              n_rx_bytes = 0;
            }
          } while(0);
          break;
        default:
          parse_state = PARSE_STATE_IDLE;
          n_rx_bytes = 0;
          rsp[0] = RSP_ERROR_UNKNOWN;
          rsp_len = 1;
          break;
      }
      break;
    default:
      parse_state = PARSE_STATE_IDLE;
      n_rx_bytes = 0;
      break;    
  }
  return rsp_len;
}

void setup() {
  jtag_setup();
  jtagduino_setup();
  Serial.begin(DEFAULT_BAUD_RATE);
}

void loop() {
  if(Serial.available()) {
    byte c = Serial.read();
    int rsp_len;
    rsp_len = jtagduino_parse(c);
    if(rsp_len > 0) {
      Serial.write(rsp, rsp_len);
    }
    if(new_baud != 0) {
      /* Wait TX complete vefore changing BAUD rate */
      delay(27); /* worst case is 300 baud, so ~27ms to transmit a byte. */
      Serial.begin(new_baud);
      new_baud = 0;
    }
  }
}


