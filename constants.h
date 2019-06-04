#pragma once

namespace alpaca {

const byte I2C_ADDR = 0x60;

const byte CMD_START = 0x81;
const byte CMD_SEND_DIGITAL = 0x90;
const byte CMD_SEND_ANALOG = 0xA0;
const byte CMD_SEND_ANALOG_RAW = 0xB0;
const byte CMD_RESPONSE = 0xC0;
const byte CMD_END = 0x80;

const byte CMD_TARGET = 0xD0;
const byte CMD_TARGET_JACK0 = 0x0;
const byte CMD_TARGET_JACK1 = 0x1;
const byte CMD_TARGET_JACK_BOTH = 0x2;
const byte CMD_TARGET_MATRIX = 0x3;
const byte CMD_JACK_SET_MODE = 0x10;
const byte CMD_MATRIX_SET_BRIGHTNESS = 0x10;
const byte CMD_MATRIX_SET_PIXEL = 0x20;
const byte CMD_MATRIX_CLEAR_PIXEL = 0x30;
const byte CMD_MATRIX_CLEAR = 0x40;
const byte CMD_MATRIX_FILL = 0x50;
const byte CMD_MATRIX_DRAW = 0x60;
const byte CMD_MATRIX_CHAR = 0x70;

enum JackMode {
  MODE_NONE = 0,
  MODE_ANALOG = 1,
  MODE_DIGITAL1 = 2,
  MODE_DIGITAL2 = 3,
  MODE_DIGITAL_BOTH = 4,
  MODE_ANALOG_RAW = 5
};

enum ResponseCode {
  OK = 0,
  EMPTY_COMMAND = 1,
  NO_TARGET = 2,
  UNKNOWN_TARGET = 3,
  UNKNOWN_COMMAND = 4,
  NO_CHAR = 5
};

}
