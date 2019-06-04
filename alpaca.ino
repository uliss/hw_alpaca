// i2c
#include "Wire.h"
#include "TroykaLedMatrix.h"
#include <EEPROM.h>
#include "abc.h"
#include "array.h"
#include "constants.h"

using namespace alpaca;

//const int BAUD = 57600;
//const int BAUD = 38400;
//const int BAUD = 19200;
const int BAUD = 9600;

const int N_INPUTS = 2;
const int N_PINS = 2 * N_INPUTS;
const byte input_pins[N_PINS] = { A1, A0, A3, A2 };
const byte input_led_pixel[N_PINS][2] = { {0, 0}, {0, 7}, {7, 7}, {7, 0} };

JackMode jack_modes[N_INPUTS] = { MODE_DIGITAL_BOTH, MODE_DIGITAL_BOTH };
TroykaLedMatrix matrix(I2C_ADDR);

typedef Array<byte, 10> ByteStack;
ByteStack cmd_stack;

#define L7BIT(v) (v & 0x7F)
#define U7BIT(v) ((v & (~0x007F)) >> 7)

void setup() {
  pinMode(input_pins[0], INPUT_PULLUP);
  pinMode(input_pins[1], INPUT_PULLUP);
  pinMode(input_pins[2], INPUT_PULLUP);
  pinMode(input_pins[3], INPUT_PULLUP);

  analogReference(EXTERNAL);
  Serial.begin(BAUD);
  matrix.begin();
  matrix.clear();

  delay(250);

  matrix.drawRow(0, "\1\1\1\1\1\1\1\1");
  matrix.drawRow(7, "\1\1\1\1\1\1\1\1");
  matrix.drawColumn(0, "\1\1\1\1\1\1\1\1");
  matrix.drawColumn(7, "\1\1\1\1\1\1\1\1");

  delay(250);

  matrix.drawColumn(0, "\0\0\0\0\0\0\0\0");
  matrix.drawColumn(7, "\0\0\0\0\0\0\0\0");
  matrix.drawRow(0, "\0\0\0\0\0\0\0\0");
  matrix.drawRow(7, "\0\0\0\0\0\0\0\0");


  setJackMode(0, MODE_DIGITAL1);
  setJackMode(1, MODE_DIGITAL1);
}

void clearSubStr(byte x, byte side) {
  matrix.clearPixel(x, side ? 0 : 7, false);
  matrix.clearPixel(x, side ? 1 : 6, false);
  matrix.clearPixel(x, side ? 2 : 5, false);
  matrix.clearPixel(x, side ? 3 : 4, false);
  matrix.update();
}

void draw4x2(uint8_t v, byte y_off, byte x_off = 0)
{
  const byte X = 7 - x_off;
  for (byte x = 0; x < 4; x++) {
    bool bit0 = v & (0x1 << (3 - x));
    bool bit1 = v & (0x1 << (7 - x));

    if (bit0)
      matrix.drawPixel(y_off, X - x, false);
    else
      matrix.clearPixel(y_off, X - x, false);

    if (bit1)
      matrix.drawPixel(y_off +  1, X - x, false);
    else
      matrix.clearPixel(y_off + 1, X - x, false);
  }
}


void drawLetter(const uint8_t* l, byte x_offset = 0) {
  draw4x2(l[0], 1, x_offset);
  draw4x2(l[1], 3, x_offset);
  draw4x2(l[2], 5, x_offset);

  matrix.update();
}

void drawChar(int ch, int off) {
  drawLetter(abc[ch - '+'], off);
}

void setJackMode(byte n, JackMode m) {
  if (n >= N_INPUTS) return;
  jack_modes[n] = m;

  switch (m) {
    case MODE_ANALOG:
      clearSubStr(0, n);
      matrix.drawPixel(0, n ? 0 : 7);
      matrix.drawPixel(0, n ? 1 : 6);
      matrix.drawPixel(0, n ? 2 : 5);
      break;
    case MODE_DIGITAL1:
      clearSubStr(0, n);
      matrix.drawPixel(0, n ? 0 : 7);
      break;
    case MODE_DIGITAL2:
      clearSubStr(0, n);
      matrix.drawPixel(0, n ? 1 : 6);
      break;
    case MODE_DIGITAL_BOTH:
      clearSubStr(0, n);
      matrix.drawPixel(0, n ? 0 : 7);
      matrix.drawPixel(0, n ? 1 : 6);
      break;
    case MODE_NONE:
      clearSubStr(0, n);
      break;
    case MODE_ANALOG_RAW:
      matrix.drawPixel(0, n ? 0 : 7);
      matrix.drawPixel(0, n ? 1 : 6);
      matrix.drawPixel(0, n ? 2 : 5);
      matrix.drawPixel(0, n ? 3 : 4);
      break;
  }
}

void sendResultCode(int e) {
  byte buf[] = { CMD_START, CMD_RESPONSE | e, CMD_END };
  Serial.write(buf, sizeof(buf));
}

void jackCommand(byte n, byte cmd) {
  if (cmd == CMD_JACK_SET_MODE) {
    byte mode = cmd_stack[1] & 0x0F;
    setJackMode(n, mode);
  }
  else
    sendResultCode(UNKNOWN_COMMAND);
}

/**
   command format:
   CMD_START (0x81)
   CMD_TARGET (0xD0) | TARGET
   CMD_COMMAND (0x10)
   CMD_END   (0x80)

   0x81, 0xD0, 0xC0 | 0x08, 0x80
*/
void parseCommand() {
  if (cmd_stack.size() < 2)
    return sendResultCode(EMPTY_COMMAND);

  if ((cmd_stack[0] & 0xF0) != CMD_TARGET)
    return sendResultCode(NO_TARGET);

  byte target = cmd_stack[0] & 0x0F;
  byte cmd = cmd_stack[1] & 0xF0;

  switch (target) {
    case CMD_TARGET_JACK0:
      jackCommand(0, cmd);
      break;
    case CMD_TARGET_JACK1:
      jackCommand(1, cmd);
      break;
    case CMD_TARGET_JACK_BOTH:
      jackCommand(0, cmd);
      jackCommand(1, cmd);
      break;
    case CMD_TARGET_MATRIX: {
        switch (cmd) {
          case CMD_MATRIX_CHAR: {
              if (cmd_stack.size() < 4)
                return sendResultCode(NO_CHAR);

              byte lb = cmd_stack[2] & 0x3F;
              byte ub = cmd_stack[3] & 0x3F;
              byte ch = lb | (ub << 6);

              byte off = 0;
              if (cmd_stack.size() > 4)
                off = cmd_stack[4] & 0x8F;

              drawChar(ch, off);
            }
            break;
          case CMD_MATRIX_CLEAR:
            for (byte x = 1; x < 7; x++) {
              for (byte y = 0; y < 8; y++)
                matrix.clearPixel(x, y, false);
            }

            matrix.update();
            break;
        }
      }
      break;
    default:
      return sendResultCode(UNKNOWN_TARGET);
  }

  sendResultCode(OK);
}

void pushCommand(byte v) {
  if (v == CMD_START)
    cmd_stack.clear();
  else if (v == CMD_END)
    parseCommand();
  else {
    if (!cmd_stack.push_back(v))
      cmd_stack.clear();
  }
}

void processSerial() {
  int n = Serial.available();
  if (n < 1)
    return;

  while (n-- > 0)
    pushCommand(Serial.read());
}

void sendDigital(byte n) {
  static int prev_values[N_PINS] = { 1, 1, 1, 1 };

  // pullup input reverted
  int v = digitalRead(input_pins[n]) ? 0 : 1;
  byte y = n < 2 ? 7 : 0;

  if (v)
    matrix.drawPixel(7, y);
  else
    matrix.clearPixel(7, y);

  matrix.update();

  if (prev_values[n] != v) {
    prev_values[n] = v;

    byte buf[] = {CMD_START, CMD_SEND_DIGITAL | (n << 1) | v, CMD_END};
    Serial.write(buf, sizeof(buf));
  }
}

void sendAnalog(byte n) {
  static int prev_values[N_INPUTS] = { 0, 0 };

  int v0 = 1023 - analogRead(input_pins[2 * n]);
  int v1 = 1023 - analogRead(input_pins[2 * n + 1]);
  int v = abs(v0 - v1);

  if (v != prev_values[n]) {
    prev_values[n] = v;
    const byte buf[] = { CMD_START, CMD_SEND_ANALOG | (n << 1), L7BIT(v), U7BIT(v), CMD_END };
    Serial.write(buf, sizeof(buf));
  }
}

void sendAnalogRaw(byte n) {
  int v0 = 1023 - analogRead(input_pins[2 * n]);
  int v1 = 1023 - analogRead(input_pins[2 * n + 1]);
  const byte buf[] = { CMD_START, CMD_SEND_ANALOG_RAW | (n << 1), L7BIT(v0), U7BIT(v0), L7BIT(v1), U7BIT(v1), CMD_END };
  Serial.write(buf, sizeof(buf));
}

void processJack(int n) {
  JackMode m = jack_modes[n];

  switch (m) {
    case MODE_DIGITAL1:
      sendDigital(2 * n);
      break;
    case MODE_DIGITAL2:
      sendDigital(2 * n + 1);
      break;
    case MODE_DIGITAL_BOTH:
      sendDigital(2 * n);
      sendDigital(2 * n + 1);
      break;
    case MODE_ANALOG:
      sendAnalog(n);
      break;
    case MODE_ANALOG_RAW:
      sendAnalogRaw(n);
      break;
    default:
      break;
  }
}

void loop() {
  processSerial();

  processJack(0);
  processJack(1);

}