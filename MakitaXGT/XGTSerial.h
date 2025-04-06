#include <inttypes.h>
#include "OneWire_direct_regtype.h"
#include "OneWire_direct_gpio.h"

#ifndef XGTSerial_h
#define XGTSerial_h


class XGTSerial {
  public:
    //the makita uart implementation is not so tolerant so each cpu cycle counts here
    XGTSerial(uint8_t pin) {
      pinMode(pin, INPUT);
      bitmask = PIN_TO_BITMASK(pin);
      baseReg = PIN_TO_BASEREG(pin);
      DIRECT_WRITE_LOW(baseReg,bitmask);

      #ifdef ESP32
           _tx_delay = 1000000/9600;
      #else
      _tx_delay = ((F_CPU / 9600) >> 2) - 3;
      #endif
    }

  void write(uint8_t * buff, int len) {
    for (int i = 0; i < len; i++) write(buff[i]);
  }

  void read(uint8_t * buff, int len) {
    //noInterrupts();
    for (int i = 0; i < len; i++) buff[i] = recv();
   // interrupts();
  }

  int recv() {
     uint8_t data = 0;


    //noInterrupts();

    DIRECT_MODE_INPUT(baseReg, bitmask);

    //wait until we find the start bit
    int waitIter = 4096;
    for (; waitIter > 0 && !DIRECT_READ(baseReg, bitmask); waitIter--) _delay_loop_2(10);
    if (waitIter == 0) return 0;

    //wait half a bit to poll in the center, then another wait to skip the start bit ( at the beginning of the loop )
    _delay_loop_2(_tx_delay >> 1);

    // Read each of the 8 bits
    for (uint8_t i = 0X80; i > 0; i >>= 1) {
      _delay_loop_2(_tx_delay);
      if (!DIRECT_READ(baseReg, bitmask)) data |= i;
    }

    // skip the parity and stop bits
    _delay_loop_2(_tx_delay << 1);

    //interrupts();

    return data;
  }

  size_t write(uint8_t b) {
    DIRECT_MODE_OUTPUT(baseReg, bitmask);

    uint8_t parity = 0;
    for (uint8_t t = 1; t; t <<= 1)
      if (!(b & t)) parity++;

    // Write the start bit
    DIRECT_WRITE_HIGH(baseReg, bitmask);

    _delay_loop_2(_tx_delay);

    // Write each of the 8 bits
    for (uint8_t i = 0X80; i > 0; i >>= 1) {
      if (b & i) // choose bit
        DIRECT_WRITE_LOW(baseReg, bitmask);
      else
        DIRECT_WRITE_HIGH(baseReg, bitmask);

      _delay_loop_2(_tx_delay);
    }

    // parity
    if (parity & 0x01)
      DIRECT_WRITE_LOW(baseReg, bitmask);
    else
      DIRECT_WRITE_HIGH(baseReg, bitmask);

    _delay_loop_2(_tx_delay);

    // restore pin to natural state
    DIRECT_WRITE_LOW(baseReg, bitmask);

    _delay_loop_2(_tx_delay);

    return 1;
  }


  private: 

  #ifdef ESP32
     inline  void     _delay_loop_2(uint16_t delay){
        esp_rom_delay_us(delay);
      }
  #endif
  IO_REG_TYPE bitmask;
  volatile IO_REG_TYPE * baseReg;
  uint16_t _tx_delay;
};

#endif