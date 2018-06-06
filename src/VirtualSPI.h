/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * Copyright (c) 2014 by Andrew J. Kroll <xxxajk@gmail.com> (atomicity fixes)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _VirtualSPI_H_INCLUDED
#define _VirtualSPI_H_INCLUDED

#include <Arduino.h>

// HardwareSPI_HAS_TRANSACTION means SPI has beginTransaction(), endTransaction(),
// usingInterrupt(), and SPISetting(clock, bitOrder, dataMode)
#define HardwareSPI_HAS_TRANSACTION 1

// HardwareSPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define HardwareSPI_HAS_NOTUSINGINTERRUPT 1

// HardwareSPI_ATOMIC_VERSION means that SPI has atomicity fixes and what version.
// This way when there is a bug fix you can check this define to alert users
// of your code if it uses better version of this library.
// This also implies everything that HardwareSPI_HAS_TRANSACTION as documented above is
// available too.
#define HardwareSPI_ATOMIC_VERSION 1

// Uncomment this line to add detection of mismatched begin/end transactions.
// A mismatch occurs if other libraries fail to use SPI.endTransaction() for
// each SPI.beginTransaction().  Connect an LED to this pin.  The LED will turn
// on if any mismatch is ever detected.
//#define HardwareSPI_TRANSACTION_MISMATCH_LED 5

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define HardwareSPI_CLOCK_DIV4 0x00
#define HardwareSPI_CLOCK_DIV16 0x01
#define HardwareSPI_CLOCK_DIV64 0x02
#define HardwareSPI_CLOCK_DIV128 0x03
#define HardwareSPI_CLOCK_DIV2 0x04
#define HardwareSPI_CLOCK_DIV8 0x05
#define HardwareSPI_CLOCK_DIV32 0x06

#define HardwareSPI_MODE0 0x00
#define HardwareSPI_MODE1 0x04
#define HardwareSPI_MODE2 0x08
#define HardwareSPI_MODE3 0x0C

#define HardwareSPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define HardwareSPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define HardwareSPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR

// define HardwareSPI_AVR_EIMSK for AVR boards with external interrupt pins
#if defined(EIMSK)
  #define HardwareSPI_AVR_EIMSK  EIMSK
#elif defined(GICR)
  #define HardwareSPI_AVR_EIMSK  GICR
#elif defined(GIMSK)
  #define HardwareSPI_AVR_EIMSK  GIMSK
#endif

class VirtualSPISettings {
public:
  VirtualSPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
    if (__builtin_constant_p(clock)) {
      init_AlwaysInline(clock, bitOrder, dataMode);
    } else {
      init_MightInline(clock, bitOrder, dataMode);
    }
  }
  VirtualSPISettings() {
    init_AlwaysInline(4000000, MSBFIRST, HardwareSPI_MODE0);
  }
  
  void init_MightInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
    init_AlwaysInline(clock, bitOrder, dataMode);
  }
  void init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
    __attribute__((__always_inline__)) {
    // Clock settings are defined as follows. Note that this shows SPI2X
    // inverted, so the bits form increasing numbers. Also note that
    // fosc/64 appears twice
    // SPR1 SPR0 ~SPI2X Freq
    //   0    0     0   fosc/2
    //   0    0     1   fosc/4
    //   0    1     0   fosc/8
    //   0    1     1   fosc/16
    //   1    0     0   fosc/32
    //   1    0     1   fosc/64
    //   1    1     0   fosc/64
    //   1    1     1   fosc/128

    // We find the fastest clock that is less than or equal to the
    // given clock rate. The clock divider that results in clock_setting
    // is 2 ^^ (clock_div + 1). If nothing is slow enough, we'll use the
    // slowest (128 == 2 ^^ 7, so clock_div = 6).
    uint8_t clockDiv;

    // When the clock is known at compiletime, use this if-then-else
    // cascade, which the compiler knows how to completely optimize
    // away. When clock is not known, use a loop instead, which generates
    // shorter code.
    if (__builtin_constant_p(clock)) {
      if (clock >= F_CPU / 2) {
        clockDiv = 0;
      } else if (clock >= F_CPU / 4) {
        clockDiv = 1;
      } else if (clock >= F_CPU / 8) {
        clockDiv = 2;
      } else if (clock >= F_CPU / 16) {
        clockDiv = 3;
      } else if (clock >= F_CPU / 32) {
        clockDiv = 4;
      } else if (clock >= F_CPU / 64) {
        clockDiv = 5;
      } else {
        clockDiv = 6;
      }
    } else {
      uint32_t clockSetting = F_CPU / 2;
      clockDiv = 0;
      while (clockDiv < 6 && clock < clockSetting) {
        clockSetting /= 2;
        clockDiv++;
      }
    }

    // Compensate for the duplicate fosc/64
    if (clockDiv == 6)
    clockDiv = 7;

    // Invert the SPI2X bit
    clockDiv ^= 0x1;

    // Pack into the SPISettings class
    spcr = _BV(SPE) | _BV(MSTR) | ((bitOrder == LSBFIRST) ? _BV(DORD) : 0) |
      (dataMode & HardwareSPI_MODE_MASK) | ((clockDiv >> 1) & HardwareSPI_CLOCK_MASK);
    spsr = clockDiv & HardwareSPI_2XCLOCK_MASK;
  }
  uint8_t spcr;
  uint8_t spsr;
};


class VirtualSPIClass {
public:
  virtual void begin();
  virtual void usingInterrupt(uint8_t interruptNumber);
  virtual void notUsingInterrupt(uint8_t interruptNumber);
  virtual void beginTransaction(VirtualSPISettings settings);
  virtual uint8_t transfer(uint8_t data);
  virtual uint16_t transfer16(uint16_t data);
  virtual void transfer(void *buf, size_t count);
  virtual void endTransaction(void);
  virtual void end();

  virtual void setBitOrder(uint8_t bitOrder);
  virtual void setDataMode(uint8_t dataMode);
  virtual void setClockDivider(uint8_t clockDiv);
  virtual void attachInterrupt();
  virtual void detachInterrupt();
};

#endif
