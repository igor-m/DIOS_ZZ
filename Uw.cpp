/*******************************************************************************
 * ChipKitForth
 * Interactive Forth environment for PIC32 based ChipKit boards.
 * Based on DIOSFORTH. http://www.forth.cz/Download/DIOSForth/DIOSForth.html
 * Developed under MPIDE.
 * Public repository: https://github.com/jvvood/ChipKitForth
 * Published under GPLv3.
 * Created by Janos Waldhauser (2013).
 * Email: janos.waldhauser@gmail.com
 ******************************************************************************/


/**************************************************************************************************
 * This file contains funtions of the UserDefined words, which is written in C.
 * And contains the "interface" functions for call the MPIDE base or library functions from Forth.
 **************************************************************************************************/

#include <plib.h>
#include<p32xxxx.h>
#include <errno.h>
#include <stdarg.h>

#include "WProgram.h"
#include "GenericTypeDefs.h"
#include "VMcore.h"
#include "VMword.h"
#include "Uw.h"


#ifdef WITH_SOFTPWM
extern int SoftPWMServoPWMWrite(uint32_t pin, uint8_t value);
extern int SoftPWMServoServoWrite(uint32_t pin, float value);
extern int32_t SoftPWMServoPinDisable(uint32_t Pin);
#endif

// pm!
void Fpinmode() {
  int pin, mode;
  pin = POP;
  mode = POP;
#ifdef WITH_SOFTPWM
  SoftPWMServoPinDisable(pin);
#endif  
  pinMode(pin, mode);
}

// d!
void Fdigitalwrite() {
  int pin, value;
  pin = POP;
  value = POP;
  digitalWrite(pin, value);
}

// delay
void Fdelay() {
  int ms;
  ms = POP;
  delay(ms);
}

// delayus
void Fdelayus() {
  int us;
  us = POP;
  delayMicroseconds(us);
}


// d@
void Fdigitalread() {
  int pin;
  pin = POP;
  PUSH(digitalRead(pin));
}

// a@
void Fanalogread() {
  int pin;
  pin = POP;
  PUSH(analogRead(pin));
}

// millis
void Fmillis() {
  int m;
  m = millis();
  PUSH(m);
}

// micros
void Fmicros() {
  int m;
  m = micros();
  PUSH(m);
}


#ifdef WITH_SOFTPWM
// a!
void Fpwmwrite(void) {
  int pin;
  int value;
  pin = POP;
  value = POP;
  SoftPWMServoPWMWrite(pin, value);
}


// s!
void Fservowrite(void) {
  int pin;
  int value;
  pin = POP;
  value = POP;
  SoftPWMServoServoWrite(pin, (float)value);
}
#else
void Fanalogwrite(void) {
  int pin;
  int value;
  pin = POP;
  value = POP;
  analogWrite(pin, value);
}
#endif

// dump
void Fdump(void) {
  // (addr, len ---  )
  uint8_t *p;
  int l, ll;
  int i;
  int c;
  l = POP;
  ll = l;
  if (!l) {
    l=1;
  }
  crf();
  emit();
  p = (uint8_t*)POP;
  while (l>0) {
    //    f_printf("%08.8x ", (unsigned int)p);
    f_puthex((unsigned int)p, 8);
    f_puts(" ");
    for(i=0; i<16;++i) {
      if ( (i % 4) == 0) {
        f_puts(" ");
      }
      //      f_printf("%02.2x ", *(p+i));
      f_puthex(*(p+i), 2);
      f_puts(" ");
    }
    for(i=0; i<16;++i) {
      c = *(p+i);
      c &= 0x7f;
      if ( (i % 4) == 0) {
        f_puts(" ");
      }
      if ( (c>=32) && (c<0x7f) ) {
        f_putc(c);
      } 
      else {
        f_puts(".");
      }
    }
    f_puts("\n");
    p += 16;
    l -= 16;
  }
  PUSH((int)p);
  PUSH(ll);
}


// ++ ( addr --- )
void plusplus(void) {
  UINT addr;
  addr = POP;
  ++(pDATA(addr));
}

// -- ( addr --- )
void minusminus(void) {
  UINT addr;
  addr = POP;
  --(pDATA(addr));
}


// ? ( addr --- )
void question(void) {
  TOS=pDATA TOS;
  dot();
}

// u? ( addr --- )
void uquestion(void) {
  TOS=pDATA TOS;
  udot();
}


// tone1 ( pin frq dur --- )
void Ftone(void) {
  UINT dur = POP;
  UINT frq = POP;
  UINT pin = POP;
  tone(pin, frq, dur);
}

// notone ( pin  --- )
void no_tone(void) {
  UINT pin = POP;
  noTone(pin);
}

// shiftout ( dataPin  clockPin  bitOrder value --- )
void shiftout(void) {
  UINT value = POP;
  UINT bitOrder = POP;
  UINT clockPin = POP;
  UINT dataPin = POP;
  shiftOut(dataPin, clockPin, bitOrder, value);
}



uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
	uint8_t value = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		digitalWrite(clockPin, HIGH);
		if (bitOrder == LSBFIRST)
			value |= digitalRead(dataPin) << i;
		else
			value |= digitalRead(dataPin) << (7 - i);
		digitalWrite(clockPin, LOW);
	}
	return value;
}

// shiftout ( dataPin  clockPin  bitOrder --- value )
void shiftin(void) {
  UINT bitOrder = POP;
  UINT clockPin = POP;
  UINT dataPin = POP;
  PUSH((uint8_t)shiftIn(dataPin, clockPin, bitOrder));
}


// pulsein ( pin vale timeout  --- )
void pulsein(void) {
  UINT timeout = POP;
  UINT value = POP;
  UINT pin = POP;
  PUSH(pulseIn(pin, value, timeout));
}



#ifdef WITH_WIRE
/*
void Wire.begin()
 void Wire.requestFrom(address, quantity)
 void Wire.beginTransmission(address)
 int Wire.endTransmission()
 void Wire.send(value)
 void Wire.send(string)
 void Wire.send(data, quantity)
 int Wire.available()
 byte byte Wire.receive()
 */
#include <Wire.h>


// i2c_init ( --- )
void wire_begin(void) {
  Wire.begin();
}

// i2c_req ( addr cnt --- )
void wire_requestFrom(void) {
  UINT quantity = POP;
  UINT address = POP;
  Wire.requestFrom((int)address, (int)quantity);
}

// i2c_begin ( addr --- )
void wire_beginTransmission(void) {
  UINT address = POP;
  Wire.beginTransmission((int)address);
}

// i2c_done ( --- f )
void wire_endTransmission(void) {
  PUSH(Wire.endTransmission());
}

// ?i2c ( --- cnt )
void wire_available(void) {
  PUSH(Wire.available());
}

// i2c_@ ( --- u )
void wire_receive(void) {
  PUSH(Wire.receive());
}

// i2c_type ( addr cnt --- )
void wire_send(void) {
  UINT quantity = POP;
  UINT data = POP;
  Wire.send((uint8_t*)data, (uint8_t)quantity);
}

// i2c_c! ( c --- )
void wire_send_byte(void) {
  UINT data = POP;
  Wire.send((uint8_t)data);
}

// i2c_! ( u --- )
void wire_send_word(void) {
  UINT data = POP;
  Wire.send((int)data);
}
#endif


#ifdef WITH_SPI
#include <SPI.h>

// spi_init ( --- )
void spi_begin(void) {
  SPI.begin();
}

// spi_done ( --- )
void spi_end(void) {
  SPI.end();
}

// spi_bitorder ( order --- )
void spi_setBitOrder(void) {
  UINT order = POP;
  SPI.setBitOrder(order);
}

// spi_clockdiv ( clockdiv --- )
void spi_setClockDivider(void) {
  UINT divider = POP;
  SPI.setClockDivider(divider);
}

// spi_mode ( mode --- )
void spi_setDataMode(void) {
  UINT mode = POP;
  SPI.setDataMode(mode);
}

// spi_transfer ( c --- c )
void spi_transfer(void) {
  UINT val = POP;
  PUSH(SPI.transfer(val));
}

#endif


/*
 * Reverse pin mapping
 * Determine pin number from port and bit.
 */
#if defined(_BOARD_FUBARINO_MINI_)
const uint8_t porta_bit_to_pin[] = {
   5, // RA0
   6, // RA1
  14, // RA2
  15, // RA3
  18, // RA4
  -1, // RA5
  -1, // RA6
   2, // RA7
  16, // RA8
  19, // RA9
   1, // RA10
  -1, // RA11
  -1, // RA12
  -1, // RA13
  -1, // RA14
  -1  // RA15
};
const uint8_t portb_bit_to_pin[] = {
   7, // RB0
   8, // RB1
   9, // RB2
  10, // RB3
  17, // RB4
  23, // RB5
  -1, // RB6
  24, // RB7
  25, // RB8
  26, // RB9
  31, // RB10
  32, // RB11
  -1, // RB12
   0, // RB13
   3, // RB14
   4  // RB15
};
const uint8_t portc_bit_to_pin[] = {
  11, // RC0
  12, // RC1
  13, // RC2
  20, // RC3
  21, // RC4
  22, // RC5
  27, // RC6
  28, // RC7
  29, // RC8
  30, // RC9
  -1, // RC10
  -1, // RC11
  -1, // RC12
  -1, // RC13
  -1, // RC14
  -1  // RC15
};
#endif // defined(_BOARD_FUBARINO_MINI_)
#if defined(_BOARD_DP32_)
const uint8_t porta_bit_to_pin[] = {
  9,  // RA0
  10, // RA1
  15, // RA2
  16, // RA3
  18, // RA4
  -1, // RA5
  -1, // RA6
  -1, // RA7
  -1, // RA8
  -1, // RA9
  -1, // RA10
  -1, // RA11
  -1, // RA12
  -1, // RA13
  -1, // RA14
  -1  // RA15
};
const uint8_t portb_bit_to_pin[] = {
  11, // RB0
  12, // RB1
  13, // RB2
  14, // RB3
  17, // RB4
   0, // RB5
  -1, // RB6
   1, // RB7
   2, // RB8
   3, // RB9
   4, // RB10
   5, // RB11
  -1, // RB12
   6, // RB13
   7, // RB14
   8  // RB15
};
#endif // defined(_BOARD_DP32_)

uint8_t portbit_to_pin(int port, int portbit) {
  if (port == PORTA || port == 0) {
    return porta_bit_to_pin[portbit];
  }
  if (port == PORTB || port == 1) {
    return portb_bit_to_pin[portbit];
  }
#if defined(_BOARD_FUBARINO_MINI_)
  if (port == PORTC || port == 2) {
    return portc_bit_to_pin[portbit];
  }
#endif // #if defined(_BOARD_FUBARINO_MINI_)
  return -1;  
}


// ( port bit --- pin )
void portbit2pin(void) {
  UINT portbit = POP;
  UINT port = POP;
  PUSH(portbit_to_pin(port, portbit));
}
// ( port mask --- pin )
void portmask2pin(void) {
  UINT mask = POP;
  UINT port = POP;
  int i;
  for (i=0;i<32;++i) {
    if ( mask & (1<<i) ) {
      PUSH(portbit_to_pin(port, i));
      return;
    }
  }
}
// ( mask --- bitnum )
void mask2bit(void) {
  UINT mask = POP;
  int i;
  for (i=0;i<32;++i) {
    if ( mask & (1<<i) ) {
      PUSH(i);
      return;
    }
  }
}


#ifdef WITH_LCD
#include <LiquidCrystal.h>
LiquidCrystal lcd(255,255,255,255,255,255);

uint32_t lcd_active = 0;


// ( fourbit rs rw en d0 d1 d2 d3 d4 d5 d6 d7 --- )
void lcd_init(void) {
  UINT d7 = POP;
  UINT d6 = POP;
  UINT d5 = POP;
  UINT d4 = POP;
  UINT d3 = POP;
  UINT d2 = POP;
  UINT d1 = POP;
  UINT d0 = POP;
  UINT en = POP;
  UINT rw = POP;
  UINT rs = POP;
  UINT fourbit = POP;
  lcd.init(fourbit, rs, rw, en, d0, d1, d2, d3, d4, d5, d6, d7);
}

// ( cols rows --- )
void lcd_begin(void) {
  UINT rows = POP;
  UINT cols = POP;
  lcd.begin(cols, rows);
}

// ( --- )
void lcd_clear(void) {
  lcd.clear();
}

// ( --- )
void lcd_home(void) {
  lcd.home();
}

// ( cols rows --- )
void lcd_setcursor(void) {
  UINT row = POP;
  UINT col = POP;
  lcd.setCursor(col, row);
}

// ( c --- )
void lcd_emit(void) {
  lcd.write(POP);
}

// ( --- )
void lcd_on(void) {
  lcd_active = 1;
}

// ( --- )
void lcd_off(void) {
  lcd_active = 0;
}


#endif // #ifdef WITH_LCD

