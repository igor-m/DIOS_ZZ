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
#include "p32_defs.h"
#include "GenericTypeDefs.h"
#include "VMcore.h"
#include "VMword.h"
#include "Uw.h"


/*******************************************************************************
 * Autoload
 ******************************************************************************/
char *autoload_ptr = 0;
int extdict_loaded= 0;
const char *autoload = {
  
": delayms ( ms --- )\r"
"  millis +\r"
"  begin\r"
"    millis over >=\r"
"  until\r"
"  drop\r"
";\r"


": delayus ( us --- )\r"
"  150 - micros +\r"
"  begin\r"
"    micros over >=\r"
"  until\r"
"  drop\r"
";\r"



": delayct ( coretick --- )\r"
"  3150 - coretim +\r"
"  begin\r"
"    coretim over >=\r"
"  until\r"
"  drop\r"
";\r"


" \\ Print number with decimal point.\r"
": (..) ( n dp --- )\r"
"  <#\r"
"    0 do # loop\r"
"    46 hold\r"
"    #s\r"
"  #> \r"
";\r"

": .. \r"
"  (..)\r"
"  type\r"
";\r"


  
};

void extdict(void) {
  if (! extdict_loaded) {
    autoload_ptr = (char *)autoload;
    extdict_loaded = 1;
  } else {
    f_puts("\nDictionary extension already loaded !\n");
  }
}




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


// pulsein ( pin value timeout  --- )
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

UINT i2c_inited = 0;
// i2c_init ( --- )
void wire_begin(void) {
  if ( ! i2c_inited) {
    Wire.begin();
    i2c_inited = 1;
  }
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

// i2c@ ( --- u )
void wire_receive(void) {
  PUSH(Wire.receive());
}

// i2c_type ( addr cnt --- )
void wire_send(void) {
  UINT quantity = POP;
  UINT data = POP;
  Wire.send((uint8_t*)data, (uint8_t)quantity);
}

// i2c! ( c --- )
void wire_send_byte(void) {
  UINT data = POP;
  Wire.send((uint8_t)data);
}

/*
// i2c_! ( u --- )
void wire_send_word(void) {
  UINT data = POP;
  Wire.send((int)data);
}
*/
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



#ifdef WITH_OW

uint8_t ow_power;

uint8_t ow_reset(uint8_t pin) {
  IO_REG_TYPE bitmask = PIN_TO_BITMASK(pin);
  volatile IO_REG_TYPE *baseReg = PIN_TO_BASEREG(pin);
  IO_REG_TYPE mask = bitmask;
  volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
  uint8_t r;
  uint8_t retries = 125;

  pinMode(pin, INPUT);
  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  interrupts();
  // wait until the wire is high... just in case
  do {
    if (--retries == 0) {
      return 0;
    }
    delayMicroseconds(2);
  } while ( !DIRECT_READ(reg, mask));

  noInterrupts();
  DIRECT_WRITE_LOW(reg, mask);
  DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
  interrupts();
  delayMicroseconds(500);
  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);	// allow it to float
  delayMicroseconds(80);
  r = !DIRECT_READ(reg, mask);
  interrupts();
  delayMicroseconds(420);
  return r;
}


void ow_write_bit(uint8_t pin, uint8_t v) {
  IO_REG_TYPE bitmask = PIN_TO_BITMASK(pin);
  volatile IO_REG_TYPE *baseReg = PIN_TO_BASEREG(pin);
  IO_REG_TYPE mask = bitmask;
  volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	if (v & 1) {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(10);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(55);
	} else {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(65);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(5);
	}
}

uint8_t ow_read_bit(uint8_t pin)
{
  IO_REG_TYPE bitmask = PIN_TO_BITMASK(pin);
  volatile IO_REG_TYPE *baseReg = PIN_TO_BASEREG(pin);
  IO_REG_TYPE mask = bitmask;
  volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	uint8_t r;

	noInterrupts();
	DIRECT_MODE_OUTPUT(reg, mask);
	DIRECT_WRITE_LOW(reg, mask);
	delayMicroseconds(3);
	DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
	delayMicroseconds(10);
	r = DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(53);
	return r;
}

void ow_write(uint8_t pin, uint8_t v) {
  IO_REG_TYPE bitmask = PIN_TO_BITMASK(pin);
  volatile IO_REG_TYPE *baseReg = PIN_TO_BASEREG(pin);
  IO_REG_TYPE mask = bitmask;
  volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	ow_write_bit(pin, (bitMask & v)?1:0);
    }
    if ( !ow_power) {
	noInterrupts();
	DIRECT_MODE_INPUT(baseReg, bitmask);
	DIRECT_WRITE_LOW(baseReg, bitmask);
	interrupts();
    }
}

void ow_write_bytes(uint8_t pin, const uint8_t *buf, uint16_t count) {
  IO_REG_TYPE bitmask = PIN_TO_BITMASK(pin);
  volatile IO_REG_TYPE *baseReg = PIN_TO_BASEREG(pin);
  IO_REG_TYPE mask = bitmask;
  volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
  for (uint16_t i = 0 ; i < count ; i++)
    ow_write(pin, buf[i]);
  if (!ow_power) {
    noInterrupts();
    DIRECT_MODE_INPUT(baseReg, bitmask);
    DIRECT_WRITE_LOW(baseReg, bitmask);
    interrupts();
  }
}

uint8_t ow_read(uint8_t pin) {
  IO_REG_TYPE bitmask = PIN_TO_BITMASK(pin);
  volatile IO_REG_TYPE *baseReg = PIN_TO_BASEREG(pin);
  IO_REG_TYPE mask = bitmask;
  volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	if ( ow_read_bit(pin)) r |= bitMask;
    }
    return r;
}

void ow_read_bytes(uint8_t pin, uint8_t *buf, uint16_t count) {
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = ow_read(pin);
}


void ow_select( uint8_t pin, uint8_t rom[8])
{
    int i;
    ow_write(pin, 0x55);           // Choose ROM
    for( i = 0; i < 8; i++) {
      ow_write(pin, rom[i]);
    }
}

void ow_skip(uint8_t pin)
{
    ow_write(pin, 0xCC);           // Skip ROM
}

void ow_depower(uint8_t pin)
{
  IO_REG_TYPE bitmask = PIN_TO_BITMASK(pin);
  volatile IO_REG_TYPE *baseReg = PIN_TO_BASEREG(pin);
  IO_REG_TYPE mask = bitmask;
  volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	noInterrupts();
	DIRECT_MODE_INPUT(baseReg, bitmask);
	interrupts();
}

uint8_t ROM_NO[8];
uint8_t LastDiscrepancy;
uint8_t LastDeviceFlag;
uint8_t LastFamilyDiscrepancy;

void ow_reset_search()
  {
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = FALSE;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--)
    {
    ROM_NO[i] = 0;
    if ( i == 0) break;
    }
  }

uint8_t ow_search(uint8_t pin, uint8_t *newAddr)
{
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number, search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = 0;

   // if the last call was not the last one
   if (!LastDeviceFlag)
   {
      // 1-Wire reset
      if (!ow_reset(pin))
      {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = FALSE;
         LastFamilyDiscrepancy = 0;
         return FALSE;
      }

      // issue the search command
      ow_write(pin, 0xF0);

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = ow_read_bit(pin);
         cmp_id_bit = ow_read_bit(pin);

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1))
            break;
         else
         {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit)
               search_direction = id_bit;  // bit write value for search
            else
            {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy)
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               else
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);

               // if 0 was picked then record its position in LastZero
               if (search_direction == 0)
               {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            ow_write_bit(pin, search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0)
            {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65))
      {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0)
            LastDeviceFlag = TRUE;

         search_result = TRUE;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0])
   {
      LastDiscrepancy = 0;
      LastDeviceFlag = FALSE;
      LastFamilyDiscrepancy = 0;
      search_result = FALSE;
   }
   for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   return search_result;
  }



uint8_t ow_crc8( uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;
	
	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

uint16_t ow_crc16(uint8_t* input, uint16_t len)
{
    static const uint8_t oddparity[16] =
        { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
    uint16_t crc = 0;    // Starting seed is zero.

    for (uint16_t i = 0 ; i < len ; i++) {
      // Even though we're just copying a byte from the input,
      // we'll be doing 16-bit computation with it.
      uint16_t cdata = input[i];
      cdata = (cdata ^ (crc & 0xff)) & 0xff;
      crc >>= 8;

      if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
          crc ^= 0xC001;

      cdata <<= 6;
      crc ^= cdata;
      cdata <<= 1;
      crc ^= cdata;
    }
    return crc;
}

bool ow_check_crc16(uint8_t* input, uint16_t len, uint8_t* inverted_crc)
{
    uint16_t crc = ~ow_crc16(input, len);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}

// ( p --- )
void Fow_power(void) {
  ow_power = POP;
}
//uint8_t ow_reset(uint8_t pin) 
// ( pin --- f )
void Fow_reset(void) {
  UINT pin = POP;
  PUSH(ow_reset(pin));
}

//void ow_write_bit(uint8_t pin, uint8_t v) {
// ( v pin --- )  
void Fow_write_bit(void) {
  UINT pin = POP;
  UINT v = POP;
  v = (v != 0);
  ow_write_bit(pin, v);
}

//uint8_t ow_read_bit(uint8_t pin)
// ( pin --- v )
void Fow_read_bit(void) {
  UINT pin = POP;
  PUSH(ow_read_bit(pin));
}

//void ow_write(uint8_t pin, uint8_t v)
// ( b pin --- )
void Fow_write(void) {
  UINT pin = POP;
  UINT b = POP;
  ow_write(pin, b);
}  

//void ow_write_bytes(uint8_t pin, const uint8_t *buf, uint16_t count) 
// ( addr cnt pin --- )
void Fow_write_bytes(void) {
  UINT pin = POP;
  UINT cnt = POP;
  UINT addr = POP;
  ow_write_bytes(pin, (const uint8_t *)addr, cnt);
}

//uint8_t ow_read(uint8_t pin) 
// ( pin --- b )
void Fow_read(void) {
  UINT pin = POP;
  PUSH(ow_read(pin));
}

//void ow_read_bytes(uint8_t pin, uint8_t *buf, uint16_t count) 
// ( addr cnt pin --- )
void Fow_read_bytes(void) {
  UINT pin = POP;
  UINT cnt = POP;
  UINT addr = POP;
  ow_read_bytes(pin, (uint8_t*)addr, cnt);
}

//void ow_select( uint8_t pin, uint8_t rom[8])
// ( addr pin --- )
void Fow_select(void) {
  int i;
  uint8_t rom[8];
  UINT pin = POP;
  uint8_t *addr = (uint8_t*)POP;
  for(i=0;i<8;++i) {
    rom[i] = *addr;
    ++addr;
  }
  ow_select(pin, rom);
}
//void ow_skip(uint8_t pin)
// ( pin --- )
void Fow_skip(void) {
  UINT pin = POP;
  ow_skip(pin);
}

//void ow_depower(uint8_t pin)
// ( pin --- )
void Fow_depower(void) {
  UINT pin = POP;
  ow_depower(pin);
}

//void ow_reset_search()
// ( --- )
void Fow_reset_search(void) {
  ow_reset_search();
}

//uint8_t ow_search(uint8_t pin, uint8_t *newAddr)
// ( addr pin --- )
void Fow_search(void) {
  UINT pin = POP;
  UINT addr = POP;
  PUSH(ow_search(pin, (uint8_t*)addr));
}

//uint8_t ow_crc8( uint8_t *addr, uint8_t len)
// ( addr len --- f )
void Fow_crc8(void) {
  UINT len = POP;
  UINT addr = POP;
  PUSH(ow_crc8((uint8_t*)addr, len));
}

//uint16_t ow_crc16(uint8_t* input, uint16_t len)
//bool ow_check_crc16(uint8_t* input, uint16_t len, uint8_t* inverted_crc)



#endif // #ifdef WITH_OW


#if defined(__PIC32MX2XX__)
/*******************************************************************************
 * PPS
 ******************************************************************************/
#ifdef WITH_PPS
void F_PPS_OUT_GPIO(void) { PUSH(PPS_OUT_GPIO); }
void F_PPS_OUT_U1TX(void) { PUSH(PPS_OUT_U1TX); }
void F_PPS_OUT_U2RTS(void) { PUSH(PPS_OUT_U2RTS); }
void F_PPS_OUT_SS1(void) { PUSH(PPS_OUT_SS1); }
void F_PPS_OUT_OC1(void) { PUSH(PPS_OUT_OC1); }
void F_PPS_OUT_C2OUT(void) { PUSH(PPS_OUT_C2OUT); }
void F_PPS_OUT_SDO1(void) { PUSH(PPS_OUT_SDO1); }
void F_PPS_OUT_SDO2(void) { PUSH(PPS_OUT_SDO2); }
void F_PPS_OUT_OC2(void) { PUSH(PPS_OUT_OC2); }
void F_PPS_OUT_OC4(void) { PUSH(PPS_OUT_OC4); }
void F_PPS_OUT_OC5(void) { PUSH(PPS_OUT_OC5); }
void F_PPS_OUT_REFCLKO(void) { PUSH(PPS_OUT_REFCLKO); }
void F_PPS_OUT_U1RTS(void) { PUSH(PPS_OUT_U1RTS); }
void F_PPS_OUT_U2TX(void) { PUSH(PPS_OUT_U2TX); }
void F_PPS_OUT_SS2(void) { PUSH(PPS_OUT_SS2); }
void F_PPS_OUT_OC3(void) { PUSH(PPS_OUT_OC3); }
void F_PPS_OUT_C1OUT(void) { PUSH(PPS_OUT_C1OUT); }
void F_PPS_IN_INT1(void) { PUSH(PPS_IN_INT1); }
void F_PPS_IN_INT2(void) { PUSH(PPS_IN_INT2); }
void F_PPS_IN_INT3(void) { PUSH(PPS_IN_INT3); }
void F_PPS_IN_INT4(void) { PUSH(PPS_IN_INT4); }
void F_PPS_IN_T2CK(void) { PUSH(PPS_IN_T2CK); }
void F_PPS_IN_T3CK(void) { PUSH(PPS_IN_T3CK); }
void F_PPS_IN_T4CK(void) { PUSH(PPS_IN_T4CK); }
void F_PPS_IN_T5CK(void) { PUSH(PPS_IN_T5CK); }
void F_PPS_IN_IC1(void) { PUSH(PPS_IN_IC1); }
void F_PPS_IN_IC2(void) { PUSH(PPS_IN_IC2); }
void F_PPS_IN_IC3(void) { PUSH(PPS_IN_IC3); }
void F_PPS_IN_IC4(void) { PUSH(PPS_IN_IC4); }
void F_PPS_IN_IC5(void) { PUSH(PPS_IN_IC5); }
void F_PPS_IN_OCFA(void) { PUSH(PPS_IN_OCFA); }
void F_PPS_IN_OCFB(void) { PUSH(PPS_IN_OCFB); }
void F_PPS_IN_U1RX(void) { PUSH(PPS_IN_U1RX); }
void F_PPS_IN_U1CTS(void) { PUSH(PPS_IN_U1CTS); }
void F_PPS_IN_U2RX(void) { PUSH(PPS_IN_U2RX); }
void F_PPS_IN_U2CTS(void) { PUSH(PPS_IN_U2CTS); }
void F_PPS_IN_SDI1(void) { PUSH(PPS_IN_SDI1); }
void F_PPS_IN_SS1(void) { PUSH(PPS_IN_SS1); }
void F_PPS_IN_SDI2(void) { PUSH(PPS_IN_SDI2); }
void F_PPS_IN_SS2(void) { PUSH(PPS_IN_SS2); }
void F_PPS_IN_REFCLKI(void) { PUSH(PPS_IN_REFCLKI); }

#endif //#ifdef WITH_PPS
#endif //#if defined(__PIC32MX2XX__)


