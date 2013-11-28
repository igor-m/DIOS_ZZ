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
 

/*******************************************************************************
 * Autoload
 ******************************************************************************/

const extern char *autoload;
extern char *autoload_ptr;
extern int extdict_loaded;
void extdict(void);

/*******************************************************************************
 * Forward declarations of functions in "uw.cpp"
 ******************************************************************************/


extern void crf(void);
extern void emit(void);

extern void plusplus(void);
extern void minusminus(void);
extern void question(void);
extern void uquestion(void);
extern void Fpinmode();
extern void Fdigitalwrite();
extern void Fdigitalread();
extern void Fanalogread();
extern void Fmillis();
extern void Fmicros();

#ifdef WITH_SOFTPWM
extern void Fpwmwrite(void);
extern void Fservowrite(void);
#else
extern void Fanalogwrite(void);
#endif

extern void Fafetch(void);
extern void Fastore(void);
extern void Facfetch(void);
extern void Facstore(void);

extern void Ffindmagic(void);
extern void FFindFreeFlash(void);

extern void Fdump(void);




extern void Ftone(void);
extern void no_tone(void);
extern void shiftout(void);
extern void shiftin(void);
extern void pulsein(void);


#ifdef WITH_WIRE
extern void wire_begin(void);
extern void wire_requestFrom(void);
extern void wire_beginTransmission(void);
extern void wire_endTransmission(void);
extern void wire_available(void);
extern void wire_receive(void);
extern void wire_send(void);
extern void wire_send_byte(void);
extern void wire_send_word(void);
#endif

#ifdef WITH_SPI
extern void spi_begin(void);
extern void spi_end(void);
extern void spi_setBitOrder(void);
extern void spi_setClockDivider(void);
extern void spi_setDataMode(void);
extern void spi_transfer(void);
#endif



extern uint8_t portbit_to_pin(int port, int portbit);
extern void portbit2pin(void);
extern void portmask2pin(void);
extern void mask2bit(void);


#ifdef WITH_LCD
extern uint32_t lcd_active;


extern void lcd_init(void);
extern void lcd_begin(void);
extern void lcd_clear(void);
extern void lcd_home(void);
extern void lcd_setcursor(void);
extern void lcd_emit(void);
extern void lcd_on(void);
extern void lcd_off(void);
#endif // #ifdef WITH_LCD

#ifdef WITH_OW
#include <inttypes.h>

#ifndef ONEWIRE_SEARCH
#define ONEWIRE_SEARCH 1
#endif

#ifndef ONEWIRE_CRC
#define ONEWIRE_CRC 1
#endif



#ifndef ONEWIRE_CRC16
#define ONEWIRE_CRC16 1
#endif


#define PIN_TO_BASEREG(pin)             (portModeRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (((*(base+4)) & (mask)) ? 1 : 0)  //PORTX + 0x10
#define DIRECT_MODE_INPUT(base, mask)   ((*(base+2)) = (mask))            //TRISXSET + 0x08
#define DIRECT_MODE_OUTPUT(base, mask)  ((*(base+1)) = (mask))            //TRISXCLR + 0x04
#define DIRECT_WRITE_LOW(base, mask)    ((*(base+8+1)) = (mask))          //LATXCLR  + 0x24
#define DIRECT_WRITE_HIGH(base, mask)   ((*(base+8+2)) = (mask))          //LATXSET + 0x28




// ( p --- )
extern void Fow_power(void);
// ( pin --- f )
extern void Fow_reset(void);
// ( v pin --- )  
extern void Fow_write_bit(void);
// ( pin --- v )
extern void Fow_read_bit(void);
// ( b pin --- )
extern void Fow_write(void);
// ( addr cnt pin --- )
extern void Fow_write_bytes(void);
// ( pin --- b )
extern void Fow_read(void);
// ( addr cnt pin --- )
extern void Fow_read_bytes(void);
// ( romh roml pin --- )
extern void Fow_select(void);
// ( pin --- )
extern void Fow_skip(void);
// ( pin --- )
extern void Fow_depower(void);
// ( --- )
extern void Fow_reset_search(void);
// ()
extern void Fow_search(void);
// ( addr len --- f )
extern void Fow_crc8(void);

#endif // #ifdef WITH_OW



