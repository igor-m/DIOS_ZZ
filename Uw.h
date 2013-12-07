/*******************************************************************************
 * ChipKitForth
 * Interactive Forth environment for PIC32 based ChipKit boards.
 * Based on DIOSFORTH. http://www.forth.cz/Download/DIOSForth/DIOSForth.html
 * Developed under MPIDE.
 * Public repository: https://github.com/jvvood/CKF
 * Published under GPLv3.
 * Created by Janos Waldhauser (2013).
 * Email: janos.waldhauser@gmail.com
 ******************************************************************************/
 

/*******************************************************************************
 * extdict
 ******************************************************************************/
#define EXTDICTMARKER  "}ed"
const extern char *extdict;
extern char *extdict_ptr;
void Fextdict(void);
void Fisextdict(void);

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





#ifdef WITH_LCD
extern uint32_t lcd_active;


extern void lcd_init(void);
extern void lcd_begin(void);
extern void lcd_clear(void);
extern void lcd_home(void);
extern void lcd_setcursor(void);
extern void lcd_blink(void);
extern void lcd_noblink(void);
extern void lcd_cursor(void);
extern void lcd_nocursor(void);
extern void lcd_emit(void);
extern void lcd_typef(void);
//extern void lcd_on(void);
//extern void lcd_off(void);
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


#if defined(__PIC32MX2XX__)
/*******************************************************************************
 * PPS
 ******************************************************************************/
#ifdef WITH_PPS
extern void F_PPS_OUT_GPIO(void);
extern void F_PPS_OUT_U1TX(void);
extern void F_PPS_OUT_U2RTS(void);
extern void F_PPS_OUT_SS1(void);
extern void F_PPS_OUT_OC1(void);
extern void F_PPS_OUT_C2OUT(void);
extern void F_PPS_OUT_SDO1(void);
extern void F_PPS_OUT_SDO2(void);
extern void F_PPS_OUT_OC2(void);
extern void F_PPS_OUT_OC4(void);
extern void F_PPS_OUT_OC5(void);
extern void F_PPS_OUT_REFCLKO(void);
extern void F_PPS_OUT_U1RTS(void);
extern void F_PPS_OUT_U2TX(void);
extern void F_PPS_OUT_SS2(void);
extern void F_PPS_OUT_OC3(void);
extern void F_PPS_OUT_C1OUT(void);
extern void F_PPS_IN_INT1(void);
extern void F_PPS_IN_INT2(void);
extern void F_PPS_IN_INT3(void);
extern void F_PPS_IN_INT4(void);
extern void F_PPS_IN_T2CK(void);
extern void F_PPS_IN_T3CK(void);
extern void F_PPS_IN_T4CK(void);
extern void F_PPS_IN_T5CK(void);
extern void F_PPS_IN_IC1(void);
extern void F_PPS_IN_IC2(void);
extern void F_PPS_IN_IC3(void);
extern void F_PPS_IN_IC4(void);
extern void F_PPS_IN_IC5(void);
extern void F_PPS_IN_OCFA(void);
extern void F_PPS_IN_OCFB(void);
extern void F_PPS_IN_U1RX(void);
extern void F_PPS_IN_U1CTS(void);
extern void F_PPS_IN_U2RX(void);
extern void F_PPS_IN_U2CTS(void);
extern void F_PPS_IN_SDI1(void);
extern void F_PPS_IN_SS1(void);
extern void F_PPS_IN_SDI2(void);
extern void F_PPS_IN_SS2(void);
extern void F_PPS_IN_REFCLKI(void);

extern void pps(void);

#endif //#ifdef WITH_PPS
#endif //#if defined(__PIC32MX2XX__)


extern void find_word(char *ptr);
extern void find_and_execute(char *ptr);

