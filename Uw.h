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
 * This file contains forward declarations of functions in "uw.cpp"
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


/*******************************************************************************
 * Autoload
 ******************************************************************************/

const extern char *autoload;
extern char *autoload_ptr;
extern int extdict_loaded;
void extdict(void);



