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
 * This file contains extensions of dictionary with user defined C/C++ words.
 ******************************************************************************/


//	{2,  pr|8,     "?extdict",   (void *) Fisextdict}, 
//	{2,  pr|7,     "extdict",    (void *) Fextdict}, 
        {18, pr|4,     "dump",       (void *) Fdump}, 
	{5,  pr|2,     "++",         (void *) plusplus}, 
	{5,  pr|2,     "--",         (void *) minusminus}, 
	{5,  pr|1,     "?",          (void *) question}, 
	{5,  pr|2,     "u?",         (void *) uquestion}, 
	{5,  pr|6,     "millis",     (void *) Fmillis}, 
	{12, pr|6,     "micros",     (void *) Fmicros}, 
	{6,  pr|2,     "pm",         (void *) Fpinmode}, 
	{7,  pr|2,     "d!",         (void *) Fdigitalwrite}, 
	{10, pr|2,     "d@",         (void *) Fdigitalread}, 
	{11, pr|2,     "a@",         (void *) Fanalogread},
#ifdef WITH_SOFTPWM
	{11, pr|2,     "a!",         (void *) Fpwmwrite}, 
	{11, pr|2,     "s!",         (void *) Fservowrite}, 
#else
	{11, pr|2,     "a!",         (void *) Fanalogwrite}, 
#endif

	{11, pr|4,     "tone",       (void *) Ftone},
	{11, pr|6,     "notone",     (void *) no_tone},
	{11, pr|8,     "shiftout",   (void *) shiftout},
	{11, pr|7,     "shiftin",    (void *) shiftin},
	{11, pr|7,     "pulsein",    (void *) pulsein},

#ifdef WITH_WIRE
	{10, pr|8,     "i2c_init",   (void *) wire_begin}, 
	{10, pr|7,     "i2c_req",    (void *) wire_requestFrom}, 
	{10, pr|9,     "i2c_begin",  (void *) wire_beginTransmission}, 
	{10, pr|8,     "i2c_done",   (void *) wire_endTransmission}, 
	{10, pr|4,     "?i2c",       (void *) wire_available}, 
	{10, pr|4,     "i2c@",       (void *) wire_receive}, 
	{10, pr|8,     "i2c_type",   (void *) wire_send}, 
	{10, pr|4,     "i2c!",       (void *) wire_send_byte}, 
//	{10, pr|5,     "i2c!",       (void *) wire_send_word}, 
#endif



#ifdef WITH_SPI
	{10, pr|8,     "spi_init",      (void *) spi_begin}, 
	{10, pr|8,     "spi_done",      (void *) spi_end}, 
	{10, pr|12,    "spi_bitorder",  (void *) spi_setBitOrder}, 
	{10, pr|12,    "spi_clockdiv",  (void *) spi_setClockDivider}, 
	{10, pr|8,     "spi_mode",      (void *) spi_setDataMode}, 
	{10, pr|11,    "spi_transfer",  (void *) spi_transfer}, 
#endif

#ifdef WITH_ISR
	{2,  pr|2,     "ei",            (void *) isrenable}, 
	{2,  pr|2,     "di",            (void *) isrdisable}, 
//	{2,  pr|7,     "isrmask",       (void *) isrmask}, 
//	{2,  pr|9,     "isrsource",     (void *) isrsource}, 
//	{2,  pr|7,     "isrdata",       (void *) isrdata}, 
	{2,  pr|8,     "isrdata@",      (void *) isrdatafetch}, 
//	{2,  pr|12,    "ISR_DATASIZE",  (void *) c_isrdatasize}, 
//	{2,  pr|8,     "isrwords",      (void *) isrwords}, 
//	{2,  pr|6,     "isrxts",        (void *) isrxts}, 
//	{2,  pr|6,     "setisr",        (void *) setisr}, 
//	{2,  pr|9,     "enableisr",     (void *) enableisr}, 
//	{2,  pr|10,    "disableisr",    (void *) disableisr}, 
#ifdef WITH_CORETIM_ISR
	{2,  pr|6,     "uptime",        (void *) uptime}, 
#ifdef WITH_LOAD_INDICATOR
        {00, pr|4,     "load",          (void *) load},
#endif  // WITH_LOAD_INDICATOR
//	{2,  pr|13,    "ISR_CORETIMER", (void *) c_coretimer}, 
#endif  // WITH_CORETIM_ISR
#ifdef WITH_PINCHANGE_ISR
#if defined(__PIC32MX2XX__)
//	{2,  pr|13,    "ISR_PINCHANGE", (void *) c_pinchange}, 
	{2,  pr|7,     "pintocn",       (void *) pinToCN}, 
	{2,  pr|9,     "pinfromcn",     (void *) pinFromCN}, 
	{2,  pr|10,    "?pinchange",    (void *) pinchanged}, 
#endif // #if defined(__PIC32MX2XX__)
#endif // WITH_PINCHANGE_ISR
#endif  // WITH_ISR

#ifdef WITH_LCD
	{2,  pr|8,     "lcd_init",     (void *) lcd_init}, 
	{2,  pr|9,     "lcd_begin",    (void *) lcd_begin}, 
	{2,  pr|9,     "lcd_clear",    (void *) lcd_clear}, 
	{2,  pr|8,     "lcd_home",     (void *) lcd_home}, 
	{2,  pr|8,     "lcd_goto",     (void *) lcd_setcursor}, 
	{2,  pr|10,    "(lcd_emit)",   (void *) lcd_emit}, 
	{2,  pr|10,    "lcd_cursor",   (void *) lcd_cursor}, 
	{2,  pr|12,    "lcd_nocursor", (void *) lcd_nocursor}, 
	{2,  pr|9,     "lcd_blink",    (void *) lcd_blink}, 
	{2,  pr|11,    "lcd_noblink",  (void *) lcd_noblink}, 
#endif // #ifdef WITH_LCD

#ifdef WITH_OW
	{2,  pr|9,     "ow_power!",       (void *) Fow_power}, 
	{2,  pr|8,     "ow_reset",        (void *) Fow_reset}, 
	{2,  pr|5,     "ow_b!",           (void *) Fow_write_bit}, 
	{2,  pr|5,     "ow_b@",           (void *) Fow_read_bit}, 
	{2,  pr|5,     "ow_c!",           (void *) Fow_write}, 
	{2,  pr|5,     "ow_c@",           (void *) Fow_read}, 
	{2,  pr|8,     "ow_write",        (void *) Fow_write_bytes}, 
	{2,  pr|7,     "ow_read",         (void *) Fow_read_bytes}, 
	{2,  pr|9,     "ow_select",       (void *) Fow_select}, 
	{2,  pr|7,     "ow_skip",         (void *) Fow_skip}, 
	{2,  pr|10,    "ow_depower",      (void *) Fow_depower}, 
	{2,  pr|15,    "ow_reset_search", (void *) Fow_reset_search}, 
	{2,  pr|9,     "ow_search",       (void *) Fow_search}, 
	{2,  pr|7,     "ow_crc8",         (void *) Fow_crc8}, 

#endif // #ifdef WITH_OW

#if defined(__PIC32MX2XX__)
/*******************************************************************************
 * PPS
 ******************************************************************************/
#ifdef WITH_PPS
{ 0, pr|4, "pps!",    (void *) pps},
{ 0, pr|4, "GPIO",    (void *) F_PPS_OUT_GPIO},
{ 0, pr|4, "U1TX",    (void *) F_PPS_OUT_U1TX},
{ 0, pr|5, "U2RTS",   (void *) F_PPS_OUT_U2RTS},
{ 0, pr|3, "SS1",     (void *) F_PPS_OUT_SS1},
{ 0, pr|3, "OC1",     (void *) F_PPS_OUT_OC1},
{ 0, pr|5, "C2OUT",   (void *) F_PPS_OUT_C2OUT},
{ 0, pr|4, "SDO1",    (void *) F_PPS_OUT_SDO1},
{ 0, pr|4, "SDO2",    (void *) F_PPS_OUT_SDO2},
{ 0, pr|3, "OC2",     (void *) F_PPS_OUT_OC2},
{ 0, pr|3, "OC4",     (void *) F_PPS_OUT_OC4},
{ 0, pr|3, "OC5",     (void *) F_PPS_OUT_OC5},
{ 0, pr|7, "REFCLKO", (void *) F_PPS_OUT_REFCLKO},
{ 0, pr|5, "U1RTS",   (void *) F_PPS_OUT_U1RTS},
{ 0, pr|4, "U2TX",    (void *) F_PPS_OUT_U2TX},
{ 0, pr|3, "SS2",     (void *) F_PPS_OUT_SS2},
{ 0, pr|3, "OC3",     (void *) F_PPS_OUT_OC3},
{ 0, pr|5, "C1OUT",   (void *) F_PPS_OUT_C1OUT},
{ 0, pr|4, "INT1",    (void *) F_PPS_IN_INT1},
{ 0, pr|4, "INT2",    (void *) F_PPS_IN_INT2},
{ 0, pr|4, "INT3",    (void *) F_PPS_IN_INT3},
{ 0, pr|4, "INT4",    (void *) F_PPS_IN_INT4},
{ 0, pr|4, "T2CK",    (void *) F_PPS_IN_T2CK},
{ 0, pr|4, "T3CK",    (void *) F_PPS_IN_T3CK},
{ 0, pr|4, "T4CK",    (void *) F_PPS_IN_T4CK},
{ 0, pr|4, "T5CK",    (void *) F_PPS_IN_T5CK},
{ 0, pr|3, "IC1",     (void *) F_PPS_IN_IC1},
{ 0, pr|3, "IC2",     (void *) F_PPS_IN_IC2},
{ 0, pr|3, "IC3",     (void *) F_PPS_IN_IC3},
{ 0, pr|3, "IC4",     (void *) F_PPS_IN_IC4},
{ 0, pr|3, "IC5",     (void *) F_PPS_IN_IC5},
{ 0, pr|4, "OCFA",    (void *) F_PPS_IN_OCFA},
{ 0, pr|4, "OCFB",    (void *) F_PPS_IN_OCFB},
{ 0, pr|4, "U1RX",    (void *) F_PPS_IN_U1RX},
{ 0, pr|5, "U1CTS",   (void *) F_PPS_IN_U1CTS},
{ 0, pr|4, "U2RX",    (void *) F_PPS_IN_U2RX},
{ 0, pr|5, "U2CTS",   (void *) F_PPS_IN_U2CTS},
{ 0, pr|4, "SDI1",    (void *) F_PPS_IN_SDI1},
{ 0, pr|3, "SS1",     (void *) F_PPS_IN_SS1},
{ 0, pr|4, "SDI2",    (void *) F_PPS_IN_SDI2},
{ 0, pr|3, "SS2",     (void *) F_PPS_IN_SS2},
{ 0, pr|7, "REFCLKI", (void *) F_PPS_IN_REFCLKI},
#endif //#ifdef WITH_PPS
#endif //#if defined(__PIC32MX2XX__)

