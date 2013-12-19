/* 
 *  TODO
 *  a masik nyomo gombbal lehessen megallitani az automatikus sysrestore-t
 *  Megszakítások maszkolási lehetősége
 */


/*
 * EXTERNAL RESET esetén nincs autostart !
 */
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
 
 
/*
 * MPIDE or ChipKit specific extendions:
 *
 * "Uw.cpp" 
 *    contains the interface functions for use ChipKit features in Forth.
 * "Uw.h" 
 *    contains forward declarations of functions in Uw.cpp.
 * "Ud.h" 
 *    integrates finctions from Uw.cpp into the forth dictionary.
 * "exceptions.cpp" 
 *    implement basic exception handling
 * "isr.cpp" and "isr.h" 
 *    implement CoreTimer and PinChangeNotification interrupts in Forth level
 */

 



#include <setjmp.h>

#include "Config.h"
#include "VMword.h"
#include "VMcore.h"



#ifdef WITH_EEPROM
#include <EEPROM.h>
#endif

#ifdef WITH_WIRE
#include <Wire.h>
#endif

#ifdef WITH_SPI
//#include <SPI.h>
#endif


#ifdef WITH_SOFTPWM
#include <SoftPWMServo.h>
#endif

#ifdef WITH_LCD
#include <LiquidCrystal.h>
#endif



jmp_buf coldstart;

void setup() {
  Serial.begin(115200);
#ifdef WITH_UART  
  Serial1.begin(UART_BAUD);
#endif
#ifdef WITH_ISR
    initIsr();
#endif
#ifdef WITH_BREAK
  pinMode(BREAK_PIN, INPUT);
#endif
}

void loop () {
  setjmp(coldstart);
#ifdef WITH_ISR
    initIsr();
#endif
   Serial.println(VerVM);
  _cold();
}

