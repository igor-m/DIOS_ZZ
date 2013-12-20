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






static jmp_buf _excep_buf;
jmp_buf coldstart;


int tmp;



void setup() {
  Serial.begin(115200);
#ifdef WITH_UART  
  Serial1.begin(UART_BAUD);
#endif

  tmp = setjmp(_excep_buf);
  if (tmp) {
    asm volatile( "eret" );
  }


#ifdef WITH_BREAK
  pinMode(BREAK_PIN, INPUT);
#endif
}

void loop () {
  setjmp(coldstart);
#ifdef WITH_ISR
    initIsr();
#endif
  _cold();
}



/*******************************************************************************
 * Exception handling
 * If an exception occured, reexecute the "loop" function.
 * In application can read exception specific information (getexceptioninfo)
 * After readed these informations then zeroized it.
 ******************************************************************************/
//***************************************************
#define EXCEPTION_NUM_EXCEPTIONS 14

// declared static in case exception condition would prevent
// auto variable being created
static enum {
   EXCEP_IRQ = 0,         // interrupt
   EXCEP_AdEL = 4,         // address error exception (load or ifetch)
   EXCEP_AdES,            // address error exception (store)
   EXCEP_IBE,            // bus error (ifetch)
   EXCEP_DBE,            // bus error (load/store)
   EXCEP_Sys,            // syscall
   EXCEP_Bp,            // breakpoint
   EXCEP_RI,            // reserved instruction
   EXCEP_CpU,            // coprocessor unusable
   EXCEP_Overflow,         // arithmetic overflow
   EXCEP_Trap,            // trap (possible divide by zero)
   EXCEP_IS1 = 16,         // implementation specfic 1
   EXCEP_CEU,            // CorExtend Unuseable
   EXCEP_C2E            // coprocessor 2
} _excep_codes;

static unsigned int _excep_code; // exception code corresponds to _excep_codes
static unsigned int _excep_addr; // exception address
static unsigned int _excep_stat; // status register
static unsigned int mod_addr; // Skip 


#ifdef __cplusplus
extern "C" {
#endif
void _general_exception_handler(unsigned cause, unsigned status) {
  _excep_code = (cause & 0x0000007C) >> 2;
  _excep_stat = status;
  _excep_addr = __builtin_mfc0(_CP0_EPC, _CP0_EPC_SELECT);

// After an exteption we continue with "loop", which is initialise the whole system except USB !
  mod_addr = (unsigned int)((void*)loop);
  asm volatile("mtc0 %0,$14" :: "r" (mod_addr));  
//  asm volatile( "eret" );
  Serial.println("\n\rException occured !\n\rUse \"exceptioninfo\" !\n\r");
  longjmp(_excep_buf, _excep_code);
}
#ifdef __cplusplus
}
#endif


//*  
//* getexceptioninfo ( --- code stat addr )
//*    Leave on the stack the datas which is stored by exception handler into the first 3 words of EEPROM
//*    After fetching, erases desired EEPROM words.
    void getExceptionInfo(void) {
      PUSH(_excep_code);
      PUSH(_excep_stat);
      PUSH(_excep_addr);
      _excep_code = 0;
      _excep_stat = 0;
      _excep_addr = 0;
    }


//***************************************************



