//** 
//** CKF means ChipKitForth
//** Interactive Forth environment for PIC32 based ChipKit boards.
//** Based on DIOSFORTH. http://www.forth.cz/Download/DIOSForth/DIOSForth.html
//** Developed under MPIDE.
//** Public repository: https://github.com/jvvood/CKF
//** Published under GPLv3.
//** Created by Janos Waldhauser (2013).
//** Email: janos.waldhauser@gmail.com
//** 
//** 
//** 
 
/*
 * Documentation comments:
 * //* at beginning of a line are sign the word description
 * //** at beginnig of a line are sign the generic documentation
 * Documentation can be generate with "wordlist" Bash script
 */

//** 
//** ----------------------------------------------------------------------------------------------
//** 
//** TODO
//** In PinChangeNotification interrupt handle the case with multiple simultaneous pin change.
//** Simplify PWM usage. Now only possibile in forth level. (See exampes/PWM.txt !)
//** Timer handling.
//** IC, OC handling
//** Make all possibile and worthy interrupt handler.
//** 

/*
 * EXTERNAL RESET esetén nincs autostart !
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

  tmp = setjmp(_excep_buf);
  if (tmp) {
    asm volatile( "eret" );
  }


#ifdef WITH_BREAK
  pinMode(BREAK_PIN, INPUT);
#endif
  delay(500);
}

void loop () {
  setjmp(coldstart);
#ifdef WITH_ISR
    initIsr();
#endif
  _cold();
}



// *******************************************************************************
//** 
//** ----------------------------------------------------------------------------------------------
//** Exception handling
//** If an exception occured, reexecute the "loop" function, which is execute "cold" word.
//** This solution prevent the loss of USB communication after an exception. This is nice for eevelopment.
//** But the real application recommanded to use the following sequence in "autorun" word:
//** ... @exception + + if 0 reset then ... to get a clean system after an exception.
//** In application can read exception specific information (@exception)
//** fter readed these informations then zeroized to prepare to store the information from next exception.
// ******************************************************************************/
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
static unsigned int loop_addr;   // Address of "loop" function.


#ifdef __cplusplus
extern "C" {
#endif
void _general_exception_handler(unsigned cause, unsigned status) {
  _excep_code = (cause & 0x0000007C) >> 2;
  _excep_stat = status;
  _excep_addr = __builtin_mfc0(_CP0_EPC, _CP0_EPC_SELECT);

// After an exteption we continue with "loop", which is initialise the whole system except USB !
  loop_addr = (unsigned int)((void*)loop);
  asm volatile("mtc0 %0,$14" :: "r" (loop_addr));  
//  asm volatile( "eret" );
  Serial.println("\n\r\n\rException occured !\n\rUse \"@exception\" !\n\rPlease reset MCU as soon as possible !!!\n\r");
  longjmp(_excep_buf, _excep_code);
}
#ifdef __cplusplus
}
#endif


//*  
//* @exception ( --- code stat addr )
//*    Leave on the stack the datas which is stored by exception handler into the first 3 words of EEPROM
//*    After fetching, erases desired EEPROM words.
    void fetchException(void) {
      PUSH(_excep_code);
      PUSH(_excep_stat);
      PUSH(_excep_addr);
      _excep_code = 0;
      _excep_stat = 0;
      _excep_addr = 0;
    }


// ***************************************************



