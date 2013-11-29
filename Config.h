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
 


/*******************************************************************************************
 * This file contains constant, which contolls what extensions are compiled into the build.
 *******************************************************************************************/



/* 
 * Adjust RAM usage
 * 
 * 
 * Set this value to 0.
 * Build.
 * You got the following message during build:
 * ......region 'kseg1_data_mem' overflowed by xxxxx bytes .....
 * change 0 to xxxxx
 * Build
 * If you give message again then increase this number while supress message.
 */
#define USED_RAM_SIZE  18000


/* 
 * Adjust FLASH usage
 * 
 * 
 * Set this value to 0.
 * Build.
 * You got the following message during build:
 * Binary sketch size: xxxxx bytes (of a yyyyy byte maximum)
 * change 0 to xxxxx
 * Build
 */
#define BINARY_SKETCH_SIZE   65000  /*Put here xxxxx from MPIDE status window after "Binary sketch size: xxxxx bytes (.....)*/


#define DEFAULT_BASE  10
/*
 * Comment out unneccessary features
 */
 
/*******************************************************************************
 * !!! Overwrite the first 3 word (12 bytes) of emulated EEPROM !!!
 * If exception occured then save three variable into first three words of EEPROM.
 * After the variable save perform a softreset.
 * In application can read saved variables (0 e@ 1 e@ 2 e@)
 * After readed saved variables write -1 to desired EEPROM location for proper next exception handling.
 * If the EEPROM support does not compile into the build only softreset are executed.
 ******************************************************************************/
#define  WITH_EXCEPTION_HANDLING 1    

//#define  WITH_FLASH_DEBUG 1     // Create dictionary entrys of then FLASH manupulation words
//#define  WITH_BREAK  1          // A configured HW input causes warm.
#define  WITH_CORETIM_ISR    1          // Can use interrupt in forth. See the coretimer example in ChipKitForth.pde
#define  WITH_PINCHANGE_ISR  1
#define  WITH_SOFTPWM  1        // USe SoftPWMSERVO library instead analogWrite(), which is bogous on PPS devices.
#define  WITH_EEPROM  1
#define  WITH_WIRE 1
//#define  WITH_SPI 1
#define WITH_LCD 1
#define WITH_OW 1
#define WITH_PPS 1

#ifdef WITH_CORETIM_ISR
  // Counts how many NEXT exetuted in 1 s time windows.
  // The "load" word push the current value onto the stack.
  // The max. value are system dependent. Use "maxload" to determine this:
  // : maxload 100000 0 do loop load ." MaxLoad:" . cr ;
  // The value is vary the background ChipKit tasks's time consumption (eg USB, SoftPWM, ...).
  #define WITH_LOAD_INDICATOR  1
#endif  

#ifdef WITH_BREAK
  #define BREAK_PIN 1
  #define BREAK_STATUS  1
#endif

#if defined(WITH_CORETIM_ISR) || defined(WITH_PINCHANGE_ISR)
  #define WITH_ISR
#endif


