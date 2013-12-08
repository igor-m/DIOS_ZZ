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
 


/*******************************************************************************************
 * This file contains constant, which contolls what extensions are compiled into the build.
 *******************************************************************************************/





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
#define BINARY_SKETCH_SIZE   75000  /*Put here xxxxx from MPIDE status window after "Binary sketch size: xxxxx bytes (.....)*/


#define DEFAULT_BASE  16


#define HIDE_EXTDICT_LOAD 1

/*
 * Comment out unneccessary features
 */
 
/*******************************************************************************
 * Exception handling
 * !!! Overwrite the first 3 word (12 bytes) of emulated EEPROM !!!
 * If exception occured then save three variable into first three words of EEPROM.
 * After the variable save perform a softreset.
 * In application can read saved variables (getexceptioninfo)
 * If the EEPROM support does not compile into the build only softreset are executed.
 ******************************************************************************/
#define  WITH_EXCEPTION_HANDLING 1    
#ifdef WITH_EXCEPTION_HANDLING
  #define  WITH_EEPROM  1
#endif // #ifdef WITH_EXCEPTION_HANDLING


#define  WITH_CORETIM_ISR    1          // Can use interrupt in forth. See the coretimer example in ChipKitForth.pde
#define  WITH_PINCHANGE_ISR  1   // Currently only for PIC32MX2 series.
#define  WITH_SOFTPWM  1        // USe SoftPWMSERVO library instead analogWrite(), which is bogous on PPS devices.
#define  WITH_WIRE 1
#define  WITH_LCD 1
#define  WITH_OW 1
#define  WITH_PPS 1
#define  WITH_SEE  1
#define  WITH_UART 1
#ifdef WITH_UART
#define UART_BAUD 9600
#endif // WITH_UART


//#define  WITH_SPI 1
//#define  WITH_FLASH_DEBUG 1     // Create dictionary entrys of then FLASH manupulation words
#define  WITH_BREAK  1          // A configured HW input causes warm.


#ifdef WITH_CORETIM_ISR
  // Counts how many NEXT exetuted in 1 s time windows.
  // The "load" word push the current value onto the stack.
  // The max. value are system dependent. Use "maxload" to determine this:
  // : maxload 100000 0 do loop load ." MaxLoad:" . cr ;
  // The value is vary the background ChipKit tasks's time consumption (eg USB, SoftPWM, ...).
  #define WITH_LOAD_INDICATOR  1
#endif  

#ifdef WITH_BREAK
  #define BREAK_PIN PIN_BTN1
  #define BREAK_STATUS  1
#endif

#if defined(WITH_CORETIM_ISR) || defined(WITH_PINCHANGE_ISR)
  #define WITH_ISR
#endif


#ifdef WITH_ISR
/* 
 * Names of isr processing words
 */
#ifdef WITH_CORETIM_ISR  
  #define ISR_1MS_WORD    "isr_1ms"
  #define ISR_10MS_WORD   "isr_10ms"
  #define ISR_100MS_WORD  "isr_100ms"
  #define ISR_1000MS_WORD "isr_1000ms"
#endif // #ifdef WITH_CORETIM_ISR  

#ifdef WITH_PINCHANGE_ISR
  #define ISR_PINCHANGE_WORD "isr_cn"
#endif // #ifdef WITH_PINCHANGE_ISR  

/* 
 * Max 32 ISR source
 * Every ISR source has a bit in isr_source
 * Currenty only 2 has implemented
 */
#ifdef WITH_CORETIM_ISR  
  #define ISR_SOURCE_1MS    0
  #define ISR_SOURCE_10MS   1
  #define ISR_SOURCE_100MS  2
  #define ISR_SOURCE_1000MS 3
#endif // #ifdef WITH_CORETIM_ISR  

#ifdef WITH_PINCHANGE_ISR
  #define ISR_SOURCE_PIN_CHANGE  4
#endif // #ifdef WITH_PINCHANGE_ISR  

#define ISR_SOURCE_LAST  5


#endif // #ifdef WITH_ISR


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
 *
 */
#define BASE_RAM  10800

#ifdef  WITH_CORETIM_ISR
  #define CORETIM_RAM 500
#else  
  #define CORETIM_RAM  0
#endif  

#ifdef  WITH_PINCHANGE_ISR
  #define PINCHANGE_RAM 500
#else  
  #define PINCHANGE_RAM  0
#endif  

#ifdef  WITH_SOFTPWM
  #define SOFTPWM_RAM 5500
#else  
  #define SOFTPWM_RAM  0
#endif  

#ifdef  WITH_WIRE
  #define WIRE_RAM 500
#else  
  #define WIRE_RAM  0
#endif  

#ifdef  WITH_LCD
  #define LCD_RAM 0
#else  
  #define LCD_RAM  0
#endif  

#ifdef  WITH_OW
  #define OW_RAM 0
#else  
  #define OW_RAM  0
#endif  

#ifdef  WITH_PPS
  #define PPS_RAM 0
#else  
  #define PPS_RAM  0
#endif  

#ifdef  WITH_SEE
  #define SEE_RAM 0
#else  
  #define SEE_RAM  0
#endif  

#ifdef  WITH_EXCEPTION_HANDLING
  #define EXCEPTION_RAM 1000
#else  
  #define EXCEPTION_RAM  0
#endif  


#define USED_RAM_SIZE  (BASE_RAM + EXCEPTION_RAM + CORETIM_RAM + PINCHANGE_RAM + SOFTPWM_RAM + WIRE_RAM + LCD_RAM + OW_RAM + PPS_RAM + SEE_RAM)



