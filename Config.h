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
 * This file contains constant, which contolls what feaures are compiled into the build.
 * sections:
 * [ FEAUTRE SELECTIONS ]
 * [ Fine tuning and depedencies ]
 * [ Memory usage ]
*******************************************************************************************/



// ==============================================================================
// ============================[ FEAUTRE SELECTIONS ]============================
// ==============================================================================

/*
 * Comment out unneccessary features
 */
 
/*******************************************************************************
 * After the reset wait 10 secs for abort system restore.
 ******************************************************************************/
#define  WITH_BOOTWAIT 1    

/*******************************************************************************
 * SLEEP power saving mode support
 ******************************************************************************/
#define  WITH_SLEEP 1    


/*******************************************************************************
 * SoftPWMSERVO
 * USe SoftPWMSERVO library instead analogWrite(), which is bogous on PPS devices.
 ******************************************************************************/
//#define  WITH_SOFTPWM  1       

/*******************************************************************************
 * Wire
 * Add I2C words.
 ******************************************************************************/
//#define  WITH_WIRE 1


/*******************************************************************************
 * LCD
 * Add LCD handing words
 ******************************************************************************/
#define  WITH_LCD 1

/*******************************************************************************
 * OW
 * Add OneWire handling words
 ******************************************************************************/
#define  WITH_OW 1

/*******************************************************************************
 * PPS
 * Add words for PPS mapping handing
 ******************************************************************************/
#define  WITH_PPS 1

/*******************************************************************************
 * SEE
 * Add word "see" which is the Fort decompiler
 ******************************************************************************/
#define  WITH_SEE  1

/*******************************************************************************
 * UART
 * Add words for handling HW UART1 device
 ******************************************************************************/
#define  WITH_UART 1



/*******************************************************************************
 * SPI
 * Add words for handling HW SPI device
 ******************************************************************************/
//#define  WITH_SPI 1

/*******************************************************************************
 * DEBUG
 * Add words for debugging "syssave" and "sysrestore"
 ******************************************************************************/
//#define  WITH_FLASH_DEBUG 1

/*******************************************************************************
 * BREAK
 * Allow to configure a pin and his active state.
 * When the pin goes the active state executes a "warm". 
 * This is stoping all running words, and restart the interpreter mode.
 ******************************************************************************/
#define  WITH_BREAK  1          

/*******************************************************************************
 * marker
 * Add words "marker"
 ******************************************************************************/
#define  WITH_MARKER 1

/*******************************************************************************
 * Extended dictionary
 * When defined, all console output omitted, while loading extended dictionary
 ******************************************************************************/
#define HIDE_EXTDICT_LOAD 1




/*******************************************************************************************
 * What is the default radix
 *******************************************************************************************/
#define DEFAULT_BASE  16





// ==============================================================================
// ========================[ Fine tuning and depedencies ]=======================
// ==============================================================================

/*******************************************************************************************
 * Fine tuning and depedencies handling of above feature selections.
 *******************************************************************************************/

/* 
 * UART specific
 */
#ifdef WITH_UART
#define UART_BAUD 9600
#endif // WITH_UART


/* 
 * CoreTimer ISR specific
 */
  // Counts how many NEXT exetuted in 1 s time windows.
  // The "load" word push the current value onto the stack.
  // The max. value are system dependent. Use "maxload" to determine this:
  // : maxload 100000 0 do loop load ." MaxLoad:" . cr ;
  // The value is vary the background ChipKit tasks's time consumption (eg USB, SoftPWM, ...).
  #define WITH_LOAD_INDICATOR  1

/* 
 * Break specific
 */
#ifdef WITH_BREAK
  #define BREAK_PIN PIN_BTN1
  #define BREAK_STATUS  1
#endif


/* 
 * Names of isr processing words
 */
/*******************************************************************************
 * CoreTimer
 * Can use CoreTimer based interrupts.
 * Creates deferred words, which is executed in every 1, 10, 100, 100 ms
 ******************************************************************************/
#define ISR_1MS_WORD    "isr_1ms"
#define ISR_SOURCE_1MS    0

#define ISR_1S_WORD     "isr_1s"
#define ISR_SOURCE_1S 1

/*******************************************************************************
 * PinChange
 * Can use Pin change notification interrupts.
 ******************************************************************************/
#define ISR_PINCHANGE_WORD "isr_cn"
#define ISR_SOURCE_PIN_CHANGE  2

/*******************************************************************************
 * External interrupt
 * Can use external interrupts.
 ******************************************************************************/
#define ISR_EXT0_WORD "isr_ext0"
#define ISR_SOURCE_EXT0  3

#define ISR_EXT1_WORD "isr_ext1"
#define ISR_SOURCE_EXT1  4

#define ISR_EXT2_WORD "isr_ext2"
#define ISR_SOURCE_EXT2  5

#define ISR_EXT3_WORD "isr_ext3"
#define ISR_SOURCE_EXT3  6

#define ISR_EXT4_WORD "isr_ext4"
#define ISR_SOURCE_EXT4  7


#define ISR_SOURCE_LAST  8






// ==============================================================================
// ===============================[ Memory usage ]===============================
// ==============================================================================

 
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
#define BASE_RAM  11250


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




#define USED_RAM_SIZE  (BASE_RAM + SOFTPWM_RAM + WIRE_RAM)



