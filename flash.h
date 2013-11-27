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
 * This file contains processor dependent constants for FLASH memory
 ******************************************************************************/

#include "cpudefs.h"
#include "WProgram.h"

//#if defined(_BOARD_FUBARINO_MINI_) || defined(_BOARD_DP32_)
#if defined(__PIC32MX2XX__)
    //*********************************************************************
    //
    //                 MX1,2 Page info
    //
    //*********************************************************************
    #define PAGE_SIZE               256                // # of 32-bit Instructions per Page
    #define ROW_SIZE                32                 // # of 32-bit Instructions per Row
#else
    //*********************************************************************
    //
    //                 MX3,4,5,6,7 Page info
    //
    //*********************************************************************
    #define PAGE_SIZE               1024                // # of 32-bit Instructions per Page
    #define ROW_SIZE                128                 // # of 32-bit Instructions per Row
#endif

//*********************************************************************
//
//                 MX1,2,3,4,5,6,7 Page info
//
//*********************************************************************
#define BYTE_ROW_SIZE           (4 * ROW_SIZE)      // # Row size in Bytes
#define BYTE_PAGE_SIZE          (4 * PAGE_SIZE)     // Page size in Bytes
#define NUM_ROWS_PAGE           8                   //Number of Rows per Page

//*********************************************************************
//
//                  Macros
//
//*********************************************************************
// convert to a physical address
#define KVA_2_PA(v)         (((unsigned long) v) & 0x1fffffff)  // you can find this in <sys/kmen.h>
#define NVMOP_PAGE_ERASE    0x4004                              // Page erase operation
#define NVMOP_ROW_PGM       0x4003                              // Row program operation
#define NVMOP_WORD_PGM      0x4001                              // Word program operation

#define SuspendINT(status) asm volatile ("di %0" : "=r" (status))
#define RestoreINT(status) {if(status & 0x00000001) asm volatile ("ei"); else asm volatile ("di");}

#define ReadK0(dest) __asm__ __volatile__("mfc0 %0,$16" : "=r" (dest))
#define WriteK0(src) __asm__ __volatile__("mtc0 %0,$16" : : "r" (src))
#define K0_UNCACHED 0x00000002
#define K0_CACHED   0x00000003

#define FLASH_START_ADDR        0xBD000000ul
#define MAX_FLASH_ADDR          0xC0000000ul
#define MAX_PAGES               ((MAX_FLASH_ADDR - FLASH_START_ADDR) / BYTE_PAGE_SIZE) 

#define StartOfFlashPage(a) (a & (~(BYTE_PAGE_SIZE - 1)))
#define NextFlashPage(a) (StartOfFlashPage(a) + BYTE_PAGE_SIZE)
#define PageIndex(a) ((StartOfFlashPage(a) - FLASH_START_ADDR) / BYTE_PAGE_SIZE)





/*

General MPIDE memory map:
FLASH_END-0x1000 --- FLASH_END: "Split code" commonly used by the bootloader and application (typically USB Serial routines)
FLASH_END-0x2000 -- FLASH_END-0x1000 EEPROM emulation space

ViewHeaderInfo.pde:

Primary Image pointers:
_IMAGE_HEADER_ADDR
9D0000FC
9D003CA0

Flash Header Info:

cbHeader: 64
Bootloader Version: 0x1000108
MPIDE Version: 0x1000304
Bootloader Capabilities: 0x330F0612
Vendor ID: 0x3
Product ID: 0x4
Image Type: 0x1020001
Execution Start Address: 0x9D001000
Program Flash Base Address: 0x9D000000
Length of Program Flash: 0x1E000
EEProm Address: 0x9D01E000
Length of EEProm: 0x1000
Config Bit Address: 0xBFC00BF0
Length of Config Bits: 0x10
Address of Ram Header as placed by the sketch's linker script: 0xA0000200
Length of Ram Header as known by the linker script: 8
The max amount of RAM reserved for Debug data, Ram header, and persistent data as preserved by the bootloader: 0x600

Ram Header Info:

Amount of Ram Header written by the bootloader: 8
RCON value before bootloader execution: 0x80

*/




