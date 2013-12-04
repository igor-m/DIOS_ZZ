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
 * This file contains base dictionary functions and tables
 ******************************************************************************/

#ifndef __VMword_H__
#define __VMword_H__



#include <cpudefs.h>
#include <plib.h>
#include<p32xxxx.h>
#include <System_Defs.h>
#include "Config.h"
#include "GenericTypeDefs.h"
#include "flash.h"
#include "isr.h"


/*************************************
 * Determine the EEPROM area address *
 *************************************/
extern const uint32_t _IMAGE_HEADER_ADDR;   
static const IMAGE_HEADER_INFO * pImageHeader = getImageHeaderInfoStructure();      


#define EEPROM_START (pImageHeader->pEEProm)

#define FREE_FLASH_START  (0x9d000000 + BINARY_SKETCH_SIZE + 0x800)
#define FREE_FLASH_END  (EEPROM_START-1)

typedef struct
{
	WORD wlink;		// Link
	WORD wlen;		// Flg+length
	char *wname;	// Name
	void *wcall;	// Call fce
} PRIMWORD;

typedef struct {
  char magic[8];
  uint32_t bootword;
  char* here;
  char* head;
  uint32_t savedbytes;
} SYSVARS;

extern const PRIMWORD primwords[];

#define RAMSIZE  (RAMEND+1)

#define VerVM		"PIC32 forth by WJ (2013.)"
#define dictsize	(RAMSIZE-USED_RAM_SIZE)	// All available RAM allocated to dictionary
#define tibsize		80		// terminal input buf
#define padsize		80		// pad buf
#define primsize	sizeof(PRIMWORD)  // 12B for primitive word

#define im		0x80	// Type immediate
//#define rs		0x40	// Type reserve
#define sm		0x40	// Type reserve
#define pr		0x20	// Type primitive

#define CRD		0x0D	// Cursor return
#define CRA		0x0A	// LF
#define BackSpc1	0x08	// Backspace
#define BackSpc2	0x7f	// Backspace
#define Spc		0x20	// Space
#define EndState	0xEE	// Bye
#define iEXIT		0x00	// Index of (exit)
#define iEXEC		0x01	// Index of execute
#define iNEXT		0x02	// Index of next
#define iENTER		0x03	// Index of (colon)
#define iDOLIT		0x04	// Index of (lit)
#define iDOSLIT		0x05	// Index of (slit)
#define iDOCON		0x06	// Index of (con)
#define iDOVAR		0x07	// Index of (var)
#define iDODOES		0x08	// Index of (does>)
#define iDODEF		0x09	// Index of (dodefer)
#define iDODO		0x0A	// Index of (do)
#define iISDO		0x0B	// Index of (?do)
#define iLOOP		0x0C	// Index of (loop)
#define iPLOOP		0x0D	// Index of (+loop)
#define iDOBR		0x0E	// Index of (branch)
#define iDOCBR		0x0F	// Index of (?branch)
#define iDOVAL		0x10	// Index of (val)
#define iNOP		0x11	// Index of nop


void comma(void);
void abortf(void);
void align(void);
extern uint8_t *findFreeFlash(uint32_t bsize);
extern int f_putc(int c);
extern void f_inputline(void);
extern int f_getc(void);
extern void f_puts(char *p);
extern void f_puthex(UINT x, int l);
extern void cold(void);
extern char NVMerase(UINT *pAddr);
extern char NVMwrite(UINT *pAddr, UINT Data);
extern uint8_t *pagealign(uint8_t *addr);
extern void EE_WR_Word(uint32_t address, uint32_t data);
extern void dot(void);
extern void udot(void);
extern void find(void);
extern void swap(void);
extern void drop(void);



#endif		








