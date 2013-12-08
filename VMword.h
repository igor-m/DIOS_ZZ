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
#include <setjmp.h>
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

#define FLASH_START  0x9d000000
#define FLASH_END  EEPROM_START

#define FREE_FLASH_START  (0x9d000000 + BINARY_SKETCH_SIZE + 0x800)
#define FREE_FLASH_END  (EEPROM_START-1)

// ********************************************************************************
// *** IO redirection
#define IO_XT_EMIT  0
#define IO_XT_KEY  1
#define IO_XT_ISKEY 2
#define IO_XT_LAST 3
// ********************************************************************************


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
#define iEXIT		0	// Index of (exit)
#define iEXEC		1	// Index of execute
#define iNEXT		2	// Index of next
#define iENTER		3	// Index of (colon)
#define iDOLIT		4	// Index of (lit)
#define iDOSLIT		5	// Index of (slit)
#define iDOCON		6	// Index of (con)
#define iDOVAR		7	// Index of (var)
#define iDODOES		8	// Index of (does>)
#define iDODEF		9	// Index of (dodefer)
#define iDODO		10	// Index of (do)
#define iISDO		11	// Index of (?do)
#define iLOOP		12	// Index of (loop)
#define iPLOOP		13	// Index of (+loop)
#define iDOBR		14	// Index of (branch)
#define iDOCBR		15	// Index of (?branch)
#define iDOVAL		16	// Index of (val)
#define iDOBR_ELSE	17	// Index of (branch) for "else"
#define iDOBR_REPEAT	18	// Index of (branch) for "repeat"
#define iDOBR_AGAIN	19	// Index of (branch) for "again"
#define iDOCBR_IF	20	// Index of (?branch) for "if"
#define iDOCBR_WHILE	21	// Index of (?branch) for "while"
#define iDOCBR_UNTIL	22	// Index of (?branch) for "until"
#define iDOBR_ENDOF	23	// Index of (branch) for "endof"


void comma(void);
void abortf(void);
void align(void);
extern uint8_t *findFreeFlash(uint32_t bsize);
extern void f_puts(char *p);
extern void f_puthex(UINT x, int l);
extern void cold(void);
extern void _cold(void);
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








