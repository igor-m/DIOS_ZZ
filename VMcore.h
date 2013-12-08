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
 * This file contains the core of Forth
 ******************************************************************************/

#ifndef __VMcore_H__
#define __VMcore_H__


#include "GenericTypeDefs.h"
#include "Config.h"
#include "isr.h"


#define cell 		int
#define ucell 		unsigned int
#define dcell 		long long
#define udcell 		unsigned long long
#define cellsize	4
#define dcellsize	2*cellsize
#define DSsize 		128
#define RSsize 		128

extern cell  DS[DSsize];
extern ucell RS[RSsize];
extern cell *pDS, *pDSzero;
extern ucell *pRS, *pRSzero, *pRSbak;
//extern WORD DScnt, RScnt, RSCntBak;
extern ucell PC, WORK;

extern void deferfetch(void);
extern void callForthWord(UINT xt);
extern void (*pFce)(void);

extern void warm(void);
void exitw(void);
void executew(void);
void nextw(void);
void enterw(void);
void dolitw(void);
void doslitw(void);
void doconw(void);
void dovarw(void);
void dovalw(void);
void dodefer(void);
void dodoes(void);
void dodo(void);
void doisdo(void);
void doloop(void);
void doplusloop(void);
void dobranch(void);
void dobranch_else(void);
void dobranch_endof(void);
void dobranch_repeat(void);
void dobranch_again(void);
void docbranch(void);
void docbranch_if(void);
void docbranch_while(void);
void docbranch_until(void);

#define PUSH(val)	(*++pDS=val)		// DS[++DScnt]=val
#define POP			(*pDS--)			// DS[DScnt--]
#define TOS			(*pDS)				// DS[DScnt]
#define TOSi(val)	(*(pDS-(val)))		// DS[DScnt-val]
#define PUSHR(val)	(*++pRS=val)		// RS[++RScnt]=val
#define POPR		(*pRS--)			// RS[RScnt--]
#define TOSR		(*pRS)				// RS[RScnt]
#define TOSRi(val)	(*(pRS-(val)))		// RS[RScnt-val]

#define DPUSH(val)	*++pDS=(ucell)(val); *++pDS=(udcell)(val)>>32
#define DPOP(val)	val=(dcell)*pDS--; val<<=32; val|=(ucell)*pDS--

#define DScnt		(((cell)((ucell)pDS-(ucell)pDSzero))>>2)
#define RScnt		(((cell)((ucell)pRS-(ucell)pRSzero))>>2)

#define pDATA 		*(ucell *)
#define EXIT		PC=POPR;
#define EXECUTE		PC=(ucell)&TOS; pFce=(void (*)())pDATA POP; (*pFce)();
#define _NEXT           PC+= cellsize; pFce=(void (*)())pDATA(pDATA PC); (*pFce)(); 
#ifdef WITH_BREAK
  #define BREAK_PROLOG  if(digitalRead(BREAK_PIN) == BREAK_STATUS) {warm();}
#else // WITH_BREAK
  #define BREAK_PROLOG
#endif // WITH_BREAK
#ifdef WITH_ISR
  #define ISR_PROLOG    isrw()
  #ifdef WITH_LOAD_INDICATOR
    #define LOAD_PROLOG  {++load_counter;}
  #else // WITH_LOAD_INDICATOR
    #define LOAD_PROLOG
  #endif // WITH_LOAD_INDICATOR
#else //WITH_ISR
  #define ISR_PROLOG
  #define LOAD_PROLOG
#endif  //WITH_ISR
#define _NEXT           BREAK_PROLOG; PC+= cellsize; pFce=(void (*)())pDATA(pDATA PC); (*pFce)(); 

#define NEXT            LOAD_PROLOG; ISR_PROLOG;  _NEXT
#define ENTER 		pRSbak=pRS; PUSHR(PC); PC=pDATA PC; while(pRSbak<pRS){NEXT};
#define ENTERDOES	pRSbak=pRS; PUSHR(PC); PC=pDATA(WORK); while(pRSbak<pRS){NEXT};
#define DOLIT 		PC+=cellsize; PUSH(pDATA PC);
#define DOSLIT 		PC+=cellsize; PUSH(PC+1); PUSH(*(uint8_t *)PC); PC+=TOS;
#define DOCON 		PUSH(pDATA(pDATA(PC)+cellsize));
#define DOVAR 		PUSH(pDATA(PC)+cellsize);
#define DODEFER		PUSH(pDATA(pDATA(pDATA(PC)+cellsize)));
#define DOVAL 		PUSH(pDATA(pDATA(PC)+cellsize));

#endif		
