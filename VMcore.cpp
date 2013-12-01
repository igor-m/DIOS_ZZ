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


#include <plib.h>
#include<p32xxxx.h>
#include "GenericTypeDefs.h"
#include "VMcore.h"
#include "WProgram.h"
#include "Config.h"
#include "isr.h"



cell  DS[DSsize];    // DataStack
cell  *pDSzero=DS+8;
cell  *pDS=DS+8; 
ucell RS[RSsize];    // ReturnStack
ucell *pRSzero=RS+8;
ucell *pRS=RS+8;
ucell *pRSbak;

ucell PC;
ucell WORK;

void (*pFce)(void) = (void(*)())NULL;



void exitw(void)  {
  EXIT;
}

void executew(void) {
  EXECUTE;
}

void nextw(void)  {
  NEXT;
}


void callForthWord(UINT xt) {
  ucell *pRSbak; 
  cell *pDSbak; 
  if (xt) {
    pRSbak=pRS; 
    pDSbak=pDS; 
    PUSHR(PC); 
    PC=xt; 
    while(pRSbak<pRS){ 
#ifdef WITH_LOAD_INDICATOR      
      ++load_counter;
#endif      
      _NEXT;
    }
    pDS=pDSbak; 
  }
}




void enterw(void) {
  ucell *pRSbak; 
  ENTER;
}

void dolitw(void) {
  DOLIT;
}

void doslitw(void){
  DOSLIT  
  PC&=~3;
}  // align (PC-cellsize)

void doconw(void) {
  DOCON;
}

void dovarw(void) {
  DOVAR;
}

void dovalw(void) {
  DOVAL;
}


void dodoes(void)
{
	WORK=pDATA(PC);	
        PUSH(WORK+dcellsize);	// PFA
	WORK+=cellsize;				// Must be used WORK
	ucell *pRSbak; 
        ENTERDOES	// Words behind does>
}




void dodefer(void)
{
	DOCON;
        EXECUTE;
}





void dodo(void)
{
	cell vStart=POP, vEnd=POP;
	PC+=cellsize;
	PUSHR(pDATA PC);  // For leave
	PUSHR(vEnd); 
        PUSHR(vStart);
}


void doisdo(void)
{
	cell vStart=POP, vEnd=POP;
	PC+=cellsize;
	if (vStart==vEnd) {
          PC=pDATA PC;
        } else {
          PUSHR(pDATA PC); 
          PUSHR(vEnd); 
          PUSHR(vStart);
      }
}


void doloop(void)
{
	cell vStart=POPR, vEnd=POPR; 
	PC+=cellsize; 
        vStart++;

	if (vStart<vEnd) {
		PC=pDATA PC;
		PUSHR(vEnd); 
                PUSHR(vStart);
	} else {
          pRS--;
        }  // POPR or leave
}


void doplusloop(void)
{
	cell vInc=POP, vStart=POPR, vEnd=POPR;
	char k; 

	PC+=cellsize; vStart+=vInc;
	if (vInc>=0) {
          k=(char)(vStart<vEnd);
        } else 
          {k=(char)(vStart>vEnd);
        }
	if (k) {
		PC=pDATA PC;
		PUSHR(vEnd); 
                PUSHR(vStart);
	}
	else {
          pRS--;
        }  // POPR or leave
}


void dobranch(void)
{
	PC+=cellsize;
	PC=pDATA PC;
}


void docbranch(void)
{
	PC+=cellsize;
	if (!POP) {
          PC=pDATA PC;
        }
}


