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
 * This file contains the ISR processing
 * 
 * 
 * 
 * 
 * 
 ******************************************************************************/

#include "pins_arduino.h"
#include "Config.h"
#include "Uw.h"

#ifdef WITH_ISR

/*
 * GENERIC ISR handling services
 */

#include "isr.h"
#include "GenericTypeDefs.h"
#include "VMcore.h"


UINT isr_enabled;
UINT isr_data[ISR_SOURCE_LAST][ISR_DATA_SIZE];
UINT isr_source;
UINT isr_mask;
UINT isr_processing;
UINT isr_words[ISR_SOURCE_LAST];

/*
 * Executes ALL occured and enabled (in isr_mask) ISR processing FORTH words.
 */
void isrw(void) {
  int i;
  uint32_t mask;
  if (!isr_processing) {
    isr_processing = 1;  // Set flag to indicate the ISR processing is in progress.
    if ( (isr_enabled) && (isr_mask & isr_source) ) {  // only if not running other ISR processing and have enabled ISR request.
      for (i=0; i<ISR_SOURCE_LAST; ++i) {
        mask = (1<<i);
        if ( (isr_mask & isr_source) & mask) {          // determine which interrupt handler need to start
          isr_source ^= mask;                           // Clear flag
          callForthWord(isr_words[i]);
        }
      }
    }
    isr_processing = 0;  // Clear flag to indicate the ISR processing is done.
  }
}



void initIsr(void) {
  int i, j;
  isr_mask = 0;        // Disabled all ISR sources.
  isr_processing = 0;  // Clear flag to indicate that currently don't running ISR processing..
  isr_enabled = 0;     // initially globally disabled ISR
  for (i=0;i<ISR_SOURCE_LAST;++i) { // clear table
    isr_words[i] = 0;
    for (j=0; j<ISR_DATA_SIZE;++j) {
      isr_data[i][j] = 0;
    }
  }
  
#ifdef WITH_CORETIM_ISR
  initCoreTimerIsr();
#endif  // WITH_CORETIM_ISR
#ifdef WITH_PINCHANGE_ISR
#endif  // WITH_PINCHANGE_ISR
}


/*
 * FORTH interface
 */
// ( --- &xt )
void isrdata(void) {
  PUSH((UINT)isr_data);
}
void c_isrdatasize(void) {
  PUSH(ISR_DATA_SIZE);
}
void isrenable(void) {
  isr_enabled = 1;
}
void isrdisable(void) {
  isr_enabled = 0;
}
void isrwords(void) {
  PUSH((UINT)isr_words);
}
void isrmask(void) {
  PUSH((UINT)&isr_mask);
}
void isrsource(void) {
  PUSH((UINT)&isr_source);
}
// ( isr_source xt --- )
void setisr(void) {
  UINT xt = POP;
  int source = POP;
  int mask = (1<<source);
  isr_words[source] = xt;
  isr_mask |= mask;
}
// ( isr_source index --- data )
void isrdatafetch(void) {
  UINT index = POP;
  UINT src = POP;
  PUSH(isr_data[src][index]);
}
// ( isr_source --- )
void disableisr(void) {
  int source = POP;
  int mask = (1<<source);
  isr_mask &= (~mask);
}
// ( isr_source --- )
void enableisr(void) {
  int source = POP;
  int mask = (1<<source);
  isr_mask |= mask;
}

/*
 * Individual specific ISR handling services
 */


#ifdef WITH_CORETIM_ISR

#ifdef WITH_LOAD_INDICATOR
UINT load_counter, load_value;
#endif  // WITH_LOAD_INDICATOR


UINT uptime_10ms, uptime_sec;
uint32_t myCoreTimerService(uint32_t curTime) {

    static int nextInt = 0;
    uint32_t relWait = 0;
    uint32_t relTime = curTime - nextInt;

    while(relWait <= relTime) {
        relWait += (CORE_TICK_RATE * 10);          // every 10 ms
     }
    nextInt += relWait;                     // calculate the absolute interrupt time we want.
    isr_source = (1<<ISR_SOURCE_CORE_TIMER);
    ++isr_data[ISR_SOURCE_CORE_TIMER][0];
    ++uptime_10ms;
    if (uptime_10ms >= 100) {
      uptime_10ms = 0;
      ++uptime_sec;
#ifdef WITH_LOAD_INDICATOR
      load_value = load_counter;
      load_counter = 0;
#endif  // WITH_LOAD_INDICATOR
    }
  return(nextInt);
}

void initCoreTimerIsr(void) {
  uptime_10ms=0;
  uptime_sec=0;
  if (attachCoreTimerService(myCoreTimerService)) {
  }
}

void uptime(void) {
  PUSH(uptime_sec);
}

void c_coretimer(void) {
  PUSH(ISR_SOURCE_CORE_TIMER);
}

#ifdef WITH_LOAD_INDICATOR
void load(void) {
  PUSH(load_value);
}
#endif  //WITH_LOAD_INDICATOR
#endif  // WITH_CORETIM_ISR  

#ifdef WITH_PINCHANGE_ISR
/*
 * This sequence from PIC32MX12 family Sect. 12 IO Ports.pdf page 15.
 */
// ( pin --- )
void pinToCN(void) {
  int s;
  UINT pin = POP;
  uint8_t port = digital_pin_to_port_PGM[pin];
  uint32_t *tris = (uint32_t*)port_to_tris_PGM[port];
  uint8_t portbit = digital_pin_to_bit_mask_PGM[pin];
  noInterrupts();
#if defined(__PIC32MX2XX__)
  if (tris == &TRISA) {
    ANSELA &= ~(portbit);     // ANSEL
    TRISA  |= (portbit);      // TRIS
    CNENA  |= (portbit);      // CNEN
    CNCONA  = (1<<15);                            // CNCON
    s = PORTA;
  }
  if (tris == &TRISB) {
    ANSELB &= ~(portbit);     // ANSEL
    TRISB  |= (portbit);      // TRIS
    CNENB  |= (portbit);      // CNEN
    CNCONB  = (1<<15);                            // CNCON
    s = PORTB;
  }
#if defined(_BOARD_FUBARINO_MINI_)
  if (tris == &TRISC) {
    ANSELC &= ~(portbit);     // ANSEL
    TRISC  |= (portbit);      // TRIS
    CNENC  |= (portbit);      // CNEN
    CNCONC  = (1<<15);                            // CNCON
    s = PORTC;
  }
#endif // #if defined(_BOARD_FUBARINO_MINI_)  
  IPC8bits.CNIP	=	_CN_IPL_IPC;
  IPC8bits.CNIS	=	_CN_SPL_IPC;
  if (tris == &TRISA) {
    IFS1bits.CNAIF=0;
    IEC1bits.CNAIE=1;
  }
  if (tris == &TRISB) {
    IFS1bits.CNBIF=0;
    IEC1bits.CNBIE=1;
  }
#if defined(_BOARD_FUBARINO_MINI_)
  if (tris == &TRISC) {
    IFS1bits.CNCIF=0;
    IEC1bits.CNCIE=1;
  }
#endif // #if defined(_BOARD_FUBARINO_MINI_)  
#else // __PIC32MX2XX__  
  #pragma message "CN interrupts for NOT PIC32MX2xxx is not done !!!"
#endif // __PIC32MX2XX__
  interrupts();  
}
// ( pin --- )
void pinFromCN(void) {
  int s;
  UINT pin = POP;
  uint8_t port = digital_pin_to_port_PGM[pin];
  uint32_t *tris = (uint32_t*)port_to_tris_PGM[port];
  uint8_t portbit = digital_pin_to_bit_mask_PGM[pin];
  noInterrupts();
#if defined(__PIC32MX2XX__)
  if (tris == &TRISA) {
    CNENA  &= (~portbit);      // CNEN
    s = PORTA;
  }
  if (tris == &TRISB) {
    CNENB  &= (~portbit);      // CNEN
    s = PORTB;
  }
#if defined(_BOARD_FUBARINO_MINI_)  
  if (tris == &TRISC) {
    CNENC  &= (~portbit);      // CNEN
    s = PORTC;
  }
#endif // #if defined(_BOARD_FUBARINO_MINI_)  
  IPC8bits.CNIP	=	_CN_IPL_IPC;
  IPC8bits.CNIS	=	_CN_SPL_IPC;
  if (tris == &TRISA) {
    IFS1bits.CNAIF=0;
    IEC1bits.CNAIE=1;
  }
  if (tris == &TRISB) {
    IFS1bits.CNBIF=0;
    IEC1bits.CNBIE=1;
  }
#if defined(_BOARD_FUBARINO_MINI_)  
  if (tris == &TRISC) {
    IFS1bits.CNCIF=0;
    IEC1bits.CNCIE=1;
  }
#endif // #if defined(_BOARD_FUBARINO_MINI_)  



#else // __PIC32MX2XX__  
  #pragma message "CN interrupts for NOT PIC32MX2xxx is not done !!!"
#endif // __PIC32MX2XX__
  interrupts();
}



extern "C" {
	void __ISR(_CHANGE_NOTICE_VECTOR, _CN_IPL_ISR) cn_isr()
	{
	int s;
        isr_source = (1<<ISR_SOURCE_PIN_CHANGE);
        isr_data[ISR_SOURCE_PIN_CHANGE][0] = CNSTATA;
        isr_data[ISR_SOURCE_PIN_CHANGE][1] = CNSTATB;
#if defined(_BOARD_FUBARINO_MINI_)  
        isr_data[ISR_SOURCE_PIN_CHANGE][2] = CNSTATC;
#endif // #if defined(_BOARD_FUBARINO_MINI_)  
        if (isr_data[0]) {
          s = PORTA;
        }
        if (isr_data[1]) {
          s = PORTB;
        }
#if defined(_BOARD_FUBARINO_MINI_)  
        if (isr_data[2]) {
          s = PORTB;
        }
#endif // #if defined(_BOARD_FUBARINO_MINI_)  
        IFS1bits.CNAIF=0;
        IFS1bits.CNBIF=0;
	}
}



void c_pinchange(void) {
  PUSH(ISR_SOURCE_PIN_CHANGE);
}
// ( --- pin state)
void pinchanged(void) {
  int mask;
  int i;
  mask = isr_data[ISR_SOURCE_PIN_CHANGE][0];
  if (mask) {
    for (i=0;i<32;++i) {
      if ( mask & (1<<i) ) {
        PUSH(portbit_to_pin(0, i));
        PUSH((PORTA & mask) != 0);
      }
    }
  }
  mask = isr_data[ISR_SOURCE_PIN_CHANGE][1];
  if (mask) {
    for (i=0;i<32;++i) {
      if ( mask & (1<<i) ) {
        PUSH(portbit_to_pin(1, i));
        PUSH((PORTB & mask) != 0);
      }
    }
  }
#if defined(_BOARD_FUBARINO_MINI_)  
  mask = isr_data[ISR_SOURCE_PIN_CHANGE][0];
  if (mask) {
    for (i=0;i<32;++i) {
      if ( mask & (1<<i) ) {
        PUSH(portbit_to_pin(2, i));
        PUSH((PORTB & mask) != 0);
      }
    }
  }
#endif // #if defined(_BOARD_FUBARINO_MINI_)  
}


#endif // WITH_PINCHANGE_ISR

#endif  // WITH_ISR



