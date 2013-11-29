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
#if defined(__PIC32MX2XX__)
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
#if defined(__32MX250F128D__)
  if (tris == &TRISC) {
    ANSELC &= ~(portbit);     // ANSEL
    TRISC  |= (portbit);      // TRIS
    CNENC  |= (portbit);      // CNEN
    CNCONC  = (1<<15);                            // CNCON
    s = PORTC;
  }
#endif // #if defined(__32MX250F128D__)  
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
#if defined(__32MX250F128D__)
  if (tris == &TRISC) {
    IFS1bits.CNCIF=0;
    IEC1bits.CNCIE=1;
  }
#endif // #if defined(__32MX250F128D__)  
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
  if (tris == &TRISA) {
    CNENA  &= (~portbit);      // CNEN
    s = PORTA;
  }
  if (tris == &TRISB) {
    CNENB  &= (~portbit);      // CNEN
    s = PORTB;
  }
#if defined(__32MX250F128D__)  
  if (tris == &TRISC) {
    CNENC  &= (~portbit);      // CNEN
    s = PORTC;
  }
#endif // #if defined(__32MX250F128D__)  
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
#if defined(__32MX250F128D__)  
  if (tris == &TRISC) {
    IFS1bits.CNCIF=0;
    IEC1bits.CNCIE=1;
  }
#endif // #if defined(__32MX250F128D__)  
  interrupts();
}



extern "C" {
	void __ISR(_CHANGE_NOTICE_VECTOR, _CN_IPL_ISR) cn_isr()
	{
	int s;
        isr_source = (1<<ISR_SOURCE_PIN_CHANGE);
        isr_data[ISR_SOURCE_PIN_CHANGE][0] = CNSTATA;
        isr_data[ISR_SOURCE_PIN_CHANGE][1] = CNSTATB;
#if defined(__32MX250F128D__)  
        isr_data[ISR_SOURCE_PIN_CHANGE][2] = CNSTATC;
#endif // #if defined(__32MX250F128D__)  
        if (isr_data[0]) {
          s = PORTA;
        }
        if (isr_data[1]) {
          s = PORTB;
        }
#if defined(__32MX250F128D__)  
        if (isr_data[2]) {
          s = PORTB;
        }
#endif // #if defined(__32MX250F128D__)  
        IFS1bits.CNAIF=0;
        IFS1bits.CNBIF=0;
	}
}



void c_pinchange(void) {
  PUSH(ISR_SOURCE_PIN_CHANGE);
}


/*
 * Reverse pin mapping
 * Determine pin number from port and bit.
 */
#if defined(__32MX250F128D__)
const uint8_t porta_bit_to_pin[] = {
   5, // RA0
   6, // RA1
  14, // RA2
  15, // RA3
  18, // RA4
  -1, // RA5
  -1, // RA6
   2, // RA7
  16, // RA8
  19, // RA9
   1, // RA10
  -1, // RA11
  -1, // RA12
  -1, // RA13
  -1, // RA14
  -1  // RA15
};
const uint8_t portb_bit_to_pin[] = {
   7, // RB0
   8, // RB1
   9, // RB2
  10, // RB3
  17, // RB4
  23, // RB5
  -1, // RB6
  24, // RB7
  25, // RB8
  26, // RB9
  31, // RB10
  32, // RB11
  -1, // RB12
   0, // RB13
   3, // RB14
   4  // RB15
};
const uint8_t portc_bit_to_pin[] = {
  11, // RC0
  12, // RC1
  13, // RC2
  20, // RC3
  21, // RC4
  22, // RC5
  27, // RC6
  28, // RC7
  29, // RC8
  30, // RC9
  -1, // RC10
  -1, // RC11
  -1, // RC12
  -1, // RC13
  -1, // RC14
  -1  // RC15
};
#endif // defined(__32MX250F128D__)
#if defined(__32MX250F128B__)
const uint8_t porta_bit_to_pin[] = {
  9,  // RA0
  10, // RA1
  15, // RA2
  16, // RA3
  18, // RA4
  -1, // RA5
  -1, // RA6
  -1, // RA7
  -1, // RA8
  -1, // RA9
  -1, // RA10
  -1, // RA11
  -1, // RA12
  -1, // RA13
  -1, // RA14
  -1  // RA15
};
const uint8_t portb_bit_to_pin[] = {
  11, // RB0
  12, // RB1
  13, // RB2
  14, // RB3
  17, // RB4
   0, // RB5
  -1, // RB6
   1, // RB7
   2, // RB8
   3, // RB9
   4, // RB10
   5, // RB11
  -1, // RB12
   6, // RB13
   7, // RB14
   8  // RB15
};
#endif // defined(__32MX250F128B__)

uint8_t portbit_to_pin(int port, int portbit) {
  if (port == PORTA || port == 0) {
    return porta_bit_to_pin[portbit];
  }
  if (port == PORTB || port == 1) {
    return portb_bit_to_pin[portbit];
  }
#if defined(__32MX250F128D__)
  if (port == PORTC || port == 2) {
    return portc_bit_to_pin[portbit];
  }
#endif // #if defined(__32MX250F128D__)
  return -1;  
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
#if defined(__32MX250F128D__)  
  mask = isr_data[ISR_SOURCE_PIN_CHANGE][0];
  if (mask) {
    for (i=0;i<32;++i) {
      if ( mask & (1<<i) ) {
        PUSH(portbit_to_pin(2, i));
        PUSH((PORTB & mask) != 0);
      }
    }
  }
#endif // #if defined(__32MX250F128D__)  
}

#else // __PIC32MX2XX__  
  #pragma message "CN interrupts for other than PIC32MX2xxx is not done !!!"
#endif // __PIC32MX2XX__

#endif // WITH_PINCHANGE_ISR

#endif  // WITH_ISR



