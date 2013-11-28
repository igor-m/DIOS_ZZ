
/*
ToDo:
  forget
*/

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
 
 
/*
 * MPIDE or ChipKit specific extendions:
 * Uw.cpp contains the interface functions for use ChipKit features in Forth.
 * Uw.h contains forward declarations of functions in Uw.cpp.
 * Ud.h integrates finctions from Uw.cpp into the forth dictionary.
 */

 
/*
 * Non MPIDE or ChipKit specific extendions:
 * syssave:
 *  Saves the current contents of the dictionary and some vital system variables into the free FLASH area.
 *  To minimize of eating the FLASH lifetime using the following algoithm:
 *   Every savings are marked with a "magic" string.
 *   Searches the last occurence of "magic" in flash.
 *   From the address of last "magic" searches empty FLASH area, which has enough size for current savings.
 *   If found, use this for save and mark it whit "magic".
 *   If not found enough free area, then erase FLASH from first occurence of "magic" to end of usable FLASH.
 *   If not found "magic", use the beginning of the usable FLASH.
 *   The begginnign of the usable FLASH area are computing based on "Binary sketch size".
 *   The end of the usable FLASH area are computing based on bootloader's IMAGE_HEADER. 
 *   Exclude from usable FLASH area the simulated EEPROM area.
 *  sysrestore:
 *   Searches the last occurence of "magic" in flash.
 *   Load the dictionary, and other vital variables from marked FLASH area.
 *   If not found "magic", this is meansm that not exist restorable data in FLASH.
 *    
 */


#include <setjmp.h>

#include "Config.h"
#include "VMword.h"
#include "VMcore.h"



#ifdef WITH_EEPROM
#include <EEPROM.h>
#endif

#ifdef WITH_WIRE
#include <Wire.h>
#endif

#ifdef WITH_SPI
#include <SPI.h>
#endif


#ifdef WITH_SOFTPWM
#include <SoftPWMServo.h>
#endif

#ifdef WITH_LCD
#include <LiquidCrystal.h>
#endif






//#pragma message "Write xxxxx from \"Binary sketch size: xxxxx bytes ( of a yyyyyyyy byte maximum)\" into Config.h after the BINARY_SKETCH_SIZE !"
#pragma message sizeof(char)


void setup() {
  Serial.begin(115200);
#ifdef WITH_ISR
    initIsr();
#endif
#ifdef WITH_BREAK
  pinMode(BREAK_PIN, INPUT);
#endif
}

void loop () {
  cold();
}

