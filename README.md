ChipKitForth
============
This is an interactive Forth environment for PIC32 ChipKit boards.


This program based on DIOSFORTH. 

http://www.forth.cz/Download/DIOSForth/DIOSForth.html

Tested boards:
DP32, Fubarino Mini, Fubarino SD.

Builds under MPIDE. 
Easy to integrate into the Forth dictionary the MPIDE environment's libraries and built in features:

* pinMode     --> pm!
* digitalRead --> d@
* digitalWrite --> d!
* analogRead --> a@
* analogWrite --> a! (uses SoftPWMServo library)
* servoWrite --> s!
* EEPROM.write --> e!
* EEPROM.read --> e@
* delay --> delay
* delayus --> delayus
* millis --> millis
* micros             --> micros
* Full memory access --> m@, m! (FLASH, RAM, SFR. FLASH are readonly)
 
* For demonstration purposes contains an interrupt service environment based on CoreTimerService.
* Basic exception handling are implemented.

Entire dictionary resident in RAM. 
Possibile to save the used dictionary area into the FLASH.
After the MCU reset find the saved dictionary and restore it.
If bootword's CFA is not 0, then execute it.
The dictionary restore and autostart can be cancelled with ESC key the first 5 seconds after the reset.

Keywords: ChipKit MPIDE Forth PIC32


