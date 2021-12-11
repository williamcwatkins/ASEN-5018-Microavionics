/****** ASEN 5067 Final Project ************************************************
 * Author: William Watkins
 * Date  : 1 December 2021
 * 
 * Description:
 * [POSSIBLE]
 *  On power up, the PIC uses V-USB and bit-banging to etablish communication 
 *      with the PC as an HID compatible keyboard using USB 1.11.
 * 
 * [CONFIRMED]
 *  After establishing communication with the PC, the following occurs forever:
 *      The RPG is polled every 2ms or better
 *          When the RPG has been through an entire cycle, a "KEY_VOLUMEUP" or 
 *          "KEY_VOLUMEDOWN" HID keyboard stroke (rep'd by 0x80 and 0x81)
 *          is sent to the computer via EUSART 2
 *      The key matrix (consisting of 3 column inputs and 1 row input) is
 *          continuously polled.
 *          When a key is detected as pressed, the software debounce routine is
 *              called.  Once the debounce routine has completed, the keypress
 *              is stored using the corresponding keystroke code in a FIFO stack.
 *          If another keystroke (or rpg movement) is detected within 10ms of
 *              the keystroke storage, the process for saving that input is 
 *              completed.  If not, all of the keypresses in the storage stack are 
 *              sent via EUSART 2 to the PC, beginning with the first keypress.
 *          At the conclusion of the debounce routine, an LED on PORT J is lit up
 *              for 1 second, before being shut off.
 * 
 *******************************************************************************
 * Program Hierarchy
 * 
 * Pre-processor directives
 * 
 * Declare variables
 * Declare function prototypes
 * 
 * Main()
 *  Initialize program
 *  Sample RPG for changes
 *  Sample key matrix for button presses
 *  Send first byte in keypress stack
 * 
 * Initial()
 *  Setup I/O
 *  Setup timers
 *  Setup ECCP
 *  Setup EUSART 2
 *  Setup Interrupts
 *  Final config to run
 * 
 * HiPriISR
 *  Handle loading next byte into txreg
 * 
 * LoPriISR
 *  Handle ECCP 1 Rollover
 *  Handle Timer 1 Rollover
 * 
 * EUSARTHandler()
 * CCP1Handler()
 * TMR1Handler()
 * 
 * sendData()
 * 
 * pollRPG()
 * pollMatrix()
 * debounceRoutine()
 *******************************************************************************
 * Program Philosophy
 * 
 * The program is laid out so that similar things are grouped together.  The 
 * global variables are grouped together according to what they are used for.
 * The functions are arranged in a like way.
 * 
 * Most processes are handled in the mainline code.  This includes sampling the 
 * RPG for changes, polling the keyboard matrix, and sending the first byte in 
 * the keypress stack.
 * 
 * The High Priority ISR handles loading the next byte in the keypress stack 
 * into the transmit register for EUSART 2.
 * 
 * The Low Priority ISR handles the interrupts triggered by rollovers.  
 * Specifically, the ECCP 1 rollover, used for determining when 10 ms has passed;
 * as well as the Timer 1 rollover.
 * 
 ******************************************************************************/

// <editor-fold defaultstate="collapsed" desc="Pre-processor Directives">
#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <proc/pic18f87k22.h>

#define _XTAL_FREQ 16000000 //Required in XC8 for delays - 16MHz osc clock
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Global Variables">
/******************************************************************************
 * Global variables
 ******************************************************************************/

// <editor-fold defaultstate="collapsed" desc="EUSART Vars">
// Flags set when the corresponding command has been received
char sendPOT = 0;
char sendTEMP = 0;
char sendCONT = 0;
char sendERROR = 0; // flag that error occurred and need to send command again (if I get to it)
char temporary = 0; // Temp var for throwing away framing error byte
char buffer[10];    // Array for storing incoming data (largest packet size is 8)
char checkTEMP[10] = {'T','E','M','P', 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00};
char checkPOT[10] = {'P','O','T', 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char checkCONTON[10] = {'C','O','N','T', '_', 'O', 'N', 0x0A, 0x00, 0x00};
char checkCONTOFF[10] = {'C','O','N','T', '_', 'O', 'F', 'F', 0x0A, 0x00};
char byteToSend = 0;// Next byte to send
// Strings to send via EUSART
char tempSend[7] = {'0', '0', '.', '0', 'C', 0x0A, 0x00};
char potSend[7]  = {'0', '.', '0', '0', 'V', 0x0A, 0x00};
char contSend[19] = {'T', '=', '0', '0', '.', '0', 'C', ';', ' ', 
                    'P', 'T', '=', '0', '.', '0', '0', 'V', 0x0A, 0x00};
char sendI = 0; // Index for current string
char i = 0;     // Index for receiving data
char rxFin = 0; // Flag for if receiving is finished, triggered on "\n"
// </editor-fold>

// </editor-fold>

// * The software debounce routine uses a 16 bit variable
// *              (specifically, an unsigned short) to determine when the switch
// *              has finished bouncing.  Once the port detects a high state on 
// *              the desired input pin, it begins shifting the state of the input
// *              pin into the unsigned short.  Once the variable 