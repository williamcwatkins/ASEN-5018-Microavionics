
/****** ASEN 4/5067 Lab 6 ******************************************************
 * Author: YOUR NAME HERE
 * Date  : DATE HERE
 *
 * Updated for XC8
 * 
 * Description
 * On power up execute the following sequence:
 *      RD5 ON for 0.5s +/- 10ms then off
 *      RD6 ON for 0.5s +/- 10ms then off
 *      RD7 ON for 0.5s +/- 10ms then off
 * The following then occurs forever:
 *      RD4 blinks: 100ms +/- 10ms ON, then 900ms +/- 10ms OFF
 *      LCD Displays the following lines:
 *          'T=xx.x C'
 *          'PT=x.xxV'
 *      Where the 'x' is replaced by a digit in the measurement.
 *          Temperature data must be calculated / displayed with one digit to
 *          the right of the decimal as shown.  The sensor itself can have
 *          errors up to +/- 5 degrees Celsius.
 *          Potentiometer data must be calculated / displayed with two digits
 *          to the right of the decimal as shown.
 *          These measurements must be refreshed at LEAST at a frequency of 5Hz.
 *      USART Commands are read / executed properly. '\n' is a Line Feed char (0x0A)
 *          ASEN 4067:
 *              'TEMP\n'     - Transmits temperature data in format: 'XX.XC'
 *              'POT\n'      - Transmits potentiometer data in format: X.XXV'
 *          ASEN 5067: Same as ASEN 4067, plus two additional commands
 *              'CONT_ON\n'  - Begins continuous transmission of data over USART
 *              'CONT_OFF\n' - Ends continuous transmission of data over USART
 *
 *              Continuous transmission should output in the following format:
 *                  'T=XX.XC; PT = X.XXV\n'
 *      DAC is used to output analog signal onto RA5 with jumper cables. 
 *          ASEN 4067:
 *              Potentiometer voltage is converted from a digital value to analog 
 *              and output on the DAC. 
 *          ASEN 5067: 
 *              A 0.5 Hz 0-3.3V triangle wave is output on the DAC. 
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR
 *  USARTHandler
 *
 * LoPriISR
 *  TMR1handler
 *  TMR3Handler
 *  BlinkLEDHandler
 *  DACHandler
 * 
 * SampleADC
 *  
 ******************************************************************************/

#include <xc.h>
#include "LCDroutinesEasyPic.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#define _XTAL_FREQ 16000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
const char LCDRow1[] = {0x80,'T','E','S','T','I','N','G','!',0x00};  //const puts into prog memory
unsigned int Alive_count = 0;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void TMR0handler(void);     // Interrupt handler for TMR0, typo in main

/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
     Initial();                 // Initialize everything
      while(1) {
        // Put mainline code here
     }
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * 
 ******************************************************************************/
void Initial() {
    // Configure the IO ports
    TRISD  = 0b00001111;
    LATD = 0;
    TRISC  = 0b10010011;
    LATC = 0;
    
    // Configure the LCD pins for output. Defined in LCDRoutinesEasyPic.h
    LCD_RS_TRIS   = 0;              // Register Select Control line
    LCD_E_TRIS    = 0;              // Enable control line 
    LCD_DATA_TRIS = 0b11000000;     // Note the LCD data is only on the upper nibble RB0:3
                                    // Redundant to line above RB 4:5 for control
                                    // RB 6:7 set as inputs for other use, not used by LCD
    LCD_DATA_LAT = 0;           // Initialize LCD data LAT to zero


    // Initialize the LCD and print to it
    InitLCD();
    DisplayC(LCDRow1);

    // Initializing TMR0
    T0CON = 0b01001000;             // 8-bit, Fosc / 4, no pre/post scale timer
    TMR0L = 0;                      // Clearing TMR0 registers
    TMR0H = 0;

    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt

    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts

    T0CONbits.TMR0ON = 1;           // Turning on TMR0
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/

void __interrupt() HiPriISR(void) {
    
}	// Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until TMR0IF is clear.
 ******************************************************************************/

void __interrupt(low_priority) LoPriISR(void) 
{
    // Save temp copies of WREG, STATUS and BSR if needed.
    while(1) {
        if( INTCONbits.TMR0IF ) {
            TMR0handler();
            continue;
        }
        // Save temp copies of WREG, STATUS and BSR if needed.
        break;      // Supports RETFIE automatically
    }
}


/******************************************************************************
 * TMR0handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter 
 ******************************************************************************/
void TMR0handler() {
    if( Alive_count < 4880 ) { Alive_count++; }
    else {
        LATDbits.LATD4 = ~LATDbits.LATD4;
        Alive_count = 0;
    }
    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}
