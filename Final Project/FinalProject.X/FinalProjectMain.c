/****** ASEN 5067 Final Project ************************************************
 * Author: William Watkins
 * Date  : 1 December 2021
 * 
 * Description:
 * 
 *  After establishing communication with the PC, the following occurs forever:
 *      The RPG is polled every 2ms or better
 *          When the keypresses are sent, the number of cycles the RPG has been 
 *          through is sent as continuous Volume Up or Down commands
 *      The keys are continuously polled.
 *          When a key is detected as pressed, the software debounce routine is
 *              called.  Once the debounce routine has completed, the keypress
 *              is stored.
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
 *  Setup EUSART 1
 *  Setup Interrupts
 *  Final config to run
 * 
 * pollRPG()
 * pollMatrix()
 * debounceRoutine()
 * 
 * HiPriISR
 *  Handle loading next byte into txreg
 * 
 * sendData()
 * 
 *******************************************************************************
 * Program Philosophy
 * 
 * The program is laid out so that similar things are grouped together.  The 
 * global variables are grouped together according to what they are used for.
 * The functions are arranged in a like way.
 * 
 * Most processes are handled in the mainline code.  This includes sampling the 
 * RPG for changes, polling the keys, and sending the keycode and volume change.
 * 
 * The High Priority ISR handles loading the next byte in the keypress stack 
 * into the transmit register for EUSART 1.
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
 * 
 * This section is a modified version of the one from my Lab 6 code, found in 
 * "Watkins_William_Lab6_Part2.c"
 ******************************************************************************/

// <editor-fold defaultstate="collapsed" desc="Button Press Vars">
// Implementing a stack for the button presses
char readyToSend = 0;   // Flag for indicating if data is ready to be sent.
char keypress = 0;
char buttonPress = 0;
unsigned short debouncedVar = 0;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="RPG Vars">
signed char rpgDir;
char oldPortD = 0;
char rpgTemp;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="EUSART Vars">
// Flags set when the corresponding command has been received
char byteToSend = 0;// Next byte to send
// Strings to send via EUSART
char keyCodes[5] = {0xA8, 0xAB, 0xAC, 0xAE, 0x00};    // Mute, Next, 
                        // Previous, Play/Pause
char volumeControls[3]  = {0xA9, 0xAA, 0x00};   // Vol Up, Vol Down
// </editor-fold>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Function Prototypes">
/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void TMR1Handler(void);     // Interrupt handler for TMR1 for input delay
void CCP1Handler(void);     // Interrupt handler for CCP1 for input delay
void sendData(void);        // loads next byte into TXREG1
void pollRPG(void);         // Polls the RPG for an update
void pollMatrix(void);      // Polls the key matrix for a button press
void debounceRoutine(void); // Debounces a keypress          
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Mainline Code">
/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
    Initial();                 // Initialize everything
    while(1) {
        pollRPG();
        pollMatrix();
        if(readyToSend == 1 && PIR1bits.TX1IF)
            sendData();
     }  
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Initial()">
/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.  This
 * is a modified version of the one found in "Watkins_William_Lab6_Part2.c" from
 * my Lab 6 code.
 ******************************************************************************/

void Initial() {
    
    // <editor-fold defaultstate="collapsed" desc="IO Port Config">
    // Configure the IO ports
    TRISA   = 0b00101001;
    LATA    = 0;
    TRISD   = 0b11111111;
    LATD    = 0;
    TRISC   = 0b10010011;
    LATC    = 0;
    TRISE   = 0b00000000;
    LATEbits.LATE0    = 1;
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="EUSART 1 Config">
    TXSTA1 = 0b00100000;            // Async, 8N1, 19200 baud, 16MHz, Transmit enable
    RCSTA1 = 0b00010000;            // Async, 8N1, Receive enable
    BAUDCON1 = 0b01000000;          // 8-bit mode
    SPBRG1 = 12;                    // Load value of 12 into baud reg - Gives 19230.8 baud
    // </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="Interrupt Config">
    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts
    
    // Timer 1
    IPR1bits.TMR1IP = 0;            // Assign low priority to TMR1 interrupts
    PIE1bits.TMR1IE = 1;            // Enable TMR1 interrupts
    PIR1bits.TMR1IF = 0;            // Clear the interrupt flag
    
    // CCP 1
    IPR3bits.CCP1IP = 0;            // Assign low priority to CCP1 interrupts
    PIE3bits.CCP1IE = 1;            // Enable CCP1 interrupts
    PIR3bits.CCP1IF = 0;            // Clear interrupt flag
    
    // EUSART 1
    IPR1bits.TX1IP = 1;             // Assign high priority to tx interrupts
    PIE1bits.TX1IE = 0;             // Disable EUSART 1 Transmit interrupts (for now)
    
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="Final Config to Start">
    
    // EUSART Comms
    RCSTA1bits.SPEN = 1;        // Enable Serial port

    // Turn on Timers
//    T1CONbits.TMR1ON = 1; // for copying to 

    // </editor-fold>
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Primary Functions">

// <editor-fold defaultstate="collapsed" desc="pollRPG()">
/******************************************************************************
 * pollRPG function
 *
 * Polls the status of the RPG and compares it to the previous value.  Count
 * increases or decreases according to the direction the RPG is turned.
 ******************************************************************************/

void pollRPG() {
    rpgDir = 0;
    rpgTemp = (PORTD ^ oldPortD) & 0x03;
    if(rpgTemp == 0)
        return;
    switch(oldPortD)
    {
        case 0:
            oldPortD = 2;
            break;
        case 2:
            oldPortD = 3;
            break;
        case 3:
            oldPortD = 1;
            break;
        case 1:
            oldPortD = 0;
            break;
    }
    rpgTemp = oldPortD ^ rpgTemp;
    if(rpgTemp == 0)
    {
        rpgDir = -1;
        readyToSend = 1;
    }
    else
    {
        rpgDir = 1;
        readyToSend = 1;
    }
    oldPortD = PORTD & 0b00000011;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="pollMatrix()">
/******************************************************************************
 * pollMatrix function
 *
 * Polls the keyboard matrix and stores the appropriate keycodes in a buffer to 
 * send.  Assumes only one button pressed at a time.
 ******************************************************************************/

void pollMatrix() {
    if((PORTD & 0b11110000) == 0)
        return;
    debounceRoutine();
    buttonPress = PORTD & 0b11110000;
    switch(buttonPress)
    {
        case 0b10000000:
            byteToSend = 0xA8;
            break;
        case 0b01000000:
            byteToSend = 0xAC;
            break;
        case 0b00100000:
            byteToSend = 0xAE;
            break;
        case 0b00010000:
            byteToSend = 0xAB;
            break;
    }
    if(buttonPress > 0)
    {
        readyToSend = 1;
        keypress = 1;
    }
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="debounceRoutine()">
/******************************************************************************
 * debounceRoutine function
 *
 * Debounces a keypress by waiting until a short is greater than 4095, while the
 * state of the pin in question is continuously shifted into the short
 ******************************************************************************/

void debounceRoutine() {
    while((debouncedVar<<4) != 65520)
    { 
        buttonPress = PORTD & 0b11110000;
        if(buttonPress >= 1)
        {
            debouncedVar = (debouncedVar<<1) | 0x01;
        }
        else
        {
            debouncedVar = debouncedVar<<1;
        }
        if(debouncedVar<<4 == 0)   
        {
            debouncedVar = 0;
            break;
        }
    }
    debouncedVar = 0;
}

// </editor-fold>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Interrupt Service Routines">

// <editor-fold defaultstate="collapsed" desc="HiPriISR">
/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Handles all high-priority interrupts.  Currently only triggered by EUSART TX 
 * flag.
 ******************************************************************************/

void __interrupt() HiPriISR(void) {
    while(1)
    {
        if(PIR1bits.TX1IF && PIE1bits.TX1IE)
        {
            sendData();
            continue;
        }
        break;
    }
}	
//</editor-fold>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Interrupt Handlers">

// <editor-fold defaultstate="collapsed" desc="sendData()">
/******************************************************************************
 * sendData interrupt service routine.
 *
 * Sends the next byte of data
 ******************************************************************************/

void sendData() {
    if(keypress == 1)
    {
        TXREG1 = byteToSend;
        PIE1bits.TX1IE = 1;
        keypress = 0;
    }
    else if(rpgDir != 0)
    {
        switch(rpgDir)
        {
            case 1: 
                TXREG1 = 0xA9;
                PIE1bits.TX1IE = 1;
                rpgDir = 0;
                break;
            case -1:
                TXREG1 = 0xAA;
                PIE1bits.TX1IE = 1;
                rpgDir = 0;
                break;
        }
    }
    else
    {
        PIE1bits.TX1IE = 0;
        readyToSend = 0;
    }
}
// </editor-fold>

// </editor-fold>