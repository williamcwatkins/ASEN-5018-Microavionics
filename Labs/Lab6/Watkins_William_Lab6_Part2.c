/****** ASEN 5067 Lab 6 ********************************************************
 * Author: William Watkins
 * Date  : 9 November 2021
 *
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
 * Program hierarchy 
 *
 * Mainline
 *  Initialize code
 *  Sample ADC for both temperature and potentiometer value
 *  Determine what command was sent via EUSART (if req'd)
 *  Send Temperature Data (if req'd)
 *  Send Potentiometer Data (if req'd)
 *  Send Continuous Data (if req'd)
 *  Display to LCD
 *  
 * Initial()
 * displayLCD()
 * readADC()
 *
 * HiPriISR
 *  Handle incoming EUSART commands
 *  Load next byte into TXREG1
 *
 * LoPriISR
 *  Handle CCP1 rollover (DAC)
 *  Handle CCP3 rollover (Alive LED)
 *  Handle Timer0 rollover (Initial LED sequence)
 *  Handle Timer1 rollover (DAC)
 *  Handle Timer3 rollover (Alive LED)
 *  
 * CCP1Handler()
 * CCP3Handler()
 * TMR0Handler()
 * TMR1Handler()
 * TMR3Handler()
 * EUSARTCommandHandler()
 * readCommand()
 * sendContinuous()
 * sendTemperature()
 * sendPotentiometer()
 * sendNext()
 * aliveLED()
 * updateDAC()  
 *******************************************************************************
 * Program Philosophy
 * The program is laid out so that similar things are grouped together.  The 
 * global variables are grouped together according to what they are used for.
 * The functions are arranged in a like way.
 * 
 * Most processes are handled in the mainline code.  This includes reading the 
 * ADC, deciphering the command sent via EUSART, sending the first byte for said
 * command, and displaying the current temperature and potentiometer values to 
 * the LCD.
 * 
 * The Part 1 Functions are the functions used to accomplish part 1 of Lab 6,
 * specifically, they read the values from the ADC, update the strings to be 
 * displayed on the LCD and sent via EUSART, and display said values to the LCD.
 * 
 * The interrupt service routines are mainly used to set (or clear, in the case
 * of CONT_OFF) flags the program then uses in the mainline.  
 * 
 * The High Priority ISR handles the interrupts triggered when a new byte 
 * arrives in the EUSART receive register.  The EUSARTCommandHandler reads this 
 * byte into a buffer array before resetting the interrupt flag.  It also
 * handles the transmit interrupt flag, and sendNext() loads the next byte into
 * the transmit register.
 * 
 * The Low Priority ISR handles all of the CCP and Timer rollovers, changing 
 * CCP registers and incrementing extension registers as needed.  The alive LED 
 * blink is also handled here as is the DAC update.
 * 
 * The EUSART functions are used by the mainline code to decipher commands 
 * sent via EUSART, send responses to those commands, and load bytes into the 
 * transmit register.
 ******************************************************************************/

// <editor-fold defaultstate="collapsed" desc="Pre-processor Directives">
#include <xc.h>
#include "LCDroutinesEasyPic.h"
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

// <editor-fold defaultstate="collapsed" desc="Init LED Seq Vars">
char LED_count = 1; // TMR0Handler will increment this each time
char oldLATD = 0;   // Used to compare for initial sequence
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="LCD Display Vars">
// Output strings for LCD
char row1LCD[9] = {0x80, 'T', '=', '0', '0', '.', '0', 'C', 0x00};
char row2LCD[10] = {0xC0, 'P', 'T', '=', '0', '.', '0', '0', 'V', 0x00};
// Strings to simplify conversion from double to string
char tempLCD[6] = {'0','0','0',0x00};
char potLCD[6] = {'0','0','0',0x00};
// Doubles for conversion to string
double temp = 0;
double pot = 0;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ADC Vars">
char sensor = 0; // sensor flag; 0 is LM35, 1 is P2
char firstread = 1; // flag to throw away first read
// Empty variables for reading from ADC
unsigned short temperature = 0;
unsigned short potentiometer = 0;
unsigned short trash = 0;
// </editor-fold>

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

// <editor-fold defaultstate="collapsed" desc="Alive LED Vars">
// Constants for timing (inst cycles)
unsigned long timeON = 400000;
unsigned long timeOFF = 3600000;
// Temporary variables for full registers (with extension)
unsigned long currentCCPR3 = 0;
unsigned long currentTMR3 = 0;
char DTIMEONL = 0;  // Delta time 24 bit variable for Alive LED on time
char DTIMEONH = 0;  // Might not actually be necessary...
char DTIMEONX = 0;
char DTIMEOFFL = 0; // Delta time 24 bit variable for Alive LED off time
char DTIMEOFFH = 0;
char DTIMEOFFX = 0;
// Extension bytes
char TMR3X = 0;
char CCPR3X = 0;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="DAC Output Vars">
// Flags
char upDown = 0; // flag to indicate that DAC is going up or down (0 is up)
// Constant for timing (inst cycles)
const unsigned short DACInterval = 3906;
unsigned short currentBin = 0;  // current bin for the DAC
unsigned short DACData = 0b0011000000000000; // 16 bits to shift into SSPBUF
unsigned short DACTemp = 0;
unsigned short DACTempSend = 0;
char trashSPI = 0; // Trash var for SSPBUF
char DTIMEDACL = 0;// Delta time 24 bit variable for DAC interval
char DTIMEDACH = 0;// might not be needed...
// </editor-fold>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Function Prototypes">
/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void handleADC(void);       // Function to call readADC() in correct order
void readADC(void);         // Function to read value from ADC
void displayLCD(void);      // Displays the temp and pot values on LCD
void TMR0Handler(void);     // Interrupt handler for TMR0 for initial LED seq
void TMR1Handler(void);     // Interrupt handler for TMR1 for SPI/DAC output
void TMR3Handler(void);     // Interrupt handler for TMR3 for Alive LED
void CCP1Handler(void);     // Interrupt handler for CCP1 for SPI/DAC output
void CCP3Handler(void);     // Interrupt handler for CCP3 for Alive LED
void EUSARTCommandHandler(void); // Handles incoming EUSART commands
void readCommand(void);     // Read buffer after EUSART command sent
void sendContinuous(void);  // Might be unnecessary, but will handle continuous data sending
void sendTemperature(void); // Sends Temperature via EUSART when command is received
void sendPotentiometer(void);   // Send Potentiometer via EUSART when command is received
void sendNext(void);        // loads next byte into TXREG1
void aliveLED(void);        // Blinks the Alive LED
void updateDAC(void);       // Updates the output of the DAC for a triangle wave
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Mainline Code">
/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
    Initial();                 // Initialize everything
    while(1) {
        handleADC();          // Call ADC handler, get readings
        if(rxFin == 1)        // If a command has been received over EUSART,
            readCommand();      // determine what command it was
        // ALL of the following send... functions only send the first byte.
        // Sending the following bytes is handled via interrupt when the tx flag
        // has been set.  It is assumed that only one send... FLAG is set at a 
        // time
        if(sendTEMP == 1 && sendI == 0)       // If TEMP was sent,
            sendTemperature();  // Send the first byte of the temperature string
        if(sendPOT == 1 && sendI == 0)        // Similarly for POT
            sendPotentiometer();
        if(sendCONT == 1 && sendI == 0)       // And CONT_ON
            sendContinuous();
        displayLCD();           // Finally, display the values to the LCD
     }  
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Initial()">
/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 ******************************************************************************/

void Initial() {
    
    // <editor-fold defaultstate="collapsed" desc="IO Port Config">
    // Configure the IO ports
    TRISA   = 0b00101001;
    LATA    = 0;
    TRISD   = 0b00001111;
    LATD    = 0;
    TRISC   = 0b10010011;
    LATC    = 0;
    TRISE   = 0b00000000;
    LATEbits.LATE0    = 1;
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="LCD Config">
    // Copied from Lab 6 provided code
    // Configure the LCD pins for output. Defined in LCDRoutinesEasyPic.h
    LCD_RS_TRIS   = 0;              // Register Select Control line
    LCD_E_TRIS    = 0;              // Enable control line 
    LCD_DATA_TRIS = 0b11000000;     // Note the LCD data is only on the upper nibble RB0:3
                                    // Redundant to line above RB 4:5 for control
                                    // RB 6:7 set as inputs for other use, not used by LCD
    LCD_DATA_LAT = 0;               // Initialize LCD data LAT to zero

    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="ADC Config">
    // Configure the ADC to read LM35 first
    ANCON0 = 0b00001001; // AN3 and AN1 used
    ANCON1 = 0;
    ANCON2 = 0;
    
    ADCON0              = 0b00001100;   // Use channel 3 first
    ADCON1bits.VCFG     = 0b00;         // Use Vdd
    ADCON1bits.VNCFG    = 0;            // Use Ground
    ADCON2              = 0b10010101;   // INSERT
    // </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="Timer 0, 1, 3 Config">
    // Initializing TMR0 for Initial LED blinks
    T0CON = 0b00000100;             // 16-bit, Fosc / 4, 1:32 prescale
    TMR0H = 0x0B;                   // Loading for 0.5s delay
    TMR0L = 0xDC;
    
    // Initializing TMR1 for SPI/DAC output
    T1CON = 0b00000010;             // 16-bit, Fosc/4, no prescale
    
    // Initializing TMR3 for Alive LED blink
    T3CON = 0b00000010;             // 16-bit, Fosc/4, no prescale
    
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="CCP Config">
    
    // Variable Config
    currentCCPR3 = timeON;
    
    // CCP 1
    CCP1CON = 0b00001010;       // Set CCP1 to Compare mode
    CCPTMRS0bits.C1TSEL = 0;    // Set CCP1 Timer to TMR1
    CCPR1 = DACInterval;
    
    // CCP 3
    CCP3CON = 0b00001010;       // Set CCP3 to Compare mode
    CCPTMRS0bits.C3TSEL = 0b01; // Set CCP3 Timer to TMR3
    CCPR3X = timeON>>16;
    CCPR3 = timeON;
    
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="EUSART Config">
    TXSTA1 = 0b00100000;            // Async, 8N1, 19200 baud, 16MHz, Transmit enable
    RCSTA1 = 0b00010000;            // Async, 8N1, Receive enable
    BAUDCON1 = 0b01000000;          // 8-bit mode
    SPBRG1 = 12;                    // Load value of 12 into baud reg - Gives 19230.8 baud
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="SPI Config">
    SSP1STATbits.SMP = 1;       // Sample time
    // SPI Mode 0,0
    SSP1STATbits.CKE = 1;
    SSP1CON1bits.CKP = 0;
    SSP1CON1bits.SSPM = 0b0000; // Fosc/4
    // </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="Interrupt Config">
    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts
    
    // Timer 0
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupts
    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.TMR0IF = 0;          // Clear the interrupt flag
    
    // Timer 1
    IPR1bits.TMR1IP = 0;            // Assign low priority to TMR1 interrupts
    PIE1bits.TMR1IE = 1;            // Enable TMR1 interrupts
    PIR1bits.TMR1IF = 0;            // Clear the interrupt flag
    
    // Timer 3
    IPR2bits.TMR3IP = 0;            // Assign low priority to TMR3 interrupts
    PIE2bits.TMR3IE = 1;            // Enable TMR3 interrupts
    PIR2bits.TMR3IF = 0;            // Clear interrupt flag
    
    // CCP 1
    IPR3bits.CCP1IP = 0;            // Assign low priority to CCP1 interrupts
    PIE3bits.CCP1IE = 1;            // Enable CCP1 interrupts
    PIR3bits.CCP1IF = 0;            // Clear interrupt flag
    
    // CCP 3
    IPR4bits.CCP3IP = 0;            // Assign low priority to CCP3 interrupts
    PIE4bits.CCP3IE = 1;            // Enable CCP3 interrupts
    PIR4bits.CCP3IF = 0;            // Clear interrupt flag
    
    // EUSART 1
    IPR1bits.RC1IP = 1;             // Assign high priority to rx interrupts
    PIE1bits.RC1IE = 1;             // Enable EUSART1 Receive interrupts
    PIR1bits.RC1IF = 0;             // Clear interrupt flag
    IPR1bits.TX1IP = 1;             // Assign high priority to tx interrupts
    
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="Initial LED Sequence">
    LATD = 0b00100000;          // Turn on RD5
    T0CONbits.TMR0ON = 1;       // Turn on TMR0
    while(LED_count < 4) {}     // Let sequence run
    
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="Final Config to Start">
    // LCD Init
    InitLCD();
    
    // Enable ADC
    ADCON0bits.ADON = 1;        // Turn on ADC
    
    // EUSART Comms
    RCSTA1bits.SPEN = 1;        // Enable Serial port
    
    // SPI Communication
    SSP1CON1bits.SSPEN = 1;
    
    // Turn on Alive LED
    LATDbits.LATD4 = 1;
    
    // Turn on Timers
    T1CONbits.TMR1ON = 1;
    T3CONbits.TMR3ON = 1;
    
    // </editor-fold>
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Part 1 Functions (LCD, ADC)">

// <editor-fold defaultstate="collapsed" desc="handleADC()">
/******************************************************************************
 * handleADC function
 *
 * Calls readADC() to read both temperature and potentiometer values.  After
 * reading the temperature data, the channel is changed, the first read flag is set,
 * and the sensor flag is changed.  After the potentiometer value is read, both 
 * values are converted to doubles, and then stored in the appropriate strings
 * for output via EUSART or to LCD.
 ******************************************************************************/

void handleADC() {
    readADC();
    ADCON0bits.CHS = 0b00000;   // Change channel to potentiometer
    firstread = 1;  // Reset first read flag after channel change
    sensor = 1;     // Change sensor flag
    readADC();
    ADCON0bits.CHS = 0b00011;   // Back to LM35
    firstread = 1;
    sensor = 0;
    
    // Get double values of temperature in C and potentiometer V
    temp = temperature * 0.0806;
    pot = potentiometer * 0.000806;
    
    // Using sprintf, convert doubles to strings and insert into display strings
    sprintf(potLCD, "%1.2f", pot);
    row2LCD[4] = potLCD[0];
    row2LCD[6] = potLCD[2];
    row2LCD[7] = potLCD[3];
    potSend[0] = potLCD[0];
    potSend[2] = potLCD[2];
    potSend[3] = potLCD[3];
    contSend[12] = potLCD[0];
    contSend[14] = potLCD[2];
    contSend[15] = potLCD[3];
    
    // Similarly for temperature
    sprintf(tempLCD, "%2.1f", temp);
    
    if (temp<10.0) {
        row1LCD[3] = '0';
        row1LCD[4] = tempLCD[0];
        row1LCD[6] = tempLCD[2];
        tempSend[0] = '0';
        tempSend[1] = tempLCD[0];
        tempSend[3] = tempLCD[2];
        contSend[2] = '0';
        contSend[3] = tempLCD[0];
        contSend[5] = tempLCD[2];
    }
    else {
        row1LCD[3] = tempLCD[0];
        row1LCD[4] = tempLCD[1];
        row1LCD[6] = tempLCD[3];
        tempSend[0] = tempLCD[0];
        tempSend[1] = tempLCD[1];
        tempSend[3] = tempLCD[3];
        contSend[2] = tempLCD[0];
        contSend[3] = tempLCD[1];
        contSend[5] = tempLCD[3];
    }
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="readADC()">
/******************************************************************************
 * readADC function
 *
 * Reads temp or pot values from ADC and stores them in global variable
 ******************************************************************************/

void readADC() {
    __delay_us(4);
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO) {} // Wait after starting the ADC conversion
    trash = ADRES; // trash the first variable
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO) {}
    if (sensor != 1) {
        temperature = ADRES;
    }
    else if(sensor == 1) {
        potentiometer = ADRES;
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="displayLCD()">
/******************************************************************************
 * displayLCD function
 *
 * Displays temperature and potentiometer values to LCD
 ******************************************************************************/

void displayLCD() {
    DisplayC(row1LCD);
    DisplayC(row2LCD);
}
// </editor-fold>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Interrupt Service Routines">

// <editor-fold defaultstate="collapsed" desc="HiPriISR">
/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Handles all high-priority interrupts
 *  - Currently only triggered by EUSART commands and TX flag
 ******************************************************************************/

void __interrupt() HiPriISR(void) {
    while(1)
    {
        if(PIR1bits.RC1IF)
        {
            EUSARTCommandHandler();
            continue;
        }
        
        if(PIR1bits.TX1IF && PIE1bits.TX1IE)
        {
            sendNext();
            continue;
        }
        break;
    }
}	
//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="LoPriISR">
/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. Will handle all timer and CCP rollover, 
 * as well as the triangle wave on the DAC.
 ******************************************************************************/

void __interrupt(low_priority) LoPriISR(void) 
{
    while(1) {
        if(PIR3bits.CCP1IF)
        {
            CCP1Handler();
            continue;
        }
        if(PIR4bits.CCP3IF)
        {
            CCP3Handler();
            continue;
        }
        if(INTCONbits.TMR0IF)
        {
            TMR0Handler();
            continue;
        }
        if(PIR1bits.TMR1IF)
        {
            TMR1Handler();
            continue;
        }
        if(PIR2bits.TMR3IF)
        {
            TMR3Handler();
            continue;
        }
        break;      // Supports RETFIE automatically
    }
}
// </editor-fold>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Interrupt Handlers">

// <editor-fold defaultstate="collapsed" desc="CCP1Handler()">
/******************************************************************************
 * CCP1handler interrupt service routine.
 *
 * Handles CCP1 for SPI/DAC output and updates the DAC.  Makes this handler 
 * slightly slow in comparison to others, but is required in order to handle 
 * correct timing of DAC.
 ******************************************************************************/

void CCP1Handler() {
    // Check if Timer 1 has overflowed
    // No Extension bit, so just reset it
    if(PIR1bits.TMR1IF)
        PIR1bits.TMR1IF = 0;    
    // Update CCP1 regs
    CCPR1 += DACInterval;
    updateDAC();
    PIR3bits.CCP1IF = 0;            // Clear interrupt flag
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="CCP3Handler()">
/******************************************************************************
 * CCP3handler interrupt service routine.
 *
 * Handles CCP3 for Alive LED
 ******************************************************************************/

void CCP3Handler() {
    // Check if Timer 3 has overflowed and it matched after rolling over
    // If so, increment extension byte
    if(PIR2bits.TMR3IF && ((CCPR3H & 0b10000000) > 1) )
    {
        TMR3X++;
        PIR2bits.TMR3IF = 0;
    }
    if(CCPR3X == TMR3X)
    {
        aliveLED();
    }
    PIR4bits.CCP3IF = 0;      //Clear flag and return to polling routine
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="TMR0Handler()">
/******************************************************************************
 * TMR0Handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter 
 ******************************************************************************/
void TMR0Handler() {
    T0CONbits.TMR0ON = 0;   // Turn off timer 0
    TMR0H = 0x0B;           // Reloading for 0.5s delay
    TMR0L = 0xDC;
    LED_count++;

    if(LED_count > 3)
    {
        INTCONbits.TMR0IF = 0;
        LATD = 0;
        return;
    }
    oldLATD = ((char) LATD<<1);
    LATD = oldLATD;
    
    INTCONbits.TMR0IF = 0;  // Clear flag and return to polling routine
    T0CONbits.TMR0ON = 1;   // Turn on timer 0
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="TMR1Handler()">
/******************************************************************************
 * TMR1Handler interrupt service routine.
 *
 * Handles DAC triangle wave output
 ******************************************************************************/

void TMR1Handler() {
    
    PIR1bits.TMR1IF = 0;      //Clear flag and return to polling routine
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="TMR3Handler()">
/******************************************************************************
 * TMR3Handler interrupt service routine.
 *
 * Handles Alive LED blink
 ******************************************************************************/

void TMR3Handler() {
    TMR3X++;                    // Increment Timer 3 Extension
    PIR2bits.TMR3IF = 0;        //Clear flag and return to polling routine
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="EUSARTCommandHandler()">
/******************************************************************************
 * EUSARTCommandHandler interrupt service routine.
 *
 * Handles incoming EUSART Commands
 ******************************************************************************/

void EUSARTCommandHandler() {
    if(RCSTA1bits.FERR)     // Handles framing error
    {
        temporary = RCREG1;
        return;
    }
    if(RCSTA1bits.OERR)     // Handles Overrun error
    {
        RCSTA1bits.CREN = 0;
        RCSTA1bits.CREN = 1;
        return;
    }
    buffer[i] = RCREG1; // If no errors, load byte into buffer register
    if(buffer[i] != 0x0A)
    {
        i++;
    }
    else
    {
        rxFin = 1;  // If the line feed character is received, we're done receiving
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="aliveLED()">
/******************************************************************************
 * aliveLED function
 *
 * Blinks LED when CCP3 triggered
 ******************************************************************************/

void aliveLED() {
    // Toggle the LED
    if(LATDbits.LATD4 == 1)
    {
        LATDbits.LATD4 = 0;
        currentCCPR3 += timeOFF;
    }
    else
    {
        LATDbits.LATD4 = 1;
        currentCCPR3 += timeON;
    }
    // Load in new CCPR
    CCPR3X = currentCCPR3>>16 & 0xFF;
    CCPR3H = currentCCPR3>>8 & 0xFF;
    CCPR3L = currentCCPR3 & 0xFF;
}
// </editor-fold>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="EUSART Functions">

// <editor-fold defaultstate="collapsed" desc="readCommand()">
/******************************************************************************
 * readCommand() subroutine
 *
 * Reads buffer after \n detected over EUSART
 ******************************************************************************/

void readCommand() {
    // Check if buffer has a correct length command & is the correct command and deal accordingly
    switch(i)
    {
        case 3:
            if(strcmp(buffer, checkPOT) == 0)
                sendPOT = 1;
            break;
            
        case 4:
            if(strcmp(buffer, checkTEMP) == 0)
                sendTEMP = 1;
            break;
        
        case 7:
            if(strcmp(buffer, checkCONTON) == 0)
                sendCONT = 1;
            break;
            
        case 8:
            if(strcmp(buffer, checkCONTOFF) == 0)
                sendCONT = 0;
            break;
            
        default:
            sendERROR = 1;
    }
    i = 0;
    rxFin = 0;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="sendContinuous()">
/******************************************************************************
 * sendContinuous function
 *
 * Loads first byte into TXREG1
 ******************************************************************************/

void sendContinuous() {
    TXREG1 = contSend[sendI];
    PIE1bits.TX1IE = 1;             // Enable EUSART1 Transmit interrupts
    sendI++;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="sendTemperature()">
/******************************************************************************
 * sendTemperature function
 *
 * Loads first byte into TXREG1
 ******************************************************************************/

void sendTemperature() {
    TXREG1 = tempSend[sendI];
    PIE1bits.TX1IE = 1;             // Enable EUSART1 Transmit interrupts
    sendI++;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="sendPotentiometer()">
/******************************************************************************
 * sendPotentiometer function
 *
 * Loads first byte into TXREG1
 ******************************************************************************/

void sendPotentiometer() {
    TXREG1 = potSend[sendI];
    PIE1bits.TX1IE = 1;             // Enable EUSART1 Transmit interrupts
    sendI++;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="sendNext()">
/******************************************************************************
 * sendNext function
 *
 * Sends the next byte to be transmitted over EUSART
 ******************************************************************************/

void sendNext() {
    // Since one command is assumed at a time, if-else-if is used
    // sendI indicates the byte to be sent
    
    if(sendTEMP == 1 && sendI < 6)
    {
        TXREG1 = tempSend[sendI];   // Load next byte into TXREG1
        sendI++;                    // Increment location of next byte
    }
    else if(sendPOT == 1 && sendI < 6)
    {
        TXREG1 = potSend[sendI];    // Load next byte into TXREG1
        sendI++;                    // Increment location of next byte
    }
    else if(sendCONT == 1 && sendI < 18)
    {
        TXREG1 = contSend[sendI];    // Load next byte into TXREG1
        sendI++;                    // Increment location of next byte
    }
    // reset everything - a little inefficient, but it works
    else
    {
        sendTEMP = 0;
        sendPOT = 0;
        sendI = 0;
        PIE1bits.TX1IE = 0;             // Disable EUSART1 Transmit interrupts
        // ^ Needed because otherwise the flag will continue to be set bc reg is empty
    }
}
// </editor-fold>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="updateDAC()">
/******************************************************************************
 * updateDAC function
 *
 * Updates the output of the DAC from CCP1 handler.  Was in mainline, but 
 * the timing would not work.
 ******************************************************************************/

void updateDAC() {
    // Update the DAC output value!
    DACTempSend = 0;    // Make sure variable is empty
    LATEbits.LATE0 = 0; // Drive CS low
    DACTempSend = DACData | currentBin; // Combine command and data bits
    SSP1BUF = DACTempSend>>8;   // Load first byte into SSP1BUF
    while(SSP1STATbits.BF == 0){}   // Wait until the buffer is full to read
    trashSPI = SSP1BUF; // Read trash data
    SSP1BUF = DACTempSend & 0xFF;   // Load last 8 bits into SSP1BUF
    while(SSP1STATbits.BF == 0){}
    trashSPI = SSP1BUF;
    LATEbits.LATE0 = 1; // Drive CS high to set DAC output
    
    // Simple up or down checker to change the current bin appropriately
    // Using +=4 because the delay with 1024 bins was more precise/accurate
    // than using 4096
    if(upDown == 0)
    {
        if(currentBin == 4092)
        {
            upDown = 1;
            currentBin -= 4;
            LATAbits.LA4 = 1; // Illustration purposes only
        }
        else
        {
            currentBin += 4;
        }
    }
    else
    {
        if(currentBin == 0)
        {
            upDown = 0;
            currentBin += 4;
            LATAbits.LA4 = 0;   // Again, illustration purposes only
        }
        else
        {
            currentBin -= 4;
        }
    }
    
}
// </editor-fold>