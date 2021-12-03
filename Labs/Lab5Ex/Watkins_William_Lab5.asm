;;;;;; ASEN 5067 LAB 5 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Author:	    William Watkins
; Date:		    12 Oct 21
; Target:	    PIC18F87K22
;
; !!!!!!!!!!!!!!!IMPORTANT!!!!!!!! 
; Compiler Notes: 
; Add this line to the Compiler flags i.e
;   Right click on project name -> Properties -> pic-as Global Options -> 
;   Additional options: 
;    -Wl,-presetVec=0h,-pHiPriISR_Vec=0008h,-pLoPriISR_Vec=0018h
;
; Template taken from both Lab 3 template and Lab 4 example

; Decription:
; On power up execute the following sequence:
;    Set RD4, RD5, RD6, RD7 LEDs OFF
;    "ASEN5067" displayed on LCD first line
;    "PW1.00ms" displayed on LCD second line
;    RD5 ON for 0.5s (+/- 10ms) then OFF
;    RD6 ON for 0.5s (+/- 10ms) then OFF
;    RD7 ON for 0.5s (+/- 10ms) then OFF
; Loop forever:
;    Blink "Alive" LED (RD4): ON 200ms (+/-100us), OFF 800ms (+/-10us)
;    Output PWM signal on RC2:
;	Period of 20.00ms
;	5% duty cycle
;	ON +/-100us, OFF +/-10us
;    Use RPG to change pulse width
;	1/64th of a turn +/-0.01ms (CW=+)
;	Hard limit at 1.00,2.00ms
;	Update LCD
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;; Program hierarchy ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;    
; Mainline
; Loop
; Initial	- Initialize ports, display strings on LCD, perform LED seq
; Wait05s	- Wait 0.5s using Timer0 for initialization
; InitLCD	- Subroutine to initialize LCD
; Wait10ms	- Subroutine to wait 10ms for the LCD
; T50		- Subroutine to wait 50ms for LCD
; UpdateLCD	- Subroutine to update LCD with new PW value
; DisplayC	- Subroutine to display a constant display string
; DisplayV	- Subroutine to display a variable display string
; HiPriISR	- High Priority Interrupt Subroutine
; LoPriISR	- Low Priority Interrupt Subroutine
; CCP1Handler	- Handler subroutine for CCP1 interrupt
; CCP2Handler	- Handler subroutine for CCP2 interrupt
; TMR1Handler	- Handler subroutine for Timer1
; RPGChecker	- Subroutine for receiving RPG input
			      
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Hardware notes ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; This section is copied from my Lab 4 code
// <editor-fold defaultstate="collapsed" desc="Pin Mapping">
/*
    Pin | Pin Name/Register Name
     1  | RH2/AN21/A18
     2  | RH3/AN20/A19
     3  | RE1/P2C/WR/AD9
     4  | RE0/P2D/RD/AD8
     5  | RG0/ECCP3/P3A
     6  | RG1/TX2/CK2/AN19/C3OUT
     7  | RG2/RX2/DT2/AN18/C3INA
     8  | RG3/CCP4/AN17/P3D/C3INB
     9  | MCLR/RG5
     10 | RG4/RTCC/T7CKI(Note:2)/T5G/CCP5/AN16/P1D/C3INC
     11 | VSS
     12 | VDDCORE/VCAP
     13 | RF7/AN5/SS1
     14 | RF6/AN11/C1INA
     15 | RF5/AN10/C1INB
     16 | RF4/AN9/C2INA
     17 | RF3/AN8/C2INB/CTMUI
     18 | RF2/AN7/C1OUT
     19 | RH7/CCP6(Note:3)/P1B/AN15
     20 | RH6/CCP7(Note:3)/P1C/AN14/C1INC
     21 | RH5/CCP8(Note:3)/P3B/AN13/C2IND
     22 | RH4/CCP9(Note:2,3)/P3C/AN12/C2INC
     23 | RF1/AN6/C2OUT/CTDIN
     24 | ENVREG
     25 | AVDD
     26 | AVSS
     27 | RA3/AN3/VREF+
     28 | RA2/AN2/VREF-
     29 | RA1/AN1
     30 | RA0/AN0/ULPWU
     31 | VSS
     32 | VDD
     33 | RA5/AN4/T1CKI/T3G/HLVDIN
     34 | RA4/T0CKI
     35 | RC1/SOSC/ECCP2/P2A
     36 | RC0/SOSCO/SCKLI
     37 | RC6/TX1/CK1
     38 | RC7/RX1/DT1
     39 | RJ4/BA0
     40 | RJ5/CE
     41 | RJ6/LB
     42 | RJ7/UB
     43 | RC2/ECCP1/P1A
     44 | RC3/SCK1/SCL1
     45 | RC4/SDI1/SDA1
     46 | RC5/SDO1
     47 | RB7/KBI3/PGD
     48 | VDD
     49 | OSC1/CLKI/RA7
     50 | OSC2/CLKO/RA6
     51 | VSS
     52 | RB6/KBI2/PGC
     53 | RB5/KBI1/T3CKI/T1G
     54 | RB4/KBI0
     55 | RB3/INT3/CTED2/ECCP2(Note:1)/P2A
     56 | RB2/INT2/CTED1
     57 | RB1/INT1
     58 | RB0/INT0/FLT0
     59 | RJ3/WRH
     60 | RJ2/WRL
     61 | RJ1/OE
     62 | RJ0/ALE
     63 | RD7/SS2/PSP7/AD7
     64 | RD6/SCK2/SCL2/PSP6/AD6
     65 | RD5/SDI2/SDA2/PSP5/AD5
     66 | RD4/SDO2/PSP4/AD4
     67 | RD3/PSP3/AD3
     68 | RD2/PSP2/AD2
     69 | RD1/T5CKI/T7G/PSP1/AD1
     70 | VSS
     71 | VDD
     72 | RD0/PSP0/CTPLS/AD0
     73 | RE7/ECCP2/P2A/AD15
     74 | RE6/P1B/CCP6(Note:3)/AD14
     75 | RE5/P1C/CCP7(Note:3)/AD13
     76 | RE4/P3B/CCP8(Note:3)/AD12
     77 | RE3/P3C/CCP9(Note:2,3)/REF0/AD11
     78 | RE2/P2B/CCP10(Note:2)/CS/AD10
     79 | RH0/AN23/A16
     80 | RH1/AN22/A17

Note (1) The ECCP2 pin placement depends on the CCP2MX Configuration bit 
	setting and whether the device is in Microcontroller or Extended 
	Microcontroller mode.
     (2) Not available on the PIC18F65K22 and PIC18F85K22 devices.
     (3) The CC6, CCP7, CCP8 and CCP9 pin placement depends on the 
	setting of the ECCPMX Configuration bit (CONFIG3H<1>).
*/
// </editor-fold>

;;;;;;;;;;;;;;;;;;;;;;;;; Assembler Directives ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; This section is copied from my Lab 4 code
// <editor-fold defaultstate="collapsed" desc="Assembler Directives">    
; Processor Definition
PROCESSOR   18F87K22
; Radix Definition 
RADIX	DEC	      

; The following code should control macro expansions but needs further editing
    ; Macro Expansions
;EXPAND ;To expand macros
NOEXPAND ;To collapse macros
    
; The following code is from a previous compiler and no longer works 			      
    ; List Definition
    ;   C: Set the page (i.e., Column) width
    ;   N: Set the page length
    ;   X: Turn MACRO expansion on or off
    ;LIST	C = 160, N = 15, X = OFF
    
    
; Include File:
#include <xc.inc>
// </editor-fold>    

;;;;;;;;;;;;;;;;;;;;;;;;; PIC18F87K22 Configuration Bit Settings ;;;;;;;;;;;;;;; 
; This section is copied from my Lab 4 code
// <editor-fold defaultstate="collapsed" desc="CONFIG Definitions">
			      
; CONFIG1L
CONFIG  RETEN = ON            ; VREG Sleep Enable bit (Enabled)
CONFIG  INTOSCSEL = HIGH      ; LF-INTOSC Low-power Enable bit (LF-INTOSC in 
                              ;	    High-power mode during Sleep)
CONFIG  SOSCSEL = HIGH        ; SOSC Power Selection and mode Configuration bits 
			      ;	    (High Power SOSC circuit selected)
CONFIG  XINST = OFF           ; Extended Instruction Set (Disabled)

; CONFIG1H
CONFIG  FOSC = HS1            ; Oscillator
CONFIG  PLLCFG = OFF          ; PLL x4 Enable bit (Disabled)
CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor (Disabled)
CONFIG  IESO = OFF            ; Internal External Oscillator Switch Over Mode 
			      ;	    (Disabled)

; CONFIG2L
CONFIG  PWRTEN = ON           ; Power Up Timer (Enabled)
CONFIG  BOREN = ON            ; Brown Out Detect (Controlled with SBOREN bit)
CONFIG  BORV = 1              ; Brown-out Reset Voltage bits (2.7V)
CONFIG  BORPWR = ZPBORMV      ; BORMV Power level (ZPBORMV instead of BORMV 
			      ;	    is selected)

; CONFIG2H
CONFIG  WDTEN = OFF           ; Watchdog Timer (WDT disabled in hardware; 
			      ;	    SWDTEN bit disabled)
CONFIG  WDTPS = 1048576       ; Watchdog Postscaler (1:1048576)

; CONFIG3L
CONFIG  RTCOSC = SOSCREF      ; RTCC Clock Select (RTCC uses SOSC)
CONFIG  EASHFT = ON           ; External Address Shift bit (Address Shifting 
			      ;	    enabled)
CONFIG  ABW = MM              ; Address Bus Width Select bits (8-bit 
			      ;	    address bus)
CONFIG  BW = 16               ; Data Bus Width (16-bit external bus mode)
CONFIG  WAIT = OFF            ; External Bus Wait (Disabled)

; CONFIG3H
CONFIG  CCP2MX = PORTC        ; CCP2 Mux (RC1)
CONFIG  ECCPMX = PORTE        ; ECCP Mux (Enhanced CCP1/3 [P1B/P1C/P3B/P3C] 
			      ;	    muxed with RE6/RE5/RE4/RE3)
; CONFIG  MSSPMSK = MSK7        ; MSSP address masking (7 Bit address masking 
			      ;	    mode)
CONFIG  MCLRE = ON            ; Master Clear Enable (MCLR Enabled, RG5 Disabled)

; CONFIG4L
CONFIG  STVREN = ON           ; Stack Overflow Reset (Enabled)
CONFIG  BBSIZ = BB2K          ; Boot Block Size (2K word Boot Block size)

; CONFIG5L
CONFIG  CP0 = OFF             ; Code Protect 00800-03FFF (Disabled)
CONFIG  CP1 = OFF             ; Code Protect 04000-07FFF (Disabled)
CONFIG  CP2 = OFF             ; Code Protect 08000-0BFFF (Disabled)
CONFIG  CP3 = OFF             ; Code Protect 0C000-0FFFF (Disabled)
CONFIG  CP4 = OFF             ; Code Protect 10000-13FFF (Disabled)
CONFIG  CP5 = OFF             ; Code Protect 14000-17FFF (Disabled)
CONFIG  CP6 = OFF             ; Code Protect 18000-1BFFF (Disabled)
CONFIG  CP7 = OFF             ; Code Protect 1C000-1FFFF (Disabled)

; CONFIG5H
CONFIG  CPB = OFF             ; Code Protect Boot (Disabled)
CONFIG  CPD = OFF             ; Data EE Read Protect (Disabled)

; CONFIG6L
CONFIG  WRT0 = OFF            ; Table Write Protect 00800-03FFF (Disabled)
CONFIG  WRT1 = OFF            ; Table Write Protect 04000-07FFF (Disabled)
CONFIG  WRT2 = OFF            ; Table Write Protect 08000-0BFFF (Disabled)
CONFIG  WRT3 = OFF            ; Table Write Protect 0C000-0FFFF (Disabled)
CONFIG  WRT4 = OFF            ; Table Write Protect 10000-13FFF (Disabled)
CONFIG  WRT5 = OFF            ; Table Write Protect 14000-17FFF (Disabled)
CONFIG  WRT6 = OFF            ; Table Write Protect 18000-1BFFF (Disabled)
CONFIG  WRT7 = OFF            ; Table Write Protect 1C000-1FFFF (Disabled)

; CONFIG6H
CONFIG  WRTC = OFF            ; Config. Write Protect (Disabled)
CONFIG  WRTB = OFF            ; Table Write Protect Boot (Disabled)
CONFIG  WRTD = OFF            ; Data EE Write Protect (Disabled)

; CONFIG7L
CONFIG  EBRT0 = OFF           ; Table Read Protect 00800-03FFF (Disabled)
CONFIG  EBRT1 = OFF           ; Table Read Protect 04000-07FFF (Disabled)
CONFIG  EBRT2 = OFF           ; Table Read Protect 08000-0BFFF (Disabled)
CONFIG  EBRT3 = OFF           ; Table Read Protect 0C000-0FFFF (Disabled)
CONFIG  EBRT4 = OFF           ; Table Read Protect 10000-13FFF (Disabled)
CONFIG  EBRT5 = OFF           ; Table Read Protect 14000-17FFF (Disabled)
CONFIG  EBRT6 = OFF           ; Table Read Protect 18000-1BFFF (Disabled)
CONFIG  EBRT7 = OFF           ; Table Read Protect 1C000-1FFFF (Disabled)

; CONFIG7H
CONFIG  EBRTB = OFF           ; Table Read Protect Boot (Disabled)
// </editor-fold>

;;;;;;;;;;;;;;;;;;;;;;;;; MACRO Definitions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; This section is copied from my Lab 4 code
// <editor-fold defaultstate="collapsed" desc="MACRO Definitions">			    
; MACRO Definitions:

; MOVLF
; Description:
;   Move literal value to given register. 
; Input: 
;   lit: literal value
;   dest: destination 
;   access: Access bank or not. Possible values are 'a' for access bank or
;	'b' for banked memory.
  MOVLF	    MACRO   lit, dest, access
    MOVLW   lit	    ; Move literal into WREG
    BANKSEL	(dest)	; Select Bank for next file instruction
    MOVWF   BANKMASK(dest), access  ; Move WREG into destination file
  ENDM
 
;; POINT adapted from Reference: Peatman CH 7 LCD
;POINT
; Description:
;   Loads strings into table pointer. 
; Input: 
;   stringname: name of the variable containg the desired string.
  POINT	    MACRO stringname
    MOVLF high stringname, TBLPTRH, A 
    MOVLF low stringname, TBLPTRL, A
  ENDM
  
;DISPLAY
; Description:
;   Displays a given register in binary on the LCD. 
; Input: 
;   register: The register that is to be displayed on the LCD. 
  DISPLAY   MACRO register
    MOVFF register, BYTE 
    CALL ByteDisplay
  ENDM
  
// </editor-fold>
   
;;;;;;;;;;;;;;;;;;;;;;;;; Project Sections ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
; This section is a modified version of the one from my Lab 4 code
// <editor-fold defaultstate="collapsed" desc="Project Sections">
  ;;;;;;;;;;;;;;;;;;;;;; Power-On-Reset entry point ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PSECT	resetVec, class = CODE, reloc = 2
resetVec:
    NOP	    ; No Operation
    goto    main    ; Go to main after reset

;;;;;;;;;;;;;;;;;;; Interrupt Service Routine Vectors ;;;;;;;;;;;;;;;;;;;;;;;;;;
; High Priority ISR Vector Definition:
PSECT	HiPriISR_Vec, class = CODE, reloc = 2
HiPriISR_Vec:
    GOTO    HiPriISR	; Go to Program Counter (For Now)
    
; Low Priority ISR Vector Definition:
PSECT	LoPriISR_Vec, class = CODE, reloc = 2
LoPriISR_Vec:
    GOTO    LoPriISR	; Go to Program Counter (For Now)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Variables ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Constants
AliveON	    EQU 800000	; 800000  = 0x0C3500. Inst. cycles in 200ms
AliveOFF    EQU	3200000	; 3200000 = 0x30D400. Inst. cycles in 800ms
Bignum	    EQU 65536-62500 ; 3036. Preload for 0.5s delay
Bignumdos   EQU	65536-40000 ; Used in 10 ms delay
PWMMIN	    EQU	0	
PWMMAX	    EQU 100
strMax	    EQU 0x39
strMin	    EQU 0x30

; Objects to be defined in Access Bank
PSECT	udata_acs
GLOBAL PWMON, PWMOFF, PWMNow
COUNT:		DS 1	; Reserve 1 byte for COUNT in access bank
PWM:		DS 1	; Reserve 1 byte for PWM in access bank
PW:		DS 11	; Reserve 8 bytes for PW in access bank
press:		DS 1	; Reserve 1 byte for press in access bank
WREG_TEMP:	DS 1	; Reserve 1 byte for storing the WREG during LoPriISR
STATUS_TEMP:	DS 1	; Reserve 1 byte for storing STATUS during LoPriISR
BSR_TEMP:	DS 1	; Reserve 1 byte for BSR during LoPriISR
TMR1X:		DS 1	; 1-byte extension for Timer1
CCPR1X:		DS 1	; 1-byte extension for ECCP1
CCPR3X:		DS 1	; 1-byte extension for ECCP2
DTIME1L:	DS 1	; Delta Time for ECCP1
DTIME1H:	DS 1	; Delta Time for ECCP1
DTIME1X:	DS 1	; Delta Time for ECCP1
DTIME3L:	DS 1	; Delta Time for ECCP2
DTIME3H:	DS 1	; Delta Time for ECCP2
DTIME3X:	DS 1	; Delta Time for ECCP2
PWMON:		DS 2	; Reserve 2 bytes for time to leave PWM output on
PWMOFF:		DS 3	; Reserve 3 bytes for time to leave PWM output off
DIR_RPG:	DS 1	; Direction of RPG
RPG_TEMP:	DS 1	; Used for RPG state
OLDPORTD:	DS 1	; Previous state of RPG
PWMNow:		DS 1	; Used for ease of changing
    
; Objects to be defined in Bank 1
PSECT	udata_bank1
    NOP
    
;;;;;;; Constant Strings (Program Memory) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PSECT romData, space = 0, class = CONST  
LCDstr:  
    DB  0x33,0x32,0x28,0x01,0x0C,0x06,0x00  ;Initialization string for LCD
    
TopPerm:
    DB 0x80,'A','S','E','N','5','0','6','7',0x00    ;Write "ASEN5067" to first 
;						    line of LCD
    
Bot1Perm:
    DB 0xC0,'P','W','=','1','.','0','0','m','s',0x00	;Write "PW=1.00ms" to second line of LCD
    
// </editor-fold>  
    
;;;;;;; Code Start ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	 
PSECT	code	

;;;;;;; Mainline program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
main:
    rcall   Initial			; Initialize everything
loop:
    RCALL RPG	; Continuously call RPG checker
    MOVF DIR_RPG, w, a ; Move DIR_RPG into the WREG 
    BZ loop
    RCALL RPGUpdate
    RCALL UpdateLCD
    BRA loop
    
;;;;;;; Initial subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
// <editor-fold defaultstate="collapsed" desc="Initial Subroutine">
; This subroutine performs all initializations of variables and registers.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
Initial:
    MOVLF 0xA0, PWMON, a	    ; Load  initial time into PWMON (1ms)
    MOVLF 0x0F, PWMON+1, a
    MOVLF 0xE0, PWMOFF, a	    ; Load initial time into PWMOFF (19ms)
    MOVLF 0x28, PWMOFF+1, a
    MOVLF 0x01, PWMOFF+2, a
    MOVLF low AliveON, DTIME1L, a   ; Load AliveON time into DTIME1
    MOVLF high AliveON, DTIME1H, a
    MOVLF low highword AliveON, DTIME1X, a
    MOVLF low PWMON, DTIME3L, a	    ; Load PWMON time into DTIME2
    MOVLF high PWMON, DTIME3H, a    
    MOVLF low highword PWMON, DTIME3X, a
    
    MOVLF   11000000B,TRISB,A	; Set I/O for PORTB
    MOVLF   00000000B,LATB,A	; Initialize PORTB
    CLRF    TRISC, a		; Set I/O for PORTC
    CLRF    LATC, a		; Clear outputs on PORTC
    MOVLF   00000011B, TRISD, a	; Set I/O for PORTD
    CLRF    LATD, a		; Clear outputs for PORTD
    BSF	    RCON, 7, a		; Set IPEN bit <7>
    BCF	    IPR1, 0, a		; Assign Low priority to TMR1 interrupts
    BCF	    IPR3, 1, a		; Assign Low Priority to ECCP1 interrupts
    BCF	    IPR4, 0, a		; and ECCP2
    MOVLB   0x0F		    ; Set BSR to bank F for SFRs outside of access bank				
    MOVLF   00000000B, CCPTMRS0, b  ; Set TMR1 for use with ECCP1/2, Using BSR!

;;;;;; 0.5s Timer Setup
    MOVLF 00000000B, INTCON, a	; Clears overflow flag if set
    MOVLF 00000100B, T0CON, a	; Setup timer for delay of 0.5s
    MOVLF high Bignum, TMR0H, a	; Writing 3036 to TMR0
    MOVLF low Bignum, TMR0L, a
    
;;;;;; 10ms Timer 3 Setup
    MOVLF   00000000B,PIR2,A		; This clears overflow flag if set
    MOVLF   00000110B,T3CON,A		; Set up Timer3 for a delay of 10ms
    MOVLF   high Bignumdos,TMR3H,A	; Writing binary 25536 to TMR3H / TMR3L
    MOVLF   low Bignumdos,TMR3L,A	; Write high byte first, then low!
    
;;;;;; Alive LED/PWM Output Setup
    MOVLF   00000010B, T1CON, a	    ; 16 bit timer, buffer H/L registers
    MOVLF   00001010B, CCP1CON, a   ; Select compare mode, software interrupt only
    MOVLF   00001010B, CCP3CON, a
    BSF	    PIE1, 0, a		    ; TMR1IE bit<0> enables TMR1 interrupts
    BSF	    PIE3, 1, a		    ; CCP1IE bit<1> enables ECCP1 interrupts
    BSF	    PIE3, 2, a		    ; CCP2IE bit<2> enables ECCP2 interrupts
    BSF	    INTCON, 6, a	    ; GIEL bit<6> enables low-priority interrupts
    BSF	    INTCON, 7, a	    ; GIEH bit<7> enables high-priority interrupts
    
;;;;;; Setup LCD
    RCALL   InitLCD			; Initialize LCD
    RCALL   Wait10ms			; 10 ms delay subroutine
    
;;;;;; Display ASEN5067 to LCD
    POINT   TopPerm			; ASEN5067
    RCALL   DisplayC			; Display character subroutine
    
;;;;;; Display initial second line on LCD
    POINT Bot1Perm			; Point to the first output for second line
    RCALL DisplayC			; and display it
    MOVLF 0xC0, PW,A			; This and following lines configure the
    MOVLF 'P', PW+1, A			; PW variable, used for outputting
    MOVLF 'W', PW+2, A			; data to the LCD second line
    MOVLF '=', PW+3, A
    MOVLF '1', PW+4, A
    MOVLF '.', PW+5, A
    MOVLF '0', PW+6, A
    MOVLF '0', PW+7, A
    MOVLF 'm', PW+8, A
    MOVLF 's', PW+9, A
    MOVLF 0x00, PW+10, A
    LFSR 0, PW				; Set FSR0 to PW
    
;;;;;; Begin LED sequence
    BSF	    LATD,5,A			; Turn on RE5
    BSF	    T0CON,7,A			; Start 1s timer
    RCALL   Wait05s			; 1s delay subroutine
    BCF	    LATD,5,A			; Turn off RE5
    
    BSF	    LATD,6,A			; Turn on RE6
    BSF	    T0CON,7,A			; Start 1s timer
    RCALL   Wait05s			; 1s delay subroutine
    BCF	    LATD,6,A			; Turn off RE6
    
    BSF	    LATD,7,A			; Turn on RE7
    BSF	    T0CON,7,A			; Start 1s timer
    RCALL   Wait05s			; 1s delay subroutine
    BCF	    LATD,7,A			; Turn off RE7
    
    BSF	    T1CON,0,a
    
    RETURN
// </editor-fold>  
    
;;;;;;;; Wait0.5s subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="Wait0.5s Subroutine">
; This subroutine waits for Timer0 to complete its one second count
; sequence. It does so by waiting for sixteen-bit Timer0 to roll over. To obtain
; a period of 1s/250ns = 2000000 clock periods, it needs to remove
; 65536-62500 or 3036 counts from the sixteen-bit count sequence.  It uses a
; prescale of 32.
; (This subroutine is a modified version of the Wait1s subroutine in my Lab 4 code)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Wait05s:
    BTFSS INTCON,2,A		    ; Read Timer0 TMR0IF rollover flag and ...
    BRA Wait05s			    ; Loop if timer has not rolled over
    BCF	T0CON,7,A		    ; Stop the timer
    MOVLF high Bignum,TMR0H,A	    ; Then write the timer values into
    MOVLF low Bignum,TMR0L,A	    ; the timer high and low registers
    BCF INTCON,2,A		    ; Clear Timer0 TMR0IF rollover flag
    RETURN
// </editor-fold>

;;;;;;; UpdateLCD subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; / <editor-fold defaultstate="collapsed" desc="UpdateLCD Subroutine">
; Subroutine to update the LCD
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
UpdateLCD:
    LFSR 0, PW			    ; Make sure FSR0 has address of PW
    RCALL DisplayV		    ; Display PW
    RETURN
// </editor-fold>
    
;;;;;;; InitLCD subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="InitLCD Subroutine">
; InitLCD - modified version of subroutine in Reference: Peatman CH7 LCD
; Initialize the LCD.
; First wait for 0.1 second, to get past display's power-on reset time.
; (This subroutine is a modified version of the InitLCD subroutine in the 
;   lab4_example_picas.asm file)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        
InitLCD:
        MOVLF  10,COUNT,A	    ; Wait 0.1 second for LCD to power up
	BSF T3CON,0,A		    ; Set bit 0 to start Timer 3
Loop3:
        RCALL  Wait10ms		    ; Call wait10ms 10 times to 0.1 second
        DECF  COUNT,F,A
        BNZ	Loop3
        BCF     LATB,4,A	    ; RS=0 for command mode to LCD
        POINT   LCDstr		    ; Set up table pointer to initialization string
        TBLRD*			    ; Get first byte from string into TABLAT
Loop4:
	CLRF LATB,A		    ; First set LATB to all zero	
        BSF   LATB,5,A		    ; Drive E high - enable LCD
	MOVF TABLAT,W,A		    ; Move byte from program memory into working register
	ANDLW 0xF0		    ; Mask to get only upper nibble
	SWAPF WREG,W,A		    ; Swap so that upper nibble is in right position to move to LATB (RB0:RB3)
	IORWF PORTB,W,A		    ; Mask with the rest of PORTB to retain existing RB7:RB4 states
	MOVWF LATB,A		    ; Update LATB to send upper nibble
        BCF   LATB,5,A		    ; Drive E low so LCD will process input
        RCALL Wait10ms		    ; Wait ten milliseconds
	
	CLRF LATB,A		    ; Reset LATB to all zero	    
        BSF  LATB,5,A		    ; Drive E high
        MOVF TABLAT,W,A		    ; Move byte from program memory into working register
	ANDLW 0x0F		    ; Mask to get only lower nibble
	IORWF PORTB,W,A		    ; Mask lower nibble with the rest of PORTB
	MOVWF LATB,A		    ; Update LATB to send lower nibble
        BCF   LATB,5,A		    ; Drive E low so LCD will process input
        RCALL Wait10ms		    ; Wait ten milliseconds
        TBLRD+*			    ; Increment pointer and get next byte
        MOVF  TABLAT,F,A	    ; Check if we are done, is it zero?
        BNZ	Loop4
        RETURN
// </editor-fold>

;;;;;;;; Wait10ms subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="Wait10ms Subroutine">
; This subroutine waits for Timer3 to complete its ten millisecond count
; sequence. It does so by waiting for sixteen-bit Timer3 to roll over. To obtain
; a period of 10ms/250ns = 40000 clock periods, it needs to remove
; 65536-40000 or 25536 counts from the sixteen-bit count sequence.
; (This subroutine is copied from the Wait10ms subroutine in my Lab 4 code)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Wait10ms:
        BTFSS 	PIR2,1,A		    ; Read Timer3 TMR3IF rollover flag and ...
        BRA     Wait10ms		    ; Loop if timer has not rolled over
        MOVLF  	high Bignumdos,TMR3H,A	    ; Then write the timer values into
        MOVLF  	low Bignumdos,TMR3L,A	    ; the timer high and low registers
        BCF  	PIR2,1,A		    ; Clear Timer3 TMR3IF rollover flag
        RETURN
// </editor-fold>

;;;;;;; T50 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="T50 Subroutine">

; T50 modified version of T40 taken from Reference: Peatman CH 7 LCD
; Pause for 50 microseconds or 50/0.25 = 200 instruction cycles.
; Assumes 16/4 = 4 MHz internal instruction rate (250 ns)
; rcall(2) + movlw(1) + movwf(1) + COUNT*3 - lastBNZ(1) + return(2) = 200 
; Then COUNT = 195/3
; (This subroutine is a copy of the T50 subroutine in the 
;   lab4_example_picas.asm file)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        
T50:
        MOVLW  195/3          ;Each loop L4 takes 3 ins cycles
        MOVWF  COUNT,A		    
L4:
        DECF  COUNT,F,A
        BNZ	L4
        RETURN
// </editor-fold>
	    
;;;;;;;;DisplayC subroutine;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="DisplayC Subroutine">
; DisplayC taken from Reference: Peatman CH7 LCD
; This subroutine is called with TBLPTR containing the address of a constant
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position hex to ASCII.
; This subroutine expects a normal one-byte cursor-positioning code, 0xhh, and
; a null byte at the end of the string 0x00
; (This subroutine is a copy of the DisplayC subroutine in the 
;   lab4_example_picas.asm file)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

DisplayC:
        BCF   LATB,4,A		    ;Drive RS pin low for cursor positioning code
        TBLRD*			    ;Get byte from string into TABLAT
        MOVF  TABLAT,F,A	    ;Check for leading zero byte
        BNZ	Loop5
        TBLRD+*			    ;If zero, get next byte
Loop5:
	MOVLW 0xF0
	ANDWF LATB,F,A		    ;Clear RB0:RB3, which are used to send LCD data
        BSF   LATB,5,A		    ;Drive E pin high
        MOVF TABLAT,W,A		    ;Move byte from table latch to working register
	ANDLW 0xF0		    ;Mask to get only upper nibble
	SWAPF WREG,W,A		    ;swap so that upper nibble is in right position to move to LATB (RB0:RB3)
	IORWF PORTB,W,A		    ;Mask to include the rest of PORTB
	MOVWF LATB,A		    ;Send upper nibble out to LATB
        BCF   LATB,5,A		    ;Drive E pin low so LCD will accept nibble
	
	MOVLW 0xF0
	ANDWF LATB,F,A		    ;Clear RB0:RB3, which are used to send LCD data
        BSF   LATB,5,A		    ;Drive E pin high again
        MOVF TABLAT,W,A		    ;Move byte from table latch to working register
	ANDLW 0x0F		    ;Mask to get only lower nibble
	IORWF PORTB,W,A		    ;Mask to include the rest of PORTB
	MOVWF LATB,A		    ;Send lower nibble out to LATB
        BCF   LATB,5,A		    ;Drive E pin low so LCD will accept nibble
        RCALL T50		    ;Wait 50 usec so LCD can process
	
        BSF   LATB,4,A		    ;Drive RS pin high for displayable characters
        TBLRD+*			    ;Increment pointer, then get next byte
        MOVF  TABLAT,F,A	    ;Is it zero?
        BNZ	Loop5
        RETURN
// </editor-fold>
	
;;;;;;; DisplayV subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="DisplayV Subroutine">
; DisplayV taken from Reference: Peatman CH7 LCD
; This subroutine is called with FSR0 containing the address of a variable
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position.
; (This subroutine is a copy of the DisplayV subroutine in the 
;   lab4_example_picas.asm file)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	

DisplayV:
        BCF     LATB,4,A	    ;Drive RS pin low for cursor positioning code
Loop6:
	MOVLW 0xF0
	ANDWF LATB,F,A		    ;Clear RB0:RB3, which are used to send LCD data
        BSF   LATB,5,A		    ;Drive E pin high
        MOVF INDF0,W,A		    ;Move byte from FSR to working register
	ANDLW 0xF0		    ;Mask to get only upper nibble
	SWAPF WREG,W,A		    ;swap so that upper nibble is in right position to move to LATB (RB0:RB3)
	IORWF PORTB,W,A		    ;Mask to include the rest of PORTB
	MOVWF LATB,A		    ;Send upper nibble out to LATB
        BCF   LATB,5,A		    ;Drive E pin low so LCD will accept nibble
	
	MOVLW 0xF0
	ANDWF LATB,F,A		    ;Clear RB0:RB3, which are used to send LCD data
        BSF   LATB,5,A		    ;Drive E pin high again
        MOVF INDF0,W,A		    ;Move byte from table latch to working register
	ANDLW 0x0F		    ;Mask to get only lower nibble
	IORWF PORTB,W,A		    ;Mask to include the rest of PORTB
	MOVWF LATB,A		    ;Send lower nibble out to LATB
        BCF   LATB,5,A		    ;Drive E pin low so LCD will accept nibble
        RCALL T50		    ;Wait 50 usec so LCD can process
	  
        BSF   LATB,4,A		    ;Drive RS pin high for displayable characters
        MOVF  PREINC0,W,A	    ;Increment pointer, then get next byte
        BNZ   Loop6
        RETURN
// </editor-fold>

;;;;;;; RPG subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="RPG Subroutine">
; Credit: This subroutine modified from Peatman book Chapter 8 - RPG
; This subroutine decyphers RPG changes into values of DIR_RPG of 0, +1, or -1.
; DIR_RPG = +1 for CW change, 0 for no change, and -1 for CCW change.
; (This is a modified version from the lab5_orig.asm)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
RPG:
        CLRF	DIR_RPG, a	; Clear for "no change" return value.
        MOVF	PORTD, w, a	; Copy PORTD into W.
        MOVWF	RPG_TEMP, a	;  and RPG_TEMP.
        XORWF	OLDPORTD, w, a	; Check for any change?
        ANDLW	00000011B	; Masks just the RPG pins          
        BZ  L8		; If zero, RPG has not moved, ->return
			; But if the two bits have changed then...
	; Form what a CCW change would produce.          	
	RRCF	OLDPORTD, w, a	; Rotate right once into carry bit   
	BNC	L9	; If no carry, then bit 0 was a 0 -> branch to L9
        BCF	WREG, 1, a	; Otherwise, bit 0 was a 1. Then clear bit 1
				; to simulate what a CCW change would produce
        BRA	L10	; Branch to compare if RPG actually matches new CCW pattern in WREG
L9:
        BSF	WREG, 1, a  ; Set bit 1 since there was no carry
			    ; again to simulate what CCW would produce
L10:			    ; Test direction of RPG
        XORWF	RPG_TEMP, w, a	; Did the RPG actually change to this output?
        ANDLW	00000011B	; Masks the RPG pins
        BNZ	L11		; If not zero, then branch to L11 for CW case
        DECF	DIR_RPG, f, a	; If zero then change DIR_RPG to -1, must be CCW. 
        BRA	L8		; Done so branch to return
L11:	; CW case 
        INCF	DIR_RPG, f, a	; Change DIR_RPG to +1 for CW.
L8:
        MOVFF	RPG_TEMP, OLDPORTD  ; Save current RPG state as OLDPORTD
        RETURN
// </editor-fold>
	
;;;;;;; HiPriISR interrupt service routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="High Priority ISR">
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	

HiPriISR:                        ; High-priority interrupt service routine
        BTFSS	PIR3, 1, a	; Test CCP1IF bit <1> for this interrupt
        BRA	HP1
        RCALL	CCP1handler	; Call CCP1handler for blinking RD4 LED
        BRA	HiPriISR
HP1:
        BTFSS	PIR3, 2, a	; Test CCP1IF bit <1> for this interrupt
        BRA	HP2
        RCALL	CCP3handler	; Call CCP1handler for blinking RD4 LED
        BRA	HP1
HP2:
        RETFIE  1	    ; Return and restore STATUS, WREG, and BSR
			    ; from shadow registers
// </editor-fold>
			    
;;;;;;; LoPriISR interrupt service routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="Low Priority ISR">
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
			    
LoPriISR:				; Low-priority interrupt service routine
        MOVFF	STATUS, STATUS_TEMP	; Set aside STATUS and WREG
        MOVWF	WREG_TEMP, a
	MOVFF	BSR, BSR_TEMP        
LP1:
        BTFSS	PIR3, 1, a	; Test CCP1IF bit <1> for this interrupt
        BRA	LP2
        RCALL	CCP1handler	; Call CCP1handler for blinking RD4 LED
        BRA	LP1
LP2:
        BTFSS	PIR3, 2, a	; Test CCP1IF bit <1> for this interrupt
        BRA	LP3
        RCALL	CCP3handler	; Call CCP1handler for blinking RD4 LED
        BRA	LP2
LP3:
        BTFSS	PIR1, 0, a	; Test TMR1IF bit <0> for this interrupt
        BRA	LP4
        RCALL	TMR1handler	; Call TMR1handler for timing with CCP1
        BRA	LP1
LP4:
        MOVF	WREG_TEMP, w, a	    ; Restore WREG and STATUS
        MOVFF	STATUS_TEMP, STATUS
	MOVFF	BSR_TEMP, BSR        
        RETFIE			; Return from interrupt, reenabling GIEL
// </editor-fold>
	
;;;;;;;; CCP1 Handler ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="CCP1 Handler Subroutine">
; ECCP1 is used for the Alive LED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

CCP1handler:			; First must test of TMR1IF occurred at the same time
        BTFSS	PIR1, 0, a	; If TMR1's overflow flag is set? skip to test CCP bit7
        BRA	CCP1Test	; If TMR1F was clear, branch to check extension bytes
        BTFSC	CCPR1H, 7, a	; Is bit 7 a 0? Then TMR1/CCP just rolled over, need to inc TMR1X
        BRA	CCP1Test	; Is bit 7 a 1? Then let TMR1handler inc TMR1X 
        INCF	TMR1X, f, a	; TMR1/CCP just rolled over, must increment TMR1 extension
        BCF	PIR1, 0, a	; and clear TMR1IF bit <0> flag 
				;(Since TMR1 handler was unable to and arrived here first!)
CCP1Test:
        MOVF	TMR1X, w, a	; Check whether extensions are equal
        SUBWF	CCPR1X, w, a	; by subtracting TMR1X and CCPR1X, check if 0
        BNZ	CCP1Exit		; If not, branch to return
        
	BTFSS	IPR3, 1, a	; Check if ECCP1 is Hi or Lo Pri Interrupt
	BRA	LoPriCCP1
HiPriCCP1:  ; LED just turned ON - set CCP1 to trigger when it should turn off
	BSF	LATD, 4, a	; Manually toggle RD4
	MOVF	low AliveON, w, a   ; and add half period to CCPR1 to add more pulse time
        ADDWF	CCPR1L, f, a
        MOVF	high AliveON, w, a  ; Add to each of the 3 bytes to get 24 bit CCP
        ADDWFC	CCPR1H, f, a
        MOVF	low highword AliveON, w, a
        ADDWFC	CCPR1X, f, a
	BRA	CCP1Exit
LoPriCCP1:  ; LED just turned OFF - set CCP1 to trigger when it should turn on
	BCF	LATD, 4, a	; Manually toggle RD4
	MOVF	low AliveOFF, w, a   ; and add half period to CCPR1 to add more pulse time
        ADDWF	CCPR1L, f, a
        MOVF	high AliveOFF, w, a  ; Add to each of the 3 bytes to get 24 bit CCP
        ADDWFC	CCPR1H, f, a
        MOVF	low highword AliveOFF, w, a
        ADDWFC	CCPR1X, f, a
CCP1Exit:
        BCF	PIR3, 1, a	; Clear the CCP1IF bit <1> interrupt flag
	BTG	IPR3, 1, a	; Toggle ECCP1 Int. pri. bit
        RETURN
// </editor-fold>
	
;;;;;;;; CCP2 Handler ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="CCP2 Handler Subroutine">
; ECCP2 is used for the PWM
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

CCP3handler:			; First must test of TMR1IF occurred at the same time
        BTFSS	PIR1, 0, a	; If TMR1's overflow flag is set? skip to test CCP bit7
        BRA	CCP3Test	; If TMR1F was clear, branch to check extension bytes
        BTFSC	CCPR3H, 7, a	; Is bit 7 a 0? Then TMR1/CCP just rolled over, need to inc TMR1X
        BRA	CCP3Test	; Is bit 7 a 1? Then let TMR1handler inc TMR1X 
        INCF	TMR1X, f, a	; TMR1/CCP just rolled over, must increment TMR1 extension
        BCF	PIR1, 0, a	; and clear TMR1IF bit <0> flag 
				;(Since TMR1 handler was unable to and arrived here first!)
CCP3Test:
        MOVF	TMR1X, w, a	; Check whether extensions are equal
        SUBWF	CCPR3X, w, a	; by subtracting TMR1X and CCPR2X, check if 0
        BNZ	CCP3Exit	; If not, branch to return
	BTFSS	IPR4, 0, a	; Check if ECCP2 is Hi or Lo Pri Interrupt
	BRA	LoPriCCP3
HiPriCCP3:  ; PWM just turned ON - set CCP2 to trigger when it should turn off
	BSF	LATC, 2, a
	MOVF	low PWMON, w, a   ; and add half period to CCPR1 to add more pulse time
        ADDWF	CCPR3L, f, a
        MOVF	high PWMON, w, a  ; Add to each of the 3 bytes to get 24 bit CCP
        ADDWFC	CCPR3H, f, a
        MOVF	low highword PWMON, w, a
        ADDWFC	CCPR3X, f, a
	BRA	CCP3Exit
LoPriCCP3:  ; PWM just turned OFF - set CCP2 to trigger when it should turn on
	BCF	LATC, 2, a
	MOVF	low PWMOFF, w, a   ; and add half period to CCPR1 to add more pulse time
        ADDWF	CCPR3L, f, a
        MOVF	high PWMOFF, w, a  ; Add to each of the 3 bytes to get 24 bit CCP
        ADDWFC	CCPR3H, f, a
        MOVF	low highword PWMOFF, w, a
        ADDWFC	CCPR3X, f, a
CCP3Exit:
        BCF	PIR4, 0, a	; Clear the CCP1IF bit <1> interrupt flag
	BTG	IPR4, 0, a	; Toggle ECCP1 Int. pri. bit
        RETURN
// </editor-fold>
	
;;;;;;;; TMR Handler ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="TMR1 Handler Subroutine">
; ECCP2 is used for the PWM
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
TMR1handler:
        INCF	TMR1X, f, a	;Increment Timer1 extension
        BCF	PIR1, 0, a	;Clear TMR1IF flag and return to service routine
        RETURN
// </editor-fold>
	
;;;;;;;; RPGUpdate Handler ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; // <editor-fold defaultstate="collapsed" desc="RPG Update Subroutine">
; Used to update the PWM when the RPG is turned
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

RPGUpdate:
    MOVF DIR_RPG, w, a	; Move DIR_RPG into WREG
    BN RPG_Subtract	; If negative, branch to subtract time
RPG_ADD:
    MOVF PWMNow, w, a	; Move current value to WREG
    SUBWF PWMMAX, w, a	; Check if PWMNow is 100 (e.g. 6% duty cycle, 2ms)
    BZ RPG_ADD_Exit	; if so, get out
    MOVLW 1		; Add the 100 back plus the extra for the turn of the RPG
    ADDWF PWMNow, f, a	; Move the result to PWMNow
    CLRF STATUS, a	; Increase timer time
    MOVLW 40		; Move low byte of PWMOn into WREG
    ADDWF PWMON, f, a	; Add to low bytes
    MOVLW 0		; high byte of 40 is 0
    ADDWFC PWMON+1, f, a; Add carry to high byte
    MOVLW 40		; Move low byte of change in PWMOFF into WREG
    ; Very quick and not efficient way to change PWM Off time
    SUBWF PWMOFF, f, a	; Subtract from low bytes
    MOVLW 0		; high byte of 40 is 0
    SUBWFB PWMOFF+1, f, a; Subtract carry from high byte
    SUBWFB PWMOFF+2, f, a; Subtract carry from low highword byte
RPG_ADD_Check_Hundred:
    MOVF PW+7, w, a	; Move current hundredths to WREG
    SUBWF strMax, w, a	; Subtract current num from max
    BZ RPG_ADD_Handle_Hundred_Rollover	; If ==9, handle rollver
    INCF PW+7, f, a	; Increment hundredth's place
    BRA RPG_ADD_Exit
RPG_ADD_Handle_Hundred_Rollover:
    MOVLF 0x30, PW+7, a ; Reset hundredths to 0
    MOVF PW+6, w, a	; Move current tenths to WREG
    SUBWF strMax, w, a	; Subtract current num from max
    BZ RPG_ADD_Handle_Ten_Rollover ; If ==9, handle rollover
    INCF PW+6, f, a	; Increment tenths if not
    BRA RPG_ADD_Exit
RPG_ADD_Handle_Ten_Rollover:
    MOVLF 0x30, PW+6, a ; Reset tenths to 0
    INCF PW+4, w, a	; Increment ones place
RPG_ADD_Exit:
    RETURN	; Get outta here!

RPG_Subtract:
    MOVF PWMNow, w, a	; Move current value to WREG
    SUBWF PWMMIN, w, a	; Check if PWMNow is 0 (e.g. 5% duty cycle, 1ms)
    BZ RPG_Subtract_Exit; if so, get out
    MOVLW 1		; Add the 100 back plus the extra for the turn of the RPG
    SUBWF PWMNow, f, a	; Move the result to PWMNow
    CLRF STATUS, a	; Reset status reg
    MOVLW 40		; Move low byte of change in PWMON into WREG
    SUBWF PWMON, f, a	; Add to low bytes
    MOVLW 0		; high byte of 40 is 0
    SUBWFB PWMON+1, f, a; Add carry to high byte
    MOVLW 40		; Move low byte of change in PWMOFF into WREG
    ADDWF PWMOFF, f, a	; Add to low bytes
    MOVLW 0		; high byte of 40 is 0
    ADDWFC PWMOFF+1, f, a; Add carry to high byte
    ADDWFC PWMOFF+2, f, a; Add carry to low highword byte
RPG_SUBTRACT_Handle_One_Rollover:
    MOVLW 0x32	; Move 2 to WREG
    SUBWF PW+4, w, a	; Check if ones place is 2
    BNZ RPG_SUBTRACT_Check_Hundred ; If !=2, jump to check the hundredths
    DECF PW+4, w, a	; Decrement ones place
RPG_SUBTRACT_Check_Hundred:
    MOVF PW+7, w, a	; Move current hundredths to WREG
    SUBWF strMin, w, a	; Subtract current num from min
    BZ RPG_SUBTRACT_Handle_Hundred_Rollover	; If ==0x30 (0), handle rollover
    DECF PW+7, f, a	; Decrement hundredth's place
    BRA RPG_ADD_Exit
RPG_SUBTRACT_Handle_Hundred_Rollover:
    MOVLF 0x39, PW+7, a  ; Reset hundredths to 9
    DECF PW+6, f, a	    ; Decrement te tenths place
RPG_SUBTRACT_Exit:
    RETURN	; Get outta here!
// </editor-fold>
    
    