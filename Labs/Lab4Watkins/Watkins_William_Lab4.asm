;;;;;;; ASEN 5067 LAB 4 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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

; Description:
; ~_sec means (+/- 10 ms)
; On power up execute the following sequence:
;	Display "ASEN5067" on the first line of the LCD
; 	RE5 ON for ~1 second then OFF
; 	RE6 ON for ~1 second then OFF
; 	RE7 ON for ~1 second then OFF
; LOOP on the following forever:
; 	Blink "Alive" LED (RE4) ON for ~250msec then OFF for ~250msec
;	Output PWM signal on RC2 (accurate to 100 micros):
;	    Period of 20.00 ms
;	    5% duty cycle (1.00 ms high, 19.00 ms low)
;	Display "PW=1.00ms" on second line of LCD
;	Upon press AND RELEASE of RE3, increase pulse width by 0.2ms up to 2.00ms
;	    then loop back to 1.00ms
;	    Update LCD with pulse width
; NOTE: All timing will be done with timers, not CCP or delays.
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;; Program hierarchy ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;    
; Mainline
; Loop
; Initial	-	Initialize ports, display string on LCD, perform LED seq
; Wait10ms	-	Subroutine to wait 10 ms for LCD
; InitLCD	-	Subroutine to initialize the LCD
; T50		-	Subroutine to wait 50 ms for LCD
; Wait1sec	-	Subroutine to wait one second
; CheckBu	-	Subroutine to check status of RE3 button and increase
;			pulse width/update LCD when released
; IncreasePW	-	Subroutine to increase pulse width
; UpdateLCD	-	Subroutine to update LCD with new PW value
; DisplayC	-	Subroutine to display a constant display string
; DisplayV	-	Subroutine to display a variable display string
; BlinkAlive	-	Subroutine to toggle the Alive LED (RE4) when timer dictates
; PWMOff	-	Subroutine to turn off PWM output and timer
; PWMOn		-	Subroutine to reset 20ms timer, turn on PW timer and PWM signal
			      
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Hardware notes ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; This section is copied from the Lab 4 example file
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
; This section is copied from the Lab 4 example file

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
; This section is copied from the Lab 4 example file
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
; This section is copied from the Lab 4 example file
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
; This section is a modified version of the one from the Lab 4 example file
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
    GOTO    $	; Go to Program Counter (For Now)
    
; Low Priority ISR Vector Definition:
PSECT	LoPriISR_Vec, class = CODE, reloc = 2
LoPriISR_Vec:
    GOTO    $	; Go to Program Counter (For Now)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Variables ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Objects to be defined in Access Bank
PSECT	udata_acs
COUNT:		DS  1	; Reserve 1 byte for COUNT in access bank
PWM:		DS  1	; Reserve 1 byte for PWM in access bank
PW:		DS  11	; Reserve 8 bytes for PW in access bank
press: DS 1		; Reserve 1 byte for press in access bank
    
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

;;;;;;; Code Start ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	 
PSECT	code	
// </editor-fold>  
		
;;;;;;; Mainline program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
main:
    rcall   Initial			; Initialize everything
loop:
    BTFSC INTCON,2,A			; Check if TMR0 overflow flag set and...
    RCALL BlinkAlive			; Call BlinkAlive subroutine 
    BTFSC PIR5,1,A			; Check if PWM signal ON timer overflow flag set and...
    RCALL PWMOff			; Call PWMOff to turn off PWM signal
    BTFSC PIR1,0,A			; Check if 20ms PWM signal timer overflow flag set and...
    RCALL PWMOn				; Call PWOn to turn on PWM signal and restart timers
    RCALL Check_SW1			; Call Check_SW1 to check if RE3 has been
					; pressed and released
    BRA loop

;;;;;;; Initial subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; (This subroutine is a modified version of the Initial subroutine in the 
;   lab4_example_picas.asm file)
; This subroutine performs all initializations of variables and registers.

;;;;; These are the number required for the timers
Bignum	    equ     65536-62500		; Used in 1s and 250ms timers
Bignumdos   equ	    65536-40000		; Used in 20ms and 10ms timers
Bignum10    equ	    65536-4000		; Used in 1.00ms PW
Bignum12    equ	    65536-4800		; Used in 1.20ms PW
Bignum14    equ	    65536-5600		; Used in 1.40ms PW
Bignum16    equ	    65536-6400		; Used in 1.60ms PW
Bignum18    equ	    65536-7200		; Used in 1.80ms PW
Bignum20    equ	    65536-8000		; Used in 2.00ms PW
    
Initial:
    MOVLF   11000000B,TRISB,A		; Set I/O for PORTB
    MOVLF   00000000B,LATB,A		; Initialize PORTB
    MOVLF   00000000B,TRISC,A		; Set I/0 for PORTC
    MOVLF   00000000B,LATC,A		; Initialize PORTC
    MOVLF   00001000B,TRISE,A		; Set I/O for PORTE
    MOVLF   00000000B,LATE,A		; Initialize PORTE
    
;;;;;; Setup for PWM 
    MOVLF   0x01,PWM,A			; Used for setting timer5 length
    
;;;;;; 1s Timer 0 Setup
    MOVLF   00000000B,INTCON,A		; This clears overflow flag if set
    MOVLF   00000101B,T0CON,A		; Set up Timer0 for a delay of 1 s
    MOVLF   high Bignum,TMR0H,A		; Writing binary 3036 to TMR0H / TMR0L
    MOVLF   low Bignum,TMR0L,A		; Write high byte first, then low!
    
;;;;;; 20ms Timer 1 Setup
    MOVLF   00000000B,PIR1,A		; This clears overflow flag if set
    MOVLF   00010010B,T1CON,A		; Set up Timer1 for a delay of 20ms
    MOVLF   high Bignumdos,TMR1H,A	; Writing binary 25536 to TMR1H / TMR1L
    MOVLF   low Bignumdos,TMR1L,A	; Write high byte first, then low!

;;;;;; 10ms Timer 3 Setup
    MOVLF   00000000B,PIR2,A		; This clears overflow flag if set
    MOVLF   00000110B,T3CON,A		; Set up Timer3 for a delay of 10ms
    MOVLF   high Bignumdos,TMR3H,A	; Writing binary 25536 to TMR3H / TMR3L
    MOVLF   low Bignumdos,TMR3L,A	; Write high byte first, then low!

;;;;;; 1-2ms Timer 5 Setup
    MOVLF   00000110B,T5CON,A		; Set up Timer5 for a delay of 1.00ms
    MOVLF   high Bignum10,TMR5H,A	; Writing binary 61536 to TMR5H / TMR5L
    MOVLF   low Bignum10,TMR5L,A	; Write high byte first, then low!
    
;;;;;; Setup LCD
    RCALL   InitLCD			; Initialize LCD
    RCALL   Wait10ms			; 10 ms delay subroutine
    
;;;;;; Display ASEN5067 to LCD
    POINT   TopPerm			; ASEN5067
    RCALL   DisplayC			; Display character subroutine
    
;;;;;; Begin LED sequence
    BSF	    LATE,5,A			; Turn on RE5
    BSF	    T0CON,7,A			; Start 1s timer
    RCALL   Wait1s			; 1s delay subroutine
    BCF	    LATE,5,A			; Turn off RE5
    
    BSF	    LATE,6,A			; Turn on RE6
    BSF	    T0CON,7,A			; Start 1s timer
    RCALL   Wait1s			; 1s delay subroutine
    BCF	    LATE,6,A			; Turn off RE6
    
    BSF	    LATE,7,A			; Turn on RE7
    BSF	    T0CON,7,A			; Start 1s timer
    RCALL   Wait1s			; 1s delay subroutine
    BCF	    LATE,7,A			; Turn off RE7
    
;;;;;; 250ms Timer 0 Setup
    MOVLF   00000000B,INTCON,A		; This clears overflow flag if set
    MOVLF   00000011B,T0CON,A		; Set up Timer0 for a delay of 250ms
    MOVLF   high Bignum,TMR0H,A		; Writing binary 3036 to TMR0H / TMR0L
    MOVLF   low Bignum,TMR0L,A		; Write high byte first, then low!
    BSF	    T0CON,7,A			; Start the 250ms timer before mainloop
    BSF	    LATE,4,A			; Turn on "Alive LED"
    BSF	    T1CON,0,A			; Start 20ms timer before mainloop
    BSF	    T5CON,0,A			; Start 1ms timer before mainloop
    BSF	    LATC,2,A			; Turn on RC2 pin for PWM signal
    
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
    LFSR 0,PW				; Set FSR0 to PW
    
    RETURN

;;;;;;;; Wait10ms subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine waits for Timer3 to complete its ten millisecond count
; sequence. It does so by waiting for sixteen-bit Timer3 to roll over. To obtain
; a period of 10ms/250ns = 40000 clock periods, it needs to remove
; 65536-40000 or 25536 counts from the sixteen-bit count sequence.
; (This subroutine is a modified version of the Wait10ms subroutine in the 
;   lab4_example_picas.asm file)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Wait10ms:
        BTFSS 	PIR2,1,A		    ; Read Timer3 TMR3IF rollover flag and ...
        BRA     Wait10ms		    ; Loop if timer has not rolled over
        MOVLF  	high Bignumdos,TMR3H,A	    ; Then write the timer values into
        MOVLF  	low Bignumdos,TMR3L,A	    ; the timer high and low registers
        BCF  	PIR2,1,A		    ; Clear Timer3 TMR3IF rollover flag
        RETURN
	
;;;;;;; InitLCD subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
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
	
;;;;;;; T50 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
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

;;;;;;;; Wait1s subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine waits for Timer0 to complete its one second count
; sequence. It does so by waiting for sixteen-bit Timer0 to roll over. To obtain
; a period of 1s/250ns = 4000000 clock periods, it needs to remove
; 65536-62500 or 3036 counts from the sixteen-bit count sequence.  It uses a
; prescale of 64.
; (This subroutine is a modified version of the Wait10ms subroutine in the 
;   lab4_example_picas.asm file)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Wait1s:
    BTFSS INTCON,2,A		    ; Read Timer0 TMR0IF rollover flag and ...
    BRA Wait1s			    ; Loop if timer has not rolled over
    BCF	T0CON,7,A		    ; Stop the timer
    MOVLF high Bignum,TMR0H,A	    ; Then write the timer values into
    MOVLF low Bignum,TMR0L,A	    ; the timer high and low registers
    BCF INTCON,2,A		    ; Clear Timer0 TMR0IF rollover flag
    RETURN
	
;;;;;;; Check_SW1 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to check the status of RE3 button and change PW 
; (This subroutine is a copy of the Check_SW1 subroutine my Lab 3)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
				
Check_SW1:
    BTFSC PORTE, 0x3, A		; Check if button is pressed and...
    BRA pressedbr		; if yes, jump to the pressed subroutine
    NOP				; NOP to make both pressed and non pressed cycles the same length
    BTFSS press, 0x3, A		; Check if button WAS pressed previously and...
    RETURN 1			; Return to the main loop if not
debouncelp:
    BSF T3CON,0,A		; Start Timer 3 for 10 ms delay
    CALL Wait10ms		; 10 ms delay subroutine
    BTFSC PORTE, 0x3, A		; Check if RE3 has been released and...
    BRA pressedbr		; jump to the pressed subroutine if not
    BCF LATC, 0x2, A		; If it HAS been released, stop PW output
    MOVLF 0x00, press, A	; Reset pressed flag
    RCALL IncreasePW		; Call IncreasePW to increase pulse width
    RCALL UpdateLCD		; Call UpdateLCD to update LCD data
    RETURN 1
pressedbr:
    MOVLF 0x08, press, A	; Set press variable
    RETURN 1
    
;;;;;;; IncreasePW subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to cycle through PW
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

IncreasePW:
    MOVLF '1',PW+4,A		    ; Just to save some time and space, set the 1 char for LCD
    RLNCF PWM,F,A		    ; rotate previous state left
    BTFSC PWM,6,A		    ; Check if bit 6 in PWM set and...
    BRA LCD1			    ; branch to LCD1 to set PW timer and LCD
    BTFSC PWM,1,A		    ; Check if bit 1 in PWM set and...
    BRA LCD12			    ; branch to LCD12 to set PW timer and LCD
    BTFSC PWM,2,A		    ; Check if bit 2 in PWM set and...
    BRA LCD14			    ; branch to LCD14 to set PW timer and LCD
    BTFSC PWM,3,A		    ; Check if bit 3 in PWM set and...
    BRA LCD16			    ; branch to LCD16 to set PW timer and LCD
    BTFSC PWM,4,A		    ; Check if bit 4 in PWM set and...
    BRA LCD18			    ; branch to LCD18 to set PW timer and LCD
    BTFSC PWM,5,A		    ; Check if bit 5 in PWM set and...
    BRA LCD2			    ; branch to LCD2 to set PW timer and LCD
LCD1:
    MOVLF high Bignum10,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum10,TMR5L,A	    ; the timer high and low registers
    MOVLF 0x01,PWM,A		    ; Reset the PWM variable to set bit 0
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
    MOVLF '0', PW+6,A		    ; Make sure we're showing 1.00ms
    BRA fine			    
LCD12:
    MOVLF high Bignum12,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum12,TMR5L,A	    ; the timer high and low registers
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
    MOVLF '2', PW+6,A		    ; Make sure we're showing 1.20ms
    BRA fine
LCD14:
    MOVLF high Bignum14,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum14,TMR5L,A	    ; the timer high and low registers
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
    MOVLF '4', PW+6,A		    ; Make sure we're showing 1.40ms
    BRA fine
LCD16:
    MOVLF high Bignum16,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum16,TMR5L,A	    ; the timer high and low registers
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
    MOVLF '6', PW+6,A		    ; Make sure we're showing 1.60ms
    BRA fine
LCD18:
    MOVLF high Bignum18,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum18,TMR5L,A	    ; the timer high and low registers
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
    MOVLF '8', PW+6,A		    ; Make sure we're showing 1.80ms
    BRA fine
LCD2:
    MOVLF high Bignum20,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum20,TMR5L,A	    ; the timer high and low registers
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
    MOVLF '0', PW+6,A		    
    MOVLF '2', PW+4,A		    ; Make sure we're showing 2.00ms
fine:
    BSF T5CON,0,A		    ; Start the PW timer
    BSF LATC,2,A		    ; Set RC2 to high
    CLRF PW+10,A
    RETURN
    
;;;;;;; UpdateLCD subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to update the LCD
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
UpdateLCD:
    LFSR 0, PW			    ; Make sure FSR0 has address of PW
    RCALL DisplayV		    ; Display PW
    RETURN
    
;;;;;;;;DisplayC subroutine;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
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
	
;;;;;;; DisplayV subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
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
	
;;;;;;; BlinkAlive subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine toggles the Alive LED when called.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

BlinkAlive:
    BTG LATE,4,A			; Toggle Alive LED
    MOVLF   high Bignum,TMR0H,A		; Writing binary 3036 to TMR0H / TMR0L
    MOVLF   low Bignum,TMR0L,A		; Write high byte first, then low!
    BCF INTCON,2,A			; Reset TMR0 overflow flag
    RETURN
    
;;;;;;; PWMOff subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine turns off the output to RC2, resets the Timer 5 overflow flag,
; and turns off Timer 5
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
   
PWMOff:
    BCF T5CON,0,A			; Turn off Timer 5
    BCF LATC,2,A			; Turn off PWM output
    BCF PIR5,1,A			; Reset TMR5 overflow flag
    RETURN
    
;;;;;;; PWMOn subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine resets the 20 ms timer, turns on the 1-2ms timer depending on 
; if a bit is set in PWM, and turns on the output to RC2.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
PWMOn:
    MOVLF high Bignumdos,TMR1H,A    ; Write the timer values into
    MOVLF low Bignumdos,TMR1L,A	    ; the timer high and low registers
    BCF PIR1,0,A		    ; Clear Timer1 TMR1IF rollover flag
    BTFSC PWM,0,A		    ; Check if bit 0 in PWM set and...
    BRA ms1			    ; branch to ms1 to set PW timer
    BTFSC PWM,1,A		    ; Check if bit 1 in PWM set and...
    BRA ms12			    ; branch to ms12 to set PW timer
    BTFSC PWM,2,A		    ; Check if bit 2 in PWM set and...
    BRA ms14			    ; branch to ms14 to set PW timer
    BTFSC PWM,3,A		    ; Check if bit 3 in PWM set and...
    BRA ms16			    ; branch to ms16 to set PW timer
    BTFSC PWM,4,A		    ; Check if bit 4 in PWM set and...
    BRA ms18			    ; branch to ms18 to set PW timer
    BTFSC PWM,5,A		    ; Check if bit 5 in PWM set and...
    BRA ms2			    ; branch to ms2 to set PW timer
ms1:
    MOVLF high Bignum10,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum10,TMR5L,A	    ; the timer high and low registers
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
    BRA fin			    
ms12:
    MOVLF high Bignum12,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum12,TMR5L,A	    ; the timer high and low registers
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
    BRA fin	
ms14:
    MOVLF high Bignum14,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum14,TMR5L,A	    ; the timer high and low registers
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
    BRA fin
ms16:
    MOVLF high Bignum16,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum16,TMR5L,A	    ; the timer high and low registers
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
    BRA fin
ms18:
    MOVLF high Bignum18,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum18,TMR5L,A	    ; the timer high and low registers
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
    BRA fin
ms2:
    MOVLF high Bignum20,TMR5H,A	    ; Write the timer values into
    MOVLF low Bignum20,TMR5L,A	    ; the timer high and low registers
    BCF PIR5,1,A		    ; Clear Timer5 TMR5IF rollover flag
fin:
    BSF T5CON,0,A		    ; Start the PW timer
    BSF LATC,2,A		    ; Set RC2 to high
    RETURN