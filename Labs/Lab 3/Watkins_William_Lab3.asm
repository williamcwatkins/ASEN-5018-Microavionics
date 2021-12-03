;;;;;;; Lab 3 Original Template for ASEN 4067/5067 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Author:	    YOUR NAME HERE
; Date:		    DATE HERE
; Target:	    PIC18F87K22
; 	    
; REFERENCES:	    Ruben Hinojosa Torres, Lara Lufkin
;		Using as reference also: main.s by Dan1138
;		lab3_orig.asm by Scott Palo, Doug Weibel, Gabe LoDolce and Trudy Schwartz
; Date (Original):  2021-06-5
; 	    
; Compiler: pic-as(v2.32)
; IDE:      MPLABX v5.50
			      
			      
; !!!!!!!!!!!!!!!IMPORTANT!!!!!!!! 
; Compiler Notes: 
; Add this line to the Compiler flags i.e
;   Right click on project name -> Properties -> pic-as Global Options -> 
;   Additional options: 
;    -Wl,-presetVec=0h,-pHiPriISR_Vec=0008h,-pLoPriISR_Vec=0018h
			      
; Description: 
; On power up execute the following sequence:
; 	RD5 ON for ~1 second then OFF
; 	RD6 ON for ~1 second then OFF
; 	RD7 ON for ~1 second then OFF
; LOOP on the following forever:
; 	Blink "Alive" LED (RD4) ON for ~1sec then OFF for ~1sec
; 	Read input from RPG (at least every 2ms) connected to pins 
;		RD0 and RD1 and mirror the output onto pins RJ2 and RJ3
; 	ASEN5067 ONLY: Read input from baseboard RD3 button and toggle the value 
;		of RD2 such that the switch being pressed and RELEASED causes 
;		RD2 to change state from ON to OFF or OFF to ON
;	NOTE: ~1 second means +/- 100msec
			      
;;;;;;;;;;;;;;;;;;;;;;;;;;;; Program hierarchy ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Mainline
; Loop
; Initial 	- 	Initialize ports and perform LED sequence
; WaitXXXms	- 	Subroutine to wait XXXms
; Wait1sec 	- 	Subroutine to wait 1 sec 
; Check_SW 	- 	Subroutine to check the status of RD3 button and change RD2 (ASEN5067 ONLY)
; Check_RPG	- 	Read the values of the RPG from RD0 and RD1 and display on RJ2 and RJ3	
			      
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Hardware notes ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;	RPG-A port/pin is RJ2
;	RPG-B port/pin is RJ3
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

;;;;;;;;;;;;;;;;;;;;;;;;;; Assembler Directives ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Processor Definition
PROCESSOR   18F87K22
; Radix Definition 
RADIX	DEC
; List Definition
;   C: Set the page (i.e., Column) width
;   N: Set the page length
;   X: Turn MACRO expansion on or off
; LIST	C = 160, N = 0, X = OFF
; Include File:
#include <xc.inc>

; PIC18F87K22 Configuration Bit Settings
// <editor-fold defaultstate="collapsed" desc="CONFIG Definitions">
; CONFIG1L
CONFIG  RETEN = ON            ; VREG Sleep Enable bit (Enabled)
CONFIG  INTOSCSEL = HIGH      ; LF-INTOSC Low-power Enable bit (LF-INTOSC in 
                              ;	    High-power mode during Sleep)
CONFIG  SOSCSEL = HIGH        ; SOSC Power Selection and mode Configuration bits 
			      ;	    (High Power SOSC circuit selected)
CONFIG  XINST = OFF           ; Extended Instruction Set (Disabled)

; CONFIG1H
CONFIG  FOSC = HS1            ; Oscillator (HS oscillator 
			      ;	    (Medium power, 4 MHz - 16 MHz))
CONFIG  PLLCFG = OFF
			      ; PLL x4 Enable bit (Disabled)
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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Variables ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Objects to be defined in Access Bank for Variables.
; Examples:
PSECT	udata_acs
CNT1:	DS  1	; Reserve 1 byte for CNT in access bank at 0x000 (literal or file location)
CNT2:	DS  1
VAL1:   DS  1   ; Reserve 1 byte for VAL1 in access bank at 0x001 (literal or file location)  
GLOBAL L1
L1:	DS  1
GLOBAL L2
L2:	DS  1
GLOBAL L3
L3:	DS  1
GLOBAL L1A
L1A:	DS  1
GLOBAL L2A
L2A:	DS  1
GLOBAL L3A
L3A:	DS  1
GLOBAL tmp1
tmp1:	DS 1
GLOBAL press
press: DS 1
GLOBAL release
release: DS 1
    
; Objects to be defined in Bank 1
PSECT	udata_bank1
; not used

;;;;;;;;;;;;;;;;;;;;;; Power-On-Reset entry point ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PSECT	resetVec, class = CODE, reloc = 2
resetVec:
    NOP	    ; No Operation, give time for reset to occur
    goto    main    ; Go to main after reset

;;;;;;;;;;;;;;;;;;; Interrupt Service Routine Vectors ;;;;;;;;;;;;;;;;;;;;;;;;;;
; High Priority ISR Vector Definition:
PSECT	HiPriISR_Vec, class = CODE, reloc = 2
HiPriISR_Vec:
    GOTO    $	; Return to current Program Counter (For Now - no code here yet)
    
; Low Priority ISR Vector Definition:
PSECT	LoPriISR_Vec, class = CODE, reloc = 2
LoPriISR_Vec:
    GOTO    $	; Return to current Program Counter (For Now - no code here yet)


; Program Section: All Code Starts here
PSECT	code
;;;;;;;;;;;;;;;;;;;;;;;;;;;; Mainline Code ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
main:
    RCALL    Initial	; Call to Initial Routine
loop:
    BTG LATD, 0x4, A
    Call Wait1secloop
    BRA	    loop

;;;;;;;;;;;;;;;;;;;;;; Initialization Routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Initial:
    ; Initialization
    BCF STATUS, 0, A
    BCF STATUS, 1, A
    BCF STATUS, 2, A
    MOVLW 0x19
    ADDLW 0xE7
    
    
    MOVLF 0x0B, TRISD, A	; Set TRISD
    MOVLF 0xF3, TRISJ, A	; Set TRISJ
    MOVLF 0xFE, TRISA, A	; Set TRISA for timing wait subroutine
    MOVLF 0x00, LATA, A
    MOVLF 0x00, LATJ, A
    MOVLF 0x00, LATD, A		; Turn off all LEDS
    RCALL Wait1secInit		; call subroutine to wait 1 second
    BSF LATD, 0x5, A		; Turn ON RD5
    RCALL Wait1secInit		; call subroutine to wait 1 second
    BCF LATD, 0x5, A		; Turn OFF RD5
    BSF LATD, 0x6, A		; Turn ON RD6
    RCALL Wait1secInit		; call subroutine to wait 1 second
    BCF LATD, 0x6, A		; Turn OFF RD6
    BSF LATD, 0x7, A		; Turn ON RD7
    RCALL Wait1secInit		; call subroutine to wait 1 second
    BCF LATD, 0x7, A		; Turn OFF RD7
    RETURN			    ; Return to Mainline code

;;;;;;; Wait2ms subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to wait 2 ms
; NOTE - STUDENTS replace XXX with some value of your choosing
; Choose a suitable value to decrement a counter in a loop structure and 
; not using an excessive amount of program memory - i.e. don't use 100 nop's
		
Wait2ms:
    BSF	LATA, 0x0, A	    ; Set LED0 on LATA for timing; Add code here
    MOVLF 0x28, CNT1, A
    MOVLF 0x83, CNT2, A
loop1:
    CALL loop2, 1
    BCF STATUS, 0x2, A
    DECF CNT1, F, A
    BNZ loop1
    BCF LATA, 0x0, A
    RETURN
loop2: 
    DECF CNT2, F, A
    BNZ loop2
    RETURN 1
    
;;;;;;; Wait1secloop subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to wait 1 sec for the main loop based on calling
Wait1secloop:
    MOVLF 0x0A, L2A, A		; Set length of second loop (2 cycles)
    MOVLF 0x32, L3A, A		; Set length of third loop (2 cycles)
	
    loop1s2l:
	CALL loop1s3l, 1
	DECF L2A, F, A
	BNZ loop1s2l
	RETURN 1
	
    loop1s3l:	    ; calls the 2ms subroutine, ends up being approx 100 ms
	CALL Check_SW1, 1
	CALL Check_RPG, 1
	CALL Wait2ms, 1
	DECF L3A, F, A
	BNZ loop1s3l
	RETURN 1
;;;;;;; Wait1secInit subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to wait 1 sec based on calling WaitXXXms YYY times or up to 3 nested loops
				
Wait1secInit:
    MOVLF 0x20, L1, A		; Set length of first loop (2 cycles)
    MOVLF 0xF8, L2, A		; Set length of second loop (2 cycles)
    MOVLF 0xA6, L3, A		; Set length of third loop (2 cycles)
    
    loop1s1:
	CALL loop1s2, 1		; Call second loop (L1*loop2 cycles + L1*5 cycles)
	BCF STATUS, 0x2, A
	DECF L1, F, A		; Decrement L1 (1 cycle)
	BNZ loop1s1		; branch back to the start of loop until L1=0
	RETURN 1
	
    loop1s2:
	DECF L2, F, A		;
	CALL loop1s3, 1
	BNZ loop1s2
	NOP			; Set Zero bit to 0 in STATUS register
	RETURN 1
	
    loop1s3:
	DECF L3, F, A
	BNZ loop1s3
	RETURN 1

;;;;;;; Check_SW1 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to check the status of RD3 button and change RD2 (ASEN5067 ONLY)
				
Check_SW1:
    BTFSC PORTD, 0x3, A		; Add code here
    BRA pressedbr
    NOP				; NOP to make both pressed and non pressed cycles the same length
    BTFSS press, 0x3, A
    RETURN 1
    MOVLF 0x05, VAL1, A
debouncelp:
    CALL Wait2ms
    DECF VAL1, F, A
    BNZ debouncelp
    BTFSC PORTD, 0x3, A
    BRA pressedbr
    BTG LATD, 0x2, A
    MOVLF 0x00, press, A
    RETURN 1
pressedbr:
    MOVLF 0x08, press, A
    RETURN 1	

;;;;;;; Check_RPG subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to read the values of the RPG and display on RJ2 and RJ3
				
Check_RPG:
    MOVF PORTD, W, A
    RLNCF WREG, W, A
    RLNCF WREG, W, A
    MOVWF LATJ, A		; Add code here
    NOP
    RETURN 1	 
		
    END     resetVec		    ; End program, return to reset vector ;;;;;;; ASEN 4-5067 Lab3 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

