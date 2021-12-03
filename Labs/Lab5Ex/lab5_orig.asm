;;;;;;; ASEN 4-5067 Lab 5 Example code ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;	Originally Created:	Scott Palo (scott.palo@colorado.edu)
;	Modified:	Trudy Schwartz (trudy.schwartz@colorado.edu)
;	Updated By:	Ruben Hinojosa Torres (ruhi9621@colorado.edu)
;	Modified:       05-AUGUST-21
;
; NOTES:
;   Use Timer 0 for looptime timing requirements
;
;   This code Generate a jitterfree 10 Hz square wave on CCP1 output using 
;    compare mode with 24bit extension bytes.
;
;   You may re-use parts of lab 4 code (especially the LCD functions!) but 
;    remember to give credit in your comments for any code your didnt write.
;
;;;;;;; Program hierarchy ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Mainline
;   Initial
;   Example RPG Code
;   High Priority ISR Example Shell
;   Low Priority ISR
;	CCP1 Handler
;	TMR1 Handler
;
;;;;;;; Compiler Notes ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;   
; Compiler Notes: 
; Add this line to the Compiler flags i.e
;   Right click on project name -> Properties -> pic-as Global Options -> 
;   Additional options: 
;    -Wl,-presetVec=0h,-pHiPriISR_Vec=0008h,-pLoPriISR_Vec=0018h
;
;;;;;;;;;;;;;;;;;;;;;;;;; Hardware Notes ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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

;;;;;;;;;;;;;;;;;;;;;;;;; Assembler Directives ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
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
// <editor-fold defaultstate="collapsed" desc="MACRO Definitions">			    
; MACRO Definitions:

; MOVLF
; Description:
;   Move literal value to given register. 
; Input: 
;   lit: literal value
;   dest: destination should be a full 3 byte address
;   access: Access bank or not. Possible values are 'a' for access bank or
;	'b' for banked memory.
  MOVLF	    MACRO   lit, dest, access
    MOVLW   lit	    ; Move literal into WREG
    BANKSEL	(dest)	; Determine bank and set BSR for next file instruction
    MOVWF   BANKMASK(dest), access  ; Move WREG into destination file
  ENDM
  
// </editor-fold>
  
;;;;;;;;;;;;;;;;;;;;;;;;; Program Vectors ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  
// <editor-fold defaultstate="collapsed" desc="Program Vectors">
  ;;;;;;;;;;;;;;;;;;;;;; Power-On-Reset entry point ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PSECT	resetVec, class = CODE, reloc = 2
resetVec:
    NOP	    ; No Operation
    goto    main    ; Go to main after reset

;;;;;;;;;;;;;;;;;;; Interrupt Service Routine Vectors ;;;;;;;;;;;;;;;;;;;;;;;;;;
; High Priority ISR Vector Definition:
PSECT	HiPriISR_Vec, class = CODE, reloc = 2
HiPriISR_Vec:
    GOTO    HiPriISR	; Go to High Priority ISR
    
; Low Priority ISR Vector Definition:
PSECT	LoPriISR_Vec, class = CODE, reloc = 2
LoPriISR_Vec:
    GOTO    LoPriISR	; Go to Low Priority ISR
// </editor-fold>  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Variables ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
// <editor-fold defaultstate="collapsed" desc="Variables">
; Objects to be defined in Access Bank
PSECT	udata_acs
WREG_TEMP:	DS	1   ; Temp variables used in Low Pri ISR
STATUS_TEMP:	DS	1
BSR_TEMP:	DS	1    
TMR1X:		DS	1   ; Eight-bit extension to TMR1
CCPR1X:		DS	1   ; Eight-bit extension to CCPR1
DTIMEX:		DS	1   ; Delta time variable of half period of square wave
DTIMEH:		DS	1   ; Will copy HalfPeriod constant into these 3 registers
DTIMEL:		DS	1
DIR_RPG:	DS	1   ; Direction of RPG
RPG_TEMP:	DS	1   ; Temp variable used for RPG state
OLDPORTD:	DS	1   ; Used to hold previous state of RPG

; Objects to be defined in Bank 1
PSECT	udata_bank1
    NOP
// </editor-fold>
    
;;;;;;; Code Start ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PSECT	code	

;;;;;;;;;;;;;;;;;;;;;; Definitions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
HalfPeriod  EQU	    200000  ; Number of 250 ns instruction cycles in 0.05 sec 
			    ; (Half of 10 Hz)
			    ; Only for example, not useful directly for Lab 5

;;;;;;; Mainline program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
main:
    RCALL   Initial			; Initialize everything
L1:
    BRA	    L1

;;;;;;; Initial subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine performs SOME of the initializations of variables and
; registers. YOU will need to add those that are omitted/needed for your 
; specific code
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Initial:
    MOVLF   low HalfPeriod, DTIMEL, a	; Load DTIME with HalfPeriod constant
    MOVLF   high HalfPeriod, DTIMEH, a
    MOVLF   low highword HalfPeriod, DTIMEX, a
    
    CLRF    TRISC, a		    ; Set I/O for PORTC
    CLRF    LATC, a		    ; Clear lines on PORTC
    MOVLF   00000010B, T1CON, a	    ; 16 bit timer, buffer H/L registers
    MOVLF   00001010B, CCP1CON, a   ; Select compare mode, software interrupt only
    MOVLB   0x0F		    ; Set BSR to bank F for SFRs outside of access bank				
    MOVLF   00000000B, CCPTMRS0, b  ; Set TMR1 for use with ECCP1, Using BSR!
    BSF	    RCON, 7, a		    ; Set IPEN bit <7> enables priority levels
    BCF	    IPR1, 0, a		    ; TMR1IP bit <0> assigns low priority to TMR1 interrupts
    BCF	    IPR3, 1, a		    ; CCP1IP bit<1> assign low pri to ECCP1 interrupts
    CLRF    TMR1X, a		    ; Clear TMR1X extension
    MOVLF   low highword HalfPeriod, CCPR1X, a	; Make first 24-bit compare 
						; occur quickly 16bit+8bit ext 
						; Note: 200000 (= 0x30D40)
    BSF	    PIE3, 1, a	    ; CCP1IE bit <1> enables ECCP1 interrupts
    BSF	    PIE1, 0, a	    ; TMR1IE bit <0> enables TMR1 interrupts
    BSF	    INTCON, 6, a    ; GIEL bit <6> enable low-priority interrupts to CPU
    BSF	    INTCON, 7, a    ; GIEH bit <7> enable all interrupts
    BSF	    T1CON, 0, a	    ; TMR1ON bit <0> turn on timer1

    RETURN
    
;;;;;;; RPG subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Credit: This subroutine modified from Peatman book Chapter 8 - RPG
; This subroutine decyphers RPG changes into values of DIR_RPG of 0, +1, or -1.
; DIR_RPG = +1 for CW change, 0 for no change, and -1 for CCW change.
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

;;;;;;; HiPriISR interrupt service routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

HiPriISR:                        ; High-priority interrupt service routine
;       <execute the handler for interrupt source>
;       <clear that source's interrupt flag>
        RETFIE  1	    ; Return and restore STATUS, WREG, and BSR
			    ; from shadow registers

;;;;;;; LoPriISR interrupt service routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

LoPriISR:				; Low-priority interrupt service routine
        MOVFF	STATUS, STATUS_TEMP	; Set aside STATUS and WREG
        MOVWF	WREG_TEMP, a
	MOVFF	BSR, BSR_TEMP        
L2:
        BTFSS	PIR3, 1, a	; Test CCP1IF bit <1> for this interrupt
        BRA	L3
        RCALL	CCP1handler	; Call CCP1handler for generating RC2 output
        BRA	L2
L3:
        BTFSS	PIR1, 0, a	; Test TMR1IF bit <0> for this interrupt
        BRA	L4
        RCALL	TMR1handler	; Call TMR1handler for timing with CCP1
        BRA	L2
L4:
        MOVF	WREG_TEMP, w, a	    ; Restore WREG and STATUS
        MOVFF	STATUS_TEMP, STATUS
	MOVFF	BSR_TEMP, BSR        
        RETFIE			; Return from interrupt, reenabling GIEL
	
;;;;;;;; CCP Handler ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
CCP1handler:			; First must test of TMR1IF occurred at the same time
        BTFSS	PIR1, 0, a	; If TMR1's overflow flag is set? skip to test CCP bit7
        BRA	L5		; If TMR1F was clear, branch to check extension bytes
        BTFSC	CCPR1H, 7, a	; Is bit 7 a 0? Then TMR1/CCP just rolled over, need to inc TMR1X
        BRA	L5		; Is bit 7 a 1? Then let TMR1handler inc TMR1X 
        INCF	TMR1X, f, a	; TMR1/CCP just rolled over, must increment TMR1 extension
        BCF	PIR1, 0, a	; and clear TMR1IF bit <0> flag 
				;(Since TMR1 handler was unable to and arrived here first!)
L5:
        MOVF	TMR1X, w, a	; Check whether extensions are equal
        SUBWF	CCPR1X, w, a	; by subtracting TMR1X and CCPR1X, check if 0
        BNZ	L7		; If not, branch to return
        BTG	LATC, 2, a	; Manually toggle RC2
	MOVF	DTIMEL, w, a	; and add half period to CCPR1 to add more pulse time
        ADDWF	CCPR1L, f, a
        MOVF	DTIMEH, w, a	; Add to each of the 3 bytes to get 24 bit CCP
        ADDWFC	CCPR1H, f, a
        MOVF	DTIMEX, w, a
        ADDWFC	CCPR1X, f, a
L7:
        BCF	PIR3, 1, a	; Clear the CCP1IF bit <1> interrupt flag
        RETURN

;;;;;;;; TMR Handler ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
TMR1handler:
        INCF	TMR1X, f, a	;Increment Timer1 extension
        BCF	PIR1, 0, a	;Clear TMR1IF flag and return to service routine
        RETURN
	
;;;;;;; End of Program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
    END     resetVec  