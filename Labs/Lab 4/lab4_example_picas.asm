;;;;;;; ASEN 4-5067 Lab 4 Example code ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;   ____ THIS IS **NOT** A TEMPLATE TO USE FOR LAB 4 _____
;
;   .... THIS CODE PROVIDES EXAMPLE FUNCTIONALITY ....
;
;   .... THE TIMING IN THIS CODE IS DIFFERENT THAN REQUIRED FOR LAB 4 ....
;
;   .... USE YOUR LAB 3 SOURCE FILE AS A STARTING POINT FOR LAB 4 ...
; 
;   .... YOU MAY REUSE PARTS OF THIS CODE (ESPECIALLY THE LCD FUNCTIONS!)
;	 IF THEY SUIT YOUR PURPOSE, BUT GIVE CREDIT IN YOUR COMMENTS FOR 
;	 ANY CODE YOU USE FROM HERE
;        FOR EXAMPLE (;   This subroutine is copied (or a modified version) of 
;                     ;   the subroutine XXX in the lab4_example.asm file)
;
;
;	Created:	Scott Palo (scott.palo@colorado.edu)
;	Updated By:	Trudy.Schwartz@colorado.edu	
;	Updated By:	Lara Lufkin (Lara.Lufkin@colorado.edu)
;	Modified:       10-JUNE-21
;	    Update Notes: Updated for MPLAB X IDE v5.50	and XC8 compiler v2.32
;			  References code written by Ruben Hinojosa Torres
; 
;
; NOTES:
;   ~1 second means +/- 10msec, ~250 ms means +/- 10msec
;   Use Timer 0 for main looptime timing requirements
;;;;;;; Program hierarchy ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Mainline
;   Initial
;      BlinkAlive
;      LoopTime
;
;;;;;;; Compiler Notes ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;   
; Compiler Notes: 
; Add this line to the Compiler flags i.e
;   Right click on project name -> Properties -> pic-as Global Options -> 
;   Additional options: 
;    -Wl,-presetVec=0h,-pHiPriISR_Vec=0008h,-pLoPriISR_Vec=0018h
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
ALIVECNT:	DS  1	; Reserve 1 byte for ALIVECNT in access bank
BYTE:		DS  1	; Reserve 1 byte for BYTE in access bank
BYTESTR:	DS  10	; Reserve 10 bytes for BYTESTR in access bank
    
; Objects to be defined in Bank 1
PSECT	udata_bank1
    NOP
    
;;;;;;; Constant Strings (Program Memory) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PSECT romData, space = 0, class = CONST  
LCDstr:  
    DB  0x33,0x32,0x28,0x01,0x0C,0x06,0x00  ;Initialization string for LCD
    
BYTE_1:
    DB 0x80,'B','Y','T','E','=',0x00	    ;Write "BYTE=" to first line of LCD
    
;This method does not work! Follow the method shown avbove.  
;BYTE_1:
;    DB 0x80,"BYTE=",0x00		    ;Write "BYTE=" to first line of LCD

LCDs:
    DB 0x80,'H','e','l','l','o',0x00	    ;Write "Hello" to first line of LCD
    
;The following lines of code are an alternative way of writing bytes to ROM
LCDs2:					    ;Write "World!" to first line of LCD
	DB 0xC0				    ;0xC0 Use the bottom LCD line
IRPC char, World			    ;Write the ASCII "World" characters
	DB 'char'			    ; into program memory
ENDM
	DB 0x21				    ;Exclamation Point
	DB 0x00				    ;End the statement 

;;;;;;; Code Start ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	 
PSECT	code	
// </editor-fold>  
	
;;;;;;; Mainline program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
main:
    rcall   Initial			; Initialize everything
loop:
    BTG LATC,2,A			; Toggle pin, to support measuring loop time
    RCALL BlinkAlive			; Blink "Alive" LED
    MOVLW 10101111B			; Move 0xAF to WREG
    DISPLAY WREG			; Display WREG to the LCD
    RCALL   Wait10ms			; Wait for 10ms
    BRA loop

;;;;;;; Initial subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine performs all initializations of variables and registers.
Initial:
    MOVLF   11000000B,TRISB,A		; Set I/O for PORTB
    MOVLF   00000000B,LATB,A		; Initialize PORTB
    MOVLF   00000000B,TRISC,A		; Set I/0 for PORTC
    MOVLF   00000000B,LATC,A		; Initialize PORTC
    MOVLF   00000000B,TRISE,A		; Set I/O for PORTE
    MOVLF   00000000B,LATE,A		; Initialize PORTE

    MOVLF   00000000B,INTCON,A
    MOVLF   00001000B,T0CON,A		; Set up Timer0 for a delay of 10 ms
    MOVLF   high Bignum,TMR0H,A		; Writing binary 25536 to TMR0H / TMR0L
    MOVLF   low Bignum,TMR0L,A		; Write high byte first, then low!

    MOVLF   250,ALIVECNT,A		; Initializing Alive counter
    BSF     T0CON,7,A			; Turn on Timer0

    RCALL   InitLCD			; Initialize LCD
    RCALL   Wait10ms			; 10 ms delay subroutine

    POINT   LCDs			; Hello
    RCALL   DisplayC			; Display character subroutine
    POINT   LCDs2			; World!
    RCALL   DisplayC

    RETURN
    
;;;;;;; InitLCD subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; InitLCD - modified version of subroutine in Reference: Peatman CH7 LCD
; Initialize the LCD.
; First wait for 0.1 second, to get past display's power-on reset time.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        
InitLCD:
        MOVLF  10,COUNT,A	    ; Wait 0.1 second for LCD to power up
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
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        
T50:
        MOVLW  195/3          ;Each loop L4 takes 3 ins cycles
        MOVWF  COUNT,A		    
L4:
        DECF  COUNT,F,A
        BNZ	L4
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
; This subroutine briefly blinks the LED RD4 every two-and-a-half
; seconds.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

BlinkAlive:
        BCF     LATD,4,A	    ; Turn off LED RD4
        DECF    ALIVECNT,F,A	    ; Decrement loop counter and ...
        BNZ     END1		    ; return if not zero
        MOVLF   250,ALIVECNT,A	    ; Reinitialize BLNKCNT
        BSF     LATD,4,A	    ; Turn on LED RD4 for ten milliseconds every 2.5 sec
END1:
        RETURN

;;;;;;;; LoopTime subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; This subroutine waits for Timer0 to complete its ten millisecond count
;; sequence. It does so by waiting for sixteen-bit Timer0 to roll over. To obtain
;; a period of 10ms/250ns = 40000 clock periods, it needs to remove
;; 65536-40000 or 25536 counts from the sixteen-bit count sequence.  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
Bignum  equ     65536-40000

Wait10ms:
        BTFSS 	INTCON,2,A		    ; Read Timer0 TMR0IF rollover flag and ...
        BRA     Wait10ms		    ; Loop if timer has not rolled over
        MOVLF  	high Bignum,TMR0H,A	    ; Then write the timer values into
        MOVLF  	low Bignum,TMR0L,A	    ; the timer high and low registers
        BCF  	INTCON,2,A		    ; Clear Timer0 TMR0IF rollover flag
        RETURN

;;;;;; ByteDisplay subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Display whatever is in BYTE as a binary number.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
ByteDisplay:
        POINT   BYTE_1			    ;Display "ASEN5067"
        RCALL   DisplayC
        LFSR    0,BYTESTR+8
L10:
        CLRF  WREG,A
        RRCF  BYTE,F,A			    ;Move bit into carry
        RLCF  WREG,F,A			    ;and from there into WREG
        IORLW 0x30			    ;Convert to ASCII
        MOVWF POSTDEC0,A		    ; and move to string
        MOVF  FSR0L,W,A			    ;Done?
        SUBLW low BYTESTR
        BNZ	L10

        LFSR    0,BYTESTR		    ;Set pointer to display string
        MOVLF   0xc0,BYTESTR,A		    ;Add cursor-positioning code
        CLRF    BYTESTR+9,A		    ;and end-of-string terminator
        RCALL   DisplayV
        RETURN

	
;;;;;;; End of Program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
    END     resetVec  