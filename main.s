;****************** main.s ***************
; Program written by: Heather Minke and Ajem McConico
; Lab 4
; Date Created: 4/7/2018 
; Last Modified:  4/22/2018
; Section ***Tuesday 1-2***update this***
; Instructor: ***Ramesh Yerraballi**update this***
; Lab number: 4
; Brief description of the program
;   If the switch is presses, the LED toggles at 8 Hz
; Hardware connections
;  PE1 is switch input  (1 means pressed, 0 means not pressed)
;  PE0 is LED output (1 activates external LED on protoboard) 
;Overall functionality of this system is the similar to Lab 3, with three changes:
;1-  initialize SysTick with RELOAD 0x00FFFFFF 
;2-  add a heartbeat to PF2 that toggles every time through loop 
;3-  add debugging dump of input, output, and time
; Operation
;	1) Make PE0 an output and make PE1 an input. 
;	2) The system starts with the LED on (make PE0 =1). 
;   3) Wait about 62 ms
;   4) If the switch is pressed (PE1 is 1), then toggle the LED once, else turn the LED on. 
;   5) Steps 3 and 4 are repeated over and over

SWITCH                  EQU 0x40024004  ;PE0
LED                     EQU 0x40024008  ;PE1
SYSCTL_RCGCGPIO_R       EQU 0x400FE608
SYSCTL_RCGC2_GPIOE      EQU 0x00000010   ; port E Clock Gating Control
SYSCTL_RCGC2_GPIOF      EQU 0x00000020   ; port F Clock Gating Control
GPIO_PORTE_DATA_R       EQU 0x400243FC
GPIO_PORTE_DIR_R        EQU 0x40024400
GPIO_PORTE_AFSEL_R      EQU 0x40024420
GPIO_PORTE_PUR_R        EQU 0x40024510
GPIO_PORTE_DEN_R        EQU 0x4002451C
GPIO_PORTF_DATA_R       EQU 0x400253FC
GPIO_PORTF_DIR_R        EQU 0x40025400
GPIO_PORTF_AFSEL_R      EQU 0x40025420
GPIO_PORTF_DEN_R        EQU 0x4002551C
NVIC_ST_CTRL_R          EQU 0xE000E010
NVIC_ST_RELOAD_R        EQU 0xE000E014
NVIC_ST_CURRENT_R       EQU 0xE000E018
GPIO_PORTF_LOCK_R  		EQU 0x40025520
GPIO_PORTF_CR_R    		EQU 0x40025524
GPIO_PORTF_PUR_R   		EQU 0x40025510
NVIC_ST_CTRL_COUNT    	EQU 0x00010000  ; Count flag
NVIC_ST_CTRL_CLK_SRC  	EQU 0x00000004  ; Clock Source
NVIC_ST_CTRL_INTEN    	EQU 0x00000002  ; Interrupt enable
NVIC_ST_CTRL_ENABLE   	EQU 0x00000001  ; Counter mode
NVIC_ST_RELOAD_M      	EQU 0x00FFFFFF  ; Counter load value

	
           THUMB
           AREA    DATA, ALIGN=4
SIZE       EQU    50
;You MUST use these two buffers and two variables
;You MUST not change their names
;These names MUST be exported
           EXPORT DataBuffer  
           EXPORT TimeBuffer  
           EXPORT DataPt [DATA,SIZE=4] 
           EXPORT TimePt [DATA,SIZE=4]
DataBuffer SPACE  SIZE*4
TimeBuffer SPACE  SIZE*4
DataPt     SPACE  4							;pointer to DataBuffer
TimePt     SPACE  4							;pointer to TimeBuffer
DELAYTIME       EQU 1650000
ledOn           EQU 0x01        ;turns LED on
ledOff          EQU 0x00        ;turns LED off
    
      ALIGN          
      AREA    |.text|, CODE, READONLY, ALIGN=2
      THUMB
      EXPORT  Start
      IMPORT  TExaS_Init

Start BL   TExaS_Init  ; running at 80 MHz, scope voltmeter on PD3
;initialize Port E
	LDR R1, =SYSCTL_RCGCGPIO_R      ;1) activate clock for Port E
    LDR R0, [R1]
    ORR R0, R0, #0x10               ;set bit 4 to turn on clock
    STR R0, [R1]
    NOP
    NOP                             ;allow time for clock to finish
    LDR R1, =GPIO_PORTE_DIR_R       ;5) set direction register
    MOV R0,#0x01                    ;PE0 output and PE1 input
    STR R0, [R1]
    LDR R1, =GPIO_PORTE_AFSEL_R     ;6) regular port function
    MOV R0, #0                      ;0 means disable alternate function
    STR R0, [R1]
    LDR R1, =GPIO_PORTE_DEN_R       ;7) enable Port E digital port
    MOV R0, #0xFF                   ;1 means enable digital I/O
    STR R0, [R1]
;initialize Port F
	LDR R1, =SYSCTL_RCGCGPIO_R      ;1) activate clock for Port F
    LDR R0, [R1]
    ORR R0, R0, #0x20               ;set bit 5 to turn on clock
    STR R0, [R1]
    NOP
    NOP                             ;allow time for clock to finish
    LDR R1, =GPIO_PORTF_LOCK_R      ;2) unlock the lock register
    LDR R0, =0x4C4F434B             ;unlock GPIO Port F Commit Register
    STR R0, [R1]
    LDR R1, =GPIO_PORTF_CR_R        ;enable commit for Port F
    MOV R0, #0xFF                   ;1 means allow access
    STR R0, [R1]
    LDR R1, =GPIO_PORTF_DIR_R       ;5) set direction register
    MOV R0,#0x0E                    ;PF7-4 input
    STR R0, [R1]
    LDR R1, =GPIO_PORTF_AFSEL_R     ;6) regular port function
    MOV R0, #0                      ;0 means disable alternate function
    STR R0, [R1]
    LDR R1, =GPIO_PORTF_PUR_R       ;pull-up resistors for PF4,PF0
    MOV R0, #0x11                   ;enable weak pull-up on PF0 and PF4
    STR R0, [R1]
    LDR R1, =GPIO_PORTF_DEN_R       ;7) enable Port F digital port
    MOV R0, #0xFF                   ;1 means enable digital I/O
    STR R0, [R1]
;initialize debugging dump, including SysTick
	BL Debug_Init

      CPSIE  I    ; TExaS voltmeter, scope runs on interrupts
	  
	;Registers
    ;R0 = Port E memory location
    ;R1 = LED on/off
    ;R2 = DELAYTIME
    ;R3 = percent
    ;R4 = 
    ;R5 = button history
    ;R6 = Port E register
    ;R7 = 
    ;R8 = subroutine LR
	;R9 = 
	;R10 = Port F memory location
	;R11 = 
	;R12 = NEntries
	
	MOV R5, #0                      ;initial button history
    MOV R3, #20                     ;initial duty cycle of 20%
	  
loop  
	BL Debug_Capture
;check PE0
    LDR R0, =GPIO_PORTE_DATA_R      ;loading memory location of Port E
    LDR R6, [R0]
    LSR R6, #1
    CMP R6, #1
    BEQ IF1
    CMP R5, #1
    BEQ ELSE1
    B DONE
IF1 
    MOV R5, #1
    B DONE
ELSE1 
    MOV R5, #0
    BL increaseDutyCycle
DONE
	
;LED on
    BL LedOn
    
;check PE0
    LDR R0, =GPIO_PORTE_DATA_R      ;loading memory location of Port E
    LDR R6, [R0]
    LSR R6, #1
    CMP R6, #1
    BEQ IF3
    CMP R5, #1
    BEQ ELSE3
    B ButtonDone
IF3 
    MOV R5, #1
    B ButtonDone
ELSE3 
    MOV R5, #0
    BL increaseDutyCycle
ButtonDone
	
;LED off
    BL LedOff
	
;heartbeat
	LDR R10, =GPIO_PORTF_DATA_R		;Port F data address 
	;turn blue LED on
	LDR R2, =DELAYTIME
	MOV R1, #20						;beginning of on duty cycle calculation
    MUL R2, R1					
    MOV R4, #100
    UDIV R2, R4						;end of on duty cycle calculation
	MOV R1, #4						;blue LED on PF4 turns on
    STR R1, [R10]
heartbeatDelay
    SUBS R2, #1                     ;R2 = R2 - 1 (count = count - 1)
    BNE heartbeatDelay              ;if count (R2) != 0, branch to 'delay'
	;turn blue LED off
	LDR R2, =DELAYTIME
    MOV R4, #100					;beginning of off duty cycle calculation
	MOV R1, #80
    MUL R2, R1
    UDIV R2, R4						;end of off duty cycle calculation
    MOV R1, #0						;blue LED on PF4 turns off
    STR R1, [R10]
heartbeatDelay1
	SUBS R2, #1                     ;R2 = R2 - 1 (count = count - 1)
    BNE heartbeatDelay1             ;if count (R2) != 0, branch to 'delay'
;Delay
;input PE1 test output PE0
	  B    loop


;------------Debug_Init------------
;Initializes the debugging instrument
;Input: none
;Output: none
;Modifies: none
;Note: push/pop an even number of registers so C compiler is happy

Debug_Init
;init SysTick
	MOV R6, LR
	BL SysTick_Init
	MOV LR, R6
	LDR R2, =DataBuffer		;pointer to the structure DataBuffer
	LDR R3, =DataPt			;pointer to the DataPt
	STR R2, [R3]			;DataPt is a pointer to DataBuffer[0]
	MOV R0, #50				;counter
	MOV R1, #0xFFFFFFFF		;number to fill array with
dataBuffer
	CMP R0, #0
	BEQ DoneData
	STR R1, [R3]			;stores 1 byte in R1 to where the DataPt is pointing
	ADD R3, #4				;increments the memory address the pointer is pointing to
	SUB R0, #1				;decrements the counter
	B dataBuffer
DoneData
	STR R2, [R3]
	LDR R4, =TimeBuffer		;pointer to the structure TimeBuffer
	LDR R5, =TimePt			;pointer to the TimePt
	STR R4, [R5]			;TimePt is a pointer to TimeBuffer[0]
	MOV R0, #50				;counter
timeBuffer
	CMP R0, #0
	BEQ DoneTime
	STR R1, [R5]			;stores 1 byte in R1 to where the TimePt is pointing
	ADD R5, #4				;increments the memory address the pointeer is pointing to
	SUB R0, #1				;increments the counter
DoneTime
	STR R4, [R5]
	MOV R12, #0				;set NEntries to 0.
	BX LR

;------------Debug_Capture------------
;Dump Port E and time into buffers
;Input: none
;Output: none
;Modifies: none
;Note: push/pop an even number of registers so C compiler is happy

;Registers
;	R0 = Port E memeory location
;	R1 = NVIC_ST_CURRENT_R memeory location
;	R2 = DataBuffer					;array structure
;	R3 = DataPt						;pointer to DataBuffer
;	R4 = TimeBuffer					;array structure
;	R5 = TimePt						;pointer to TimeBuffer
;	R6 = Port E
;	R7 = NVIC_ST_CURRENT_R pointer
;	R8 = 
;	R9 = 
;	R10 = 
;	R11 = 
;	R12 = NEntries

Debug_Capture
	PUSH {R0-R11}					;push registers R0 through R11 onto stack
	CMP R12, #50					;check to see if NEntries is 50
	BEQ return
	LDR R0, =GPIO_PORTE_DATA_R      ;loading memory location of Port E
    LDR R6, [R0]
	AND R6, #0x2					;just the input/output bits
	LDR R1, =NVIC_ST_CURRENT_R		;pointer to SysTick
	LSR R7, R6, #3					;shifted Port E data bit 1 into bit 4 position and left bit 0 in the bit 0 position
	AND R7, #0x10
	AND R6, #1
	ADD R6, R7
	STR R6, [R3]					;dump port info into DataBuffer using DataPt
	ADD R3, #4						;increment DataPt to next address.
	LDR R7, [R1]					;NVIC_ST_CURRENT_R pointer
	STR R7, [R5]					;stores SysTick data in TimeBuffer using TimePt
	ADD R5, #4						;increment TimePt to next address.
	ADD R12, #1						;increment NEntries
return
	POP {R0-R11}					;pop 12 words from stack and place them in registers R0 - R11
    BX LR
	  
SysTick_Init
    ;disable SysTick during setup
    LDR R1, =NVIC_ST_CTRL_R         ;R1 = &NVIC_ST_CTRL_R
    MOV R0, #0                      ;R0 = 0
    STR R0, [R1]                    ;[R1] = R0 = 0
;maximum reload value
    LDR R1, =NVIC_ST_RELOAD_R       ;R1 = &NVIC_ST_RELOAD_R
    LDR R0, =NVIC_ST_RELOAD_M;      ;R0 = NVIC_ST_RELOAD_M
    STR R0, [R1]                    ;[R1] = R0 = NVIC_ST_RELOAD_M
;any write to current clears it
    LDR R1, =NVIC_ST_CURRENT_R      ;R1 = &NVIC_ST_CURRENT_R
    MOV R0, #0                      ;R0 = 0
    STR R0, [R1]                    ;[R1] = R0 = 0
;enable SysTick with core clock
    LDR R1, =NVIC_ST_CTRL_R         ;R1 = &NVIC_ST_CTRL_R
                                    ;R0 = ENABLE and CLK_SRC bits set
    MOV R0, #(NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC)
    STR R0, [R1]                    ;[R1] = R0 = (NVIC_ST_CTRL_ENABLE|NVIC_ST_CTRL_CLK_SRC)
    BX  LR                          ;return

;------------SysTick_Wait------------
;Time delay using busy wait.
;Input: R0  delay parameter in units of the core clock (units of 12.5 nsec for 80 MHz clock)
;Output: none
;Modifies: R0, R1, R3
SysTick_Wait
    LDR  R1, =NVIC_ST_RELOAD_R      ;R1 = &NVIC_ST_RELOAD_R
    SUB  R0, #1
    STR  R0, [R1]                   ;delay-1;  // number of counts to wait
    LDR  R1, =NVIC_ST_CTRL_R        ;R1 = &NVIC_ST_CTRL_R
SysTick_Wait_loop
    LDR  R3, [R1]                   ;R3 = NVIC_ST_CTRL_R
    ANDS R3, R3, #0x00010000		;Count set?
    BEQ  SysTick_Wait_loop
    BX   LR                         ;return

;------------SysTick_Wait10ms------------
;Time delay using busy wait.  This assumes 50 MHz clock
;Input: R0  number of times to wait 10 ms before returning
;Output: none
;Modifies: R0
DELAY10MS             EQU 800000    ;clock cycles in 10 ms (assumes 80 MHz clock)
SysTick_Wait10ms
    PUSH {R4, LR}                   ;save current value of R4 and LR
    MOVS R4, R0                     ;R4 = R0 = remainingWaits
    BEQ SysTick_Wait10ms_done       ;R4 == 0, done
SysTick_Wait10ms_loop
    LDR R0, =DELAY10MS              ;R0 = DELAY10MS
    BL  SysTick_Wait                ;wait 10 ms
    SUBS R4, R4, #1                 ;R4 = R4 - 1; remainingWaits--
    BHI SysTick_Wait10ms_loop       ;if(R4 > 0), wait another 10 ms
SysTick_Wait10ms_done
    POP {R4, LR}                    ;restore previous value of R4 and LR
    BX  LR                          ;return

;delay subroutine
delay
    CMP R2, #0
    BEQ delayDone                   ;if R2 = 0, branch to delayDone
    SUBS R2, #1                     ;R2 = R2 - 1 (count = count - 1)
    BNE delay                       ;if count (R2) != 0, branch to 'delay'
delayDone
    BX LR                          	;return
    
onDutyCycle
    MUL R2, R3
    MOV R4, #100
    UDIV R2, R4
    BX LR
    
offDutyCycle
    MOV R4, #100
    SUB R7, R4, R3
    MUL R2, R7
    UDIV R2, R4
    BX LR
    
increaseDutyCycle
    ADD R3, R3, #20
    CMP R3, #100
    BHI IF2
    B doneIncrease
IF2 
    MOV R3, #0
doneIncrease
    BX LR
    
LedOn
    LDR R2, =DELAYTIME
    MOV R8, LR
    BL onDutyCycle
    MOV LR, R8
    CMP R2, #0
    BEQ off
    LDR R1, =ledOn
    STR R1, [R0]
    BL delay
    MOV LR, R8
off
    BX LR
	
LedOff
    LDR R2, =DELAYTIME
    MOV R8, LR
    BL offDutyCycle
    MOV LR, R8
    CMP R2, #0
    BEQ on
    LDR R1, =ledOff
    STR R1, [R0]
    BL delay
    MOV LR, R8
on
    BX LR
	
switch1									;PF4
	LDR R10, =GPIO_PORTF_DATA_R			;Port F data address 
    LDR R6, [R10]						;R10 data to R6 
    LSR R6, R6, #4						;Second button value  
    EOR R6, R6, #1						;Flip from 0 <-> 1 
	BX LR

    ALIGN								;make sure the end of this section is aligned
    END									;end of file