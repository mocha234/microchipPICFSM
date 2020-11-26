; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        ; WONG KAH QUAN
        ; HA18015
        ; MIKRORECHNER
        ; nelsonwongisme@gmail.com

        ; 015_LetzteHausfgabe_FinalEx
        ; Datum: 12/10/2020
; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include <p16f648A.inc> ;MC type
    errorlevel -302     ;supress the 'not in bank0' warning

    cblock  0x20    ; Start variables @ 0x20
TicCnt          ; Tick Counter
Temp            ; Temporary Register to store temporary data
FSMStateP       ; FSM's State Pointer, indicates the current state of the system.
FSMTime         ; Time delay duration in FSM, used for Timeout down counting.
InPort          ; Input Port Shadows
OutPort         ; OutPut Port Shadows
    endc

    cblock  0x70    ; Flags and save
w_save              ; save workreg.
STATUS_save         ; save Flags Z, C etc.
Flags               ; System flags
OutFlags            ; Write to ports by drivers
InFlags             ; Read from Input Ports
    endc

;   define Flags inside Flags register as Flags,#
; in file register of Flags:
; Bit 0 = TicFlag, Bit 1 - SekFlag, Bit 2 = FSMF
TicFlag     equ     0   
SekF        equ     1
FSMF        equ     2   ; if State transition is taken place ---> FSMF = 0   
                        ; Previous State != Next State ---------> FSMF = 0
;
;   Portbelegung == Ports' Assignment
; Bb, Bell Button, is a stimulus, a sensor.
Bb      equ     0   ; Pin 0 of PORTA activ high, RA0.

;
;  define event bits in InFlags
BbF         equ     0  ; Bit 0 of InFlags is BbF
nBbF        equ     1  ; Bit 1 of InFlags is nBbF

; Define Outputs to OutFlags --> OutPort --> PORTB
; Below literals are the Bit addresses in OutFlags
; Define bits in OutFlags
; Output Port: Bit 0 = DoorBell, Bit 1 = HL, Bit 2 = DL.
Doorbell    equ     0	; Door Bell: DoorBell = 1, it rings, else, not ringing.
HL  	    equ     1   ; Hallway Light: HL = 1, Lights on, else, off.
DL          equ     2   ; Door Lock: DL = 1, Door is unlock, else, lock.
; More about the Output in the Table below.

;   define constant values
T0reload    equ     0x00    ; TMR0 reload value
Ticload     equ     d'255'  ; Downcounts from 255
;-------------------------------------------------------------
    org 0   ;start with code memory adr. 0
    goto    Start   ; jump to Start
    nop             ; No operation
    nop 
    nop
;-------------------------------------------------------------
ISR         ;Interrupt service routine at adr. 4
            ; Basically Save up the STATUS of the interrupt in STATUS_save
            ; Clear the interrupt overflow(bit T0IF in INTCON)
            ; Set TicFlag
            ; Reload timer0 with T0reload
            ; load back from STATUS_save to STATUS
    movwf   w_save  ;no Z Flag change 
                    ;Move W to F
                    ; Save data from Work Register to w_save
    movf    STATUS,w ; Move STATUS to w, can also be movf STATUS
    movwf   STATUS_save ; Move W to F
                    ; Above two lines is move STATUS -> W -> STATUS_save
                    ; STATUS -> STATUS_save
                    ; save the STATUS of ISR
ServiceT0   ; no other source enabled
    bcf     INTCON,T0IF     ; Bit of T0IF in file register INTCON si cleared.     
                            ; TMR0 Overflow Interrupt Flag bit must be cleared in software
                            ; Timer0 interrupt is generated when the TMR0 register
                            ; timer/counter overflows from FFh to 00h. This overflow
                            ; sets the T0IF bit. The interrupt can be masked by
                            ; clearing the T0IE bit (INTCON<5>). The T0IF bit
                            ; (INTCON<2>) must be cleared in software by the
                            ; Timer0 module interrupt service routine before reenabling this interrupt.
                            
    bsf     Flags,TicFlag
    movlw   T0reload        ; T0reload = 0 --> WREG --> TMR0
    movwf   TMR0            ; TMR0 = T0reload = 0x00. 
Exit    
    movf    STATUS_save,w
    movwf   STATUS      ;from here no change of Z flag
                        ; STATUS_save --> STATUS
    swapf   w_save,f
    swapf   w_save,w
   	retfie
ISR_e
;---------------------------------------------------------------

Start   

PortInit             ; Initialize PortA as Input and PORTB as Output
    banksel PORTA    ; Select Bank where PORTA resides.
    clrf    PORTA
    clrf    PORTB
    banksel TRISA
    movlw   0xff
    movwf   TRISA ; move 0xff to TRISA
    clrf    TRISB
    banksel CMCON  ;switch to page with comparator contrl
    movlw   0x07
    movwf   CMCON
    banksel VRCON
    clrf    VRCON  ;deactivate voltage reference
    banksel OPTION_REG
    movlw   B'11000000' ; maskword to disable PORTB pullups,  
                        ; Interrupt on rising edge of RB0/INT pin.
    iorwf   OPTION_REG,f
    banksel PORTA
PortInit_e

Timer0Init          ; Initialize TMR0
    banksel OPTION_REG  ; Select bank of register of OPTION_REG
    movlw   B'11000000'
    andwf   OPTION_REG,f 
    movlw   B'00000111'
    iorwf   OPTION_REG,f ; OPTION_REG's current bits: 11000111
                         ; use Internal instruction cycle clock (CLKOUT)
                         ; Prescaler is assigned to the Timer0 module
                         ; Prescaler Rate for TMR0 --> 1:256
    movlw   B'10100000'     
    movwf   INTCON       ; INTCON's current bits: 10100000
                         ; All un-masked interrupts ENABLED 
                         ;      > Un-masked interrupt is a hardware interrupt, 
                         ;        in our case, Bell Button(Bb))
                         ;
                         ; All peripheral interrupts DISABLED. Peripheral interrupt is any interrupt other than TMR0, INT, or PORTB change.
                         ; TMR0 interrupt ENABLED
                         ;
                         ; Note that Interrupt requests are asynchronous events which means that an interrupt request
                         ; can occur at any time during the execution of a program. 
    banksel TMR0         ; Select bank of register of TMR0, then copy literal of T0reload to TMR0
    movlw   T0reload        
    movwf   TMR0         ; T0reload --> WREG --> TMR0
                         ; TMR 0 = T0reload = 0x00. 
Timer0Init_e

TicTacInit  ; Initialize TicTac
    movlw   Ticload 
    movwf   TicCnt  ; TicCnt = Ticload
TictacInit_e

FSMInit     ; Initialize Finite State Machine, initially the State is RESET.
            ; Clear the Flags
    clrf    FSMStateP
    clrf    Flags
FSMInit_e

loop        ; Loop forever
 
ShadowIn                ; PORTA is the Input of this microcontroller.
    movf    PORTA,w
    movwf   InPort      ; InPort = PORTA
                        ; It reads data from PORTA, then copy to InPort for further processes.
ShadowIn_e

TicTac                      ; Countinuos down counting.
    bcf     Flags,SekF
    btfss   Flags,TicFlag   ; Initially, TicFlag = 1(from ServiceT0 code block). Will skip "goto TicTac_e", to start to decrement.
                            ; If TicFlag = 0, means down counting already started. 2nd loop and more, not at the first loop of the counting cycles.
    goto    TicTac_e
    bcf     Flags,TicFlag   ; Clear TicFlag to note Down Counting starts.
    decfsz  TicCnt,f        ; Decrement the Tick Counter. iF zero, will skip "goto TicTac_e" to reload the count(Literal from Ticload), as counting is finished.
    goto    TicTac_e        
    movlw   Ticload         ; This line executes when TicCnt = 0.
                            ; Reload the Tick Counter, by load literal of TicLoad to WREG then to TicCnt. Then, set SekF bit in Flags f register.
    movwf   TicCnt          ; TicLoad --> WREG --> TicCnt
                            ; TicCnt = Ticload(d'255' here, it will downcount from 255).
    bsf     Flags,SekF      ; SekFlag = 1 , when TicCnt is reloaded.
TicTac_e

BellDrv       ; Bell Driver, to check for Bell Button pressed or not.
    bcf     InFlags,BbF      
    bsf     InFlags,nBbF    ; First, clear the BbF(Bb Flag) and set nBbF(Not Bell Button Flag)
                            ;  Then go to next line to test if the Bb is pressed/fired or not.
                            ; Why nBF is set? So that Next line will check if Bb is fired or not,
                            ; if Bb not fired, then "goto BellDrv_e"while maintaining nBbF=1,
                            ; show that Bell button is not fired.
    btfss   InPort,Bb       ; gedruckt = 1, nicht gedruckt = 0. Always check if the Bb is pressed.
                            ; If pressed, it will skip "goto BellDrv_e" and Set BbF = 1.
                            ; InPort gets value from PORTA(from ShadowIn Code Block)
    goto    BellDrv_e       
    bsf     InFlags,BbF     
    bcf     InFlags,nBbF    ; This and previous line executed when Bb is fired. 
                            ; Once Bb is fired, Bb = 1, it will set the Bell Button's Flag
                            ; Notifying the Microcontroller that Bell Button is fired.
BellDrv_e

; in FSM:
; Initially, it checks for FSMF, check whether there's state transition or not.
; If the current state is same as the previous state, will not execute FSMdo1, here, it will set the duration of the timeout delay of the
; current state into FSMTime and return Outpattern to WREG then to OutFlags(which then later will copy to PORTB as output of the system).
; OutPattern --> WREG --> OutFlags --> OutPort --> PORTB
; But when the FSMF = 1, the previous state and the current state is same, means there's no state transition.
; will jump to FSMdo1 straight away.

; Code blocks' fuinction/explanation(in short) regarding FSM

; FSM       :   Check for State Transition,if state changed, load Zeit to FSMTime, and load OutPattern to OutFlags. Executing that state's output.
;               if no state change, go FSMdo1.

; FSMdo1    :   No State Transition, test timeout, if Z = 1, means delay timeout, downcount finished, then call TimeExit to next State, save @FSMStateP
;               Here, is when there's no chnage in state, but when Z=0, means still counting, will jump to FSMdo2 to downcount.

; FSMdo2    : Test for Downcount lifetime & Down counting 
;FSMdo3      : Test Bell Button & go to next state(depends on stimulus)
;FSMexit     : Check Previous State & Next State same or not, then exit FSM and produce output and loop again.

; More explanation below.

FSM         ;Tabellen gesteuerter Zustandsautomat/Finite State Machine
    movf    FSMStateP,w     ; initially FSM's State Pointer is zero. Start from S0.
    movwf   Temp            ; Copy data from WREG to Temp to store temporary. Data to be passed to WREG back for W's offset in table of OutPattern
    btfsc   Flags,FSMF      ; Test bit of FSMF.
                            ; If FSMF = 0: There's state transition, the previous and next state is different.
                            ; Skip "goto FSMdo1" when there's no state transition, namely FSMF = 1.
                            ; When there's no state transition, skip to FSMdo1.
                            ; Initially FSMF = 0, then once start looping, FSMF will flag depends on state change at Code Block of FSMexit,
                            ; then FSMF = 0, will loop back here to execute FSMdo1 and so on.
    goto    FSMdo1
    bsf     Flags,FSMF      ; Executing the "Next State", call time and produce output etc. But first reset to 1.
                            ; executed if FSMF = 0 in bit test of FSMF in Flags, i.e., execute when no state transition occurs.
                            ; It will set FSMF = 1. Then invoke Zeit table and produce its corresponding output. If the state not depends on 
                            ; delay timeout to change state, it will return the same literal of state number in TimeExit. 
                            ; For example, S1 --> S1 if it only listen to stimulus such as button.
                            ; From the table "Zeit", it will return a value(time in seconds in this project) as literal to WREG.
                            ; Which then WREG --> FSMTime, i.e., Zeit --> WREG --> FSMTime
    call    Zeit
    movwf   FSMTime         ; Time obtained from Zeit's Table will copy to WREG then FSMTime. That Particular State's Time. 
    movf    Temp,w          ; Temp here holds the FSMStateP(see above)'s value, which is the current state, then copy to WREG
                            ; The reason to move the FSMStateP to Temp then to WREG again, cause later will have to call Outpattern,
                            ; which need WREG's value(FSMStateP) to offset in the table to get the right value from the table.
    call    Outpattern      ; initially StatePointer is zero. Start from S0. So will call Outpattern for S0.
    movwf   OutFlags        ; RETLW Literal(OutPattern) from Table to WREG then to OutFlags.
                            ; This OutPattern is a series of Outputs, where different bits in the OutPattern file register is output of a certain 
                            ; output/actuator. And it is passed to OutFlags for actuation(see OutDrv then ShadowOut).

                            ; In other words, when no state transition, remain this state, invoke time, and give outputs.

FSMdo1                  ; No State Transition, test timeout, if Z = 1, means delay timeout, downcount finished, then call TimeExit to next State, save @FSMStateP
                        ; Here, is when there's no chnage in state, but when Z=0, means still counting, will jump to FSMdo2 to downcount.
                        ; Test timeout & go to next state(depends on stimulus)
                        ; If no change of state, form FSM code block where FSMF =1, goto FSMdo1(here), will executes these.
    movf    FSMTime,f   ; Copy FSMTime to W. Then test bit. 
                        ; If no State Transition,
                        ; FSMTime is from Zeit invoked, and RETLW to WREG then to FSMTime, then here
                        ; If have state transition,
                        ; continue read below
    btfss   STATUS,Z    ; Z flag is set if f == 0(FSMTime is f register here). 
                        ; Note: STATUS is a PIC register
                        ; Z = 1, result of logical or operation is zero, means that FSMTime = 0,i.e., the Timeout Delay finished down counting.
                        ; It then will skip "goto FSMdo2" to "call TimeExit", which is the next state after delay timeout.
    goto    FSMdo2
    movf    Temp,w      ; execute this if Z is set.  The result of an arithmetic or logic operation is zero
                        ; Temp here, is from FSM's codeblock above, which is the FSMStateP, i.e. Current state.
                        ; This current state(0-7) is used for calling out table and offset to the column that indicates the next state
                        ; which then copy back to the FSMStateP(now is the next state) after Timeout. Means this only executed if delay is timeout.
                        ; Look at Zeit, if those that only listen to Stimulus, Zeit = 0.
    call    TimeExit    ; get next state for timeout. The next state's state's number is then store to WREG
    movwf   FSMStateP   ; from WREG, which contains the Next state, now move to FSMStateP.

FSMdo2                  ; Test for Downcount lifetime & Down counting 
    btfss   Flags,SekF  ; SekF = TicCnt is not zero, means TicCnt is loaded, ready to count(check above TicTac code block)
                        ; and SekFlag = 1, then skip next line and decrement.
    goto    FSMdo3
    decf    FSMTime,f   ; Decrement FSMTime if SekFlag(1st bit of Flags) is zero
                        ; It will downcount depends on the duration of the timeout delay of that current state.
                        ; the duration information/data obtain from FSMTime.

FSMdo3                  ; Test Bell Button & go to next state(depends on stimulus)
    movf    Temp,w      ; Temp is the Current State Index(S0,...,S7...)
    call    EventMask   ; Call EventMask table to check which state and check if that state listen to Bb or not.
                        ; then RETLW: return the literal to WREG.
    andwf   InFlags,w   ; InFlags: Bit 0 = BbF, Bit 1 = nBbF
                        ; InFlags AND WREG and store in WREG.
                        ; InFlags holds the data of BbDriver. AND it to check if the Bell Button fired and does this state listen to the stimulus.
                        ; So its kinda protecting the system, not to listen to any stimulus, and specifically set to what it is stimulated.
                        ; Say a state only listen to timer interrupt, and Bb is fired, this FSMdo3 test whether this state listen to the stimulus or not,
                        ; if EveentMask RETLW to WREG = 0 and then logical AND with InFlags, will get 0 for the BbF in InFlags, which stored in WREG.
                        ; But storing in WREG is not important, as the result of the AND logical operation is detected by Bit Z(Zero) of file register STATUS
                        ; Where Z = 1, when the result of logical operation is 0. Means that the state dont listen to Stimulus.
                        ; So if the stat don't listen to Stimulus, Z = 1, will execute "FSMexit", where in "FSMexit"
                        ; It checks for the Previous State & Next State same or not, then exit FSM and produce Outputs and loop again.
                        ; But if Z=0, means that this state listen to Stimulus/Bb, it will copy the value of Temp(which holds the current State index)
                        ; to WREG. To WREG for invoking EventExit, which return the next state Index.
    btfsc   STATUS,Z
    goto    FSMexit     ; Skip if Z = 0. I.e, when the result of operation is not zero. Both WREG and InFlags are set in the LSB.
                        ; Execute this, when Z = 1, where BbF AND W is 0. Means, either BbF or W are 0, or both. WREG is from EventMask from Temp.
                        ; Z must be 0 to skip this and call next state.
    movf    Temp,w      ; Bb is fired and EventMask = 1
                        ; Execute this if result of the logical operation is not zero, where Z = 0.
                        ; In others word, BbF = 1. Bb is pressed and the state is a state that listen to Bb Stimulus.
    call    EventExit   ; Next state after event/condition
    movwf   FSMStateP   ; EventExit -> FSMStateP. Now system/machine is in that Next State.
                        ; This code block is to change to FSMStateP to the apporpriate State.

FSMexit                 ; Check Previous State & Next State same or not, then exit FSM and produce output and loop again.
                        ; FSMF is the FSM's Flag.
                        ; No state transition is taken place --> FSMF = 1    [PS  = NS]
                        ; State transition is taken place    --> FSMF = 0    [PS != NS]
    movf    Temp,w      ; Temp is previous/current state
                        ; Then Exclusive OR with Next State
                        ; This is to check previous and next state in the regitser same or not, as machine don't know whether is changed or not,
                        ; it just copy values.
                        ; So if Previous and Next State is same, will be Zero, as XOR needs either one bit = 1. So if same states, where
                        ; PS = NS, say 0 -> 0 , XOR result = 0, Z = 1,
                        ; when 1 -> 1, XOR result = 0, Z = 1,
                        ; when NS != PS, say 1 -> 0, XOR result = 1, Z = 0.
                        ; Z is a bit in STATUS Register.

                        ; TLDR for this: Z = 1, NS = PS, goto FSM_e and produce outputs.
                        ;                Z = 0, NS != PS, there's change of state, clear FSMF...and loop up back to FSM code block for further execution.

    xorwf   FSMStateP,w ; Z Flag if no change
    btfsc   STATUS,Z    ; FSMStateP XOR W , will skip "goto FSM_e" if WREG = FSMStateP, WREG contains previous state number, where FSMStateP is Next State.
                        ; else FSMStateP != WREG, will execute "goto FSM_e"
                        ; Exit FSM when FSMStateP != WREG, as Z = 1. WREG is the StatePointer of previous state.
                        ; If Previous State is different than the next state, means there is state change.
                        ; Then exit FSM and loop again with FSMF = 1.
    goto    FSM_e       ; Execute if Next State != Previous State, means state transition happened.  
    bcf     Flags,FSMF  ; If FSMStateP XOR W = 1, Z = 0, this line will be executed., clear FSMF in Flags. Not Flagged, still at S0.
                        ; Executed when FSM Previous State is different compared to the Next state. (NS != PS)
                        ; I.e., State transition is taken place,then FSMF = 1, where FSMF is the FSM's Flag.
                        ; clear FSMF to indicates there's state transition then output will be executed.
                        
FSM_e               

OutDrv
    movf    OutFlags,w
    movwf   OutPort     ; OutFlags -> WREG -> OutPort
OutDrv_e

ShadowOut
    movf    OutPort,w
    movwf   PORTB       ; OutPort -> WREG -> PORTB
                        ; PORTB is the Output of this microcontroller.
ShadowOut_e

    goto loop
;
;-----------------------------------Tabellen;-----------------------------------
;           States    
;			S0		S1		S2		S3		S4		S5		S6		S7	
; Literal are in HEX, otherwise stated.	

Zeit        addwf   PCL,f
	dt		0,      3,      7,       5,      3,      2,     d'20',   2	 

    ; Values are in Seconds.
    ; These are the duration needed to change state depends for the state that 
    ; uses timer as interrupt.
    ; S0 is 0s as only listen to Bell Button but not timeout delay.

Outpattern  addwf   PCL,f
	dt      0,	    3,	    2,	     0,	     0,	     4, 	 0, 	 0

    ; Outpattern that will be the output at PORTB
    ; 0x00 : B'00000000', Hallway Light is OFF and DoorBell is NOT ringing, Door is LOCKED
    ; 0x02 : B'00000010', Hallway Light is ON and DoorBell is NOT ringing, Door is LOCKED
    ; 0x03 : B'00000011', Hallway Light is ON and DoorBell is ringing, Door is LOCKED
    ; 0x04 : B'00000100', Hallway Light is OFF and DoorBell is NOT ringing, Door is UNLOCKED

TimeExit    addwf   PCL,f
    dt		0,      2,      3,       6,      6,      7,      0,      0

    ; State will go to Next State after TIMEOUT, independent of Stimulus, for Stimulus, check EventExit(below):
    ;
    ;   S0 :    Only depends on Bb , hence, it will always go back to S0 in loop until stimulated.
    ;   S1 :    S1 --> S2
    ;   S2 :    S2 --> S3
    ;   S3 :    S3 --> S6
    ;   S4 :    S4 --> S6
    ;   S5 :    S5 --> S7
    ;   S6 :    S6 --> S0
    ;   S7 :    S7 --> S0
    ;  
    ;   The duration needed to change state depends 
    ;   on the duration of the delay in table "Zeit"(See above). 
    ;   Only depends on Bb: Means that this state only change reacting to Bell Button,
    ;   instead of change state when dleay timeout. Could be either Bb pressed or released.
    ;   This state will waiting for stimulus to be stimulate to change to next state.

EventMask   addwf   PCL,f
    dt		1,		0,		1,		 1,		 2,		 0,		 0,		 0

    ; Bit 0 of InFlags is BbF
    ; Bit 1 of InFlags is nBbF
    ;
    ; 0x01 == B'00000001'   : BbF = 1, nBbF = 0, only stimulated if Bb is fired
    ; 0x02 == B'00000010'   : BbF = 0, nBbF = 1. only stimulated if Bb is unfired.
    ; 0x00 == B'00000000'   : Not stimulated by Bell Button(Bb), means this state does not listen to Bb, but timeout instead.

EventExit   addwf   PCL,f 
    dt		1,		0,      1,       4,      5,      0,      0,      0

    ; Event Exit
    ; EvEx(Event Exit): Next state after event/condition(e.g.Bb executed)
    ;
    ; When Bb is fired or BbF = 1:
    ;   S0 :    S0 --> S1
    ;   S1 :    Does not listen to Bb
    ;   S2 :    S2 --> S1
    ;   S3 :    S3 --> S4
    ;   S4 :    S4 --> S5
    ;   S5 :    Does not listen to Bb
    ;   S6 :    Does not listen to Bb
    ;   S7 :    Does not listen to Bb      
    ;  
    ; "Does not listen/stimulate to Bb": this state won't change state when Bb is fired or not fired
    ;                                    Only depends on the delay timeout to change state.

    END

; Some Explanations for Table:
; 1 - The OutPattern to be invoke will RETLW to OutFLags then OutPort then PORTB, then output.
;     means if bit of, say DL(4th bit of OutFLags/OutPort/PORTB), is 00001000, only DL is HIGH, means DoorLock enabled.
;     [Reference: .pdf of this Hausaufgabe, Page ]

; 2 - For the addwf PCL,f ==> add W to PCL and store in PCL. PCL is program counter. So to jump to the specify column of
;     table. PCL + W = Address location of that RETLW, so the literal can be return to the WREG.
;     Hence, the WREG before invoking the table is equal to the column's index. 
;     In other words, offsetting in the Program Counter with the help of WREG(WREG here/now stores the FSMStateP, which is the current state).
;     For example: If FSMStateP(in FSM Block, this data is copied to WREG) = 2 (i.e., State 2/S2), then PCL = PCL + 2, where WREG = 2, 
;     That's why need Temp to temporary stores the current State of the machine, as WREG will be use to call table or RETLW(Return literal from table to WREG)
;     Where PCL, the program counter's value = index of column of the table. 
;     the f in addwf PCL,""f"", denotes the value will be saved into PCL, so now the PCL = column's index, then pass to "dt" to RETLW 
;     It will return the literal in the 2nd column of the table.

; 3 - dt : Define table uses RETLW, return literal to WREG. And to choose which column, it is offset by the 
;          addwf PCL,f , which is see above '2'. 

; Note(to self):
; > Don't put bit in Watch Tab, only bytes or register, but comment for understanding which bit is what in the register.
; > Need to take note on register, byte, bits, flag bit and enable etc.
; > Don't forget to disable WDT and use INTOSC for CLKOUT.
; > Don't forget to enable Real-Time Ppdates of Simulation in Settings in MPLAB.
