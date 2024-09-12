stm8/
	; works fine , each detent on the encoder displays 2 counts,PC7 & PC6 connected to encoder , PD4 to led
	; enable AFRO 0 in option bytes to activate aternate function tim1_ch1 and tim1_ch2 on PC6 and PC7
	; PD5 is TX for UART at 9600
	#include "mapping.inc"
	#include "stm8s103f.inc"
	
pointerX MACRO first
	ldw X,first
	MEND
pointerY MACRO first
	ldw Y,first
	MEND		
	
	
	
	segment byte at 100 'ram1'
buffer1 ds.b
buffer2 ds.b
buffer3 ds.b
buffer4 ds.b
buffer5 ds.b
buffer6 ds.b
buffer7 ds.b
buffer8 ds.b
buffer9 ds.b
buffer10 ds.b
buffer11 ds.b
buffer12 ds.b
buffer13 ds.b	; remainder byte 0 (LSB)
buffer14 ds.b	; remainder byte 1
buffer15 ds.b	; remainder byte 2
buffer16 ds.b	; remainder byte 3 (MSB)
buffer17 ds.b	; loop counter
captureH ds.b
captureL ds.b	
captureHS ds.b
captureLS ds.b
capture_state ds.b	
nibble1  ds.b
data	 ds.b
address  ds.b
signbit  ds.b
state    ds.b
temp1    ds.b
result4  ds.b
result3  ds.b
result2  ds.b
result1  ds.b
counter1 ds.b
counter2 ds.b
buffers  ds.b 23		
	
	
	segment 'rom'
main.l
	; initialize SP
	ldw X,#stack_end
	ldw SP,X

	#ifdef RAM0	
	; clear RAM0
ram0_start.b EQU $ram0_segment_start
ram0_end.b EQU $ram0_segment_end
	ldw X,#ram0_start
clear_ram0.l
	clr (X)
	incw X
	cpw X,#ram0_end	
	jrule clear_ram0
	#endif

	#ifdef RAM1
	; clear RAM1
ram1_start.w EQU $ram1_segment_start
ram1_end.w EQU $ram1_segment_end	
	ldw X,#ram1_start
clear_ram1.l
	clr (X)
	incw X
	cpw X,#ram1_end	
	jrule clear_ram1
	#endif

	; clear stack
stack_start.w EQU $stack_segment_start
stack_end.w EQU $stack_segment_end
	ldw X,#stack_start
clear_stack.l
	clr (X)
	incw X
	cpw X,#stack_end	
	jrule clear_stack

infinite_loop.l
	  mov CLK_CKDIVR,#$0    ; set max internal clock 16mhz
	  mov TIM1_CCMR1,#$01 	; CC1 channel is configured as input, IC1 is mapped on TI2FP2,PC6
	  mov TIM1_CCMR2,#$01 	; CC2 channel is configured as input, IC2 is mapped on TI2FP2,PC7
	  bres TIM1_CCER1,#1	; Trigger on a high level or rising edge of TI1F
	  bres TIM1_CCER1,#5	; Trigger on a high level or rising edge of TI2F
	  mov TIM1_SMCR,#$03	; SMS = 011 if the counter is counting on both TI1 and TI2 edges
	  ld a,#$03				; MSB of 1000
	  ld TIM1_ARRH,a		; load auto repeat register with 62500= F424, high =0xf4 ,MSB write first
	  ld a,#$E8				; LSB of 1000
	  ld TIM1_ARRL,a		; load auto repeat register with 62500= F424, low =0x24 , LSB written after MSB
	  ;bset TIM1_CR1,#1		;_URS = 1;
	  ;bset TIM1_EGR,#0		;_UG = 1;	  
	  bset TIM1_CR1,#0		; enable timer1

uart_setup:
	 ;UART1_TX PD5
	 ;UART1_RX PD6
	 ld a,#$03				;$0683 = 9600 ,$008B = 115200, 
	 ld UART1_BRR2,a		; write BRR2 firdt
	 ld a,#$68
	 ld UART1_BRR1,a		; write BRR1 next
	 bset UART1_CR2,#3		; enable TX
	 bset UART1_CR2,#2		; enable RX

setup						; timer4 setup for delay 1ms base timer
	  ldw x,#50
	  ldw counter1,x		; load counter1 with 50
	  bset PD_DDR,#4		; PD4 as output
	  bset PD_ODR,#4		; make PD4 high
	  mov TIM4_PSCR ,#$07	; select timer 4 prescaler to 128, 7 means 2^7=128
	  mov TIM4_ARR,#249		; 16mhz/128=125000 =1s,125000/1000=125,load 125-1 as 0 is counted=124
	  bset TIM4_IER,#0		; enable update interrupt in interrupt register
	  bset TIM4_CR1,#0		; enable timer4
	  rim					; enable interrupt globally
	  
	  
	pointerX #string		; call macro pointerX to point address named string (hello world)
	call stringloop			; writes null terminated string pointed by pointerX
	

here
	ld a,state				; copy state register to A
	cp a,#1					; compare A to 1, is flag set in state?
	jrne here				; if A not equal to 1 , sit in a tight loop by branching to label here  
	clr state				; is A equal to 1, state flag set, clear state register/flag
	bcpl PD_ODR,#4			; toggle PD4 led
	
	ld a,TIM1_CNTRH			; copy timer1 counter high register
	ld captureH,a			; store result in captureH
	ld a,TIM1_CNTRL			; copy timer1 counter low register
	ld captureL,a			; store result in captureL
	ldw X,TIM1_CNTRH		; copy timer1 counter high and low word to X
	ldw buffer16,X			; store in buffer16,17
	ldw X,#0				; load x $0000
	ldw buffer14,X			; store 0x0000 in buffer14,,15
	call bin_to_ascii		; procedure to convert binary value to ASCII , to be converted values in buffer15,16,17 msb to lsb , result in buffers to buffers+11
	pointerX #buffers   	; point X to buffers register , start point of ascii value storage, ascii values in buffers
	call write_from_buffers1; wirte converted values in buffers to buffers8 to UART
	pointerX #EOL 			; transmit en of line to UAART
	call stringloop  		; write null terminated string
	jp here					; loop label here
	  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;subtraction routine for BIN to ASCII procedure
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
A32bit_subtraction1:	
	ld a,result1
	sub a,buffer4
	ld result1,a
	ld a,result2
	sbc a,buffer3
	ld result2,a
	ld a,result3
	sbc a,buffer2
	ld result3,a
	ld a,result4
	sbc a,buffer1
	ld result4,a
	JRULT load_signbit_register1
	clr signbit
	ret
load_signbit_register1
	mov signbit,#1
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;converts BINARY to ASCII values , 0 to 10000000
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

bin_to_ascii:
	ldw x,buffer16
	ldw data,x			; result 16bit word stored in buffer5 + buffer6 in data and address registers
	ld a,buffer15		; result MSB in buffer4 stored in nibble register sram, concecutive bytes
	ld nibble1,a		; result MSB in buffer4 stored in nibble register sram, concecutive bytes
	clr buffer1			; clear sram registers for bin_to_ascii calculations
	clr buffer2			; clear sram registers for bin_to_ascii calculations
	clr buffer3			; clear sram registers for bin_to_ascii calculations
	clr buffer4			; clear sram registers for bin_to_ascii calculations
	clr buffer5			; clear sram registers for bin_to_ascii calculations
	clr buffer6			; clear sram registers for bin_to_ascii calculations
	clr buffer7			; clear sram registers for bin_to_ascii calculations
	clr buffer8			; clear sram registers for bin_to_ascii calculations
	clr result4			; clear sram registers for bin_to_ascii calculations
	clr result3			; clear sram registers for bin_to_ascii calculations
	clr result2			; clear sram registers for bin_to_ascii calculations
	clr result1			; clear sram registers for bin_to_ascii calculations
	mov result3,nibble1	; mov MSB of result in nibble1 to buffer6 (buffer5,6,7,8 used for holding result)
	ldw x,data			; load result word (LSB1,LSB0) to data & address register in sran (concecutive) 
	ldw result2,x		; load result word (LSB1,LSB0) to data & address register in sran (concecutive)	
onecrore:
	ldw x,#$9680		; load x with low word of 10,000,000
	ldw buffer3,x		; store in buffer3 and buffer4
	ldw x,#$0098		; load x with high word of 10,000,000
	ldw buffer1,x		; store in buffer1 and buffer2,(buffer1,2,3,4 used for holding test value)
	call A32bit_subtraction1		; call 32 bit subtraction routine, buffer5,6,7,8 - buffer1,2,3,4)
	inc temp1			; increase temp register to count how many 1 crrore in result
	ld a,signbit		; copy signbit register contents to accumulator
	jreq onecrore		; if signbit register is 0 (previous subtraction didnt result in negative) branch onecrore label
	dec temp1			; if negative value in subtraction , decrease temp register (we dont count)
revert_result0:	
	ld a,result1		; laod A with LSB of sutracted result1
	add a,buffer4		; add A with LSB0 of value subtracted. we reverse the result to pre negative value
	ld result1,a		; rectified LSB0 stored back in result1 
	ld a,result2		; laod A with LSB1 of sutracted result2
	adc a,buffer3		; add A with LSB1 of value subtracted. we reverse the result to pre negative value
	ld result2,a		; rectified LSB1 stored back in result2
	ld a,result3		; laod A with LSB2 of sutracted result3
	adc a,buffer2		; add A with LSB2 of value subtracted. we reverse the result to pre negative value
	ld result3,a		; rectified LSB2 stored back in result3 
	ld a,result4		; laod A with MSB of sutracted result4
	adc a,buffer1		; add A with MSB of value subtracted. we reverse the result to pre negative value
	ld result4,a		; rectified MSB stored back in result3 
	ld a,#$30			; ascii 0 loaded in A
	add a,temp1			; add temp1 (contains how many decimal places) to ascii 0 to get ascii value of poaition
	ld buffers ,a		; store result of ascii conversion of MSB position in buffers register SRAM
	clr temp1			; clear temp1 for next decimal position calculation
tenlakh:
	ldw x,#$4240
	ldw buffer3,x
	ldw x,#$000f
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq tenlakh
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 1} ,a	
	clr temp1
onelakh:
	ldw x,#$86A0
	ldw buffer3,x
	ldw x,#$0001
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq onelakh
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 2} ,a
	clr temp1
tenthousand:
	ldw x,#$2710
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq tenthousand
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 3} ,a
	clr temp1
thousand:
	ldw x,#$3e8
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq thousand
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 4} ,a
	clr temp1
hundred:
	ldw x,#$0064
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq hundred
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 5} ,a
	clr temp1
ten:
	ldw x,#$000A
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq ten
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 6} ,a
	clr temp1
UNIT:	
	ld a,#$30			; ascii 0
	add a,result1
	ld {buffers + 7},a
	
	clr buffer1			; clear sram registers for bin_to_ascii calculations
	clr buffer2			; clear sram registers for bin_to_ascii calculations
	clr buffer3			; clear sram registers for bin_to_ascii calculations
	clr buffer4			; clear sram registers for bin_to_ascii calculations
	clr buffer5			; clear sram registers for bin_to_ascii calculations
	clr buffer6			; clear sram registers for bin_to_ascii calculations
	clr buffer7			; clear sram registers for bin_to_ascii calculations
	clr buffer8			; clear sram registers for bin_to_ascii calculations
	clr result4			; clear sram registers for bin_to_ascii calculations
	clr result3			; clear sram registers for bin_to_ascii calculations
	clr result2			; clear sram registers for bin_to_ascii calculations
	clr result1			; clear sram registers for bin_to_ascii calculations
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;writes what is in buffers + bytes defined in temp1
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

TX_LOOP:
	ld a,(x)			; load A with value of string pointed by X
	ld data,a			; copy yte in A to data register
	call UART_TX		; call UART transmit subroutine
	incw X				; increase pointer X
	dec temp1			; decrease temp1 counter value
	jrne TX_LOOP		; loop to TX_LOOP label till temp1 is above 0
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;writes what is in buffers + bytes defined in temp1, with leading ZERO SUPPRESION
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
write_from_buffers1:
	clr state				; flag register for 1st non zero byte
	pointerX #buffers   	; point X to buffers register , start point of ascii value storage, ascii values in buffers
	mov temp1,#9			; temp1 as counter, 10 values to be printed
TX_LOOPNXZ:
	ld a, state				; copy to A flag register
	cp a,#1					; if 1 all leadings 0 finished. current 0 not ignored
	jreq noload0			; if not1  go to noload0 label and ignore all leading 0
	ld a,(x)				; load A with value of string pointed by X
	cp a,#$30				; is this ASCII 0
	jreq suppress_zero		;if ASCII 0 jump to suppress_zero
noload0:
	mov state,#1			; if above was something other than ASCII 0 load state with 1
	ld a,(x)				; load A with byte pointed by X
	ld data,a				; copy byte in A to data register
	call UART_TX			; call UART transmit subroutine
	jp not_last  			;	xxx
suppress_zero:

  	ld a,temp1				;çopy temp1 to A, if byte count is 2 next byte is last byte xxx
  	cp a,#2					;if counter is 2 we are on 8th byte/next is last byte xxxx
  	jrne not_last			;if not 2 this is not the 2nd last byte ,jump to not_last xxxx
  	ld a,(x)				; load A with byte pointed by X	, last byte ;xxxx
  	ld data,a				; copy byte in A to data register	;xxxx
  	call UART_TX			; call UART transmit subroutine, display last byte even if it is 0	;xxxx
  
not_last:
	incw X					; increase pointer X
	dec temp1				; decrease temp1 counter value
	jrne TX_LOOPNXZ			; loop to TX_LOOP label till temp1 is above 0
	ret						; return to caller
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; PRINTS a null terminated string
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
stringloop:
	ld a,(X)
	incw X
	cp a,#$00
	jreq exitstringloop
	ld data,a
	call UART_TX
	jp stringloop
exitstringloop:
	ret
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;UART writing routine, transmits a byte loaded in data register
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
UART_TX:
	ld a,data
	ld UART1_DR,a
TC_FLAG:
	btjf UART1_SR,#6 ,TC_FLAG
	ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; message strings
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

string:
	  dc.B " Hello world!" ,'\n','\n','\r',0
	  
count:
	  dc.B " ADC Counts ",0

EOL:
	  dc.B '\n','\r',0
	  	  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; IRS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	Interrupt timer4_ISR
timer4_ISR
	bres TIM4_SR,#0		; clear update interrupt flag
	ldw x,counter1		; load  with value word in counter1,2
	subw x,#1			; subtract 1 from word in x
	tnzw X				; test for zero or negative in X
	jreq reload_counter	; if 0 jump to label reload_counter
	ldw counter1,x		; write back X to counter1,2 reduced value
	iret				; return from interrupt
reload_counter
	ld a,TIM1_CNTRH		; copy value in timer1 counter high register to A
	ld captureH,a		; store A in captureH
	ld a,TIM1_CNTRL		; copy value in timer1 counter low register to A
	ld captureL,a		; store A in captureL
	ldw x,#50			; load x with counter value 50
	ldw counter1,x		; store in counter1,2
	mov state,#1		; load state register with 1,state flag indicates 50ms reached and uart transmit is started
	iret				; return from interrupt
	
	
	

	interrupt NonHandledInterrupt
NonHandledInterrupt.l
	iret

	segment 'vectit'
	dc.l {$82000000+main}									; reset
	dc.l {$82000000+NonHandledInterrupt}	; trap
	dc.l {$82000000+NonHandledInterrupt}	; irq0
	dc.l {$82000000+NonHandledInterrupt}	; irq1
	dc.l {$82000000+NonHandledInterrupt}	; irq2
	dc.l {$82000000+NonHandledInterrupt}	; irq3
	dc.l {$82000000+NonHandledInterrupt}	; irq4
	dc.l {$82000000+NonHandledInterrupt}	; irq5
	dc.l {$82000000+NonHandledInterrupt}	; irq6
	dc.l {$82000000+NonHandledInterrupt}	; irq7
	dc.l {$82000000+NonHandledInterrupt}	; irq8
	dc.l {$82000000+NonHandledInterrupt}	; irq9
	dc.l {$82000000+NonHandledInterrupt}	; irq10
	dc.l {$82000000+NonHandledInterrupt}	; irq11
	dc.l {$82000000+NonHandledInterrupt}	; irq12
	dc.l {$82000000+NonHandledInterrupt}	; irq13
	dc.l {$82000000+NonHandledInterrupt}	; irq14
	dc.l {$82000000+NonHandledInterrupt}	; irq15
	dc.l {$82000000+NonHandledInterrupt}	; irq16
	dc.l {$82000000+NonHandledInterrupt}	; irq17
	dc.l {$82000000+NonHandledInterrupt}	; irq18
	dc.l {$82000000+NonHandledInterrupt}	; irq19
	dc.l {$82000000+NonHandledInterrupt}	; irq20
	dc.l {$82000000+NonHandledInterrupt}	; irq21
	dc.l {$82000000+NonHandledInterrupt}	; irq22
	dc.l {$82000000+timer4_ISR}	; irq23	; irq23
	dc.l {$82000000+NonHandledInterrupt}	; irq24
	dc.l {$82000000+NonHandledInterrupt}	; irq25
	dc.l {$82000000+NonHandledInterrupt}	; irq26
	dc.l {$82000000+NonHandledInterrupt}	; irq27
	dc.l {$82000000+NonHandledInterrupt}	; irq28
	dc.l {$82000000+NonHandledInterrupt}	; irq29

	end
