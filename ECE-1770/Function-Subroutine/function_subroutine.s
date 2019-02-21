  INCLUDE core_cm4_constants.s		; Load Constant Definitions
  INCLUDE stm32l476xx_constants.s

  IMPORT LCD_Initialization
  IMPORT LCD_Clear
  IMPORT LCD_DisplayString
  IMPORT LCD_DisplayLetter
  IMPORT LCD_DisplayDelay

  AREA    main, CODE, READONLY
  EXPORT	__main				; make __main visible to linker
  ENTRY

__main	PROC

  BL  LCD_Initialization
  BL  LCD_Clear

  ; ----- task 1 ----- ;

  ; r4 is the register to be increased by 1 for each loop iteration
  LDR R4, =0x00
  ; r5 is the number of iterations to loop (6)
  LDR R5, =0x06

  ; r6 is comp array
  LDR R6, =Comparator

loop

  ; call rand function
  BL RAND

  ; r7 = comp_arr[i]
  LDR R7, [R6]

  ; r0 > r7 ?
  CMP R0, R7

  BGT gt

  ; display value = 0
  LDR R0, =0x00

  B lte

gt

  ; display value = 1
  LDR R0, =0x01


lte

  ; convert to ASCII code for display by adding 0x30
  ADD R0, #0x30

  ; set display position by setting R1
  MOV R1, #0x00
  BL LCD_DisplayLetter

  ; delay 1 second for observation
  MOV R0, #1
  ; call display function
  BL LCD_DisplayDelay


  ; Add r4 by 1 (increment loop variable)
  ADD R4, #0x01

  ; Increment mem offset
  ADD R6, #0x04

  ; r4 <= r5
  CMP R4, R5

  ; if comparison true, jump to loop
  ; else continue to next line of code
  BLE loop

  ; ----- task 2 ----- ;

  ; Hex disp
  LDR R0, =0xA2
  BL DISPLAYHEX
  MOV R0, #8
  BL LCD_DisplayDelay
  BL LCD_Clear


  ; Bin disp
  LDR R0, =0xA5  ;0b10100101
  BL DISPLAYBIN
  MOV R0, #8
  BL LCD_DisplayDelay
  BL LCD_Clear

  ; ----- task 3 ----- ;

  ; bit set
  LDR R0, =0x01  ; 0x01
  BL BITSET      ; 0x11
  BL DISPLAYBIN
  MOV R0, #8
  BL LCD_DisplayDelay
  BL LCD_Clear

  ; bit clear
  LDR R0, =0x11  ; 0x11
  BL BITCLEAR    ; 0x01
  BL DISPLAYBIN
  MOV R0, #8
  BL LCD_DisplayDelay
  BL LCD_Clear

  ; bit toggle
  LDR R0, =0x01	 ; 0x01
  BL BITTOGGLE   ; 0x11
  BL DISPLAYBIN
  MOV R0, #8
  BL LCD_DisplayDelay
  BL LCD_Clear

  ; bit test
  LDR R0, =0x04   ; 0x04
  BL BITTEST      ; 0x01 (true)
  BL LCD_Clear

  ; ----- task 4 ----- ;

  ; odd sum
  LDR R0, =0x0B ; N=11
  BL ODD_SUM	; SUM = 36 (0x24)
  BL DISPLAYHEX
  MOV R0, #8
  BL LCD_DisplayDelay
  BL LCD_Clear


  ; ----- task 5 ----- ;

  ; multiply
  LDR R0, =0x07
  LDR R1, =0x08
  BL MULTIPLY ; 7*8 = 56 (0x38)
  MOV R0, #8
  BL LCD_DisplayDelay
  BL LCD_Clear

  ; ----- task 6 ----- ;

  ; SUM RECURSE
  LDR R0, =0x0A   ; N = 10
  BL RECURSE_SUM  ; (1+2+3+...+N) = 55 (0x37)
  BL DISPLAYHEX
  MOV R0, #8
  BL LCD_DisplayDelay
  BL LCD_Clear


  ; ODD SUM RECURSE
  LDR R0, =0x0B      ; N =11
  BL RECURSE_SUM_ODD ; (1+3+5+...+N) = 36 (0x24)
  BL DISPLAYHEX
  MOV R0, #8
  BL LCD_DisplayDelay
  BL LCD_Clear


  ; dead loop & program hangs here
stop 	B 		stop

  ENDP



  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ; Function: RAND
  ; Purpose: Generate a random number
  ; Inputs: None
  ; Outputs: A random value returned in registers R0
  ; Registers modified: register R0, R1S
  ; Memory usage: The most recently generated random number is
  ; stored in memory with label LSTRAND
  ; This value is used to generate the next value
  ; Notes: Not the best random number generator around but does a
  ; halfway decent job.
  ; Implementation Notes :
  ; shift the last random value left and add 20
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
RAND PROC
  LDR R1, =LASTRND
  LDR R0, [R1]
  LSL R0, R0, #1
  ADD R0, R0, #20
  STR R0, [R1]
  BX LR

  ENDP

MULTIPLY PROC

  PUSH {R4, R5, R6, LR}
  MUL R4, R0, R1
  LDR R5, =RESULT
  STRB R4, [R5]
  MOV R0, R4
  BL DISPLAYHEX

  POP {R4, R5, R6, LR}
  BX LR

  ENDP



DISPLAYHEX PROC

  PUSH {R4, R5, R6, R7, R8, LR}
  MOV R4, R0
  LSR R5, R4, #0x04
  AND R6, R4, #0x0F

  ; r5 <= 9
  CMP R5, #0x09
  BGT hex_h_gt

  ADD R5, #'0'

  B hex_h_lte

hex_h_gt
  SUB R5, #0x0A
  ADD R5, #'A'

hex_h_lte

  ; r6 <= 9
  CMP R6, #0x09
  BGT hex_l_gt

  ADD R6, #'0'

  B hex_l_lte

hex_l_gt
  SUB R6, #0x0A
  ADD R6, #'A'

hex_l_lte

  MOV R0, R5
  LDR R1, =0x00
  BL LCD_DisplayLetter
  ;MOV R0, #0x01
  ;BL LCD_DisplayDelay
  MOV R0, R6
  LDR R1, =0x01
  BL LCD_DisplayLetter
  ;MOV R0, #0x01
  ;BL LCD_DisplayDelay
  POP {R4, R5, R6, R7, R8, LR}
  BX LR

  ENDP


DISPLAYBIN PROC

  PUSH {R4, R5, R6, R7, R8, LR}
  MOV R6, #0x00
  MOV R4, R0
  LSL R4, #0x18 ; 24

bin_loop

  LSLS R4, #0x01
  MOV R5, #0x00
  ADC R5, #0x00
  ADD R5, #'0'
  MOV R0, R5
  MOV R1, R6
  BL LCD_DisplayLetter
  ;MOV R0, #0x01
  ;BL LCD_DisplayDelay
  ADD R6, #0x01

  ; r6 >= 6
  CMP R6, #0x06
  BNE bin_loop

  POP {R4, R5, R6, R7, R8, LR}
  BX LR

  ENDP


BITSET PROC

  ORR R0, #0x10
  BX LR

  ENDP

BITCLEAR PROC

  AND R0, #0xEF
  BX LR

  ENDP

BITTOGGLE PROC

  EOR R0, #0x10
  BX LR

  ENDP

BITTEST PROC

  PUSH{R4, LR}

  TST R0, #0x04
  BGT test_eq

  MOV R0, #0x00
  LDR R1, =0x00
  ADD R0, #0x30
  BL LCD_DisplayLetter
  B test_ne

test_eq

  MOV R0, #0x01
  LDR R1, =0x00
  ADD R0, #0x30
  BL LCD_DisplayLetter

test_ne

  MOV R0, #8
  BL LCD_DisplayDelay

  POP{R4, LR}
  BX LR

  ENDP

ODD_SUM PROC

  PUSH {R4, R5}
  LDR R4, =0x01
  MOV R5, R0
  MOV R0, #0x00

odd_sum_loop

  ADD R0, R4
  ADD R4, #0x02
  CMP R4, R5
  BLE odd_sum_loop

  POP {R4, R5}
  BX LR

  ENDP


RECURSE_SUM PROC

  PUSH {R4, LR}
  MOV R4, R0

  CMP R0, #0x01
  BEQ recurse_sum_skip
  SUB R0, #0x01
  BL RECURSE_SUM
  ADD R0, R4

recurse_sum_skip

  POP {R4, LR}
  BX LR

  ENDP


RECURSE_SUM_ODD PROC

  PUSH {R4, LR}
  MOV R4, R0

  CMP R0, #0x01
  BEQ recurse_sum_odd_skip
  SUB R0, #0x02
  BL RECURSE_SUM_ODD
  ADD R0, R4

recurse_sum_odd_skip

  POP {R4, LR}
  BX LR

  ENDP




  ALIGN

  AREA myData, DATA, READWRITE
  ALIGN

; Rnd mem
LASTRND DCD 0
; RESULT
RESULT DCB 0
; Comp array
Comparator DCD 10, 62, 110, 299, 621, 1500

  END
