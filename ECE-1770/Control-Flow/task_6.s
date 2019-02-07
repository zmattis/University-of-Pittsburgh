  INCLUDE core_cm4_constants.s		; Load Constant Definitions
  INCLUDE stm32l476xx_constants.s

  IMPORT LCD_Initialization
  IMPORT LCD_Clear
  IMPORT LCD_DisplayString
  IMPORT LCD_DisplayLetter
  IMPORT LCD_DisplayDelay

  AREA    main, CODE, READONLY
  ;EXPORT	__main				; make __main visible to linker
  ENTRY

__main	PROC

  BL  LCD_Initialization
  BL  LCD_Clear

  ; load memory address of NUM
  LDR R6, =NUM

  ; load value of NUM into r4
  LDR R4, [R6]

  ; init registers
  MOV R5, #50000	; Cannot load 17 bit immediate #100000
  LSL R5, R5, #1  ; so load 16 bit #50000 and shift left x1
  MOV R6, #0	    ; increment
  MOV R3, #6      ; loop counter


loop

  ; R0 = R4/R5
  UDIV R0, R4, R5

  ; R4 = R4-R0*R5
  MUL R2, R0, R5
  SUB R4, R2

  ; R5 = R5/10
  MOV R2, #10
  UDIV R5, R2

  ; convert to ASCII code for display by adding 0x30
  ; R0 = R0+0x30
  ADD R0, #0x30

  ; set display position by setting R1
  ; R1 = R6
  MOV R1, R6


  BL LCD_DisplayLetter

  ; delay 1 second for observation
  ;MOV R0, #1
  ; call display function
  ;BL LCD_DisplayDelay


  ; Add r6 by 1 (increment loop variable)
  ADD R6, #0x01

  ; compare r6 with r3
  CMP R6, R3
  ; if they are not equal, jump to loop
  ; else continue to next line of code
  BNE loop

  ; dead loop & program hangs here
stop 	B 		stop

  ENDP

  ALIGN

  AREA src_data, DATA, READWRITE
  ALIGN

; Replace ECE1770 with your last name
str DCB "MATTIS",0
; 6-digit integer
NUM DCD 123456
;NUM DCD 756432

  END