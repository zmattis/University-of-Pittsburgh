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

  ; r4 is the register to be increased by 1 for each loop iteration
  LDR R4, =0x00
  ; r5 is the number of iterations to loop (6)
  LDR R5, =0x06

  ; load memory address of array
  LDR R6, =arr
  ; load word sizes
  MOV R7, #0x04
  ; Find end memory offset by multiplying array size (R5) by a word (4 bytes)
  MUL R3, R5, R7
  ; Subtract 1 byte offset
  ADD R3, #-0x04
  ; Add memory offset to array start
  ADD R6, R3


loop

  ; load value of array[i] into r0 (display register)
  LDR R0, [R6], #-0x04

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

  ; compare r4 with r5
  CMP R4, R5
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
; integer array
arr DCD 0x1, 0x2, 0x6, 0x8, 0x9, 0x5      ; Defines 6 words [32 bits each]

  END