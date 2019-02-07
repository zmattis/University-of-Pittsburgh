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
  ; r5 is the number of iterations to loop
  LDR R5, =0x05

loop
  ; compare r4 with r5
  CMP R4, R5

  ; if they are equal, exit out of loop
  ; else continue to next line of code
  BEQ stop

  ; Add r4 by 1
  ADD R4, #0x01

  ; display R4 @ LCD position 0
  MOV R0, R4

  ; convert to ASCII code for display by adding 0x30
  ADD R0, #0x30
  ; set display position by setting R1
  MOV R1, #0x00
  BL LCD_DisplayLetter

  ; delay 1 second for observation
  MOV R0, #1
  ; call display function
  BL LCD_DisplayDelay

  ; jump to start loop
  B loop

  ; dead loop & program hangs here
stop 	B 		stop

  ENDP

  ALIGN

  AREA myData, DATA, READWRITE
  ALIGN

; Replace ECE1770 with your last name
str DCB "MATTIS",0
  END
