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
  LDR R5, =0x6

  ; load memory address of array_1
  LDR R6, =arr1

  ; load memory address of array_2
  LDR R7, =arr2


array_loop

  ; load value of array_1[i] into r2
  LDRB R2, [R6], #0x01

  ; add 10 to arr1 value
  ADD R2, #0x0A

  ; store result in array_2[i]
  STRB R2, [R7], #0x01

  ; Add r4 by 1 (increment loop variable)
  ADD R4, #0x01

  ; compare r4 with r5
  CMP R4, R5
  ; if they are not equal, jump to loop
  ; else continue to next line of code
  BNE array_loop

  ;;;; Reinit registers ;;;;


  ; reload memory address of array_1
  LDR R6, =arr1

  ; reload memory address of array_2
  LDR R7, =arr2

  ; reset loop incremental variable (R4)
  LDR R4, =0x00

display_loop

  ; load value of array_2[i] into r0
  LDRB R0, [R7], #0x01

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
  BNE display_loop


  ; dead loop & program hangs here
stop 	B 		stop

  ENDP

  ALIGN

  AREA src_data, DATA, READWRITE
  ALIGN

; Replace ECE1770 with your last name
str DCB "MATTIS",0
; character array 1
arr1 DCB 'a', 'b', 'c', 'd', 'e', 'f'      ; Defines 6 bytes [8 bits each]
; null array 2
arr2 DCB 0x0, 0x1, 0x2, 0x3, 0x4, 0x5      ; Defines 6 bytes [8 bits each]

  END