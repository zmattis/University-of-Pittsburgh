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

  ; r0 is the size of the array (ARR_SIZE)
  LDR R0, =arr_size
  LDR R0, [R0]

  ; r1 is the outer loop counter register (i)
  LDR R1, =0x00

  ; r2 is the inner loop counter register (j)
  LDR R2, =0x00

  ; r3 = load memory address of arr
  LDR R3, =arr

  ; r4 is the number of outer array iterations
  MOV R4, R0
  SUB R4, #0x01


; ARR_SIZE - 1 iterations
array_outer_loop

  ; r5 is the number of inner array iterations
  ; ARR_SIZE - i - 1
  MOV R5, R0
  SUB R5, R1
  SUB R5, #0x01

  ; r3 = load memory address of arr
  LDR R3, =arr

  ; j = 0
  LDR R2, =0x00


; ARR_SIZE - i - 1 iterations
array_inner_loop

  ; r6 is the first element to be compared - arr[j]
  LDR R6, [R3]
  ; r7 is the second element to be compared - arr[j+1]
  LDR R7, [R3, #0x04]

  ; R6 > R7
  CMP R6, R7
  BLE skip

  ; swap
  STR R6, [R3, #0x04]
  STR R7, [R3]

skip
  ADD R3, #0x04 ; arr_ptr += 4
  ADD R2, #0x01 ; j+=1
  CMP R2, R5
  BNE array_inner_loop

  ADD R1, #0x01 ; i+=1
  CMP R1, R4
  BNE array_outer_loop



  ; ---- Print ---- ;
  MOV R5, R0    ; arr_size
  LDR R6, =arr  ; arr_ptr
  LDR R4, =0x00 ; i

display_loop

  ; arr[i], arr_ptr+=4
  LDR R0, [R6], #0x04

  ; set display position by setting R1
  MOV R1, #0x00
  ADD R0, #0x30 ; ASCII
  BL LCD_DisplayLetter

  ; delay 1 second for observation
  MOV R0, #1
  ; call display function
  BL LCD_DisplayDelay

  ; Add r4 by 1 (increment loop variable)
  ADD R4, #0x01

  ; r4 == r5 ?
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

; integer array
arr DCD 0x04, 0x09, 0x03, 0x01, 0x06, 0x03      ; Defines 6 words [32 bits each]
; arr size
arr_size DCB 0x06

  END