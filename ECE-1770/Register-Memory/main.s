	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s     

	AREA    main, CODE, READONLY
	EXPORT	__main				; make __main visible to linker
	ENTRY			
				
__main	PROC

	MOVS r1, #0x01
	MOVS r2, #0x02
	ADDS r3, r1, r2
	LDR r0, =value
	STR r3, [r0]
	
	; Clear NZCV flags
	MOVS r0, #0x01
	ADDS r1, r1, r1
	
	; Check C flag
	MOV r1, #0xFFFFFFFF
	ADDS r1, #0x2
	
	; Check Z flag
	MOV r1, #0xFFFFFFFF
	ADDS r1, #0x1
	
	; Check V, N flag
	MOV r1, #0x7FFFFFFF
	ADDS r1, r1, #0x1
	
stop 	B 		stop     		; dead loop & program hangs here

	ENDP
					
	ALIGN			

	AREA    myData, DATA, READWRITE
	ALIGN
array	DCD   1, 2, 3, 4
value	DCD	  0
	END