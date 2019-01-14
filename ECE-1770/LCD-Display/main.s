	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s    

	IMPORT LCD_Initialization
	IMPORT LCD_Clear		
	IMPORT LCD_DisplayString

	AREA    main, CODE, READONLY
	EXPORT	__main				; make __main visible to linker
	ENTRY			
				
__main	PROC
	
	BL  LCD_Initialization
	BL  LCD_Clear
	LDR r0,=str
	BL  LCD_DisplayString
  
stop 	B 		stop     		; dead loop & program hangs here

	ENDP
					
	ALIGN			

	AREA myData, DATA, READWRITE
	ALIGN

; Replace ECE1770 with your last name
str DCB "MATTIS",0
	END