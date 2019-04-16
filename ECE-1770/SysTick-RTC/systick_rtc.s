    INCLUDE core_cm4_constants.s     ; Load Cortex-M4 Definitions
    INCLUDE stm32l476xx_constants.s  ; Load STM32L4 Definitions
    EXPORT SysTick_Handler

    IMPORT config_NVIC_in_C

    IMPORT System_Clock_Init
    IMPORT RTC_Clock_Init

    IMPORT LCD_Initialization
    IMPORT LCD_Clear
    IMPORT LCD_DisplayString
    IMPORT LCD_DisplayLetter
    IMPORT LCD_DisplayDelay

    IMPORT BIN2BCD
    IMPORT BCD2BIN

    IMPORT LCD_Display_Seconds
    IMPORT LCD_Display_Minutes
    IMPORT LCD_Display_Hours

    AREA    main, CODE, READONLY
    EXPORT  __main         ; make __main visible to linker
    ENTRY

__main  PROC

    BL System_Clock_Init

    ; Call LCD Initilization
    BL LCD_Initialization
    BL LCD_Clear

    ; Configure RTC
    BL rtc_init

    ; Configure SysTick
    LDR r0, =1000   ; interrupt period = 1000 cycles
    BL systick_init

    MOV r2, #0x00  ; reset counter
loop
    B 		loop          ;  dead loop

    ENDP



; Read RTC every 1 second, and update LCD
SysTick_Handler PROC

    ; record the amount of occurence of systick interrupt
    ADD r4, r4, #0x01
    CMP r4, #1000
    BLT isr_skip

    MOV r4, #0x00  ; reset counter

    ; Read RTC by calling rtc_read_time when the amount achieves 1000
    PUSH {lr, r12}
    BL rtc_read_time

    ; Update LCD
    PUSH {r1, r2}
    BL LCD_Display_Hours

    POP{r0, r1}
    PUSH {r1, r2}
    BL LCD_Display_Minutes

    POP {r0, r1}
    BL LCD_Display_Seconds

    POP {lr, r12}

isr_skip

    BX LR
    ENDP

; Config:
; SysTick->CTRL
; SysTick->LOAD
; SysTick->VAL
; SysTick->CTRL
; Set Priority of systick interrupt (config_NVIC_in_C)

systick_init PROC

    ; Disable SysTick
    LDR r1, =SysTick
    LDR r2, =0x0
    STR r2, [r1, #0] ;offset SysTick_CTRL = 0

    ; Set reload register. R0 holds the argument passed to the routine
    LDR r1, =SysTick
    SUB r0, r0, #1
    STR r0, [r1, #4] ;offset SysTick_LOAD = 4

    ; Reset the SysTick counter value
    LDR r1, =SysTick
    LDR r2, =0x0
    STR r2, [r1, #8] ;offset SysTick_VAL = 8

    ; Set interrupt priority of SysTick.
    PUSH {LR, R0}
    BL config_NVIC_in_C
    POP {LR, R0}

    ; Enables SysTick interrupt, 1 = Enable, 0 = Disable
    LDR r1, =SysTick
    LDR r2, [r1, #0]
    ORR r2, #0x02
    STR r2, [r1, #0] ;offset SysTick_CTRL = 0

    ; Select processor clock: 1 = processor clock; 0 = external clock
    ; Use external 1MHz Clock
    LDR r1, =SysTick
    LDR r2, [r1, #0]
    BIC r2, #0x04
    STR r2, [r1, #0] ;offset SysTick_CTRL = 0

    ; Enable SysTick
    LDR r1, =SysTick
    LDR r2, [r1, #0]
    ORR r2, #0x01
    STR r2, [r1, #0] ;offset of SysTick_CTRL = 0

    BX LR
    ENDP


; RTC->WPR = 0xCA;
; RTC->WPR = 0x53;
rtc_disable_write_protection PROC

    LDR r0, =RTC_BASE

    LDR r1, [r0, #RTC_WPR]
    MOV r1, #0x00CA
    STR r1, [r0, #RTC_WPR]

    LDR r1, [r0, #RTC_WPR]
    MOV r1, #0x0053
    STR r1, [r0, #RTC_WPR]

    BX LR
    ENDP

; RTC->WPR = 0xFF;
rtc_enable_write_protection PROC

    LDR r0, =RTC_BASE

    LDR r1, [r0, #RTC_WPR]
    MOV r1, #0xFF
    STR r1, [r0, #RTC_WPR]

    BX LR
    ENDP

; Use three parameters R0-R2 to set 24hr RTC time
; To set time to 12:34:56, r0=0x12, r1=0x34, r2=0x56
; r0 = hour
; r1 = minute
; r2 = second
rtc_set_time PROC

    AND r4, r0, #0xF0  ; hour
    LSL r4, r4, #16
    MOV r5, r4

    AND r4, r0, #0x0F
    LSL r4, r4, #16
    ORR r5, r5, r4

    AND r4, r1, #0xF0  ; minute
    LSL r4, r4, #8
    ORR r5, r5, r4

    AND r4, r1, #0x0F
    LSL r4, r4, #8
    ORR r5, r5, r4

    AND r4, r2, #0xF0  ; second
    LSL r4, r4, #0
    ORR r5, r5, r4

    AND r4, r2, #0x0F
    LSL r4, r4, #0
    ORR r5, r5, r4

    LDR r0, =RTC_BASE
    STR r5, [r0, #RTC_TR]

    ; 24 hr format
    LDR r1, [r0, #RTC_CR]
    BIC r1, #0x40
    STR r1, [r0, #RTC_CR]

    BX LR
    ENDP

; Read RTC time and return the values
; r0 = hour
; r1 = minute
; r2 = second
rtc_read_time PROC

    LDR r3, =RTC_BASE
    LDR r3, [r3, #RTC_TR]

    LSR r4, r3, #20  ; hour
    AND r4, r4, #0x0F
    MOV r6, #10
    MUL r4, r4, r6
    LSR r5, r3, #16
    AND r5, r5, #0x0F
    ADD r0, r4, r5

    LSR r4, r3, #12  ; minute
    AND r4, r4, #0x0F
    MOV r6, #10
    MUL r4, r4, r6
    LSR r5, r3, #8
    AND r5, r5, #0x0F
    ADD r1, r4, r5

    LSR r4, r3, #4
    AND r4, r4, #0x0F
    MOV r6, #10
    MUL r4, r4, r6
    AND r5, r3, #0x0F
    ADD r2, r4, r5

    BX LR
    ENDP

; initialize RTC
rtc_init PROC

    PUSH {r0, lr}
    BL RTC_Clock_Init
    BL rtc_disable_write_protection

    ;Enter Initilization mode and wait for the confirmation
    LDR r0, =RTC_BASE
    LDR r1, [r0, #RTC_ISR]
    ORR r1, r1, #0xFFFFFFFF
    STR r1, [r0, #RTC_ISR]
wait4init
    LDR r1, [r0, #RTC_ISR]
    AND r1, r1, #0x40
    CMP r1, #0x0
    BEQ wait4init

    LDR r0, =0x14  ; hour
    LDR r1, =0x34  ; minute
    LDR r2, =0x59  ; second
    BL rtc_set_time

    ;Exit Initilization mode and wait for the confirmation
    LDR r0, =RTC_BASE
    LDR r1, [r0, #RTC_ISR]
    MVN r2, #RTC_ISR_INIT
    AND r1, r1, r2
    STR r1, [r0, #RTC_ISR]
wait4exit
    LDR r1, [r0, #RTC_ISR]
    AND r1, r1, #RTC_ISR_RSF
    CMP r1, #0x00
    BEQ wait4exit


    BL rtc_enable_write_protection

    POP {r0, lr}
    BX LR
    ENDP

    ALIGN

    AREA    myData, DATA, READWRITE
    ALIGN

    END
