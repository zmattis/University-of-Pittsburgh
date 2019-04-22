;*************************************  32L476GDISCOVERY ***********************
; STM32L4:  STM32L476VGT6 MCU = ARM Cortex-M4 + FPU + DSP,
;           LQFP100, 1 MB of Flash, 128 KB of SRAM
;           Instruction cache = 32 lines of 4x64 bits (1KB)
;           Data cache = 8 lines of 4x64 bits (256 B)
;
; Joystick (MT-008A):
;   Right = PA2        Up   = PA3         Center = PA0
;   Left  = PA1        Down = PA5
;
; User LEDs:
;   LD4 Red   = PB2    LD5 Green = PE8
;
; CS43L22 Audio DAC Stereo (I2C address 0x94):
;   SAI1_MCK = PE2     SAI1_SD  = PE6    I2C1_SDA = PB7    Audio_RST = PE3
;   SAI1_SCK = PE5     SAI1_FS  = PE4    I2C1_SCL = PB6
;
; MP34DT01 Digital MEMS microphone
;    Audio_CLK = PE9   Audio_DIN = PE7
;
; LSM303C eCompass (a 3D accelerometer and 3D magnetometer module):
;   MEMS_SCK  = PD1    MAG_DRDY = PC2    XL_CS  = PE0
;   MEMS_MOSI = PD4    MAG_CS  = PC0     XL_INT = PE1
;                      MAG_INT = PC1
;
; L3GD20 Gyro (three-axis digital output):
;   MEMS_SCK  = PD1    GYRO_CS   = PD7
;   MEMS_MOSI = PD4    GYRO_INT1 = PD2
;   MEMS_MISO = PD3    GYRO_INT2 = PB8
;
; ST-Link V2 (Virtual com port, Mass Storage, Debug port):
;   USART_TX = PD5     SWCLK = PA14      MFX_USART3_TX   MCO
;   USART_RX = PD6     SWDIO = PA13      MFX_USART3_RX   NRST
;   PB3 = 3V3_REG_ON   SWO = PB5
;
; Quad SPI Flash Memory (128 Mbit)
;   QSPI_CS  = PE11    QSPI_D0 = PE12    QSPI_D2 = PE14
;   QSPI_CLK = PE10    QSPI_D1 = PE13    QSPI_D3 = PE15
;
; LCD (24 segments, 4 commons)
;   VLCD = PC3
;   COM0 = PA8     COM1  = PA9      COM2  = PA10    COM3  = PB9
;   SEG0 = PA7     SEG6  = PD11     SEG12 = PB5     SEG18 = PD8
;   SEG1 = PC5     SEG7  = PD13     SEG13 = PC8     SEG19 = PB14
;   SEG2 = PB1     SEG8  = PD15     SEG14 = PC6     SEG20 = PB12
;   SEG3 = PB13    SEG9  = PC7      SEG15 = PD14    SEG21 = PB0
;   SEG4 = PB15    SEG10 = PA15     SEG16 = PD12    SEG22 = PC4
;   SEG5 = PD9     SEG11 = PB4      SEG17 = PD10    SEG23 = PA6
;
; USB OTG
;   OTG_FS_PowerSwitchOn = PC9    OTG_FS_VBUS = PC11    OTG_FS_DM = PA11
;   OTG_FS_OverCurrent   = PC10   OTG_FS_ID   = PC12    OTG_FS_DP = PA12
;
; PC14 = OSC32_IN      PC15 = OSC32_OUT
; PH0  = OSC_IN        PH1  = OSC_OUT
;
; PA4  = DAC1_OUT1 (NLMFX0 WAKEUP)   PA5 = DAC1_OUT2 (Joy Down)
; PA3  = OPAMP1_VOUT (Joy Up)        PB0 = OPAMP2_VOUT (LCD SEG21)
;****************************************************************************************************************


    INCLUDE core_cm4_constants.s      ; Load Constant Definitions
    INCLUDE stm32l476xx_constants.s
    EXPORT RTC_Alarm_IRQHandler

    IMPORT config_NVIC_in_C
    IMPORT System_Clock_Init
    IMPORT RTC_Clock_Init
    IMPORT RTC_Alarm_Makeup
    IMPORT BIN2BCD
    IMPORT BCD2BIN


    AREA main_code,  CODE,  READONLY
    EXPORT __main
    ALIGN
    ENTRY
__main PROC

    ; initialize the clock of center button (PA0), stepper motor driver (PB)
    BL pin_init
    BL System_Clock_Init


    ; config rtc clock
    BL rtc_init_clk


; 180 ccw stepping when joystick is pressed
loop
    ;check if pushbutton (PA.0) is pressed
    LDR r0, =GPIOA_BASE
    LDR r1, [r0, #GPIO_IDR]
    ANDS r2, r1, #0x00000001
    CMP r2, #0x00000001

    ;jump back to loop if center button is not pressed
    BNE	loop

    ;wait until center button is released
wait4release
    ;check if the center button (PA.0) is released
    ;if it is not released, we jump back to wait4release
    ;if it is released, we execute the stepping function
    LDR r1, [r0, #GPIO_IDR]
    ANDS r2, r1, #0x00000001
    CMP r2, #0x00000000

    BNE wait4release


    ; config rtc alarm
    BL RTC_Alarm_Makeup

    B loop

    ENDP

full_step_ccw_180 PROC

    PUSH {lr, r0}

full_step_run

    LDR r1, =GPIOB_BASE
    LDR r4, =steps_full_ccw
    LDR r5, =0x0400    ; 1024 == 180 deg.
    LDR r6, =0x04      ; steps_full_size
    LDR r7, =0x00      ; i
    LDR r8, =0x00      ; j

full_step_loop

    CMP r6, r8     ; j = steps_full_size ?

    BNE full_step_skip
    SUB r4, #0x10  ; ptr -= 16 (reset)
    SUB r8, #0x04  ; j   -= 4

full_step_skip

    ; 2500 clock cycles to delay
    LDR r0, =0x09C4

    ; port B ODR
    LDR r3, [r4], #0x04
    LDR r2, [r1, #GPIO_ODR]
    BIC r2, r2, #0x000000CC ; clear bits 2,3,6,7
    ORR r2, r2, r3          ; steps_full_cw[j]
    STR r2, [r1, #GPIO_ODR]

    ADD r7, #0x01            ; i++
    ADD r8, #0x01            ; j++
    BL delay

    CMP r5, r7               ; i = 2048 ?
    BNE full_step_loop


    POP {lr, r0}
    BX LR

    ENDP

; initialize the clock of center button(PA0), stepper motor driver (PB)
pin_init PROC

    ; Enable clock of GPIO Port A
    LDR r0, =RCC_BASE
    LDR r1, [r0, #RCC_AHB2ENR]
    ORR r1, r1, #RCC_AHB2ENR_GPIOAEN
    STR r1, [r0, #RCC_AHB2ENR]

    ; Enable clock of GPIO Port B
    LDR r0, =RCC_BASE
    LDR r1, [r0, #RCC_AHB2ENR]
    ORR r1, r1, #RCC_AHB2ENR_GPIOBEN
    STR r1, [r0, #RCC_AHB2ENR]


    ; Set PA.0 as push-pull Input with pull-down

    ; configure MODER register (input)
    LDR r0, =GPIOA_BASE
    LDR r1, [r0, #GPIO_MODER]
    BIC r1, r1, #0x00000003
    STR r1, [r0, #GPIO_MODER]

    ; configure OTYPER register (push-pull)
    LDR r0, =GPIOA_BASE
    LDR r1, [r0, #GPIO_OTYPER]
    BIC r1, r1, #0x00000001
    STR r1, [r0, #GPIO_OTYPER]

    ; configure PUPDR register (pull-down)
    LDR r0, =GPIOA_BASE
    LDR r1, [r0, #GPIO_PUPDR]
    BIC r1, r1, #0x00000003
    ORR r1, r1, #0x00000002
    STR r1, [r0, #GPIO_PUPDR]



    ; MODE: 00: Input mode,              01: General purpose output mode
    ;       10: Alternate function mode, 11: Analog mode (reset state)

    ; Set PB2, PB3, PB6, PB7 as output with push-pull
    LDR r0, =GPIOB_BASE
    LDR r1, [r0, #GPIO_MODER]	 ; load GPIOB->MODER
    LDR r2, =0x0000F0F0
    BIC r1, r1, r2
    LDR r2, =0x00005050
    ORR r1, r1, r2
    STR r1, [r0, #GPIO_MODER]	 ; store MODER

    ; Clear bit 2,3,6,7 of GPIOB->OTYPER
    LDR r1, [r0, #GPIO_OTYPER]  ; load GPIOB->OTYPER
    LDR r2, =0x000000CC
    BIC r1, r1, r2
    STR r1, [r0, #GPIO_OTYPER]  ; store OTYPER

    ; Clear bit set 2,3,6,7 of GPIOB->PUPDR
    LDR r1, [r0, #GPIO_PUPDR]	 ; load GPIOB->PUPD
    LDR r2, =0x0000F0F0
    BIC r1, r1, r2
    STR r1, [r0, #GPIO_PUPDR]	 ; store PUPDR


    BX lr
    ENDP


; call delay after each step
delay	PROC
    SUB r0, r0, #1      ; delay--
    NOP
    CMP r0, #0          ; exit when delay == 0
    BGT delay
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


; initialize RTC
rtc_init_clk PROC

    PUSH {r0, lr}
    BL config_NVIC_in_C

    ; Rising trigger selection
    ; 0 = trigger disabled, 1 = trigger enabled
    LDR r0, =EXTI_BASE
    LDR r1, [r0, #EXTI_RTSR1]
    ORR r1, #EXTI_RTSR1_RT18
    STR r1, [r0, #EXTI_RTSR1]

    ;Interrupt Mask Register
    ; 0 = marked, 1 = not masked (enabled)
    LDR r0, =EXTI_BASE
    LDR r1, [r0, #EXTI_IMR1]
    ORR r1, # EXTI_IMR1_IM18
    STR r1, [r0, # EXTI_IMR1]

    BL RTC_Clock_Init
    BL rtc_disable_write_protection

    ;Enter Initilization mode and wait for the confirmation
    LDR r0, =RTC_BASE
    LDR r1, [r0, #RTC_ISR]
    ORR r1, r1, #0xFFFFFFFF
    STR r1, [r0, #RTC_ISR]
wait4init_clk
    LDR r1, [r0, #RTC_ISR]
    AND r1, r1, #0x40
    CMP r1, #0x0
    BEQ wait4init_clk

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


; initialize RTC alarm
rtc_init_alrm PROC

    PUSH {r0, lr}
    BL RTC_Clock_Init

    ; disable write protection
    BL rtc_disable_write_protection

    ; disable alarm a
    LDR r0, =RTC_BASE
    LDR r1, [r0, #RTC_CR]
    BIC r1, r1, #0x1000  ; clear ALRAE bit (b_12)
    STR r1, [r0, #RTC_CR]

  ; poll RTC_ISR[ALRAWF]
wait4init_alrm
    LDR r1, [r0, #RTC_ISR]
    AND r1, r1, #0x01
    CMP r1, #0x0
    BEQ wait4init_alrm

    ; config alarm
    BL rtc_config_alrm


    ; enable alarm a
    LDR r0, =RTC_BASE
    LDR r1, [r0, #RTC_CR]
    ORR r1, r1, #0x1000  ; set ALRAE bit (b_12)
    STR r1, [r0, #RTC_CR]

    ; enable write protection
    BL rtc_enable_write_protection

    POP {r0, lr}
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


; Use RTC_TR register to set Alarm A
rtc_config_alrm PROC

    LDR r0, =RTC_BASE
    LDR r1, [r0, #RTC_TR]

    AND r1, r1, #0xFF        ; get seconds from tr register
    BIC r1, r1, #0xFF
    ORR r1, r1, #0x10

    LDR r2, =0x80808000
    ORR r1, r1, r2     ; don't care about days, hours, minutes
    BIC r1, r1, #0x80  ; seconds mask


    LDR r0, =RTC_BASE
    STR r1, [r0, #RTC_ALRMAR]

    BX LR
    ENDP


RTC_Alarm_IRQHandler PROC


    ; stepper motor
    BL full_step_ccw_180

    LDR r0, =RTC_BASE
    LDR r1, [r0, #RTC_ISR]
    AND r2, r1, #RTC_ISR_ALRAF
    CMP r2, #0x00
    BEQ irq_exit
    ; clear ALRAF bit in RTC->ISR
    MVN r2, #RTC_ISR
    AND r1, r1, r2
    STR r1, [r0, #RTC_ISR]
    ; clear PIF18 in EXTI->PR1
    LDR r0, =EXTI_BASE
    LDR r1, [r0, #EXTI_PR1]
    ORR r1, r1, #EXTI_PR1_PIF18
    STR r1, [r0, #EXTI_PR1]
irq_exit

    BX LR
    ENDP



    ALIGN

    AREA  variables,  DATA,  READWRITE
    ALIGN
steps_full_cw   DCD    0x84, 0x44, 0x48, 0x88
steps_full_ccw  DCD    0x88, 0x48, 0x44, 0x84
steps_half_cw   DCD    0x84, 0x04, 0x44, 0x40, 0x48, 0x08, 0x88, 0x80
steps_half_ccw  DCD    0x80, 0x88, 0x08, 0x48, 0x40, 0x44, 0x04, 0x84

    END