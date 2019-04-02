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

    INCLUDE core_cm4_constants.s		; Load Cortex-M4 Definitions
    INCLUDE stm32l476xx_constants.s     ; Load STM32L4 Definitions

    AREA    main, CODE, READONLY
    EXPORT	__main				; make __main visible to linker
    ENTRY

; ---------------------------- Start of main ----------------------------
__main	PROC

    ; Enable the clock to GPIO Port B
    LDR r0, =RCC_BASE
    LDR r1, [r0, #RCC_AHB2ENR]
    ORR r1, r1, #RCC_AHB2ENR_GPIOBEN
    STR r1, [r0, #RCC_AHB2ENR]

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



full_step_run

    LDR r1, =GPIOB_BASE
    LDR r4, =steps_full_cw
    LDR r5, =0x0800    ; 2048
    LDR r6, =0x04      ; steps_full_size
    LDR r7, =0x00      ; i
    LDR r8, =0x00      ; j

full_step_loop

    CMP r6, r8     ; j = steps_full_size ?

    BNE full_step_skip
    SUB r4, #0x10  ; ptr -= 16 (reset)
    SUB r8, #0x04  ; j   -= 4

full_step_skip

    ; 5000 clock cycles to delay
    LDR r0, =0x1388

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


half_step_run

    LDR r1, =GPIOB_BASE
    LDR r4, =steps_half_ccw
    LDR r5, =0x1000    ; 4096
    LDR r6, =0x08      ; steps_half_size
    LDR r7, =0x00      ; i
    LDR r8, =0x00      ; j

half_step_loop

    CMP r6, r8     ; j = steps_half_size ?

    BNE half_step_skip
    SUB r4, #0x20  ; ptr -= 32 (reset)
    SUB r8, #0x08  ; j   -= 8

half_step_skip

    ; 5000 clock cycles to delay
    LDR r0, =0x1388

    ; port B ODR
    LDR r3, [r4], #0x04
    LDR r2, [r1, #GPIO_ODR]
    BIC r2, r2, #0x000000CC ; clear bits 2,3,6,7
    ORR r2, r2, r3          ; steps_half_ccw[j]
    STR r2, [r1, #GPIO_ODR]

    ADD r7, #0x01            ; i++
    ADD r8, #0x01            ; j++
    BL delay

    CMP r5, r7
    BNE half_step_loop       ; i = 4096 ?

stop  B    stop              ;  dead loop & program hangs here

    ENDP

; ---------------------------- End of main ----------------------------


; call delay after each step
delay	PROC
    SUB r0, r0, #1      ; delay--
    NOP
    CMP r0, #0          ; exit when delay == 0
    BGT delay
    BX LR

    ENDP

    ALIGN

    AREA    myData, DATA, READWRITE
    ALIGN
steps_full_cw   DCD    0x84, 0x44, 0x48, 0x88
steps_full_ccw  DCD    0x88, 0x48, 0x44, 0x84
steps_half_cw   DCD    0x84, 0x04, 0x44, 0x40, 0x48, 0x08, 0x88, 0x80
steps_half_ccw  DCD    0x80, 0x88, 0x08, 0x48, 0x40, 0x44, 0x04, 0x84
    END
