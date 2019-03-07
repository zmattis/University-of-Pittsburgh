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


  INCLUDE core_cm4_constants.s		; Load Constant Definitions
  INCLUDE stm32l476xx_constants.s

  AREA main_code,  CODE,  READONLY
  ;EXPORT	__main
  IMPORT Delay1Second
  ALIGN
  ENTRY
__main PROC

  ; initialize the clock of center button(PA0), Red LED(PB2), Green LED(PE8)
  BL pin_init

;--------------------start of loop for task 2: uncomment the following codes for task 2------------------
;task 2: toggle Red LED when center button of joystick is pressed.
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
  ;if it is released, we execute the following codes to toggle the green LED
  LDR r1, [r0, #GPIO_IDR]
  ANDS r2, r1, #0x00000001
  CMP r2, #0x00000000

  BNE wait4release

  ;toggle red LED
  BL Toggle_RED_LED

  B loop
;--------------------end of loop for task 2------------------

  ENDP

Toggle_RED_LED PROC

  ;toggle Red LED(PB2)
  LDR r0, =GPIOB_BASE
  LDR r1, [r0, #GPIO_ODR]
  EOR r1, r1, #0x00000004 ; Toggle bit 2
  STR r1, [r0, #GPIO_ODR]

  BX LR

  ENDP

; Initialize the clock of center button(PA0), Red LED(PB2), Green LED(PE8)
pin_init PROC

  ; Enable clock of GPIO Port A
  LDR	r0, =RCC_BASE
  LDR	r1, [r0, #RCC_AHB2ENR]
  ORR	r1, r1, #RCC_AHB2ENR_GPIOAEN
  STR	r1, [r0, #RCC_AHB2ENR]

  ; Enable clock of GPIO Port B
  LDR	r0, =RCC_BASE
  LDR	r1, [r0, #RCC_AHB2ENR]
  ORR	r1, r1, #RCC_AHB2ENR_GPIOBEN
  STR	r1, [r0, #RCC_AHB2ENR]

  ; Enable clock of GPIO Port E
  LDR	r0, =RCC_BASE
  LDR	r1, [r0, #RCC_AHB2ENR]
  ORR	r1, r1, #RCC_AHB2ENR_GPIOEEN
  STR	r1, [r0, #RCC_AHB2ENR]

  ; Set PB.2 as push-pull Output without pull-up/pull-down

  ; configure MODER register (output)
  LDR r0, =GPIOB_BASE
  LDR r1, [r0, #GPIO_MODER]
  BIC r1, r1, #0x00000030
  ORR r1, r1, #0x00000010
  STR r1, [r0, #GPIO_MODER]

  ; configure OTYPER register (push-pull)
  LDR r0, =GPIOB_BASE
  LDR r1, [r0, #GPIO_OTYPER]
  BIC r1, r1, #0x00000004
  STR r1, [r0, #GPIO_OTYPER]

  ; configure PUPDR register (no pull-up/pull-down)
  LDR r0, =GPIOB_BASE
  LDR r1, [r0, #GPIO_PUPDR]
  BIC r1, r1, #0x00000030
  STR r1, [r0, #GPIO_PUPDR]

  ; Set PE.8 as push-pull Output without pull-up/pull-down

  ; configure MODER register (output)
  LDR r0, =GPIOE_BASE
  LDR r1, [r0, #GPIO_MODER]
  BIC r1, r1, #0x00030000
  ORR r1, r1, #0x00010000
  STR r1, [r0, #GPIO_MODER]

  ; configure OTYPER register (push-pull)
  LDR r0, =GPIOE_BASE
  LDR r1, [r0, #GPIO_OTYPER]
  BIC r1, r1, #0x00000100
  STR r1, [r0, #GPIO_OTYPER]

  ; configure PUPDR register (no pull-up/pull-down)
  LDR r0, =GPIOE_BASE
  LDR r1, [r0, #GPIO_PUPDR]
  BIC r1, r1, #0x00030000
  STR r1, [r0, #GPIO_PUPDR]

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


  BX lr
  ENDP

  ALIGN

  AREA	variables, 	DATA, READWRITE
  ALIGN

  END
