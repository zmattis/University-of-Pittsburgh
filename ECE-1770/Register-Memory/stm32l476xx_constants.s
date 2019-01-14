;******************** (C) Yifeng ZHU *******************************************
; @file    stm321476xx_constants.s
; @author  Yifeng Zhu
; @date    May-17-2015
; @note
;           This code is for the book "Embedded Systems with ARM Cortex-M 
;           Microcontrollers in Assembly Language and C, Yifeng Zhu, 
;           ISBN-13: 978-0982692639, ISBN-10: 0982692633
; @attension
;           This code is provided for education purpose. The author shall not be 
;           held liable for any direct, indirect or consequential damages, for any 
;           reason whatever. More information can be found from book website: 
;           http:;www.eece.maine.edu/~zhu/book
;*******************************************************************************


; This following is added to remove the compiler warning.
    AREA    __DEFINES_STM32L4_xx_DUMMY, CODE, READONLY
				
; Configuration of the Cortex-M4 Processor and Core Peripherals

__CM4_REV           EQU 0x0001  ; Cortex-M4 revision r0p1
__MPU_PRESENT       EQU 1       ; STM32L4XX provides an MPU
__NVIC_PRIO_BITS    EQU 4       ; STM32L4XX uses 4 Bits for the Priority Levels
__Vendor_SysTickConfig      EQU 0       ; Set to 1 if different SysTick Config is used
__FPU_PRESENT       EQU 1       ; FPU present

;  STM32L4XX Interrupt Number Definition, according to the selected device

; ******  Cortex-M4 Processor Exceptions Numbers ***************************************************************
NonMaskableInt_IRQn EQU  -14    ; 2 Non Maskable Interrupt
HardFault_IRQn      EQU  -13    ; 4 Cortex-M4 Memory Management Interrupt
MemoryManagement_IRQn       EQU  -12    ; 4 Cortex-M4 Memory Management Interrupt
BusFault_IRQn       EQU  -11    ; 5 Cortex-M4 Bus Fault Interrupt
UsageFault_IRQn     EQU  -10    ; 6 Cortex-M4 Usage Fault Interrupt
SVCall_IRQn         EQU  -5     ; 11 Cortex-M4 SV Call Interrupt
DebugMonitor_IRQn   EQU  -4     ; 12 Cortex-M4 Debug Monitor Interrupt
PendSV_IRQn         EQU  -2     ; 14 Cortex-M4 Pend SV Interrupt
SysTick_IRQn        EQU  -1     ; 15 Cortex-M4 System Tick Interrupt
; ******  STM32 specific Interrupt Numbers *********************************************************************
WWDG_IRQn           EQU  0      ; Window WatchDog Interrupt
PVD_PVM_IRQn        EQU  1      ; PVD/PVM1/PVM2/PVM3/PVM4 through EXTI Line detection Interrupts
TAMP_STAMP_IRQn     EQU  2      ; Tamper and TimeStamp interrupts through the EXTI line
RTC_WKUP_IRQn       EQU  3      ; RTC Wakeup interrupt through the EXTI line
FLASH_IRQn          EQU  4      ; FLASH global Interrupt
RCC_IRQn            EQU  5      ; RCC global Interrupt
EXTI0_IRQn          EQU  6      ; EXTI Line0 Interrupt
EXTI1_IRQn          EQU  7      ; EXTI Line1 Interrupt
EXTI2_IRQn          EQU  8      ; EXTI Line2 Interrupt
EXTI3_IRQn          EQU  9      ; EXTI Line3 Interrupt
EXTI4_IRQn          EQU  10     ; EXTI Line4 Interrupt
DMA1_Channel1_IRQn  EQU  11     ; DMA1 Channel 1 global Interrupt
DMA1_Channel2_IRQn  EQU  12     ; DMA1 Channel 2 global Interrupt
DMA1_Channel3_IRQn  EQU  13     ; DMA1 Channel 3 global Interrupt
DMA1_Channel4_IRQn  EQU  14     ; DMA1 Channel 4 global Interrupt
DMA1_Channel5_IRQn  EQU  15     ; DMA1 Channel 5 global Interrupt
DMA1_Channel6_IRQn  EQU  16     ; DMA1 Channel 6 global Interrupt
DMA1_Channel7_IRQn  EQU  17     ; DMA1 Channel 7 global Interrupt
ADC1_2_IRQn         EQU  18     ; ADC1, ADC2 SAR global Interrupts
CAN1_TX_IRQn        EQU  19     ; CAN1 TX Interrupt
CAN1_RX0_IRQn       EQU  20     ; CAN1 RX0 Interrupt
CAN1_RX1_IRQn       EQU  21     ; CAN1 RX1 Interrupt
CAN1_SCE_IRQn       EQU  22     ; CAN1 SCE Interrupt
EXTI9_5_IRQn        EQU  23     ; External Line[9:5] Interrupts
TIM1_BRK_TIM15_IRQn EQU  24     ; TIM1 Break interrupt and TIM15 global interrupt
TIM1_UP_TIM16_IRQn  EQU  25     ; TIM1 Update Interrupt and TIM16 global interrupt
TIM1_TRG_COM_TIM17_IRQn     EQU  26     ; TIM1 Trigger and Commutation Interrupt and TIM17 global interrupt
TIM1_CC_IRQn        EQU  27     ; TIM1 Capture Compare Interrupt
TIM2_IRQn           EQU  28     ; TIM2 global Interrupt
TIM3_IRQn           EQU  29     ; TIM3 global Interrupt
TIM4_IRQn           EQU  30     ; TIM4 global Interrupt
I2C1_EV_IRQn        EQU  31     ; I2C1 Event Interrupt
I2C1_ER_IRQn        EQU  32     ; I2C1 Error Interrupt
I2C2_EV_IRQn        EQU  33     ; I2C2 Event Interrupt
I2C2_ER_IRQn        EQU  34     ; I2C2 Error Interrupt
SPI1_IRQn           EQU  35     ; SPI1 global Interrupt
SPI2_IRQn           EQU  36     ; SPI2 global Interrupt
USART1_IRQn         EQU  37     ; USART1 global Interrupt
USART2_IRQn         EQU  38     ; USART2 global Interrupt
USART3_IRQn         EQU  39     ; USART3 global Interrupt
EXTI15_10_IRQn      EQU  40     ; External Line[15:10] Interrupts
RTC_Alarm_IRQn      EQU  41     ; RTC Alarm (A and B) through EXTI Line Interrupt
DFSDM3_IRQn         EQU  42     ; SD Filter 3 global Interrupt
TIM8_BRK_IRQn       EQU  43     ; TIM8 Break Interrupt
TIM8_UP_IRQn        EQU  44     ; TIM8 Update Interrupt
TIM8_TRG_COM_IRQn   EQU  45     ; TIM8 Trigger and Commutation Interrupt
TIM8_CC_IRQn        EQU  46     ; TIM8 Capture Compare Interrupt
ADC3_IRQn           EQU  47     ; ADC3 global  Interrupt
FMC_IRQn            EQU  48     ; FMC global Interrupt
SDMMC1_IRQn         EQU  49     ; SDMMC1 global Interrupt
TIM5_IRQn           EQU  50     ; TIM5 global Interrupt
SPI3_IRQn           EQU  51     ; SPI3 global Interrupt
UART4_IRQn          EQU  52     ; UART4 global Interrupt
UART5_IRQn          EQU  53     ; UART5 global Interrupt
TIM6_DAC_IRQn       EQU  54     ; TIM6 global and DAC1&2 underrun error  interrupts
TIM7_IRQn           EQU  55     ; TIM7 global interrupt
DMA2_Channel1_IRQn  EQU  56     ; DMA2 Channel 1 global Interrupt
DMA2_Channel2_IRQn  EQU  57     ; DMA2 Channel 2 global Interrupt
DMA2_Channel3_IRQn  EQU  58     ; DMA2 Channel 3 global Interrupt
DMA2_Channel4_IRQn  EQU  59     ; DMA2 Channel 4 global Interrupt
DMA2_Channel5_IRQn  EQU  60     ; DMA2 Channel 5 global Interrupt
DFSDM0_IRQn         EQU  61     ; SD Filter 0 global Interrupt
DFSDM1_IRQn         EQU  62     ; SD Filter 1 global Interrupt
DFSDM2_IRQn         EQU  63     ; SD Filter 2 global Interrupt
COMP_IRQn           EQU  64     ; COMP1 and COMP2 Interrupts
LPTIM1_IRQn         EQU  65     ; LP TIM1 interrupt
LPTIM2_IRQn         EQU  66     ; LP TIM2 interrupt
OTG_FS_IRQn         EQU  67     ; USB OTG FS global Interrupt
DMA2_Channel6_IRQn  EQU  68     ; DMA2 Channel 6 global interrupt
DMA2_Channel7_IRQn  EQU  69     ; DMA2 Channel 7 global interrupt
LPUART1_IRQn        EQU  70     ; LP UART1 interrupt
QUADSPI_IRQn        EQU  71     ; Quad SPI global interrupt
I2C3_EV_IRQn        EQU  72     ; I2C3 event interrupt
I2C3_ER_IRQn        EQU  73     ; I2C3 error interrupt
SAI1_IRQn           EQU  74     ; Serial Audio Interface 1 global interrupt
SAI2_IRQn           EQU  75     ; Serial Audio Interface 2 global interrupt
SWPMI1_IRQn         EQU  76     ; Serial Wire Interface 1 global interrupt
TSC_IRQn            EQU  77     ; Touch Sense Controller global interrupt
LCD_IRQn            EQU  78     ; LCD global interrupt
RNG_IRQn            EQU  80     ; RNG global interrupt
FPU_IRQn            EQU  81     ; FPU global interrupt

; Analog to Digital Converter
ADC_ISR  EQU  0x00   ; ADC Interrupt and Status Register,                 Address offset: 0x00
ADC_IER  EQU  0x04   ; ADC Interrupt Enable Register,                     Address offset: 0x04
ADC_CR   EQU  0x08   ; ADC control register,                              Address offset: 0x08
ADC_CFGR EQU  0x0C   ; ADC Configuration register,                        Address offset: 0x0C
ADC_CFGR2        EQU  0x10   ; ADC Configuration register 2,                      Address offset: 0x10
ADC_SMPR1        EQU  0x14   ; ADC sample time register 1,                        Address offset: 0x14
ADC_SMPR2        EQU  0x18   ; ADC sample time register 2,                        Address offset: 0x18
ADC_RESERVED1    EQU  0x1C   ; Reserved, 0x01C
ADC_TR1  EQU  0x20   ; ADC watchdog threshold register 1,                 Address offset: 0x20
ADC_TR2  EQU  0x24   ; ADC watchdog threshold register 2,                 Address offset: 0x24
ADC_TR3  EQU  0x28   ; ADC watchdog threshold register 3,                 Address offset: 0x28
ADC_RESERVED2    EQU  0x2C   ; Reserved, 0x02C
ADC_SQR1 EQU  0x30   ; ADC regular sequence register 1,                   Address offset: 0x30
ADC_SQR2 EQU  0x34   ; ADC regular sequence register 2,                   Address offset: 0x34
ADC_SQR3 EQU  0x38   ; ADC regular sequence register 3,                   Address offset: 0x38
ADC_SQR4 EQU  0x3C   ; ADC regular sequence register 4,                   Address offset: 0x3C
ADC_DR   EQU  0x40   ; ADC regular data register,                         Address offset: 0x40
ADC_RESERVED3    EQU  0x44   ; Reserved, 0x044
ADC_RESERVED4    EQU  0x48   ; Reserved, 0x048
ADC_JSQR EQU  0x4C   ; ADC injected sequence register,                    Address offset: 0x4C
ADC_RESERVED5    EQU  0x50   ; Reserved, 0x050 - 0x05C
ADC_OFR1 EQU  0x60   ; ADC offset register 1,                             Address offset: 0x60
ADC_OFR2 EQU  0x64   ; ADC offset register 2,                             Address offset: 0x64
ADC_OFR3 EQU  0x68   ; ADC offset register 3,                             Address offset: 0x68
ADC_OFR4 EQU  0x6C   ; ADC offset register 4,                             Address offset: 0x6C
ADC_RESERVED6    EQU  0x70   ; Reserved, 0x070 - 0x07C
ADC_JDR1 EQU  0x80   ; ADC injected data register 1,                      Address offset: 0x80
ADC_JDR2 EQU  0x84   ; ADC injected data register 2,                      Address offset: 0x84
ADC_JDR3 EQU  0x88   ; ADC injected data register 3,                      Address offset: 0x88
ADC_JDR4 EQU  0x8C   ; ADC injected data register 4,                      Address offset: 0x8C
ADC_RESERVED7    EQU  0x90   ; Reserved, 0x090 - 0x09C
ADC_AWD2CR       EQU  0xA0   ; ADC  Analog Watchdog 2 Configuration Register,     Address offset: 0xA0
ADC_AWD3CR       EQU  0xA4   ; ADC  Analog Watchdog 3 Configuration Register,     Address offset: 0xA4
ADC_RESERVED8    EQU  0xA8   ; Reserved, 0x0A8
ADC_RESERVED9    EQU  0xAC   ; Reserved, 0x0AC
ADC_DIFSEL       EQU  0xB0   ; ADC  Differential Mode Selection Register,         Address offset: 0xB0
ADC_CALFACT      EQU  0xB4   ; ADC  Calibration Factors,                          Address offset: 0xB4

ADC_CSR  EQU  0x300  ; ADC Common status register,                  Address offset: ADC1 base address + 0x300
ADC_RESERVED     EQU  0x304  ; Reserved, ADC1 base address + 0x304
ADC_CCR  EQU  0x308  ; ADC common control register,                 Address offset: ADC1 base address + 0x308
ADC_CDR  EQU  0x30C  ; ADC common regular data register for dual    Address offset: ADC1 base address + 0x30C


; Controller Area Network TxMailBox
CAN_TxMailBox_TIR     EQU 0x00  ; CAN TX mailbox identifier register
CAN_TxMailBox_TDTR    EQU 0x04  ; CAN mailbox data length control and time stamp register
CAN_TxMailBox_TDLR    EQU 0x08  ; CAN mailbox data low register
CAN_TxMailBox_TDHR    EQU 0x0C  ; CAN mailbox data high register

; Controller Area Network FIFOMailBox
CAN_FIFOMailBox_RIR   EQU 0x00  ; CAN receive FIFO mailbox identifier register
CAN_FIFOMailBox_RDTR  EQU 0x04  ; CAN receive FIFO mailbox data length control and time stamp register
CAN_FIFOMailBox_RDLR  EQU 0x08  ; CAN receive FIFO mailbox data low register
CAN_FIFOMailBox_RDHR  EQU 0x0C  ; CAN receive FIFO mailbox data high register


; Controller Area Network FilterRegister
CAN_FilterRegister_FR1 EQU 0x00 ; CAN Filter bank register 1
CAN_FilterRegister_FR2 EQU 0x04 ; CAN Filter bank register 1

; Controller Area Network
CAN_MCR    EQU  0x00     ; CAN master control register,         Address offset: 0x00
CAN_MSR    EQU  0x04     ; CAN master status register,          Address offset: 0x04
CAN_TSR    EQU  0x08     ; CAN transmit status register,        Address offset: 0x08
CAN_RF0R   EQU  0x0C     ; CAN receive FIFO 0 register,         Address offset: 0x0C
CAN_RF1R   EQU  0x10     ; CAN receive FIFO 1 register,         Address offset: 0x10
CAN_IER    EQU  0x14     ; CAN interrupt enable register,       Address offset: 0x14
CAN_ESR    EQU  0x18     ; CAN error status register,           Address offset: 0x18
CAN_BTR    EQU  0x1C     ; CAN bit timing register,             Address offset: 0x1C

CAN_RESERVED0      EQU  0x20     ; Reserved, 0x020 - 0x17F

CAN_TxMailBox0     EQU  0x180    ; CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC
CAN_TxMailBox1     EQU  0x190    ; CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC
CAN_TxMailBox2     EQU  0x1A0    ; CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC

CAN_FIFOMailBox0   EQU  0x1B0    ; CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC
CAN_FIFOMailBox1   EQU  0x1C0    ; CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC

CAN_RESERVED1      EQU  0x1D0    ; Reserved, 0x1D0 - 0x1FF

CAN_FMR    EQU  0x200    ; CAN filter master register,          Address offset: 0x200
CAN_FM1R   EQU  0x204    ; CAN filter mode register,            Address offset: 0x204
CAN_RESERVED2      EQU  0x208    ; Reserved, 0x208
CAN_FS1R   EQU  0x20C    ; CAN filter scale register,           Address offset: 0x20C
CAN_RESERVED3      EQU  0x210    ; Reserved, 0x210
CAN_FFA1R  EQU  0x214    ; CAN filter FIFO assignment register, Address offset: 0x214
CAN_RESERVED4      EQU  0x218    ; Reserved, 0x218
CAN_FA1R   EQU  0x21C    ; CAN filter activation register,      Address offset: 0x21C
CAN_RESERVED5      EQU  0x220    ; Reserved, 0x220-0x23F
CAN_FilterRegister EQU  0x240    ; CAN Filter Register,                 Address offset: 0x240-0x31C



; Comparator
COMP_CSR       EQU  0x00 ; COMP comparator control and status register, Address offset: 0x00

; CRC calculation unit
CRC_DR EQU  0x00      ; CRC Data register,                           Address offset: 0x00
CRC_IDR        EQU  0x04      ; CRC Independent data register,               Address offset: 0x04
CRC_RESERVED0  EQU  0x05      ; Reserved,                                                    0x05
CRC_RESERVED1  EQU  0x06      ; Reserved,                                                    0x06
CRC_CR EQU  0x08      ; CRC Control register,                        Address offset: 0x08
CRC_RESERVED2  EQU  0x0C      ; Reserved,                                                    0x0C
CRC_INIT       EQU  0x10      ; Initial CRC value register,                  Address offset: 0x10
CRC_POL        EQU  0x14      ; CRC polynomial register,                     Address offset: 0x14


; Digital to Analog Converter
DAC_CR EQU  0x00  ; DAC control register,                                    Address offset: 0x00
DAC_SWTRIGR    EQU  0x04  ; DAC software trigger register,                           Address offset: 0x04
DAC_DHR12R1    EQU  0x08  ; DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08
DAC_DHR12L1    EQU  0x0C  ; DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C
DAC_DHR8R1     EQU  0x10  ; DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10
DAC_DHR12R2    EQU  0x14  ; DAC channel2 12-bit right aligned data holding register, Address offset: 0x14
DAC_DHR12L2    EQU  0x18  ; DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18
DAC_DHR8R2     EQU  0x1C  ; DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C
DAC_DHR12RD    EQU  0x20  ; Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20
DAC_DHR12LD    EQU  0x24  ; DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24
DAC_DHR8RD     EQU  0x28  ; DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28
DAC_DOR1       EQU  0x2C  ; DAC channel1 data output register,                       Address offset: 0x2C
DAC_DOR2       EQU  0x30  ; DAC channel2 data output register,                       Address offset: 0x30
DAC_SR EQU  0x34  ; DAC status register,                                     Address offset: 0x34
DAC_CCR        EQU  0x38  ; DAC calibration control register,                        Address offset: 0x38
DAC_MCR        EQU  0x3C  ; DAC mode control register,                               Address offset: 0x3C
DAC_SHSR1      EQU  0x40  ; DAC Sample and Hold sample time register 1,              Address offset: 0x40
DAC_SHSR2      EQU  0x44  ; DAC Sample and Hold sample time register 2,              Address offset: 0x44
DAC_SHHR       EQU  0x48  ; DAC Sample and Hold hold time register,                  Address offset: 0x48
DAC_SHRR       EQU  0x4C  ; DAC Sample and Hold refresh time register,               Address offset: 0x4C

; DFSDM module registers
DFSDM_Filter_CR1     EQU  0x100   ; DFSDM control register1,                          Address offset: 0x100
DFSDM_Filter_CR2     EQU  0x104   ; DFSDM control register2,                          Address offset: 0x104
DFSDM_Filter_ISR     EQU  0x108   ; DFSDM interrupt and status register,              Address offset: 0x108
DFSDM_Filter_ICR     EQU  0x10C   ; DFSDM interrupt flag clear register,              Address offset: 0x10C
DFSDM_Filter_JCHGR   EQU  0x110   ; DFSDM injected channel group selection register,  Address offset: 0x110
DFSDM_Filter_FCR     EQU  0x114   ; DFSDM filter control register,                    Address offset: 0x114
DFSDM_Filter_JDATAR  EQU  0x118   ; DFSDM data register for injected group,           Address offset: 0x118
DFSDM_Filter_RDATAR  EQU  0x11C   ; DFSDM data register for regular group,            Address offset: 0x11C
DFSDM_Filter_AWHTR   EQU  0x120   ; DFSDM analog watchdog high threshold register,    Address offset: 0x120
DFSDM_Filter_AWLTR   EQU  0x124   ; DFSDM analog watchdog low threshold register,     Address offset: 0x124
DFSDM_Filter_AWSR    EQU  0x128   ; DFSDM analog watchdog status register             Address offset: 0x128
DFSDM_Filter_AWCFR   EQU  0x12C   ; DFSDM analog watchdog clear flag register         Address offset: 0x12C
DFSDM_Filter_EXMAX   EQU  0x130   ; DFSDM extreme detector maximum register,          Address offset: 0x130
DFSDM_Filter_EXMIN   EQU  0x134   ; DFSDM extreme detector minimum register           Address offset: 0x134
DFSDM_Filter_CNVTIMR EQU  0x138   ; DFSDM conversion timer,                           Address offset: 0x138

; DFSDM channel configuration registers
DFSDM_Channel_CHCFGR1    EQU  0x00  ; DFSDM channel configuration register1,            Address offset: 0x00
DFSDM_Channel_CHCFGR2    EQU  0x04  ; DFSDM channel configuration register2,            Address offset: 0x04
DFSDM_Channel_AWSCDR     EQU  0x08   ; DFSDM channel analog watchdog and short circuit detector register,                  Address offset: 0x08
DFSDM_Channel_CHWDATAR   EQU  0x0C ; DFSDM channel watchdog filter data register,      Address offset: 0x0C
DFSDM_Channel_CHDATINR   EQU  0x10 ; DFSDM channel data input register,                Address offset: 0x10

; Debug MCU
DBGMCU_IDCODE    EQU  0x00   ; MCU device ID code,                 Address offset: 0x00
DBGMCU_CR        EQU  0x04   ; Debug MCU configuration register,   Address offset: 0x04
DBGMCU_APB1FZR1  EQU  0x08   ; Debug MCU APB1 freeze register 1,   Address offset: 0x08
DBGMCU_APB1FZR2  EQU  0x0C   ; Debug MCU APB1 freeze register 2,   Address offset: 0x0C
DBGMCU_APB2FZ    EQU  0x10   ; Debug MCU APB2 freeze register,     Address offset: 0x10


; DMA Controller
DMA_Channel_CCR    EQU  0x00  ; DMA channel x configuration register
DMA_Channel_CNDTR  EQU  0x04  ; DMA channel x number of data register
DMA_Channel_CPAR   EQU  0x08  ; DMA channel x peripheral address register
DMA_Channel_CMAR   EQU  0x0C  ; DMA channel x memory address register

DMA_ISR    EQU  0x00  ; DMA interrupt status register,                 Address offset: 0x00
DMA_IFCR   EQU  0x04  ; DMA interrupt flag clear register,             Address offset: 0x04

DMA_Request_CSELR  EQU  0x00  ; DMA option register,                           Address offset: 0x00

; External Interrupt/Event Controller
EXTI_IMR1        EQU  0x00     ; EXTI Interrupt mask register 1,             Address offset: 0x00
EXTI_EMR1        EQU  0x04     ; EXTI Event mask register 1,                 Address offset: 0x04
EXTI_RTSR1       EQU  0x08     ; EXTI Rising trigger selection register 1,   Address offset: 0x08
EXTI_FTSR1       EQU  0x0C     ; EXTI Falling trigger selection register 1,  Address offset: 0x0C
EXTI_SWIER1      EQU  0x10     ; EXTI Software interrupt event register 1,   Address offset: 0x10
EXTI_PR1 EQU  0x14     ; EXTI Pending register 1,                    Address offset: 0x14
EXTI_RESERVED1   EQU  0x18     ; Reserved, 0x18
EXTI_RESERVED2   EQU  0x1C     ; Reserved, 0x1C
EXTI_IMR2        EQU  0x20     ; EXTI Interrupt mask register 2,             Address offset: 0x20
EXTI_EMR2        EQU  0x24     ; EXTI Event mask register 2,                 Address offset: 0x24
EXTI_RTSR2       EQU  0x28     ; EXTI Rising trigger selection register 2,   Address offset: 0x28
EXTI_FTSR2       EQU  0x2C     ; EXTI Falling trigger selection register 2,  Address offset: 0x2C
EXTI_SWIER2      EQU  0x30     ; EXTI Software interrupt event register 2,   Address offset: 0x30
EXTI_PR2 EQU  0x34     ; EXTI Pending register 2,                    Address offset: 0x34

; Firewall
FIREWALL_CSSA       EQU  0x00  ; Code Segment Start Address register,               Address offset: 0x00
FIREWALL_CSL        EQU  0x04  ; Code Segment Length register,                      Address offset: 0x04
FIREWALL_NVDSSA     EQU  0x08  ; NON volatile data Segment Start Address register,  Address offset: 0x08
FIREWALL_NVDSL      EQU  0x0C  ; NON volatile data Segment Length register,         Address offset: 0x0C
FIREWALL_VDSSA      EQU  0x10  ; Volatile data Segment Start Address register,      Address offset: 0x10
FIREWALL_VDSL       EQU  0x14  ; Volatile data Segment Length register,             Address offset: 0x14
FIREWALL_RESERVED1  EQU  0x18  ; Reserved1,                                         Address offset: 0x18
FIREWALL_RESERVED2  EQU  0x1C  ; Reserved2,                                         Address offset: 0x1C
FIREWALL_CR EQU  0x20  ; Configuration  register,                           Address offset: 0x20

; FLASH Registers
FLASH_ACR       EQU  0x00      ; FLASH access control register,            Address offset: 0x00
FLASH_PDKEYR    EQU  0x04      ; FLASH power down key register,            Address offset: 0x04
FLASH_KEYR      EQU  0x08      ; FLASH key register,                       Address offset: 0x08
FLASH_OPTKEYR   EQU  0x0C      ; FLASH option key register,                Address offset: 0x0C
FLASH_SR        EQU  0x10      ; FLASH status register,                    Address offset: 0x10
FLASH_CR        EQU  0x14      ; FLASH control register,                   Address offset: 0x14
FLASH_ECCR      EQU  0x18      ; FLASH ECC register,                       Address offset: 0x18
FLASH_RESERVED1 EQU  0x1C      ; Reserved1,                                Address offset: 0x1C
FLASH_OPTR      EQU  0x20      ; FLASH option register,                    Address offset: 0x20
FLASH_PCROP1SR  EQU  0x24      ; FLASH bank1 PCROP start address register, Address offset: 0x24
FLASH_PCROP1ER  EQU  0x28      ; FLASH bank1 PCROP end address register,   Address offset: 0x28
FLASH_WRP1AR    EQU  0x2C      ; FLASH bank1 WRP area A address register,  Address offset: 0x2C
FLASH_WRP1BR    EQU  0x30      ; FLASH bank1 WRP area B address register,  Address offset: 0x30
FLASH_RESERVED2 EQU  0x34      ; Reserved2,                                Address offset: 0x34
FLASH_PCROP2SR  EQU  0x44      ; FLASH bank2 PCROP start address register, Address offset: 0x44
FLASH_PCROP2ER  EQU  0x48      ; FLASH bank2 PCROP end address register,   Address offset: 0x48
FLASH_WRP2AR    EQU  0x4C      ; FLASH bank2 WRP area A address register,  Address offset: 0x4C
FLASH_WRP2BR    EQU  0x50      ; FLASH bank2 WRP area B address register,  Address offset: 0x50

; Flexible Memory Controller
FMC_Bank1_BTCR0     EQU  0x00  ; NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C
FMC_Bank1_BTCR1     EQU  0x04  ; NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C
FMC_Bank1_BTCR2     EQU  0x08  ; NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C
FMC_Bank1_BTCR3     EQU  0x0C  ; NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C
FMC_Bank1_BTCR4     EQU  0x10  ; NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C
FMC_Bank1_BTCR5     EQU  0x14  ; NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C
FMC_Bank1_BTCR6     EQU  0x18  ; NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C
FMC_Bank1_BTCR7     EQU  0x1C  ; NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C

; Flexible Memory Controller Bank1E
FMC_Bank1E_BWTR0    EQU 0x104  ; NOR/PSRAM write timing registers, Address offset: 0x104-0x11C
FMC_Bank1E_BWTR1    EQU 0x108  ; NOR/PSRAM write timing registers, Address offset: 0x104-0x11C
FMC_Bank1E_BWTR2    EQU 0x10C  ; NOR/PSRAM write timing registers, Address offset: 0x104-0x11C
FMC_Bank1E_BWTR3    EQU 0x110  ; NOR/PSRAM write timing registers, Address offset: 0x104-0x11C
FMC_Bank1E_BWTR4    EQU 0x114  ; NOR/PSRAM write timing registers, Address offset: 0x104-0x11C
FMC_Bank1E_BWTR5    EQU 0x118  ; NOR/PSRAM write timing registers, Address offset: 0x104-0x11C
FMC_Bank1E_BWTR6    EQU 0x11C  ; NOR/PSRAM write timing registers, Address offset: 0x104-0x11C

; Flexible Memory Controller Bank3
FMC_Bank3_PCR       EQU  0x80  ; NAND Flash control register,                       Address offset: 0x80
FMC_Bank3_SR        EQU  0x84  ; NAND Flash FIFO status and interrupt register,     Address offset: 0x84
FMC_Bank3_PMEM      EQU  0x88  ; NAND Flash Common memory space timing register,    Address offset: 0x88
FMC_Bank3_PATT      EQU  0x8C  ; NAND Flash Attribute memory space timing register, Address offset: 0x8C
FMC_Bank3_RESERVED0 EQU  0x90  ; Reserved, 0x90
FMC_Bank3_ECCR      EQU  0x94  ; NAND Flash ECC result registers,                   Address offset: 0x94


; General Purpose I/O
GPIO_MODER      EQU  0x00      ; GPIO port mode register,               Address offset: 0x00
GPIO_OTYPER     EQU  0x04      ; GPIO port output type register,        Address offset: 0x04
GPIO_OSPEEDR    EQU  0x08      ; GPIO port output speed register,       Address offset: 0x08
GPIO_PUPDR      EQU  0x0C      ; GPIO port pull-up/pull-down register,  Address offset: 0x0C
GPIO_IDR        EQU  0x10      ; GPIO port input data register,         Address offset: 0x10
GPIO_ODR        EQU  0x14      ; GPIO port output data register,        Address offset: 0x14
GPIO_BSRR       EQU  0x18      ; GPIO port bit set/reset  register,     Address offset: 0x18
GPIO_LCKR       EQU  0x1C      ; GPIO port configuration lock register, Address offset: 0x1C
GPIO_AFR0       EQU  0x20      ; GPIO alternate function registers,     Address offset: 0x20-0x24
GPIO_AFR1       EQU  0x24      ; GPIO alternate function registers,     Address offset: 0x20-0x24
GPIO_BRR        EQU  0x28      ; GPIO Bit Reset register,               Address offset: 0x28
GPIO_ASCR       EQU  0x2C      ; GPIO analog switch control register,   Address offset: 0x2C


; Inter-integrated Circuit Interface
I2C_CR1 EQU  0x00      ; I2C Control register 1,            Address offset: 0x00
I2C_CR2 EQU  0x04      ; I2C Control register 2,            Address offset: 0x04
I2C_OAR1        EQU  0x08      ; I2C Own address 1 register,        Address offset: 0x08
I2C_OAR2        EQU  0x0C      ; I2C Own address 2 register,        Address offset: 0x0C
I2C_TIMINGR     EQU  0x10      ; I2C Timing register,               Address offset: 0x10
I2C_TIMEOUTR    EQU  0x14      ; I2C Timeout register,              Address offset: 0x14
I2C_ISR EQU  0x18      ; I2C Interrupt and status register, Address offset: 0x18
I2C_ICR EQU  0x1C      ; I2C Interrupt clear register,      Address offset: 0x1C
I2C_PECR        EQU  0x20      ; I2C PEC register,                  Address offset: 0x20
I2C_RXDR        EQU  0x24      ; I2C Receive data register,         Address offset: 0x24
I2C_TXDR        EQU  0x28      ; I2C Transmit data register,        Address offset: 0x28

; Independent WATCHDOG
IWDG_KR EQU  0x00      ; IWDG Key register,       Address offset: 0x00
IWDG_PR EQU  0x04      ; IWDG Prescaler register, Address offset: 0x04
IWDG_RLR        EQU  0x08      ; IWDG Reload register,    Address offset: 0x08
IWDG_SR EQU  0x0C      ; IWDG Status register,    Address offset: 0x0C
IWDG_WINR       EQU  0x10      ; IWDG Window register,    Address offset: 0x10

; LCD
LCD_FCR      EQU  0x04  ; LCD frame control register,        Address offset: 0x04
LCD_SR       EQU  0x08  ; LCD status register,               Address offset: 0x08
LCD_CLR      EQU  0x0C  ; LCD clear register,                Address offset: 0x0C
LCD_RESERVED EQU  0x10  ; Reserved,                          Address offset: 0x10
LCD_RAM0     EQU  0x14  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM1     EQU  0x18  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM2     EQU  0x1C  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM3     EQU  0x20  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM4     EQU  0x24  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM5     EQU  0x28  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM6     EQU  0x2C  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM7     EQU  0x30  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM8     EQU  0x34  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM9     EQU  0x38  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM10    EQU  0x3C  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM11    EQU  0x40  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM12    EQU  0x44  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM13    EQU  0x48  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM14    EQU  0x4C  ; LCD display memory,           Address offset: 0x14-0x50
LCD_RAM15    EQU  0x50  ; LCD display memory,           Address offset: 0x14-0x50

; LPTIMER
LPTIM_ISR    EQU  0x00  ; LPTIM Interrupt and Status register,                Address offset: 0x00
LPTIM_ICR    EQU  0x04  ; LPTIM Interrupt Clear register,                     Address offset: 0x04
LPTIM_IER    EQU  0x08  ; LPTIM Interrupt Enable register,                    Address offset: 0x08
LPTIM_CFGR   EQU  0x0C  ; LPTIM Configuration register,                       Address offset: 0x0C
LPTIM_CR     EQU  0x10  ; LPTIM Control register,                             Address offset: 0x10
LPTIM_CMP    EQU  0x14  ; LPTIM Compare register,                             Address offset: 0x14
LPTIM_ARR    EQU  0x18  ; LPTIM Autoreload register,                          Address offset: 0x18
LPTIM_CNT    EQU  0x1C  ; LPTIM Counter register,                             Address offset: 0x1C
LPTIM_OR     EQU  0x20  ; LPTIM Option register,                              Address offset: 0x20


; Operational Amplifier (OPAMP)
OPAMP_CSR    EQU  0x00  ; OPAMP control/status register,                     Address offset: 0x00
OPAMP_OTR    EQU  0x04  ; OPAMP offset trimming register for normal mode,    Address offset: 0x04
OPAMP_LPOTR  EQU  0x08  ; OPAMP offset trimming register for low power mode, Address offset: 0x08


; Power Control
PWR_CR1    EQU  0x00  ; PWR power control register 1,        Address offset: 0x00
PWR_CR2    EQU  0x04  ; PWR power control register 2,        Address offset: 0x04
PWR_CR3    EQU  0x08  ; PWR power control register 3,        Address offset: 0x08
PWR_CR4    EQU  0x0C  ; PWR power control register 4,        Address offset: 0x0C
PWR_SR1    EQU  0x10  ; PWR power status register 1,         Address offset: 0x10
PWR_SR2    EQU  0x14  ; PWR power status register 2,         Address offset: 0x14
PWR_SCR    EQU  0x18  ; PWR power status reset register,     Address offset: 0x18
PWR_RESERVED    EQU  0x1C; Reserved,                            Address offset: 0x1C
PWR_PUCRA  EQU  0x20  ; Pull_up control register of portA,   Address offset: 0x20
PWR_PDCRA  EQU  0x24  ; Pull_Down control register of portA, Address offset: 0x24
PWR_PUCRB  EQU  0x28  ; Pull_up control register of portB,   Address offset: 0x28
PWR_PDCRB  EQU  0x2C  ; Pull_Down control register of portB, Address offset: 0x2C
PWR_PUCRC  EQU  0x30  ; Pull_up control register of portC,   Address offset: 0x30
PWR_PDCRC  EQU  0x34  ; Pull_Down control register of portC, Address offset: 0x34
PWR_PUCRD  EQU  0x38  ; Pull_up control register of portD,   Address offset: 0x38
PWR_PDCRD  EQU  0x3C  ; Pull_Down control register of portD, Address offset: 0x3C
PWR_PUCRE  EQU  0x40  ; Pull_up control register of portE,   Address offset: 0x40
PWR_PDCRE  EQU  0x44  ; Pull_Down control register of portE, Address offset: 0x44
PWR_PUCRF  EQU  0x48  ; Pull_up control register of portF,   Address offset: 0x48
PWR_PDCRF  EQU  0x4C  ; Pull_Down control register of portF, Address offset: 0x4C
PWR_PUCRG  EQU  0x50  ; Pull_up control register of portG,   Address offset: 0x50
PWR_PDCRG  EQU  0x54  ; Pull_Down control register of portG, Address offset: 0x54
PWR_PUCRH  EQU  0x58  ; Pull_up control register of portH,   Address offset: 0x58
PWR_PDCRH  EQU  0x5C  ; Pull_Down control register of portH, Address offset: 0x5C 


; QUAD Serial Peripheral Interface
QUADSPI_CR    EQU  0x00  ; QUADSPI Control register,                           Address offset: 0x00
QUADSPI_DCR   EQU  0x04  ; QUADSPI Device Configuration register,              Address offset: 0x04
QUADSPI_SR    EQU  0x08  ; QUADSPI Status register,                            Address offset: 0x08
QUADSPI_FCR   EQU  0x0C  ; QUADSPI Flag Clear register,                        Address offset: 0x0C
QUADSPI_DLR   EQU  0x10  ; QUADSPI Data Length register,                       Address offset: 0x10
QUADSPI_CCR   EQU  0x14  ; QUADSPI Communication Configuration register,       Address offset: 0x14
QUADSPI_AR    EQU  0x18  ; QUADSPI Address register,                           Address offset: 0x18
QUADSPI_ABR   EQU  0x1C  ; QUADSPI Alternate Bytes register,                   Address offset: 0x1C
QUADSPI_DR    EQU  0x20  ; QUADSPI Data register,                              Address offset: 0x20
QUADSPI_PSMKR EQU  0x24  ; QUADSPI Polling Status Mask register,               Address offset: 0x24
QUADSPI_PSMAR EQU  0x28  ; QUADSPI Polling Status Match register,              Address offset: 0x28
QUADSPI_PIR   EQU  0x2C  ; QUADSPI Polling Interval register,                  Address offset: 0x2C
QUADSPI_LPTR  EQU  0x30  ; QUADSPI Low Power Timeout register,                 Address offset: 0x30


; Reset and Clock Control
RCC_CR  EQU  0x00  ; RCC clock control register,                                              Address offset: 0x00
RCC_ICSCR       EQU  0x04  ; RCC Internal Clock Sources Calibration Register,                         Address offset: 0x04
RCC_CFGR        EQU  0x08  ; RCC clock configuration register,                                        Address offset: 0x08
RCC_PLLCFGR     EQU  0x0C  ; RCC System PLL configuration register,                                   Address offset: 0x0C
RCC_PLLSAI1CFGR EQU  0x10  ; RCC PLL SAI1 Configuration Register,                                     Address offset: 0x10
RCC_PLLSAI2CFGR EQU  0x14  ; RCC PLL SAI2 Configuration Register,                                     Address offset: 0x14
RCC_CIER        EQU  0x18  ; RCC Clock Interrupt Enable Register,                                     Address offset: 0x18
RCC_CIFR        EQU  0x1C  ; RCC Clock Interrupt Flag Register,                                       Address offset: 0x1C
RCC_CICR        EQU  0x20  ; RCC Clock Interrupt Clear Register,                                      Address offset: 0x20
RCC_RESERVED0   EQU  0x24  ; Reserved,                                                                Address offset: 0x24
RCC_AHB1RSTR    EQU  0x28  ; RCC AHB1 peripheral reset register,                                      Address offset: 0x28
RCC_AHB2RSTR    EQU  0x2C  ; RCC AHB2 peripheral reset register,                                      Address offset: 0x2C
RCC_AHB3RSTR    EQU  0x30  ; RCC AHB3 peripheral reset register,                                      Address offset: 0x30
RCC_RESERVED1   EQU  0x34  ; Reserved,                                                                Address offset: 0x34
RCC_APB1RSTR1   EQU  0x38  ; RCC APB1 macrocells resets Low Word,                                     Address offset: 0x38
RCC_APB1RSTR2   EQU  0x3C  ; RCC APB1 macrocells resets High Word,                                    Address offset: 0x3C
RCC_APB2RSTR    EQU  0x40  ; RCC APB2 macrocells resets,                                              Address offset: 0x40
RCC_RESERVED2   EQU  0x44  ; Reserved,                                                                Address offset: 0x44
RCC_AHB1ENR     EQU  0x48  ; RCC AHB1 peripheral clock enable register,                               Address offset: 0x48
RCC_AHB2ENR     EQU  0x4C  ; RCC AHB2 peripheral clock enable register,                               Address offset: 0x4C
RCC_AHB3ENR     EQU  0x50  ; RCC AHB3 peripheral clock enable register,                               Address offset: 0x50
RCC_RESERVED3   EQU  0x54  ; Reserved,                                                                Address offset: 0x54
RCC_APB1ENR1    EQU  0x58  ; RCC APB1 macrocells clock enables Low Word,                              Address offset: 0x58
RCC_APB1ENR2    EQU  0x5C  ; RCC APB1 macrocells clock enables High Word,                             Address offset: 0x5C
RCC_APB2ENR     EQU  0x60  ; RCC APB2 macrocells clock enabled,                                       Address offset: 0x60
RCC_RESERVED4   EQU  0x64  ; Reserved,                                                                Address offset: 0x64
RCC_AHB1SMENR   EQU  0x60  ; RCC AHB1 macrocells clocks enables in sleep mode,                        Address offset: 0x60
RCC_AHB2SMENR   EQU  0x64  ; RCC AHB2 macrocells clock enables in sleep mode,                         Address offset: 0x64
RCC_AHB3SMENR   EQU  0x70  ; RCC AHB3 macrocells clock enables in sleep mode,                         Address offset: 0x70
RCC_RESERVED5   EQU  0x74  ; Reserved,                                                                Address offset: 0x74
RCC_APB1SMENR1  EQU  0x78  ; RCC APB1 macrocells clock enables in sleep mode Low Word,                Address offset: 0x78
RCC_APB1SMENR2  EQU  0x7C  ; RCC APB1 macrocells clock enables in sleep mode High Word,               Address offset: 0x7C
RCC_APB2SMENR   EQU  0x80  ; RCC APB2 macrocells clock enabled in sleep mode,                         Address offset: 0x80
RCC_RESERVED6   EQU  0x84  ; Reserved,                                                                Address offset: 0x84
RCC_CCIPR       EQU  0x88  ; RCC IPs Clocks Configuration Register,                                   Address offset: 0x88
RCC_RESERVED7   EQU  0x8C  ; Reserved,                                                                Address offset: 0x8C
RCC_BDCR        EQU  0x90  ; RCC Vswitch Backup Domain Control Register,                              Address offset: 0x90
RCC_CSR         EQU  0x94  ; RCC clock control & status register,                                     Address offset: 0x94

; Real-Time Clock
RTC_TR         EQU  0x00   ; RTC time register,                                         Address offset: 0x00
RTC_DR         EQU  0x04   ; RTC date register,                                         Address offset: 0x04
RTC_CR         EQU  0x08   ; RTC control register,                                      Address offset: 0x08
RTC_ISR        EQU  0x0C   ; RTC initialization and status register,                    Address offset: 0x0C
RTC_PRER       EQU  0x10   ; RTC prescaler register,                                    Address offset: 0x10
RTC_WUTR       EQU  0x14   ; RTC wakeup timer register,                                 Address offset: 0x14
RTC_Reserved   EQU  0x18   ; Reserved
RTC_ALRMAR     EQU  0x1C   ; RTC alarm A register,                                      Address offset: 0x1C
RTC_ALRMBR     EQU  0x20   ; RTC alarm B register,                                      Address offset: 0x20
RTC_WPR        EQU  0x24   ; RTC write protection register,                             Address offset: 0x24
RTC_SSR        EQU  0x28   ; RTC sub second register,                                   Address offset: 0x28
RTC_SHIFTR     EQU  0x2C   ; RTC shift control register,                                Address offset: 0x2C
RTC_TSTR       EQU  0x30   ; RTC time stamp time register,                              Address offset: 0x30
RTC_TSDR       EQU  0x34   ; RTC time stamp date register,                              Address offset: 0x34
RTC_TSSSR      EQU  0x38   ; RTC time-stamp sub second register,                        Address offset: 0x38
RTC_CALR       EQU  0x3C   ; RTC calibration register,                                  Address offset: 0x3C
RTC_TAMPCR     EQU  0x40   ; RTC tamper configuration register,                         Address offset: 0x40
RTC_ALRMASSR   EQU  0x44   ; RTC alarm A sub second register,                           Address offset: 0x44
RTC_ALRMBSSR   EQU  0x48   ; RTC alarm B sub second register,                           Address offset: 0x48
RTC_OR         EQU  0x4C   ; RTC option register,                                       Address offset: 0x4C
RTC_BKP0R      EQU  0x50   ; RTC backup register 0,                                     Address offset: 0x50
RTC_BKP1R      EQU  0x54   ; RTC backup register 1,                                     Address offset: 0x54
RTC_BKP2R      EQU  0x58   ; RTC backup register 2,                                     Address offset: 0x58
RTC_BKP3R      EQU  0x5C   ; RTC backup register 3,                                     Address offset: 0x5C
RTC_BKP4R      EQU  0x60   ; RTC backup register 4,                                     Address offset: 0x60
RTC_BKP5R      EQU  0x64   ; RTC backup register 5,                                     Address offset: 0x64
RTC_BKP6R      EQU  0x68   ; RTC backup register 6,                                     Address offset: 0x68
RTC_BKP7R      EQU  0x6C   ; RTC backup register 7,                                     Address offset: 0x6C
RTC_BKP8R      EQU  0x70   ; RTC backup register 8,                                     Address offset: 0x70
RTC_BKP9R      EQU  0x74   ; RTC backup register 9,                                     Address offset: 0x74
RTC_BKP10R     EQU  0x78   ; RTC backup register 10,                                    Address offset: 0x78
RTC_BKP11R     EQU  0x7C   ; RTC backup register 11,                                    Address offset: 0x7C
RTC_BKP12R     EQU  0x80   ; RTC backup register 12,                                    Address offset: 0x80
RTC_BKP13R     EQU  0x84   ; RTC backup register 13,                                    Address offset: 0x84
RTC_BKP14R     EQU  0x88   ; RTC backup register 14,                                    Address offset: 0x88
RTC_BKP15R     EQU  0x8C   ; RTC backup register 15,                                    Address offset: 0x8C
RTC_BKP16R     EQU  0x90   ; RTC backup register 16,                                    Address offset: 0x90
RTC_BKP17R     EQU  0x94   ; RTC backup register 17,                                    Address offset: 0x94
RTC_BKP18R     EQU  0x98   ; RTC backup register 18,                                    Address offset: 0x98
RTC_BKP19R     EQU  0x9C   ; RTC backup register 19,                                    Address offset: 0x9C
RTC_BKP20R     EQU  0xA0   ; RTC backup register 20,                                    Address offset: 0xA0
RTC_BKP21R     EQU  0xA4   ; RTC backup register 21,                                    Address offset: 0xA4
RTC_BKP22R     EQU  0xA8   ; RTC backup register 22,                                    Address offset: 0xA8
RTC_BKP23R     EQU  0xAC   ; RTC backup register 23,                                    Address offset: 0xAC
RTC_BKP24R     EQU  0xB0   ; RTC backup register 24,                                    Address offset: 0xB0
RTC_BKP25R     EQU  0xB4   ; RTC backup register 25,                                    Address offset: 0xB4
RTC_BKP26R     EQU  0xB0   ; RTC backup register 26,                                    Address offset: 0xB8
RTC_BKP27R     EQU  0xB0   ; RTC backup register 27,                                    Address offset: 0xBC
RTC_BKP28R     EQU  0xC0   ; RTC backup register 28,                                    Address offset: 0xC0
RTC_BKP29R     EQU  0xC0   ; RTC backup register 29,                                    Address offset: 0xC4
RTC_BKP30R     EQU  0xC0   ; RTC backup register 30,                                    Address offset: 0xC8
RTC_BKP31R     EQU  0xC0   ; RTC backup register 31,                                    Address offset: 0xCC 


; Serial Audio Interface
SAI_GCR          EQU  0x00    ; SAI global configuration register,        Address offset: 0x00
SAI_Block_CR1    EQU  0x04    ; SAI block x configuration register 1,     Address offset: 0x04
SAI_Block_CR2    EQU  0x08    ; SAI block x configuration register 2,     Address offset: 0x08
SAI_Block_FRCR   EQU  0x0C    ; SAI block x frame configuration register, Address offset: 0x0C
SAI_Block_SLOTR  EQU  0x10    ; SAI block x slot register,                Address offset: 0x10
SAI_Block_IMR    EQU  0x14    ; SAI block x interrupt mask register,      Address offset: 0x14
SAI_Block_SR     EQU  0x18    ; SAI block x status register,              Address offset: 0x18
SAI_Block_CLRFR  EQU  0x1C    ; SAI block x clear flag register,          Address offset: 0x1C
SAI_Block_DR     EQU  0x20    ; SAI block x data register,                Address offset: 0x20

; Secure digital input/output Interface
SDMMC_POWER      EQU  0x00    ; SDMMC power control register,    Address offset: 0x00
SDMMC_CLKCR      EQU  0x04    ; SDMMC clock control register,    Address offset: 0x04
SDMMC_ARG        EQU  0x08    ; SDMMC argument register,         Address offset: 0x08
SDMMC_CMD        EQU  0x0C    ; SDMMC command register,          Address offset: 0x0C
SDMMC_RESPCMD    EQU  0x10    ; SDMMC command response register, Address offset: 0x10
SDMMC_RESP1      EQU  0x14    ; SDMMC response 1 register,       Address offset: 0x14
SDMMC_RESP2      EQU  0x18    ; SDMMC response 2 register,       Address offset: 0x18
SDMMC_RESP3      EQU  0x1C    ; SDMMC response 3 register,       Address offset: 0x1C
SDMMC_RESP4      EQU  0x20    ; SDMMC response 4 register,       Address offset: 0x20
SDMMC_DTIMER     EQU  0x24    ; SDMMC data timer register,       Address offset: 0x24
SDMMC_DLEN       EQU  0x28    ; SDMMC data length register,      Address offset: 0x28
SDMMC_DCTRL      EQU  0x2C    ; SDMMC data control register,     Address offset: 0x2C
SDMMC_DCOUNT     EQU  0x30    ; SDMMC data counter register,     Address offset: 0x30
SDMMC_STA        EQU  0x34    ; SDMMC status register,           Address offset: 0x34
SDMMC_ICR        EQU  0x38    ; SDMMC interrupt clear register,  Address offset: 0x38
SDMMC_MASK       EQU  0x3C    ; SDMMC mask register,             Address offset: 0x3C
SDMMC_RESERVED0  EQU  0x40    ; Reserved, 0x40-0x44
SDMMC_FIFOCNT    EQU  0x48    ; SDMMC FIFO counter register,     Address offset: 0x48
SDMMC_RESERVED1  EQU  0x4C    ; Reserved, 0x4C-0x7C
SDMMC_FIFO       EQU  0x80    ; SDMMC data FIFO register,        Address offset: 0x80


; Serial Peripheral Interface
SPI_CR1       EQU  0x00      ; SPI Control register 1,           Address offset: 0x00
SPI_CR2       EQU  0x04      ; SPI Control register 2,           Address offset: 0x04
SPI_SR        EQU  0x08      ; SPI Status register,              Address offset: 0x08
SPI_DR        EQU  0x0C      ; SPI data register,                Address offset: 0x0C
SPI_CRCPR     EQU  0x10      ; SPI CRC polynomial register,      Address offset: 0x10
SPI_RXCRCR    EQU  0x14      ; SPI Rx CRC register,              Address offset: 0x14
SPI_TXCRCR    EQU  0x18      ; SPI Tx CRC register,              Address offset: 0x18
SPI_RESERVED1 EQU  0x1C      ; Reserved,                         Address offset: 0x1C
SPI_RESERVED2 EQU  0x20      ; Reserved,                         Address offset: 0x20


; Single Wire Protocol Master Interface SPWMI
SWPMI_CR      EQU  0x00      ; SWPMI Configuration/Control register,     Address offset: 0x00
SWPMI_BRR     EQU  0x04      ; SWPMI bitrate register,                   Address offset: 0x04
SWPMI_RESERVED1    EQU  0x08 ; Reserved, 0x08
SWPMI_ISR     EQU  0x0C      ; SWPMI Interrupt and Status register,      Address offset: 0x0C
SWPMI_ICR     EQU  0x10      ; SWPMI Interrupt Flag Clear register,      Address offset: 0x10
SWPMI_IER     EQU  0x14      ; SWPMI Interrupt Enable register,          Address offset: 0x14
SWPMI_RFL     EQU  0x18      ; SWPMI Receive Frame Length register,      Address offset: 0x18
SWPMI_TDR     EQU  0x1C      ; SWPMI Transmit data register,             Address offset: 0x1C
SWPMI_RDR     EQU  0x20      ; SWPMI Receive data register,              Address offset: 0x20
SWPMI_OR      EQU  0x24      ; SWPMI Option register,                    Address offset: 0x24


; System configuration controller
SYSCFG_MEMRMP   EQU  0x00   ; SYSCFG memory remap register,                      Address offset: 0x00
SYSCFG_CFGR1    EQU  0x04   ; SYSCFG configuration register 1,                   Address offset: 0x04
SYSCFG_EXTICR0  EQU  0x08   ; SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14
SYSCFG_EXTICR1  EQU  0x0C   ; SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14
SYSCFG_EXTICR2  EQU  0x10   ; SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14
SYSCFG_EXTICR3  EQU  0x14   ; SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14
SYSCFG_SCSR     EQU  0x18   ; SYSCFG SRAM2 control and status register,          Address offset: 0x18
SYSCFG_CFGR2    EQU  0x1C   ; SYSCFG configuration register 2,                   Address offset: 0x1C
SYSCFG_SWPR     EQU  0x20   ; SYSCFG SRAM2 write protection register,            Address offset: 0x20
SYSCFG_SKR      EQU  0x24   ; SYSCFG SRAM2 key register,                         Address offset: 0x24


; TIM
TIM_CR1    EQU  0x00     ; TIM control register 1,                   Address offset: 0x00
TIM_CR2    EQU  0x04     ; TIM control register 2,                   Address offset: 0x04
TIM_SMCR   EQU  0x08     ; TIM slave mode control register,          Address offset: 0x08
TIM_DIER   EQU  0x0C     ; TIM DMA/interrupt enable register,        Address offset: 0x0C
TIM_SR     EQU  0x10     ; TIM status register,                      Address offset: 0x10
TIM_EGR    EQU  0x14     ; TIM event generation register,            Address offset: 0x14
TIM_CCMR1  EQU  0x18     ; TIM capture/compare mode register 1,      Address offset: 0x18
TIM_CCMR2  EQU  0x1C     ; TIM capture/compare mode register 2,      Address offset: 0x1C
TIM_CCER   EQU  0x20     ; TIM capture/compare enable register,      Address offset: 0x20
TIM_CNT    EQU  0x24     ; TIM counter register,                     Address offset: 0x24
TIM_PSC    EQU  0x28     ; TIM prescaler,                            Address offset: 0x28
TIM_ARR    EQU  0x2C     ; TIM auto-reload register,                 Address offset: 0x2C
TIM_RCR    EQU  0x30     ; TIM repetition counter register,          Address offset: 0x30
TIM_CCR1   EQU  0x34     ; TIM capture/compare register 1,           Address offset: 0x34
TIM_CCR2   EQU  0x38     ; TIM capture/compare register 2,           Address offset: 0x38
TIM_CCR3   EQU  0x3C     ; TIM capture/compare register 3,           Address offset: 0x3C
TIM_CCR4   EQU  0x40     ; TIM capture/compare register 4,           Address offset: 0x40
TIM_BDTR   EQU  0x44     ; TIM break and dead-time register,         Address offset: 0x44
TIM_DCR    EQU  0x48     ; TIM DMA control register,                 Address offset: 0x48
TIM_DMAR   EQU  0x4C     ; TIM DMA address for full transfer,        Address offset: 0x4C
TIM_OR1    EQU  0x50     ; TIM option register 1,                    Address offset: 0x50
TIM_CCMR3  EQU  0x54     ; TIM capture/compare mode register 3,      Address offset: 0x54
TIM_CCR5   EQU  0x58     ; TIM capture/compare register5,            Address offset: 0x58
TIM_CCR6   EQU  0x5C     ; TIM capture/compare register6,            Address offset: 0x5C
TIM_OR2    EQU  0x60     ; TIM option register 2,                    Address offset: 0x60
TIM_OR3    EQU  0x64     ; TIM option register 3,                    Address offset: 0x64


; Touch Sensing Controller (TSC)
TSC_CR   EQU  0x00      ; TSC control register,                      Address offset: 0x00
TSC_IER  EQU  0x04      ; TSC interrupt enable register,             Address offset: 0x04
TSC_ICR  EQU  0x08      ; TSC interrupt clear register,              Address offset: 0x08
TSC_ISR  EQU  0x0C      ; TSC interrupt status register,             Address offset: 0x0C
TSC_IOHCR        EQU  0x10      ; TSC I/O hysteresis control register,       Address offset: 0x10
TSC_RESERVED1    EQU  0x14      ; Reserved,                                  Address offset: 0x14
TSC_IOASCR       EQU  0x18      ; TSC I/O analog switch control register,    Address offset: 0x18
TSC_RESERVED2    EQU  0x1C      ; Reserved,                                  Address offset: 0x1C
TSC_IOSCR        EQU  0x20      ; TSC I/O sampling control register,         Address offset: 0x20
TSC_RESERVED3    EQU  0x24      ; Reserved,                                  Address offset: 0x24
TSC_IOCCR        EQU  0x28      ; TSC I/O channel control register,          Address offset: 0x28
TSC_RESERVED4    EQU  0x2C      ; Reserved,                                  Address offset: 0x2C
TSC_IOGCSR       EQU  0x30      ; TSC I/O group control status register,     Address offset: 0x30
TSC_IOGXCR0      EQU  0x34      ; TSC I/O group x counter register,          Address offset: 0x34-50
TSC_IOGXCR1      EQU  0x38      ; TSC I/O group x counter register,          Address offset: 0x34-50
TSC_IOGXCR2      EQU  0x3C      ; TSC I/O group x counter register,          Address offset: 0x34-50
TSC_IOGXCR3      EQU  0x40      ; TSC I/O group x counter register,          Address offset: 0x34-50
TSC_IOGXCR4      EQU  0x44      ; TSC I/O group x counter register,          Address offset: 0x34-50
TSC_IOGXCR5      EQU  0x48      ; TSC I/O group x counter register,          Address offset: 0x34-50
TSC_IOGXCR6      EQU  0x4C      ; TSC I/O group x counter register,          Address offset: 0x34-50
TSC_IOGXCR7      EQU  0x50      ; TSC I/O group x counter register,          Address offset: 0x34-50

; Universal Synchronous Asynchronous Receiver Transmitter
USART_CR1        EQU  0x00      ; USART Control register 1,                 Address offset: 0x00
USART_CR2        EQU  0x04      ; USART Control register 2,                 Address offset: 0x04
USART_CR3        EQU  0x08      ; USART Control register 3,                 Address offset: 0x08
USART_BRR        EQU  0x0C      ; USART Baud rate register,                 Address offset: 0x0C
USART_GTPR       EQU  0x10      ; USART Guard time and prescaler register,  Address offset: 0x10
USART_RESERVED2  EQU  0x12      ; Reserved, 0x12
USART_RTOR       EQU  0x14      ; USART Receiver Time Out register,         Address offset: 0x14
USART_RQR        EQU  0x18      ; USART Request register,                   Address offset: 0x18
USART_RESERVED3  EQU  0x1A      ; Reserved, 0x1A
USART_ISR        EQU  0x1C      ; USART Interrupt and status register,      Address offset: 0x1C
USART_ICR        EQU  0x20      ; USART Interrupt flag Clear register,      Address offset: 0x20
USART_RDR        EQU  0x24      ; USART Receive Data register,              Address offset: 0x24
USART_RESERVED4  EQU  0x26      ; Reserved, 0x26
USART_TDR        EQU  0x28      ; USART Transmit Data register,             Address offset: 0x28
USART_RESERVED5  EQU  0x2A      ; Reserved, 0x2A


; VREFBUF
VREFBUF_CSR    EQU  0x00      ; VREFBUF control and status register,         Address offset: 0x00
VREFBUF_CCR    EQU  0x04      ; VREFBUF calibration and control register,    Address offset: 0x04

; Window WATCHDOG
WWDG_CR    EQU  0x00       ; WWDG Control register,       Address offset: 0x00
WWDG_CFR   EQU  0x04       ; WWDG Configuration register, Address offset: 0x04
WWDG_SR    EQU  0x08       ; WWDG Status register,        Address offset: 0x08



; RNG
NRG_CR EQU 0x00  ; RNG control register, Address offset: 0x00
NRG_SR EQU 0x04  ; RNG status register,  Address offset: 0x04
NRG_DR EQU 0x08  ; RNG data register,    Address offset: 0x08

; USB_OTG_Core_register
USB_OTG_Global_GOTGCTL    EQU  0x00           ;  USB_OTG Control and Status Register          000h
USB_OTG_Global_GOTGINT    EQU  0x04           ;  USB_OTG Interrupt Register                   004h
USB_OTG_Global_GAHBCFG    EQU  0x08           ;  Core AHB Configuration Register              008h
USB_OTG_Global_GUSBCFG    EQU  0x0C           ;  Core USB Configuration Register              00Ch
USB_OTG_Global_GRSTCTL    EQU  0x10           ;  Core Reset Register                          010h
USB_OTG_Global_GINTSTS    EQU  0x14           ;  Core Interrupt Register                      014h
USB_OTG_Global_GINTMSK    EQU  0x18           ;  Core Interrupt Mask Register                 018h
USB_OTG_Global_GRXSTSR    EQU  0x1C           ;  Receive Sts Q Read Register                  01Ch
USB_OTG_Global_GRXSTSP    EQU  0x20           ;  Receive Sts Q Read & POP Register            020h
USB_OTG_Global_GRXFSIZ    EQU  0x24           ;  Receive FIFO Size Register                   024h
USB_OTG_Global_DIEPTXF0_HNPTXFSIZ  EQU  0x28  ;  EP0 / Non Periodic Tx FIFO Size Register     028h
USB_OTG_Global_HNPTXSTS    EQU  0x0C          ;  Non Periodic Tx FIFO/Queue Sts reg           02Ch
USB_OTG_Global_Reserved30  EQU  0x30          ; Reserved                                      030h
USB_OTG_Global_GCCFG       EQU  0x38          ; General Purpose IO Register                   038h
USB_OTG_Global_CID         EQU  0x3C          ; User ID Register                              03Ch
USB_OTG_Global_Reserved5   EQU  0x40         ; Reserved                                       040h-048h
USB_OTG_Global_GHWCFG3     EQU  0x4C          ; User HW config3                               04Ch
USB_OTG_Global_Reserved6   EQU  0x50          ; Reserved                                      050h
USB_OTG_Global_GLPMCFG     EQU  0x54          ; LPM Register                                  054h
USB_OTG_Global_GPWRDN      EQU  0x58          ; Power Down Register                           058h
USB_OTG_Global_GDFIFOCFG   EQU  0x5C          ; DFIFO Software Config Register                05Ch
USB_OTG_Global_GADPCTL     EQU  0x60          ; ADP Timer, Control and Status Register        60Ch
USB_OTG_Global_Reserved43  EQU  0x64          ; Reserved                                      058h-0FFh
USB_OTG_Global_HPTXFSIZ    EQU  0x100         ; Host Periodic Tx FIFO Size Reg                100h
USB_OTG_Global_DIEPTXF0    EQU  0x104         ; dev Periodic Transmit FIFO


; USB_OTG_device_Registers
USB_OTG_Device_DCFG        EQU  0x800  ; dev Configuration Register   800h
USB_OTG_Device_DCTL        EQU  0x804  ; dev Control Register         804h
USB_OTG_Device_DSTS        EQU  0x808  ; dev Status Register (RO)     808h
USB_OTG_Device_Reserved0C  EQU  0x80C  ; Reserved                     80Ch
USB_OTG_Device_DIEPMSK     EQU  0x810  ; dev IN Endpoint Mask         810h
USB_OTG_Device_DOEPMSK     EQU  0x814  ; dev OUT Endpoint Mask        814h
USB_OTG_Device_DAINT       EQU  0x818  ; dev All Endpoints Itr Reg    818h
USB_OTG_Device_DAINTMSK    EQU  0x81C  ; dev All Endpoints Itr Mask   81Ch
USB_OTG_Device_Reserved20  EQU  0x820  ; Reserved                     820h
USB_OTG_Device_Reserved9   EQU  0x824  ; Reserved                     824h
USB_OTG_Device_DVBUSDIS    EQU  0x828  ; dev VBUS discharge Register  828h
USB_OTG_Device_DVBUSPULSE  EQU  0x82C  ; dev VBUS Pulse Register      82Ch
USB_OTG_Device_DTHRCTL     EQU  0x830  ; dev thr                      830h
USB_OTG_Device_DIEPEMPMSK  EQU  0x834  ; dev empty msk                834h
USB_OTG_Device_DEACHINT    EQU  0x838  ; dedicated EP interrupt       838h
USB_OTG_Device_DEACHMSK    EQU  0x83C  ; dedicated EP msk             83Ch
USB_OTG_Device_Reserved40  EQU  0x840  ; dedicated EP mask            840h
USB_OTG_Device_DINEP1MSK   EQU  0x844  ; dedicated EP mask            844h
USB_OTG_Device_Reserved44  EQU  0x848  ; Reserved                 844-87Ch
USB_OTG_Device_DOUTEP1MSK  EQU  0x884  ; dedicated EP msk             884h

; USB_OTG_IN_Endpoint-Specific_Register
USB_OTG_INEndpoint_DIEPCTL    EQU  0x900  ; dev IN Endpoint Control Reg 900h + (ep_num * 20h) + 00h
USB_OTG_INEndpoint_Reserved04 EQU  0x904  ; Reserved                       900h + (ep_num * 20h) + 04h
USB_OTG_INEndpoint_DIEPINT    EQU  0x908  ; dev IN Endpoint Itr Reg     900h + (ep_num * 20h) + 08h
USB_OTG_INEndpoint_Reserved0C EQU  0x90C  ; Reserved                       900h + (ep_num * 20h) + 0Ch
USB_OTG_INEndpoint_DIEPTSIZ   EQU  0x910  ; IN Endpoint Txfer Size   900h + (ep_num * 20h) + 10h
USB_OTG_INEndpoint_DIEPDMA    EQU  0x914  ; IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h
USB_OTG_INEndpoint_DTXFSTS    EQU  0x918  ; IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h
USB_OTG_INEndpoint_Reserved18 EQU  0x91C  ; Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch

; USB_OTG_OUT_Endpoint-Specific_Registers
SB_OTG_OUTEndpoint_DOEPCTL    EQU  0xB00   ; dev OUT Endpoint Control Reg  B00h + (ep_num * 20h) + 00h
SB_OTG_OUTEndpoint_Reserved04 EQU  0xB04   ; Reserved                      B00h + (ep_num * 20h) + 04h
SB_OTG_OUTEndpoint_DOEPINT    EQU  0xB08   ; dev OUT Endpoint Itr Reg      B00h + (ep_num * 20h) + 08h
SB_OTG_OUTEndpoint_Reserved0C EQU  0xB0C   ; Reserved                      B00h + (ep_num * 20h) + 0Ch
SB_OTG_OUTEndpoint_DOEPTSIZ   EQU  0xB10   ; dev OUT Endpoint Txfer Size   B00h + (ep_num * 20h) + 10h
SB_OTG_OUTEndpoint_DOEPDMA    EQU  0xB14   ; dev OUT Endpoint DMA Address  B00h + (ep_num * 20h) + 14h
SB_OTG_OUTEndpoint_Reserved18 EQU  0xB18   ; Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch

; USB_OTG_Host_Mode_Register_Structures
USB_OTG_Host_HCFG     EQU  0x400  ; Host Configuration Register    400h
USB_OTG_Host_HFIR     EQU  0x404  ; Host Frame Interval Register   404h
USB_OTG_Host_HFNUM    EQU  0x408  ; Host Frame Nbr/Frame Remaining 408h
USB_OTG_Host_Reserved40C      EQU  0x40C  ; Reserved                       40Ch
USB_OTG_Host_HPTXSTS  EQU  0x410  ; Host Periodic Tx FIFO/ Queue Status 410h
USB_OTG_Host_HAINT    EQU  0x414  ; Host All Channels Interrupt Register 414h
USB_OTG_Host_HAINTMSK EQU  0x418  ; Host All Channels Interrupt Mask 418h

; USB_OTG_Host_Channel_Specific_Registers
USB_OTG_HostChannel_HCCHAR    EQU  0x00  
USB_OTG_HostChannel_HCSPLT    EQU  0x04  
USB_OTG_HostChannel_HCINT     EQU  0x08  
USB_OTG_HostChannel_HCINTMSK  EQU  0x0C  
USB_OTG_HostChannel_HCTSIZ    EQU  0x10  
USB_OTG_HostChannel_HCDMA     EQU  0x14  
USB_OTG_HostChannel_Reserved  EQU  0x18  

; Peripheral_memory_map
FLASH_BASE    EQU (0x08000000) ; FLASH(up to 1 MB) base address
SRAM1_BASE    EQU (0x20000000) ; SRAM1(96 KB) base address
PERIPH_BASE   EQU (0x40000000) ; Peripheral base address
FMC_BASE      EQU (0x60000000) ; FMC base address
SRAM2_BASE    EQU (0x10000000) ; SRAM2(32 KB) base address
FMC_R_BASE    EQU (0xA0000000) ; FMC  control registers base address
QSPI_R_BASE   EQU (0xA0001000) ; QUADSPI control registers base address
SRAM1_BB_BASE EQU (0x22000000) ; SRAM1(96 KB) base address in the bit-band region
PERIPH_BB_BASE        EQU (0x42000000) ; Peripheral base address in the bit-band region
SRAM2_BB_BASE EQU (0x12000000) ; SRAM2(32 KB) base address in the bit-band region

; Legacy defines
SRAM_BASE     EQU SRAM1_BASE
SRAM_BB_BASE  EQU SRAM1_BB_BASE

SRAM1_SIZE_MAX EQU (0x00018000) ; maximum SRAM1 size (up to 96 KBytes)
SRAM2_SIZE     EQU (0x00008000) ; SRAM2 size (32 KBytes)

; Peripheral memory map
APB1PERIPH_BASE       EQU PERIPH_BASE
APB2PERIPH_BASE       EQU (PERIPH_BASE + 0x00010000)
AHB1PERIPH_BASE       EQU (PERIPH_BASE + 0x00020000)
AHB2PERIPH_BASE       EQU (PERIPH_BASE + 0x08000000)

FMC_BANK1     EQU FMC_BASE
FMC_BANK1_1   EQU FMC_BANK1
FMC_BANK1_2   EQU (FMC_BANK1 + 0x04000000)
FMC_BANK1_3   EQU (FMC_BANK1 + 0x08000000)
FMC_BANK1_4   EQU (FMC_BANK1 + 0x0C000000)
FMC_BANK3     EQU (FMC_BASE  + 0x20000000)

; APB1 peripherals
TIM2_BASE     EQU (APB1PERIPH_BASE + 0x0000)
TIM3_BASE     EQU (APB1PERIPH_BASE + 0x0400)
TIM4_BASE     EQU (APB1PERIPH_BASE + 0x0800)
TIM5_BASE     EQU (APB1PERIPH_BASE + 0x0C00)
TIM6_BASE     EQU (APB1PERIPH_BASE + 0x1000)
TIM7_BASE     EQU (APB1PERIPH_BASE + 0x1400)
LCD_BASE      EQU (APB1PERIPH_BASE + 0x2400)
RTC_BASE      EQU (APB1PERIPH_BASE + 0x2800)
WWDG_BASE     EQU (APB1PERIPH_BASE + 0x2C00)
IWDG_BASE     EQU (APB1PERIPH_BASE + 0x3000)
SPI2_BASE     EQU (APB1PERIPH_BASE + 0x3800)
SPI3_BASE     EQU (APB1PERIPH_BASE + 0x3C00)
USART2_BASE   EQU (APB1PERIPH_BASE + 0x4400)
USART3_BASE   EQU (APB1PERIPH_BASE + 0x4800)
UART4_BASE    EQU (APB1PERIPH_BASE + 0x4C00)
UART5_BASE    EQU (APB1PERIPH_BASE + 0x5000)
I2C1_BASE     EQU (APB1PERIPH_BASE + 0x5400)
I2C2_BASE     EQU (APB1PERIPH_BASE + 0x5800)
I2C3_BASE     EQU (APB1PERIPH_BASE + 0x5C00)
CAN1_BASE     EQU (APB1PERIPH_BASE + 0x6400)
LPTIM1_BASE   EQU (APB1PERIPH_BASE + 0x7C00)
PWR_BASE      EQU (APB1PERIPH_BASE + 0x7000)
DAC_BASE      EQU (APB1PERIPH_BASE + 0x7400)
DAC1_BASE     EQU (APB1PERIPH_BASE + 0x7400)
OPAMP_BASE    EQU (APB1PERIPH_BASE + 0x7800)
OPAMP1_BASE   EQU (APB1PERIPH_BASE + 0x7800)
OPAMP2_BASE   EQU (APB1PERIPH_BASE + 0x7810)
LPUART1_BASE  EQU (APB1PERIPH_BASE + 0x8000)
SWPMI1_BASE   EQU (APB1PERIPH_BASE + 0x8800)
LPTIM2_BASE   EQU (APB1PERIPH_BASE + 0x9400)


; APB2 peripherals
SYSCFG_BASE   EQU (APB2PERIPH_BASE + 0x0000)
VREFBUF_BASE  EQU (APB2PERIPH_BASE + 0x0030)
COMP1_BASE    EQU (APB2PERIPH_BASE + 0x0200)
COMP2_BASE    EQU (APB2PERIPH_BASE + 0x0204)
EXTI_BASE     EQU (APB2PERIPH_BASE + 0x0400)
FIREWALL_BASE EQU (APB2PERIPH_BASE + 0x1C00)
SDMMC1_BASE   EQU (APB2PERIPH_BASE + 0x2800)
TIM1_BASE     EQU (APB2PERIPH_BASE + 0x2C00)
SPI1_BASE     EQU (APB2PERIPH_BASE + 0x3000)
TIM8_BASE     EQU (APB2PERIPH_BASE + 0x3400)
USART1_BASE   EQU (APB2PERIPH_BASE + 0x3800)
TIM15_BASE    EQU (APB2PERIPH_BASE + 0x4000)
TIM16_BASE    EQU (APB2PERIPH_BASE + 0x4400)
TIM17_BASE    EQU (APB2PERIPH_BASE + 0x4800)
SAI1_BASE     EQU (APB2PERIPH_BASE + 0x5400)
SAI1_Block_A_BASE     EQU (SAI1_BASE + 0x004)
SAI1_Block_B_BASE     EQU (SAI1_BASE + 0x024)
SAI2_BASE     EQU (APB2PERIPH_BASE + 0x5800)
SAI2_Block_A_BASE     EQU (SAI2_BASE + 0x004)
SAI2_Block_B_BASE     EQU (SAI2_BASE + 0x024)
DFSDM_BASE    EQU (APB2PERIPH_BASE + 0x6000)
DFSDM_Channel0_BASE   EQU (DFSDM_BASE + 0x00)
DFSDM_Channel1_BASE   EQU (DFSDM_BASE + 0x20)
DFSDM_Channel2_BASE   EQU (DFSDM_BASE + 0x40)
DFSDM_Channel3_BASE   EQU (DFSDM_BASE + 0x60)
DFSDM_Channel4_BASE   EQU (DFSDM_BASE + 0x80)
DFSDM_Channel5_BASE   EQU (DFSDM_BASE + 0xA0)
DFSDM_Channel6_BASE   EQU (DFSDM_BASE + 0xC0)
DFSDM_Channel7_BASE   EQU (DFSDM_BASE + 0xE0)
DFSDM_Filter0_BASE    EQU (DFSDM_BASE + 0x100)
DFSDM_Filter1_BASE    EQU (DFSDM_BASE + 0x180)
DFSDM_Filter2_BASE    EQU (DFSDM_BASE + 0x200)
DFSDM_Filter3_BASE    EQU (DFSDM_BASE + 0x280)

; AHB1 peripherals
DMA1_BASE     EQU (AHB1PERIPH_BASE)
DMA2_BASE     EQU (AHB1PERIPH_BASE + 0x0400)
RCC_BASE      EQU (AHB1PERIPH_BASE + 0x1000)
FLASH_R_BASE  EQU (AHB1PERIPH_BASE + 0x2000)
CRC_BASE      EQU (AHB1PERIPH_BASE + 0x3000)
TSC_BASE      EQU (AHB1PERIPH_BASE + 0x4000)


DMA1_Channel1_BASE    EQU (DMA1_BASE + 0x0008)
DMA1_Channel2_BASE    EQU (DMA1_BASE + 0x001C)
DMA1_Channel3_BASE    EQU (DMA1_BASE + 0x0030)
DMA1_Channel4_BASE    EQU (DMA1_BASE + 0x0044)
DMA1_Channel5_BASE    EQU (DMA1_BASE + 0x0058)
DMA1_Channel6_BASE    EQU (DMA1_BASE + 0x006C)
DMA1_Channel7_BASE    EQU (DMA1_BASE + 0x0080)
DMA1_CSELR_BASE       EQU (DMA1_BASE + 0x00A8)


DMA2_Channel1_BASE    EQU (DMA2_BASE + 0x0008)
DMA2_Channel2_BASE    EQU (DMA2_BASE + 0x001C)
DMA2_Channel3_BASE    EQU (DMA2_BASE + 0x0030)
DMA2_Channel4_BASE    EQU (DMA2_BASE + 0x0044)
DMA2_Channel5_BASE    EQU (DMA2_BASE + 0x0058)
DMA2_Channel6_BASE    EQU (DMA2_BASE + 0x006C)
DMA2_Channel7_BASE    EQU (DMA2_BASE + 0x0080)
DMA2_CSELR_BASE       EQU (DMA2_BASE + 0x00A8)


; AHB2 peripherals
GPIOA_BASE    EQU (AHB2PERIPH_BASE + 0x0000)
GPIOB_BASE    EQU (AHB2PERIPH_BASE + 0x0400)
GPIOC_BASE    EQU (AHB2PERIPH_BASE + 0x0800)
GPIOD_BASE    EQU (AHB2PERIPH_BASE + 0x0C00)
GPIOE_BASE    EQU (AHB2PERIPH_BASE + 0x1000)
GPIOF_BASE    EQU (AHB2PERIPH_BASE + 0x1400)
GPIOG_BASE    EQU (AHB2PERIPH_BASE + 0x1800)
GPIOH_BASE    EQU (AHB2PERIPH_BASE + 0x1C00)

USBOTG_BASE   EQU (AHB2PERIPH_BASE + 0x08000000)

ADC1_BASE     EQU (AHB2PERIPH_BASE + 0x08040000)
ADC2_BASE     EQU (AHB2PERIPH_BASE + 0x08040100)
ADC3_BASE     EQU (AHB2PERIPH_BASE + 0x08040200)
ADC123_COMMON_BASE    EQU (AHB2PERIPH_BASE + 0x08040300)


RNG_BASE      EQU (AHB2PERIPH_BASE + 0x08060800)

; FMC Banks registers base  address
FMC_Bank1_R_BASE      EQU (FMC_R_BASE + 0x0000)
FMC_Bank1E_R_BASE     EQU (FMC_R_BASE + 0x0104)
FMC_Bank2_R_BASE      EQU (FMC_R_BASE + 0x0060)
FMC_Bank3_R_BASE      EQU (FMC_R_BASE + 0x0080)
FMC_Bank4_R_BASE      EQU (FMC_R_BASE + 0x00A0)

; Debug MCU registers base address
DBGMCU_BASE   EQU (0xE0042000)

; USB registers base address
USB_OTG_FS_PERIPH_BASE       EQU (0x50000000)

USB_OTG_GLOBAL_BASE          EQU (0x000)
USB_OTG_DEVICE_BASE          EQU (0x800)
USB_OTG_IN_ENDPOINT_BASE     EQU (0x900)
USB_OTG_OUT_ENDPOINT_BASE    EQU (0xB00)
USB_OTG_EP_REG_SIZE          EQU (0x20)
USB_OTG_HOST_BASE            EQU (0x400)
USB_OTG_HOST_PORT_BASE       EQU (0x440)
USB_OTG_HOST_CHANNEL_BASE    EQU (0x500)
USB_OTG_HOST_CHANNEL_SIZE    EQU (0x20)
USB_OTG_PCGCCTL_BASE         EQU (0xE00)
USB_OTG_FIFO_BASE            EQU (0x1000)
USB_OTG_FIFO_SIZE            EQU (0x1000)



; Peripheral_Registers_Bits_Definition



; *****************************************************************************
;                      Peripheral Registers_Bits_Definition
; *****************************************************************************

; *****************************************************************************
;                        Analog to Digital Converter
; *****************************************************************************
; ********************  Bit definition for ADC_ISR register  *******************
ADC_ISR_ADRDY EQU (0x00000001) ; ADC Ready (ADRDY) flag
ADC_ISR_EOSMP EQU (0x00000002) ; ADC End of Sampling flag
ADC_ISR_EOC   EQU (0x00000004) ; ADC End of Regular Conversion flag
ADC_ISR_EOS   EQU (0x00000008) ; ADC End of Regular sequence of Conversions flag
ADC_ISR_OVR   EQU (0x00000010) ; ADC overrun flag
ADC_ISR_JEOC  EQU (0x00000020) ; ADC End of Injected Conversion flag
ADC_ISR_JEOS  EQU (0x00000040) ; ADC End of Injected sequence of Conversions flag
ADC_ISR_AWD1  EQU (0x00000080) ; ADC Analog watchdog 1 flag
ADC_ISR_AWD2  EQU (0x00000100) ; ADC Analog watchdog 2 flag
ADC_ISR_AWD3  EQU (0x00000200) ; ADC Analog watchdog 3 flag
ADC_ISR_JQOVF EQU (0x00000400) ; ADC Injected Context Queue Overflow flag

; ********************  Bit definition for ADC_IER register  *******************
ADC_IER_ADRDY EQU (0x00000001) ; ADC Ready (ADRDY) interrupt source
ADC_IER_EOSMP EQU (0x00000002) ; ADC End of Sampling interrupt source
ADC_IER_EOC   EQU (0x00000004) ; ADC End of Regular Conversion interrupt source
ADC_IER_EOS   EQU (0x00000008) ; ADC End of Regular sequence of Conversions interrupt source
ADC_IER_OVR   EQU (0x00000010) ; ADC overrun interrupt source
ADC_IER_JEOC  EQU (0x00000020) ; ADC End of Injected Conversion interrupt source
ADC_IER_JEOS  EQU (0x00000040) ; ADC End of Injected sequence of Conversions interrupt source
ADC_IER_AWD1  EQU (0x00000080) ; ADC Analog watchdog 1 interrupt source
ADC_IER_AWD2  EQU (0x00000100) ; ADC Analog watchdog 2 interrupt source
ADC_IER_AWD3  EQU (0x00000200) ; ADC Analog watchdog 3 interrupt source
ADC_IER_JQOVF EQU (0x00000400) ; ADC Injected Context Queue Overflow interrupt source

; ********************  Bit definition for ADC_CR register  *******************
ADC_CR_ADEN   EQU (0x00000001) ; ADC Enable control
ADC_CR_ADDIS  EQU (0x00000002) ; ADC Disable command
ADC_CR_ADSTART        EQU (0x00000004) ; ADC Start of Regular conversion
ADC_CR_JADSTART       EQU (0x00000008) ; ADC Start of injected conversion
ADC_CR_ADSTP  EQU (0x00000010) ; ADC Stop of Regular conversion
ADC_CR_JADSTP EQU (0x00000020) ; ADC Stop of injected conversion
ADC_CR_ADVREGEN       EQU (0x10000000) ; ADC Voltage regulator Enable
ADC_CR_DEEPPWD        EQU (0x20000000) ; ADC Deep power down Enable
ADC_CR_ADCALDIF       EQU (0x40000000) ; ADC Differential Mode for calibration
ADC_CR_ADCAL  EQU (0x80000000) ; ADC Calibration

; ********************  Bit definition for ADC_CFGR register  *******************
ADC_CFGR_DMAEN        EQU (0x00000001) ; ADC DMA Enable
ADC_CFGR_DMACFG       EQU (0x00000002) ; ADC DMA configuration

ADC_CFGR_RES  EQU (0x00000018) ; ADC Data resolution
ADC_CFGR_RES_0        EQU (0x00000008) ; ADC RES bit 0
ADC_CFGR_RES_1        EQU (0x00000010) ; ADC RES bit 1

ADC_CFGR_ALIGN        EQU (0x00000020) ; ADC Data Alignement

ADC_CFGR_EXTSEL       EQU (0x000003C0) ; ADC External trigger selection for regular group
ADC_CFGR_EXTSEL_0     EQU (0x00000040) ; ADC EXTSEL bit 0
ADC_CFGR_EXTSEL_1     EQU (0x00000080) ; ADC EXTSEL bit 1
ADC_CFGR_EXTSEL_2     EQU (0x00000100) ; ADC EXTSEL bit 2
ADC_CFGR_EXTSEL_3     EQU (0x00000200) ; ADC EXTSEL bit 3

ADC_CFGR_EXTEN        EQU (0x00000C00) ; ADC External trigger enable and polarity selection for regular channels
ADC_CFGR_EXTEN_0      EQU (0x00000400) ; ADC EXTEN bit 0
ADC_CFGR_EXTEN_1      EQU (0x00000800) ; ADC EXTEN bit 1

ADC_CFGR_OVRMOD       EQU (0x00001000) ; ADC overrun mode
ADC_CFGR_CONT EQU (0x00002000) ; ADC Single/continuous conversion mode for regular conversion
ADC_CFGR_AUTDLY       EQU (0x00004000) ; ADC Delayed conversion mode

ADC_CFGR_DISCEN       EQU (0x00010000) ; ADC Discontinuous mode for regular channels

ADC_CFGR_DISCNUM      EQU (0x000E0000) ; ADC Discontinuous mode channel count
ADC_CFGR_DISCNUM_0    EQU (0x00020000) ; ADC DISCNUM bit 0
ADC_CFGR_DISCNUM_1    EQU (0x00040000) ; ADC DISCNUM bit 1
ADC_CFGR_DISCNUM_2    EQU (0x00080000) ; ADC DISCNUM bit 2

ADC_CFGR_JDISCEN      EQU (0x00100000) ; ADC Discontinuous mode on injected channels
ADC_CFGR_JQM  EQU (0x00200000) ; ADC JSQR Queue mode
ADC_CFGR_AWD1SGL      EQU (0x00400000) ; Enable the watchdog 1 on a single channel or on all channels
ADC_CFGR_AWD1EN       EQU (0x00800000) ; ADC Analog watchdog 1 enable on regular Channels
ADC_CFGR_JAWD1EN      EQU (0x01000000) ; ADC Analog watchdog 1 enable on injected Channels
ADC_CFGR_JAUTO        EQU (0x02000000) ; ADC Automatic injected group conversion

ADC_CFGR_AWD1CH       EQU (0x7C000000) ; ADC Analog watchdog 1 Channel selection
ADC_CFGR_AWD1CH_0     EQU (0x04000000) ; ADC AWD1CH bit 0
ADC_CFGR_AWD1CH_1     EQU (0x08000000) ; ADC AWD1CH bit 1
ADC_CFGR_AWD1CH_2     EQU (0x10000000) ; ADC AWD1CH bit 2
ADC_CFGR_AWD1CH_3     EQU (0x20000000) ; ADC AWD1CH bit 3
ADC_CFGR_AWD1CH_4     EQU (0x40000000) ; ADC AWD1CH bit 4

ADC_CFGR_JQDIS        EQU (0x80000000) ; ADC Injected queue disable

; ********************  Bit definition for ADC_CFGR2 register  *******************
ADC_CFGR2_ROVSE       EQU (0x00000001) ; ADC Regular group oversampler enable
ADC_CFGR2_JOVSE       EQU (0x00000002) ; ADC Injected group oversampler enable

ADC_CFGR2_OVSR        EQU (0x0000001C) ; ADC Regular group oversampler enable
ADC_CFGR2_OVSR_0      EQU (0x00000004) ; ADC OVSR bit 0
ADC_CFGR2_OVSR_1      EQU (0x00000008) ; ADC OVSR bit 1
ADC_CFGR2_OVSR_2      EQU (0x00000010) ; ADC OVSR bit 2

ADC_CFGR2_OVSS        EQU (0x000001E0) ; ADC Regular Oversampling shift
ADC_CFGR2_OVSS_0      EQU (0x00000020) ; ADC OVSS bit 0
ADC_CFGR2_OVSS_1      EQU (0x00000040) ; ADC OVSS bit 1
ADC_CFGR2_OVSS_2      EQU (0x00000080) ; ADC OVSS bit 2
ADC_CFGR2_OVSS_3      EQU (0x00000100) ; ADC OVSS bit 3

ADC_CFGR2_TROVS       EQU (0x00000200) ; ADC Triggered regular Oversampling
ADC_CFGR2_ROVSM       EQU (0x00000400) ; ADC Regular oversampling mode

; ********************  Bit definition for ADC_SMPR1 register  *******************
ADC_SMPR1_SMP0        EQU (0x00000007) ; ADC Channel 0 Sampling time selection
ADC_SMPR1_SMP0_0      EQU (0x00000001) ; ADC SMP0 bit 0
ADC_SMPR1_SMP0_1      EQU (0x00000002) ; ADC SMP0 bit 1
ADC_SMPR1_SMP0_2      EQU (0x00000004) ; ADC SMP0 bit 2

ADC_SMPR1_SMP1        EQU (0x00000038) ; ADC Channel 1 Sampling time selection
ADC_SMPR1_SMP1_0      EQU (0x00000008) ; ADC SMP1 bit 0
ADC_SMPR1_SMP1_1      EQU (0x00000010) ; ADC SMP1 bit 1
ADC_SMPR1_SMP1_2      EQU (0x00000020) ; ADC SMP1 bit 2

ADC_SMPR1_SMP2        EQU (0x000001C0) ; ADC Channel 2 Sampling time selection
ADC_SMPR1_SMP2_0      EQU (0x00000040) ; ADC SMP2 bit 0
ADC_SMPR1_SMP2_1      EQU (0x00000080) ; ADC SMP2 bit 1
ADC_SMPR1_SMP2_2      EQU (0x00000100) ; ADC SMP2 bit 2

ADC_SMPR1_SMP3        EQU (0x00000E00) ; ADC Channel 3 Sampling time selection
ADC_SMPR1_SMP3_0      EQU (0x00000200) ; ADC SMP3 bit 0
ADC_SMPR1_SMP3_1      EQU (0x00000400) ; ADC SMP3 bit 1
ADC_SMPR1_SMP3_2      EQU (0x00000800) ; ADC SMP3 bit 2

ADC_SMPR1_SMP4        EQU (0x00007000) ; ADC Channel 4 Sampling time selection
ADC_SMPR1_SMP4_0      EQU (0x00001000) ; ADC SMP4 bit 0
ADC_SMPR1_SMP4_1      EQU (0x00002000) ; ADC SMP4 bit 1
ADC_SMPR1_SMP4_2      EQU (0x00004000) ; ADC SMP4 bit 2

ADC_SMPR1_SMP5        EQU (0x00038000) ; ADC Channel 5 Sampling time selection
ADC_SMPR1_SMP5_0      EQU (0x00008000) ; ADC SMP5 bit 0
ADC_SMPR1_SMP5_1      EQU (0x00010000) ; ADC SMP5 bit 1
ADC_SMPR1_SMP5_2      EQU (0x00020000) ; ADC SMP5 bit 2

ADC_SMPR1_SMP6        EQU (0x001C0000) ; ADC Channel 6 Sampling time selection
ADC_SMPR1_SMP6_0      EQU (0x00040000) ; ADC SMP6 bit 0
ADC_SMPR1_SMP6_1      EQU (0x00080000) ; ADC SMP6 bit 1
ADC_SMPR1_SMP6_2      EQU (0x00100000) ; ADC SMP6 bit 2

ADC_SMPR1_SMP7        EQU (0x00E00000) ; ADC Channel 7 Sampling time selection
ADC_SMPR1_SMP7_0      EQU (0x00200000) ; ADC SMP7 bit 0
ADC_SMPR1_SMP7_1      EQU (0x00400000) ; ADC SMP7 bit 1
ADC_SMPR1_SMP7_2      EQU (0x00800000) ; ADC SMP7 bit 2

ADC_SMPR1_SMP8        EQU (0x07000000) ; ADC Channel 8 Sampling time selection
ADC_SMPR1_SMP8_0      EQU (0x01000000) ; ADC SMP8 bit 0
ADC_SMPR1_SMP8_1      EQU (0x02000000) ; ADC SMP8 bit 1
ADC_SMPR1_SMP8_2      EQU (0x04000000) ; ADC SMP8 bit 2

ADC_SMPR1_SMP9        EQU (0x38000000) ; ADC Channel 9 Sampling time selection
ADC_SMPR1_SMP9_0      EQU (0x08000000) ; ADC SMP9 bit 0
ADC_SMPR1_SMP9_1      EQU (0x10000000) ; ADC SMP9 bit 1
ADC_SMPR1_SMP9_2      EQU (0x20000000) ; ADC SMP9 bit 2

; ********************  Bit definition for ADC_SMPR2 register  *******************
ADC_SMPR2_SMP10     EQU (0x00000007) ; ADC Channel 10 Sampling time selection
ADC_SMPR2_SMP10_0   EQU (0x00000001) ; ADC SMP10 bit 0
ADC_SMPR2_SMP10_1   EQU (0x00000002) ; ADC SMP10 bit 1
ADC_SMPR2_SMP10_2   EQU (0x00000004) ; ADC SMP10 bit 2

ADC_SMPR2_SMP11     EQU (0x00000038) ; ADC Channel 11 Sampling time selection
ADC_SMPR2_SMP11_0   EQU (0x00000008) ; ADC SMP11 bit 0
ADC_SMPR2_SMP11_1   EQU (0x00000010) ; ADC SMP11 bit 1
ADC_SMPR2_SMP11_2   EQU (0x00000020) ; ADC SMP11 bit 2

ADC_SMPR2_SMP12     EQU (0x000001C0) ; ADC Channel 12 Sampling time selection
ADC_SMPR2_SMP12_0   EQU (0x00000040) ; ADC SMP12 bit 0
ADC_SMPR2_SMP12_1   EQU (0x00000080) ; ADC SMP12 bit 1
ADC_SMPR2_SMP12_2   EQU (0x00000100) ; ADC SMP12 bit 2

ADC_SMPR2_SMP13     EQU (0x00000E00) ; ADC Channel 13 Sampling time selection
ADC_SMPR2_SMP13_0   EQU (0x00000200) ; ADC SMP13 bit 0
ADC_SMPR2_SMP13_1   EQU (0x00000400) ; ADC SMP13 bit 1
ADC_SMPR2_SMP13_2   EQU (0x00000800) ; ADC SMP13 bit 2

ADC_SMPR2_SMP14     EQU (0x00007000) ; ADC Channel 14 Sampling time selection
ADC_SMPR2_SMP14_0   EQU (0x00001000) ; ADC SMP14 bit 0
ADC_SMPR2_SMP14_1   EQU (0x00002000) ; ADC SMP14 bit 1
ADC_SMPR2_SMP14_2   EQU (0x00004000) ; ADC SMP14 bit 2

ADC_SMPR2_SMP15     EQU (0x00038000) ; ADC Channel 15 Sampling time selection
ADC_SMPR2_SMP15_0   EQU (0x00008000) ; ADC SMP15 bit 0
ADC_SMPR2_SMP15_1   EQU (0x00010000) ; ADC SMP15 bit 1
ADC_SMPR2_SMP15_2   EQU (0x00020000) ; ADC SMP15 bit 2

ADC_SMPR2_SMP16     EQU (0x001C0000) ; ADC Channel 16 Sampling time selection
ADC_SMPR2_SMP16_0   EQU (0x00040000) ; ADC SMP16 bit 0
ADC_SMPR2_SMP16_1   EQU (0x00080000) ; ADC SMP16 bit 1
ADC_SMPR2_SMP16_2   EQU (0x00100000) ; ADC SMP16 bit 2

ADC_SMPR2_SMP17     EQU (0x00E00000) ; ADC Channel 17 Sampling time selection
ADC_SMPR2_SMP17_0   EQU (0x00200000) ; ADC SMP17 bit 0
ADC_SMPR2_SMP17_1   EQU (0x00400000) ; ADC SMP17 bit 1
ADC_SMPR2_SMP17_2   EQU (0x00800000) ; ADC SMP17 bit 2

ADC_SMPR2_SMP18     EQU (0x07000000) ; ADC Channel 18 Sampling time selection
ADC_SMPR2_SMP18_0   EQU (0x01000000) ; ADC SMP18 bit 0
ADC_SMPR2_SMP18_1   EQU (0x02000000) ; ADC SMP18 bit 1
ADC_SMPR2_SMP18_2   EQU (0x04000000) ; ADC SMP18 bit 2

; ********************  Bit definition for ADC_TR1 register  *******************
ADC_TR1_LT1 EQU (0x00000FFF) ; ADC Analog watchdog 1 lower threshold
ADC_TR1_LT1_0       EQU (0x00000001) ; ADC LT1 bit 0
ADC_TR1_LT1_1       EQU (0x00000002) ; ADC LT1 bit 1
ADC_TR1_LT1_2       EQU (0x00000004) ; ADC LT1 bit 2
ADC_TR1_LT1_3       EQU (0x00000008) ; ADC LT1 bit 3
ADC_TR1_LT1_4       EQU (0x00000010) ; ADC LT1 bit 4
ADC_TR1_LT1_5       EQU (0x00000020) ; ADC LT1 bit 5
ADC_TR1_LT1_6       EQU (0x00000040) ; ADC LT1 bit 6
ADC_TR1_LT1_7       EQU (0x00000080) ; ADC LT1 bit 7
ADC_TR1_LT1_8       EQU (0x00000100) ; ADC LT1 bit 8
ADC_TR1_LT1_9       EQU (0x00000200) ; ADC LT1 bit 9
ADC_TR1_LT1_10      EQU (0x00000400) ; ADC LT1 bit 10
ADC_TR1_LT1_11      EQU (0x00000800) ; ADC LT1 bit 11

ADC_TR1_HT1 EQU (0x0FFF0000) ; ADC Analog watchdog 1 higher threshold
ADC_TR1_HT1_0       EQU (0x00010000) ; ADC HT1 bit 0
ADC_TR1_HT1_1       EQU (0x00020000) ; ADC HT1 bit 1
ADC_TR1_HT1_2       EQU (0x00040000) ; ADC HT1 bit 2
ADC_TR1_HT1_3       EQU (0x00080000) ; ADC HT1 bit 3
ADC_TR1_HT1_4       EQU (0x00100000) ; ADC HT1 bit 4
ADC_TR1_HT1_5       EQU (0x00200000) ; ADC HT1 bit 5
ADC_TR1_HT1_6       EQU (0x00400000) ; ADC HT1 bit 6
ADC_TR1_HT1_7       EQU (0x00800000) ; ADC HT1 bit 7
ADC_TR1_HT1_8       EQU (0x01000000) ; ADC HT1 bit 8
ADC_TR1_HT1_9       EQU (0x02000000) ; ADC HT1 bit 9
ADC_TR1_HT1_10      EQU (0x04000000) ; ADC HT1 bit 10
ADC_TR1_HT1_11      EQU (0x08000000) ; ADC HT1 bit 11

; ********************  Bit definition for ADC_TR2 register  *******************
ADC_TR2_LT2 EQU (0x000000FF) ; ADC Analog watchdog 2 lower threshold
ADC_TR2_LT2_0       EQU (0x00000001) ; ADC LT2 bit 0
ADC_TR2_LT2_1       EQU (0x00000002) ; ADC LT2 bit 1
ADC_TR2_LT2_2       EQU (0x00000004) ; ADC LT2 bit 2
ADC_TR2_LT2_3       EQU (0x00000008) ; ADC LT2 bit 3
ADC_TR2_LT2_4       EQU (0x00000010) ; ADC LT2 bit 4
ADC_TR2_LT2_5       EQU (0x00000020) ; ADC LT2 bit 5
ADC_TR2_LT2_6       EQU (0x00000040) ; ADC LT2 bit 6
ADC_TR2_LT2_7       EQU (0x00000080) ; ADC LT2 bit 7

ADC_TR2_HT2 EQU (0x00FF0000) ; ADC Analog watchdog 2 higher threshold
ADC_TR2_HT2_0       EQU (0x00010000) ; ADC HT2 bit 0
ADC_TR2_HT2_1       EQU (0x00020000) ; ADC HT2 bit 1
ADC_TR2_HT2_2       EQU (0x00040000) ; ADC HT2 bit 2
ADC_TR2_HT2_3       EQU (0x00080000) ; ADC HT2 bit 3
ADC_TR2_HT2_4       EQU (0x00100000) ; ADC HT2 bit 4
ADC_TR2_HT2_5       EQU (0x00200000) ; ADC HT2 bit 5
ADC_TR2_HT2_6       EQU (0x00400000) ; ADC HT2 bit 6
ADC_TR2_HT2_7       EQU (0x00800000) ; ADC HT2 bit 7

; ********************  Bit definition for ADC_TR3 register  *******************
ADC_TR3_LT3 EQU (0x000000FF) ; ADC Analog watchdog 3 lower threshold
ADC_TR3_LT3_0       EQU (0x00000001) ; ADC LT3 bit 0
ADC_TR3_LT3_1       EQU (0x00000002) ; ADC LT3 bit 1
ADC_TR3_LT3_2       EQU (0x00000004) ; ADC LT3 bit 2
ADC_TR3_LT3_3       EQU (0x00000008) ; ADC LT3 bit 3
ADC_TR3_LT3_4       EQU (0x00000010) ; ADC LT3 bit 4
ADC_TR3_LT3_5       EQU (0x00000020) ; ADC LT3 bit 5
ADC_TR3_LT3_6       EQU (0x00000040) ; ADC LT3 bit 6
ADC_TR3_LT3_7       EQU (0x00000080) ; ADC LT3 bit 7

ADC_TR3_HT3 EQU (0x00FF0000) ; ADC Analog watchdog 3 higher threshold
ADC_TR3_HT3_0       EQU (0x00010000) ; ADC HT3 bit 0
ADC_TR3_HT3_1       EQU (0x00020000) ; ADC HT3 bit 1
ADC_TR3_HT3_2       EQU (0x00040000) ; ADC HT3 bit 2
ADC_TR3_HT3_3       EQU (0x00080000) ; ADC HT3 bit 3
ADC_TR3_HT3_4       EQU (0x00100000) ; ADC HT3 bit 4
ADC_TR3_HT3_5       EQU (0x00200000) ; ADC HT3 bit 5
ADC_TR3_HT3_6       EQU (0x00400000) ; ADC HT3 bit 6
ADC_TR3_HT3_7       EQU (0x00800000) ; ADC HT3 bit 7

; ********************  Bit definition for ADC_SQR1 register  *******************
ADC_SQR1_L  EQU (0x0000000F) ; ADC regular channel sequence lenght
ADC_SQR1_L_0        EQU (0x00000001) ; ADC L bit 0
ADC_SQR1_L_1        EQU (0x00000002) ; ADC L bit 1
ADC_SQR1_L_2        EQU (0x00000004) ; ADC L bit 2
ADC_SQR1_L_3        EQU (0x00000008) ; ADC L bit 3

ADC_SQR1_SQ1        EQU (0x000007C0) ; ADC 1st conversion in regular sequence
ADC_SQR1_SQ1_0      EQU (0x00000040) ; ADC SQ1 bit 0
ADC_SQR1_SQ1_1      EQU (0x00000080) ; ADC SQ1 bit 1
ADC_SQR1_SQ1_2      EQU (0x00000100) ; ADC SQ1 bit 2
ADC_SQR1_SQ1_3      EQU (0x00000200) ; ADC SQ1 bit 3
ADC_SQR1_SQ1_4      EQU (0x00000400) ; ADC SQ1 bit 4

ADC_SQR1_SQ2        EQU (0x0001F000) ; ADC 2nd conversion in regular sequence
ADC_SQR1_SQ2_0      EQU (0x00001000) ; ADC SQ2 bit 0
ADC_SQR1_SQ2_1      EQU (0x00002000) ; ADC SQ2 bit 1
ADC_SQR1_SQ2_2      EQU (0x00004000) ; ADC SQ2 bit 2
ADC_SQR1_SQ2_3      EQU (0x00008000) ; ADC SQ2 bit 3
ADC_SQR1_SQ2_4      EQU (0x00010000) ; ADC SQ2 bit 4

ADC_SQR1_SQ3        EQU (0x007C0000) ; ADC 3rd conversion in regular sequence
ADC_SQR1_SQ3_0      EQU (0x00040000) ; ADC SQ3 bit 0
ADC_SQR1_SQ3_1      EQU (0x00080000) ; ADC SQ3 bit 1
ADC_SQR1_SQ3_2      EQU (0x00100000) ; ADC SQ3 bit 2
ADC_SQR1_SQ3_3      EQU (0x00200000) ; ADC SQ3 bit 3
ADC_SQR1_SQ3_4      EQU (0x00400000) ; ADC SQ3 bit 4

ADC_SQR1_SQ4        EQU (0x1F000000) ; ADC 4th conversion in regular sequence
ADC_SQR1_SQ4_0      EQU (0x01000000) ; ADC SQ4 bit 0
ADC_SQR1_SQ4_1      EQU (0x02000000) ; ADC SQ4 bit 1
ADC_SQR1_SQ4_2      EQU (0x04000000) ; ADC SQ4 bit 2
ADC_SQR1_SQ4_3      EQU (0x08000000) ; ADC SQ4 bit 3
ADC_SQR1_SQ4_4      EQU (0x10000000) ; ADC SQ4 bit 4

; ********************  Bit definition for ADC_SQR2 register  *******************
ADC_SQR2_SQ5        EQU (0x0000001F) ; ADC 5th conversion in regular sequence
ADC_SQR2_SQ5_0      EQU (0x00000001) ; ADC SQ5 bit 0
ADC_SQR2_SQ5_1      EQU (0x00000002) ; ADC SQ5 bit 1
ADC_SQR2_SQ5_2      EQU (0x00000004) ; ADC SQ5 bit 2
ADC_SQR2_SQ5_3      EQU (0x00000008) ; ADC SQ5 bit 3
ADC_SQR2_SQ5_4      EQU (0x00000010) ; ADC SQ5 bit 4

ADC_SQR2_SQ6        EQU (0x000007C0) ; ADC 6th conversion in regular sequence
ADC_SQR2_SQ6_0      EQU (0x00000040) ; ADC SQ6 bit 0
ADC_SQR2_SQ6_1      EQU (0x00000080) ; ADC SQ6 bit 1
ADC_SQR2_SQ6_2      EQU (0x00000100) ; ADC SQ6 bit 2
ADC_SQR2_SQ6_3      EQU (0x00000200) ; ADC SQ6 bit 3
ADC_SQR2_SQ6_4      EQU (0x00000400) ; ADC SQ6 bit 4

ADC_SQR2_SQ7        EQU (0x0001F000) ; ADC 7th conversion in regular sequence
ADC_SQR2_SQ7_0      EQU (0x00001000) ; ADC SQ7 bit 0
ADC_SQR2_SQ7_1      EQU (0x00002000) ; ADC SQ7 bit 1
ADC_SQR2_SQ7_2      EQU (0x00004000) ; ADC SQ7 bit 2
ADC_SQR2_SQ7_3      EQU (0x00008000) ; ADC SQ7 bit 3
ADC_SQR2_SQ7_4      EQU (0x00010000) ; ADC SQ7 bit 4

ADC_SQR2_SQ8        EQU (0x007C0000) ; ADC 8th conversion in regular sequence
ADC_SQR2_SQ8_0      EQU (0x00040000) ; ADC SQ8 bit 0
ADC_SQR2_SQ8_1      EQU (0x00080000) ; ADC SQ8 bit 1
ADC_SQR2_SQ8_2      EQU (0x00100000) ; ADC SQ8 bit 2
ADC_SQR2_SQ8_3      EQU (0x00200000) ; ADC SQ8 bit 3
ADC_SQR2_SQ8_4      EQU (0x00400000) ; ADC SQ8 bit 4

ADC_SQR2_SQ9        EQU (0x1F000000) ; ADC 9th conversion in regular sequence
ADC_SQR2_SQ9_0      EQU (0x01000000) ; ADC SQ9 bit 0
ADC_SQR2_SQ9_1      EQU (0x02000000) ; ADC SQ9 bit 1
ADC_SQR2_SQ9_2      EQU (0x04000000) ; ADC SQ9 bit 2
ADC_SQR2_SQ9_3      EQU (0x08000000) ; ADC SQ9 bit 3
ADC_SQR2_SQ9_4      EQU (0x10000000) ; ADC SQ9 bit 4

; ********************  Bit definition for ADC_SQR3 register  *******************
ADC_SQR3_SQ10       EQU (0x0000001F) ; ADC 10th conversion in regular sequence
ADC_SQR3_SQ10_0     EQU (0x00000001) ; ADC SQ10 bit 0
ADC_SQR3_SQ10_1     EQU (0x00000002) ; ADC SQ10 bit 1
ADC_SQR3_SQ10_2     EQU (0x00000004) ; ADC SQ10 bit 2
ADC_SQR3_SQ10_3     EQU (0x00000008) ; ADC SQ10 bit 3
ADC_SQR3_SQ10_4     EQU (0x00000010) ; ADC SQ10 bit 4

ADC_SQR3_SQ11       EQU (0x000007C0) ; ADC 11th conversion in regular sequence
ADC_SQR3_SQ11_0     EQU (0x00000040) ; ADC SQ11 bit 0
ADC_SQR3_SQ11_1     EQU (0x00000080) ; ADC SQ11 bit 1
ADC_SQR3_SQ11_2     EQU (0x00000100) ; ADC SQ11 bit 2
ADC_SQR3_SQ11_3     EQU (0x00000200) ; ADC SQ11 bit 3
ADC_SQR3_SQ11_4     EQU (0x00000400) ; ADC SQ11 bit 4

ADC_SQR3_SQ12       EQU (0x0001F000) ; ADC 12th conversion in regular sequence
ADC_SQR3_SQ12_0     EQU (0x00001000) ; ADC SQ12 bit 0
ADC_SQR3_SQ12_1     EQU (0x00002000) ; ADC SQ12 bit 1
ADC_SQR3_SQ12_2     EQU (0x00004000) ; ADC SQ12 bit 2
ADC_SQR3_SQ12_3     EQU (0x00008000) ; ADC SQ12 bit 3
ADC_SQR3_SQ12_4     EQU (0x00010000) ; ADC SQ12 bit 4

ADC_SQR3_SQ13       EQU (0x007C0000) ; ADC 13th conversion in regular sequence
ADC_SQR3_SQ13_0     EQU (0x00040000) ; ADC SQ13 bit 0
ADC_SQR3_SQ13_1     EQU (0x00080000) ; ADC SQ13 bit 1
ADC_SQR3_SQ13_2     EQU (0x00100000) ; ADC SQ13 bit 2
ADC_SQR3_SQ13_3     EQU (0x00200000) ; ADC SQ13 bit 3
ADC_SQR3_SQ13_4     EQU (0x00400000) ; ADC SQ13 bit 4

ADC_SQR3_SQ14       EQU (0x1F000000) ; ADC 14th conversion in regular sequence
ADC_SQR3_SQ14_0     EQU (0x01000000) ; ADC SQ14 bit 0
ADC_SQR3_SQ14_1     EQU (0x02000000) ; ADC SQ14 bit 1
ADC_SQR3_SQ14_2     EQU (0x04000000) ; ADC SQ14 bit 2
ADC_SQR3_SQ14_3     EQU (0x08000000) ; ADC SQ14 bit 3
ADC_SQR3_SQ14_4     EQU (0x10000000) ; ADC SQ14 bit 4

; ********************  Bit definition for ADC_SQR4 register  *******************
ADC_SQR4_SQ15       EQU (0x0000001F) ; ADC 15th conversion in regular sequence
ADC_SQR4_SQ15_0     EQU (0x00000001) ; ADC SQ15 bit 0
ADC_SQR4_SQ15_1     EQU (0x00000002) ; ADC SQ15 bit 1
ADC_SQR4_SQ15_2     EQU (0x00000004) ; ADC SQ15 bit 2
ADC_SQR4_SQ15_3     EQU (0x00000008) ; ADC SQ15 bit 3
ADC_SQR4_SQ15_4     EQU (0x00000010) ; ADC SQ105 bit 4

ADC_SQR4_SQ16       EQU (0x000007C0) ; ADC 16th conversion in regular sequence
ADC_SQR4_SQ16_0     EQU (0x00000040) ; ADC SQ16 bit 0
ADC_SQR4_SQ16_1     EQU (0x00000080) ; ADC SQ16 bit 1
ADC_SQR4_SQ16_2     EQU (0x00000100) ; ADC SQ16 bit 2
ADC_SQR4_SQ16_3     EQU (0x00000200) ; ADC SQ16 bit 3
ADC_SQR4_SQ16_4     EQU (0x00000400) ; ADC SQ16 bit 4

; ********************  Bit definition for ADC_DR register  *******************
ADC_DR_RDATA        EQU (0x0000FFFF) ; ADC regular Data converted
ADC_DR_RDATA_0      EQU (0x00000001) ; ADC RDATA bit 0
ADC_DR_RDATA_1      EQU (0x00000002) ; ADC RDATA bit 1
ADC_DR_RDATA_2      EQU (0x00000004) ; ADC RDATA bit 2
ADC_DR_RDATA_3      EQU (0x00000008) ; ADC RDATA bit 3
ADC_DR_RDATA_4      EQU (0x00000010) ; ADC RDATA bit 4
ADC_DR_RDATA_5      EQU (0x00000020) ; ADC RDATA bit 5
ADC_DR_RDATA_6      EQU (0x00000040) ; ADC RDATA bit 6
ADC_DR_RDATA_7      EQU (0x00000080) ; ADC RDATA bit 7
ADC_DR_RDATA_8      EQU (0x00000100) ; ADC RDATA bit 8
ADC_DR_RDATA_9      EQU (0x00000200) ; ADC RDATA bit 9
ADC_DR_RDATA_10     EQU (0x00000400) ; ADC RDATA bit 10
ADC_DR_RDATA_11     EQU (0x00000800) ; ADC RDATA bit 11
ADC_DR_RDATA_12     EQU (0x00001000) ; ADC RDATA bit 12
ADC_DR_RDATA_13     EQU (0x00002000) ; ADC RDATA bit 13
ADC_DR_RDATA_14     EQU (0x00004000) ; ADC RDATA bit 14
ADC_DR_RDATA_15     EQU (0x00008000) ; ADC RDATA bit 15

; ********************  Bit definition for ADC_JSQR register  *******************
ADC_JSQR_JL EQU (0x00000003) ; ADC injected channel sequence length
ADC_JSQR_JL_0       EQU (0x00000001) ; ADC JL bit 0
ADC_JSQR_JL_1       EQU (0x00000002) ; ADC JL bit 1

ADC_JSQR_JEXTSEL    EQU (0x0000003C) ; ADC external trigger selection for injected group
ADC_JSQR_JEXTSEL_0  EQU (0x00000004) ; ADC JEXTSEL bit 0
ADC_JSQR_JEXTSEL_1  EQU (0x00000008) ; ADC JEXTSEL bit 1
ADC_JSQR_JEXTSEL_2  EQU (0x00000010) ; ADC JEXTSEL bit 2
ADC_JSQR_JEXTSEL_3  EQU (0x00000020) ; ADC JEXTSEL bit 3

ADC_JSQR_JEXTEN     EQU (0x000000C0) ; ADC external trigger enable and polarity selection for injected channels
ADC_JSQR_JEXTEN_0   EQU (0x00000040) ; ADC JEXTEN bit 0
ADC_JSQR_JEXTEN_1   EQU (0x00000080) ; ADC JEXTEN bit 1

ADC_JSQR_JSQ1       EQU (0x00001F00) ; ADC 1st conversion in injected sequence
ADC_JSQR_JSQ1_0     EQU (0x00000100) ; ADC JSQ1 bit 0
ADC_JSQR_JSQ1_1     EQU (0x00000200) ; ADC JSQ1 bit 1
ADC_JSQR_JSQ1_2     EQU (0x00000400) ; ADC JSQ1 bit 2
ADC_JSQR_JSQ1_3     EQU (0x00000800) ; ADC JSQ1 bit 3
ADC_JSQR_JSQ1_4     EQU (0x00001000) ; ADC JSQ1 bit 4

ADC_JSQR_JSQ2       EQU (0x0007C000) ; ADC 2nd conversion in injected sequence
ADC_JSQR_JSQ2_0     EQU (0x00004000) ; ADC JSQ2 bit 0
ADC_JSQR_JSQ2_1     EQU (0x00008000) ; ADC JSQ2 bit 1
ADC_JSQR_JSQ2_2     EQU (0x00010000) ; ADC JSQ2 bit 2
ADC_JSQR_JSQ2_3     EQU (0x00020000) ; ADC JSQ2 bit 3
ADC_JSQR_JSQ2_4     EQU (0x00040000) ; ADC JSQ2 bit 4

ADC_JSQR_JSQ3       EQU (0x01F00000) ; ADC 3rd conversion in injected sequence
ADC_JSQR_JSQ3_0     EQU (0x00100000) ; ADC JSQ3 bit 0
ADC_JSQR_JSQ3_1     EQU (0x00200000) ; ADC JSQ3 bit 1
ADC_JSQR_JSQ3_2     EQU (0x00400000) ; ADC JSQ3 bit 2
ADC_JSQR_JSQ3_3     EQU (0x00800000) ; ADC JSQ3 bit 3
ADC_JSQR_JSQ3_4     EQU (0x01000000) ; ADC JSQ3 bit 4

ADC_JSQR_JSQ4       EQU (0x7C000000) ; ADC 4th conversion in injected sequence
ADC_JSQR_JSQ4_0     EQU (0x04000000) ; ADC JSQ4 bit 0
ADC_JSQR_JSQ4_1     EQU (0x08000000) ; ADC JSQ4 bit 1
ADC_JSQR_JSQ4_2     EQU (0x10000000) ; ADC JSQ4 bit 2
ADC_JSQR_JSQ4_3     EQU (0x20000000) ; ADC JSQ4 bit 3
ADC_JSQR_JSQ4_4     EQU (0x40000000) ; ADC JSQ4 bit 4


; ********************  Bit definition for ADC_OFR1 register  *******************
ADC_OFR1_OFFSET1    EQU (0x00000FFF) ; ADC data offset 1 for channel programmed into bits OFFSET1_CH[4:0]
ADC_OFR1_OFFSET1_0  EQU (0x00000001) ; ADC OFFSET1 bit 0
ADC_OFR1_OFFSET1_1  EQU (0x00000002) ; ADC OFFSET1 bit 1
ADC_OFR1_OFFSET1_2  EQU (0x00000004) ; ADC OFFSET1 bit 2
ADC_OFR1_OFFSET1_3  EQU (0x00000008) ; ADC OFFSET1 bit 3
ADC_OFR1_OFFSET1_4  EQU (0x00000010) ; ADC OFFSET1 bit 4
ADC_OFR1_OFFSET1_5  EQU (0x00000020) ; ADC OFFSET1 bit 5
ADC_OFR1_OFFSET1_6  EQU (0x00000040) ; ADC OFFSET1 bit 6
ADC_OFR1_OFFSET1_7  EQU (0x00000080) ; ADC OFFSET1 bit 7
ADC_OFR1_OFFSET1_8  EQU (0x00000100) ; ADC OFFSET1 bit 8
ADC_OFR1_OFFSET1_9  EQU (0x00000200) ; ADC OFFSET1 bit 9
ADC_OFR1_OFFSET1_10 EQU (0x00000400) ; ADC OFFSET1 bit 10
ADC_OFR1_OFFSET1_11 EQU (0x00000800) ; ADC OFFSET1 bit 11

ADC_OFR1_OFFSET1_CH    EQU (0x7C000000) ; ADC Channel selection for the data offset 1
ADC_OFR1_OFFSET1_CH_0  EQU (0x04000000) ; ADC OFFSET1_CH bit 0
ADC_OFR1_OFFSET1_CH_1  EQU (0x08000000) ; ADC OFFSET1_CH bit 1
ADC_OFR1_OFFSET1_CH_2  EQU (0x10000000) ; ADC OFFSET1_CH bit 2
ADC_OFR1_OFFSET1_CH_3  EQU (0x20000000) ; ADC OFFSET1_CH bit 3
ADC_OFR1_OFFSET1_CH_4  EQU (0x40000000) ; ADC OFFSET1_CH bit 4

ADC_OFR1_OFFSET1_EN    EQU (0x80000000) ; ADC offset 1 enable

; ********************  Bit definition for ADC_OFR2 register  *******************
ADC_OFR2_OFFSET2    EQU (0x00000FFF) ; ADC data offset 2 for channel programmed into bits OFFSET2_CH[4:0]
ADC_OFR2_OFFSET2_0  EQU (0x00000001) ; ADC OFFSET2 bit 0
ADC_OFR2_OFFSET2_1  EQU (0x00000002) ; ADC OFFSET2 bit 1
ADC_OFR2_OFFSET2_2  EQU (0x00000004) ; ADC OFFSET2 bit 2
ADC_OFR2_OFFSET2_3  EQU (0x00000008) ; ADC OFFSET2 bit 3
ADC_OFR2_OFFSET2_4  EQU (0x00000010) ; ADC OFFSET2 bit 4
ADC_OFR2_OFFSET2_5  EQU (0x00000020) ; ADC OFFSET2 bit 5
ADC_OFR2_OFFSET2_6  EQU (0x00000040) ; ADC OFFSET2 bit 6
ADC_OFR2_OFFSET2_7  EQU (0x00000080) ; ADC OFFSET2 bit 7
ADC_OFR2_OFFSET2_8  EQU (0x00000100) ; ADC OFFSET2 bit 8
ADC_OFR2_OFFSET2_9  EQU (0x00000200) ; ADC OFFSET2 bit 9
ADC_OFR2_OFFSET2_10 EQU (0x00000400) ; ADC OFFSET2 bit 10
ADC_OFR2_OFFSET2_11 EQU (0x00000800) ; ADC OFFSET2 bit 11

ADC_OFR2_OFFSET2_CH    EQU (0x7C000000) ; ADC Channel selection for the data offset 2
ADC_OFR2_OFFSET2_CH_0  EQU (0x04000000) ; ADC OFFSET2_CH bit 0
ADC_OFR2_OFFSET2_CH_1  EQU (0x08000000) ; ADC OFFSET2_CH bit 1
ADC_OFR2_OFFSET2_CH_2  EQU (0x10000000) ; ADC OFFSET2_CH bit 2
ADC_OFR2_OFFSET2_CH_3  EQU (0x20000000) ; ADC OFFSET2_CH bit 3
ADC_OFR2_OFFSET2_CH_4  EQU (0x40000000) ; ADC OFFSET2_CH bit 4

ADC_OFR2_OFFSET2_EN    EQU (0x80000000) ; ADC offset 2 enable

; ********************  Bit definition for ADC_OFR3 register  *******************
ADC_OFR3_OFFSET3    EQU (0x00000FFF) ; ADC data offset 3 for channel programmed into bits OFFSET3_CH[4:0]
ADC_OFR3_OFFSET3_0  EQU (0x00000001) ; ADC OFFSET3 bit 0
ADC_OFR3_OFFSET3_1  EQU (0x00000002) ; ADC OFFSET3 bit 1
ADC_OFR3_OFFSET3_2  EQU (0x00000004) ; ADC OFFSET3 bit 2
ADC_OFR3_OFFSET3_3  EQU (0x00000008) ; ADC OFFSET3 bit 3
ADC_OFR3_OFFSET3_4  EQU (0x00000010) ; ADC OFFSET3 bit 4
ADC_OFR3_OFFSET3_5  EQU (0x00000020) ; ADC OFFSET3 bit 5
ADC_OFR3_OFFSET3_6  EQU (0x00000040) ; ADC OFFSET3 bit 6
ADC_OFR3_OFFSET3_7  EQU (0x00000080) ; ADC OFFSET3 bit 7
ADC_OFR3_OFFSET3_8  EQU (0x00000100) ; ADC OFFSET3 bit 8
ADC_OFR3_OFFSET3_9  EQU (0x00000200) ; ADC OFFSET3 bit 9
ADC_OFR3_OFFSET3_10 EQU (0x00000400) ; ADC OFFSET3 bit 10
ADC_OFR3_OFFSET3_11 EQU (0x00000800) ; ADC OFFSET3 bit 11

ADC_OFR3_OFFSET3_CH    EQU (0x7C000000) ; ADC Channel selection for the data offset 3
ADC_OFR3_OFFSET3_CH_0  EQU (0x04000000) ; ADC OFFSET3_CH bit 0
ADC_OFR3_OFFSET3_CH_1  EQU (0x08000000) ; ADC OFFSET3_CH bit 1
ADC_OFR3_OFFSET3_CH_2  EQU (0x10000000) ; ADC OFFSET3_CH bit 2
ADC_OFR3_OFFSET3_CH_3  EQU (0x20000000) ; ADC OFFSET3_CH bit 3
ADC_OFR3_OFFSET3_CH_4  EQU (0x40000000) ; ADC OFFSET3_CH bit 4

ADC_OFR3_OFFSET3_EN    EQU (0x80000000) ; ADC offset 3 enable

; ********************  Bit definition for ADC_OFR4 register  *******************
ADC_OFR4_OFFSET4    EQU (0x00000FFF) ; ADC data offset 4 for channel programmed into bits OFFSET4_CH[4:0]
ADC_OFR4_OFFSET4_0  EQU (0x00000001) ; ADC OFFSET4 bit 0
ADC_OFR4_OFFSET4_1  EQU (0x00000002) ; ADC OFFSET4 bit 1
ADC_OFR4_OFFSET4_2  EQU (0x00000004) ; ADC OFFSET4 bit 2
ADC_OFR4_OFFSET4_3  EQU (0x00000008) ; ADC OFFSET4 bit 3
ADC_OFR4_OFFSET4_4  EQU (0x00000010) ; ADC OFFSET4 bit 4
ADC_OFR4_OFFSET4_5  EQU (0x00000020) ; ADC OFFSET4 bit 5
ADC_OFR4_OFFSET4_6  EQU (0x00000040) ; ADC OFFSET4 bit 6
ADC_OFR4_OFFSET4_7  EQU (0x00000080) ; ADC OFFSET4 bit 7
ADC_OFR4_OFFSET4_8  EQU (0x00000100) ; ADC OFFSET4 bit 8
ADC_OFR4_OFFSET4_9  EQU (0x00000200) ; ADC OFFSET4 bit 9
ADC_OFR4_OFFSET4_10 EQU (0x00000400) ; ADC OFFSET4 bit 10
ADC_OFR4_OFFSET4_11 EQU (0x00000800) ; ADC OFFSET4 bit 11

ADC_OFR4_OFFSET4_CH    EQU (0x7C000000) ; ADC Channel selection for the data offset 4
ADC_OFR4_OFFSET4_CH_0  EQU (0x04000000) ; ADC OFFSET4_CH bit 0
ADC_OFR4_OFFSET4_CH_1  EQU (0x08000000) ; ADC OFFSET4_CH bit 1
ADC_OFR4_OFFSET4_CH_2  EQU (0x10000000) ; ADC OFFSET4_CH bit 2
ADC_OFR4_OFFSET4_CH_3  EQU (0x20000000) ; ADC OFFSET4_CH bit 3
ADC_OFR4_OFFSET4_CH_4  EQU (0x40000000) ; ADC OFFSET4_CH bit 4

ADC_OFR4_OFFSET4_EN    EQU (0x80000000) ; ADC offset 4 enable

; ********************  Bit definition for ADC_JDR1 register  *******************
ADC_JDR1_JDATA      EQU (0x0000FFFF) ; ADC Injected DATA
ADC_JDR1_JDATA_0    EQU (0x00000001) ; ADC JDATA bit 0
ADC_JDR1_JDATA_1    EQU (0x00000002) ; ADC JDATA bit 1
ADC_JDR1_JDATA_2    EQU (0x00000004) ; ADC JDATA bit 2
ADC_JDR1_JDATA_3    EQU (0x00000008) ; ADC JDATA bit 3
ADC_JDR1_JDATA_4    EQU (0x00000010) ; ADC JDATA bit 4
ADC_JDR1_JDATA_5    EQU (0x00000020) ; ADC JDATA bit 5
ADC_JDR1_JDATA_6    EQU (0x00000040) ; ADC JDATA bit 6
ADC_JDR1_JDATA_7    EQU (0x00000080) ; ADC JDATA bit 7
ADC_JDR1_JDATA_8    EQU (0x00000100) ; ADC JDATA bit 8
ADC_JDR1_JDATA_9    EQU (0x00000200) ; ADC JDATA bit 9
ADC_JDR1_JDATA_10   EQU (0x00000400) ; ADC JDATA bit 10
ADC_JDR1_JDATA_11   EQU (0x00000800) ; ADC JDATA bit 11
ADC_JDR1_JDATA_12   EQU (0x00001000) ; ADC JDATA bit 12
ADC_JDR1_JDATA_13   EQU (0x00002000) ; ADC JDATA bit 13
ADC_JDR1_JDATA_14   EQU (0x00004000) ; ADC JDATA bit 14
ADC_JDR1_JDATA_15   EQU (0x00008000) ; ADC JDATA bit 15

; ********************  Bit definition for ADC_JDR2 register  *******************
ADC_JDR2_JDATA      EQU (0x0000FFFF) ; ADC Injected DATA
ADC_JDR2_JDATA_0    EQU (0x00000001) ; ADC JDATA bit 0
ADC_JDR2_JDATA_1    EQU (0x00000002) ; ADC JDATA bit 1
ADC_JDR2_JDATA_2    EQU (0x00000004) ; ADC JDATA bit 2
ADC_JDR2_JDATA_3    EQU (0x00000008) ; ADC JDATA bit 3
ADC_JDR2_JDATA_4    EQU (0x00000010) ; ADC JDATA bit 4
ADC_JDR2_JDATA_5    EQU (0x00000020) ; ADC JDATA bit 5
ADC_JDR2_JDATA_6    EQU (0x00000040) ; ADC JDATA bit 6
ADC_JDR2_JDATA_7    EQU (0x00000080) ; ADC JDATA bit 7
ADC_JDR2_JDATA_8    EQU (0x00000100) ; ADC JDATA bit 8
ADC_JDR2_JDATA_9    EQU (0x00000200) ; ADC JDATA bit 9
ADC_JDR2_JDATA_10   EQU (0x00000400) ; ADC JDATA bit 10
ADC_JDR2_JDATA_11   EQU (0x00000800) ; ADC JDATA bit 11
ADC_JDR2_JDATA_12   EQU (0x00001000) ; ADC JDATA bit 12
ADC_JDR2_JDATA_13   EQU (0x00002000) ; ADC JDATA bit 13
ADC_JDR2_JDATA_14   EQU (0x00004000) ; ADC JDATA bit 14
ADC_JDR2_JDATA_15   EQU (0x00008000) ; ADC JDATA bit 15

; ********************  Bit definition for ADC_JDR3 register  *******************
ADC_JDR3_JDATA      EQU (0x0000FFFF) ; ADC Injected DATA
ADC_JDR3_JDATA_0    EQU (0x00000001) ; ADC JDATA bit 0
ADC_JDR3_JDATA_1    EQU (0x00000002) ; ADC JDATA bit 1
ADC_JDR3_JDATA_2    EQU (0x00000004) ; ADC JDATA bit 2
ADC_JDR3_JDATA_3    EQU (0x00000008) ; ADC JDATA bit 3
ADC_JDR3_JDATA_4    EQU (0x00000010) ; ADC JDATA bit 4
ADC_JDR3_JDATA_5    EQU (0x00000020) ; ADC JDATA bit 5
ADC_JDR3_JDATA_6    EQU (0x00000040) ; ADC JDATA bit 6
ADC_JDR3_JDATA_7    EQU (0x00000080) ; ADC JDATA bit 7
ADC_JDR3_JDATA_8    EQU (0x00000100) ; ADC JDATA bit 8
ADC_JDR3_JDATA_9    EQU (0x00000200) ; ADC JDATA bit 9
ADC_JDR3_JDATA_10   EQU (0x00000400) ; ADC JDATA bit 10
ADC_JDR3_JDATA_11   EQU (0x00000800) ; ADC JDATA bit 11
ADC_JDR3_JDATA_12   EQU (0x00001000) ; ADC JDATA bit 12
ADC_JDR3_JDATA_13   EQU (0x00002000) ; ADC JDATA bit 13
ADC_JDR3_JDATA_14   EQU (0x00004000) ; ADC JDATA bit 14
ADC_JDR3_JDATA_15   EQU (0x00008000) ; ADC JDATA bit 15

; ********************  Bit definition for ADC_JDR4 register  *******************
ADC_JDR4_JDATA      EQU (0x0000FFFF) ; ADC Injected DATA
ADC_JDR4_JDATA_0    EQU (0x00000001) ; ADC JDATA bit 0
ADC_JDR4_JDATA_1    EQU (0x00000002) ; ADC JDATA bit 1
ADC_JDR4_JDATA_2    EQU (0x00000004) ; ADC JDATA bit 2
ADC_JDR4_JDATA_3    EQU (0x00000008) ; ADC JDATA bit 3
ADC_JDR4_JDATA_4    EQU (0x00000010) ; ADC JDATA bit 4
ADC_JDR4_JDATA_5    EQU (0x00000020) ; ADC JDATA bit 5
ADC_JDR4_JDATA_6    EQU (0x00000040) ; ADC JDATA bit 6
ADC_JDR4_JDATA_7    EQU (0x00000080) ; ADC JDATA bit 7
ADC_JDR4_JDATA_8    EQU (0x00000100) ; ADC JDATA bit 8
ADC_JDR4_JDATA_9    EQU (0x00000200) ; ADC JDATA bit 9
ADC_JDR4_JDATA_10   EQU (0x00000400) ; ADC JDATA bit 10
ADC_JDR4_JDATA_11   EQU (0x00000800) ; ADC JDATA bit 11
ADC_JDR4_JDATA_12   EQU (0x00001000) ; ADC JDATA bit 12
ADC_JDR4_JDATA_13   EQU (0x00002000) ; ADC JDATA bit 13
ADC_JDR4_JDATA_14   EQU (0x00004000) ; ADC JDATA bit 14
ADC_JDR4_JDATA_15   EQU (0x00008000) ; ADC JDATA bit 15

; ********************  Bit definition for ADC_AWD2CR register  *******************
ADC_AWD2CR_AWD2CH    EQU (0x0007FFFF) ; ADC Analog watchdog 2 channel selection
ADC_AWD2CR_AWD2CH_0  EQU (0x00000001) ; ADC AWD2CH bit 0
ADC_AWD2CR_AWD2CH_1  EQU (0x00000002) ; ADC AWD2CH bit 1
ADC_AWD2CR_AWD2CH_2  EQU (0x00000004) ; ADC AWD2CH bit 2
ADC_AWD2CR_AWD2CH_3  EQU (0x00000008) ; ADC AWD2CH bit 3
ADC_AWD2CR_AWD2CH_4  EQU (0x00000010) ; ADC AWD2CH bit 4
ADC_AWD2CR_AWD2CH_5  EQU (0x00000020) ; ADC AWD2CH bit 5
ADC_AWD2CR_AWD2CH_6  EQU (0x00000040) ; ADC AWD2CH bit 6
ADC_AWD2CR_AWD2CH_7  EQU (0x00000080) ; ADC AWD2CH bit 7
ADC_AWD2CR_AWD2CH_8  EQU (0x00000100) ; ADC AWD2CH bit 8
ADC_AWD2CR_AWD2CH_9  EQU (0x00000200) ; ADC AWD2CH bit 9
ADC_AWD2CR_AWD2CH_10 EQU (0x00000400) ; ADC AWD2CH bit 10
ADC_AWD2CR_AWD2CH_11 EQU (0x00000800) ; ADC AWD2CH bit 11
ADC_AWD2CR_AWD2CH_12 EQU (0x00001000) ; ADC AWD2CH bit 12
ADC_AWD2CR_AWD2CH_13 EQU (0x00002000) ; ADC AWD2CH bit 13
ADC_AWD2CR_AWD2CH_14 EQU (0x00004000) ; ADC AWD2CH bit 14
ADC_AWD2CR_AWD2CH_15 EQU (0x00008000) ; ADC AWD2CH bit 15
ADC_AWD2CR_AWD2CH_16 EQU (0x00010000) ; ADC AWD2CH bit 16
ADC_AWD2CR_AWD2CH_17 EQU (0x00020000) ; ADC AWD2CH bit 17
ADC_AWD2CR_AWD2CH_18 EQU (0x00040000) ; ADC AWD2CH bit 18

; ********************  Bit definition for ADC_AWD3CR register  *******************
ADC_AWD3CR_AWD3CH    EQU (0x0007FFFF) ; ADC Analog watchdog 3 channel selection
ADC_AWD3CR_AWD3CH_0  EQU (0x00000001) ; ADC AWD3CH bit 0
ADC_AWD3CR_AWD3CH_1  EQU (0x00000002) ; ADC AWD3CH bit 1
ADC_AWD3CR_AWD3CH_2  EQU (0x00000004) ; ADC AWD3CH bit 2
ADC_AWD3CR_AWD3CH_3  EQU (0x00000008) ; ADC AWD3CH bit 3
ADC_AWD3CR_AWD3CH_4  EQU (0x00000010) ; ADC AWD3CH bit 4
ADC_AWD3CR_AWD3CH_5  EQU (0x00000020) ; ADC AWD3CH bit 5
ADC_AWD3CR_AWD3CH_6  EQU (0x00000040) ; ADC AWD3CH bit 6
ADC_AWD3CR_AWD3CH_7  EQU (0x00000080) ; ADC AWD3CH bit 7
ADC_AWD3CR_AWD3CH_8  EQU (0x00000100) ; ADC AWD3CH bit 8
ADC_AWD3CR_AWD3CH_9  EQU (0x00000200) ; ADC AWD3CH bit 9
ADC_AWD3CR_AWD3CH_10 EQU (0x00000400) ; ADC AWD3CH bit 10
ADC_AWD3CR_AWD3CH_11 EQU (0x00000800) ; ADC AWD3CH bit 11
ADC_AWD3CR_AWD3CH_12 EQU (0x00001000) ; ADC AWD3CH bit 12
ADC_AWD3CR_AWD3CH_13 EQU (0x00002000) ; ADC AWD3CH bit 13
ADC_AWD3CR_AWD3CH_14 EQU (0x00004000) ; ADC AWD3CH bit 14
ADC_AWD3CR_AWD3CH_15 EQU (0x00008000) ; ADC AWD3CH bit 15
ADC_AWD3CR_AWD3CH_16 EQU (0x00010000) ; ADC AWD3CH bit 16
ADC_AWD3CR_AWD3CH_17 EQU (0x00020000) ; ADC AWD3CH bit 17
ADC_AWD3CR_AWD3CH_18 EQU (0x00040000) ; ADC AWD3CH bit 18

; ********************  Bit definition for ADC_DIFSEL register  *******************
ADC_DIFSEL_DIFSEL    EQU (0x0007FFFF) ; ADC differential modes for channels 1 to 18
ADC_DIFSEL_DIFSEL_0  EQU (0x00000001) ; ADC DIFSEL bit 0
ADC_DIFSEL_DIFSEL_1  EQU (0x00000002) ; ADC DIFSEL bit 1
ADC_DIFSEL_DIFSEL_2  EQU (0x00000004) ; ADC DIFSEL bit 2
ADC_DIFSEL_DIFSEL_3  EQU (0x00000008) ; ADC DIFSEL bit 3
ADC_DIFSEL_DIFSEL_4  EQU (0x00000010) ; ADC DIFSEL bit 4
ADC_DIFSEL_DIFSEL_5  EQU (0x00000020) ; ADC DIFSEL bit 5
ADC_DIFSEL_DIFSEL_6  EQU (0x00000040) ; ADC DIFSEL bit 6
ADC_DIFSEL_DIFSEL_7  EQU (0x00000080) ; ADC DIFSEL bit 7
ADC_DIFSEL_DIFSEL_8  EQU (0x00000100) ; ADC DIFSEL bit 8
ADC_DIFSEL_DIFSEL_9  EQU (0x00000200) ; ADC DIFSEL bit 9
ADC_DIFSEL_DIFSEL_10 EQU (0x00000400) ; ADC DIFSEL bit 10
ADC_DIFSEL_DIFSEL_11 EQU (0x00000800) ; ADC DIFSEL bit 11
ADC_DIFSEL_DIFSEL_12 EQU (0x00001000) ; ADC DIFSEL bit 12
ADC_DIFSEL_DIFSEL_13 EQU (0x00002000) ; ADC DIFSEL bit 13
ADC_DIFSEL_DIFSEL_14 EQU (0x00004000) ; ADC DIFSEL bit 14
ADC_DIFSEL_DIFSEL_15 EQU (0x00008000) ; ADC DIFSEL bit 15
ADC_DIFSEL_DIFSEL_16 EQU (0x00010000) ; ADC DIFSEL bit 16
ADC_DIFSEL_DIFSEL_17 EQU (0x00020000) ; ADC DIFSEL bit 17
ADC_DIFSEL_DIFSEL_18 EQU (0x00040000) ; ADC DIFSEL bit 18

; ********************  Bit definition for ADC_CALFACT register  *******************
ADC_CALFACT_CALFACT_S    EQU (0x0000007F) ; ADC calibration factors in single-ended mode
ADC_CALFACT_CALFACT_S_0  EQU (0x00000001) ; ADC CALFACT_S bit 0
ADC_CALFACT_CALFACT_S_1  EQU (0x00000002) ; ADC CALFACT_S bit 1
ADC_CALFACT_CALFACT_S_2  EQU (0x00000004) ; ADC CALFACT_S bit 2
ADC_CALFACT_CALFACT_S_3  EQU (0x00000008) ; ADC CALFACT_S bit 3
ADC_CALFACT_CALFACT_S_4  EQU (0x00000010) ; ADC CALFACT_S bit 4
ADC_CALFACT_CALFACT_S_5  EQU (0x00000020) ; ADC CALFACT_S bit 5
ADC_CALFACT_CALFACT_S_6  EQU (0x00000040) ; ADC CALFACT_S bit 6

ADC_CALFACT_CALFACT_D    EQU (0x007F0000) ; ADC calibration factors in differential mode
ADC_CALFACT_CALFACT_D_0  EQU (0x00010000) ; ADC CALFACT_D bit 0
ADC_CALFACT_CALFACT_D_1  EQU (0x00020000) ; ADC CALFACT_D bit 1
ADC_CALFACT_CALFACT_D_2  EQU (0x00040000) ; ADC CALFACT_D bit 2
ADC_CALFACT_CALFACT_D_3  EQU (0x00080000) ; ADC CALFACT_D bit 3
ADC_CALFACT_CALFACT_D_4  EQU (0x00100000) ; ADC CALFACT_D bit 4
ADC_CALFACT_CALFACT_D_5  EQU (0x00200000) ; ADC CALFACT_D bit 5
ADC_CALFACT_CALFACT_D_6  EQU (0x00400000) ; ADC CALFACT_D bit 6

; *************************  ADC Common registers  ****************************
; ********************  Bit definition for ADC_CSR register  *******************
ADC_CSR_ADRDY_MST EQU (0x00000001) ; Master ADC ready
ADC_CSR_EOSMP_MST EQU (0x00000002) ; End of sampling phase flag of the master ADC
ADC_CSR_EOC_MST   EQU (0x00000004) ; End of regular conversion of the master ADC
ADC_CSR_EOS_MST   EQU (0x00000008) ; End of regular sequence flag of the master ADC
ADC_CSR_OVR_MST   EQU (0x00000010) ; Overrun flag of the master ADC
ADC_CSR_JEOC_MST  EQU (0x00000020) ; End of injected conversion of the master ADC
ADC_CSR_JEOS_MST  EQU (0x00000040) ; End of injected sequence flag of the master ADC
ADC_CSR_AWD1_MST  EQU (0x00000080) ; Analog watchdog 1 flag of the master ADC
ADC_CSR_AWD2_MST  EQU (0x00000100) ; Analog watchdog 2 flag of the master ADC
ADC_CSR_AWD3_MST  EQU (0x00000200) ; Analog watchdog 3 flag of the master ADC
ADC_CSR_JQOVF_MST EQU (0x00000400) ; Injected context queue overflow flag of the master ADC

ADC_CSR_ADRDY_SLV EQU (0x00010000) ; Slave ADC ready
ADC_CSR_EOSMP_SLV EQU (0x00020000) ; End of sampling phase flag of the slave ADC
ADC_CSR_EOC_SLV   EQU (0x00040000) ; End of regular conversion of the slave ADC
ADC_CSR_EOS_SLV   EQU (0x00080000) ; End of regular sequence flag of the slave ADC
ADC_CSR_OVR_SLV   EQU (0x00100000) ; Overrun flag of the slave ADC
ADC_CSR_JEOC_SLV  EQU (0x00200000) ; End of injected conversion of the slave ADC
ADC_CSR_JEOS_SLV  EQU (0x00400000) ; End of injected sequence flag of the slave ADC
ADC_CSR_AWD1_SLV  EQU (0x00800000) ; Analog watchdog 1 flag of the slave ADC
ADC_CSR_AWD2_SLV  EQU (0x01000000) ; Analog watchdog 2 flag of the slave ADC
ADC_CSR_AWD3_SLV  EQU (0x02000000) ; Analog watchdog 3 flag of the slave ADC
ADC_CSR_JQOVF_SLV EQU (0x04000000) ; Injected context queue overflow flag of the slave ADC

; ********************  Bit definition for ADC_CCR register  *******************
ADC_CCR_DUAL      EQU (0x0000001F) ; Dual ADC mode selection
ADC_CCR_DUAL_0    EQU (0x00000001) ; Dual bit 0
ADC_CCR_DUAL_1    EQU (0x00000002) ; Dual bit 1
ADC_CCR_DUAL_2    EQU (0x00000004) ; Dual bit 2
ADC_CCR_DUAL_3    EQU (0x00000008) ; Dual bit 3
ADC_CCR_DUAL_4    EQU (0x00000010) ; Dual bit 4

ADC_CCR_DELAY     EQU (0x00000F00) ; Delay between 2 sampling phases
ADC_CCR_DELAY_0   EQU (0x00000100) ; DELAY bit 0
ADC_CCR_DELAY_1   EQU (0x00000200) ; DELAY bit 1
ADC_CCR_DELAY_2   EQU (0x00000400) ; DELAY bit 2
ADC_CCR_DELAY_3   EQU (0x00000800) ; DELAY bit 3

ADC_CCR_DMACFG    EQU (0x00002000) ; DMA configuration for multi-ADC mode

ADC_CCR_MDMA      EQU (0x0000C000) ; DMA mode for multi-ADC mode
ADC_CCR_MDMA_0    EQU (0x00004000) ; MDMA bit 0
ADC_CCR_MDMA_1    EQU (0x00008000) ; MDMA bit 1

ADC_CCR_CKMODE    EQU (0x00030000) ; ADC clock mode
ADC_CCR_CKMODE_0  EQU (0x00010000) ; CKMODE bit 0
ADC_CCR_CKMODE_1  EQU (0x00020000) ; CKMODE bit 1

ADC_CCR_PRESC     EQU (0x003C0000) ; ADC prescaler
ADC_CCR_PRESC_0   EQU (0x00040000) ; ADC prescaler bit 0
ADC_CCR_PRESC_1   EQU (0x00080000) ; ADC prescaler bit 1
ADC_CCR_PRESC_2   EQU (0x00100000) ; ADC prescaler bit 2
ADC_CCR_PRESC_3   EQU (0x00200000) ; ADC prescaler bit 3

ADC_CCR_VREFEN    EQU (0x00400000) ; VREFINT enable
ADC_CCR_TSEN      EQU (0x00800000) ; Temperature sensor enable
ADC_CCR_VBATEN    EQU (0x01000000) ; VBAT enable

; ********************  Bit definition for ADC_CDR register  *******************
ADC_CDR_RDATA_MST EQU (0x0000FFFF) ; Regular Data of the master ADC
ADC_CDR_RDATA_MST_0       EQU (0x00000001) ; RDATA_MST bit 0
ADC_CDR_RDATA_MST_1       EQU (0x00000002) ; RDATA_MST bit 1
ADC_CDR_RDATA_MST_2       EQU (0x00000004) ; RDATA_MST bit 2
ADC_CDR_RDATA_MST_3       EQU (0x00000008) ; RDATA_MST bit 3
ADC_CDR_RDATA_MST_4       EQU (0x00000010) ; RDATA_MST bit 4
ADC_CDR_RDATA_MST_5       EQU (0x00000020) ; RDATA_MST bit 5
ADC_CDR_RDATA_MST_6       EQU (0x00000040) ; RDATA_MST bit 6
ADC_CDR_RDATA_MST_7       EQU (0x00000080) ; RDATA_MST bit 7
ADC_CDR_RDATA_MST_8       EQU (0x00000100) ; RDATA_MST bit 8
ADC_CDR_RDATA_MST_9       EQU (0x00000200) ; RDATA_MST bit 9
ADC_CDR_RDATA_MST_10      EQU (0x00000400) ; RDATA_MST bit 10
ADC_CDR_RDATA_MST_11      EQU (0x00000800) ; RDATA_MST bit 11
ADC_CDR_RDATA_MST_12      EQU (0x00001000) ; RDATA_MST bit 12
ADC_CDR_RDATA_MST_13      EQU (0x00002000) ; RDATA_MST bit 13
ADC_CDR_RDATA_MST_14      EQU (0x00004000) ; RDATA_MST bit 14
ADC_CDR_RDATA_MST_15      EQU (0x00008000) ; RDATA_MST bit 15

ADC_CDR_RDATA_SLV EQU (0xFFFF0000) ; Regular Data of the master ADC
ADC_CDR_RDATA_SLV_0       EQU (0x00010000) ; RDATA_SLV bit 0
ADC_CDR_RDATA_SLV_1       EQU (0x00020000) ; RDATA_SLV bit 1
ADC_CDR_RDATA_SLV_2       EQU (0x00040000) ; RDATA_SLV bit 2
ADC_CDR_RDATA_SLV_3       EQU (0x00080000) ; RDATA_SLV bit 3
ADC_CDR_RDATA_SLV_4       EQU (0x00100000) ; RDATA_SLV bit 4
ADC_CDR_RDATA_SLV_5       EQU (0x00200000) ; RDATA_SLV bit 5
ADC_CDR_RDATA_SLV_6       EQU (0x00400000) ; RDATA_SLV bit 6
ADC_CDR_RDATA_SLV_7       EQU (0x00800000) ; RDATA_SLV bit 7
ADC_CDR_RDATA_SLV_8       EQU (0x01000000) ; RDATA_SLV bit 8
ADC_CDR_RDATA_SLV_9       EQU (0x02000000) ; RDATA_SLV bit 9
ADC_CDR_RDATA_SLV_10      EQU (0x04000000) ; RDATA_SLV bit 10
ADC_CDR_RDATA_SLV_11      EQU (0x08000000) ; RDATA_SLV bit 11
ADC_CDR_RDATA_SLV_12      EQU (0x10000000) ; RDATA_SLV bit 12
ADC_CDR_RDATA_SLV_13      EQU (0x20000000) ; RDATA_SLV bit 13
ADC_CDR_RDATA_SLV_14      EQU (0x40000000) ; RDATA_SLV bit 14
ADC_CDR_RDATA_SLV_15      EQU (0x80000000) ; RDATA_SLV bit 15

; *****************************************************************************
;
;                         Controller Area Network
;
; *****************************************************************************
;CAN control and status registers
; *******************  Bit definition for CAN_MCR register  *******************
CAN_MCR_INRQ                EQU (0x0001)            ;Initialization Request
CAN_MCR_SLEEP               EQU (0x0002)            ;Sleep Mode Request
CAN_MCR_TXFP                EQU (0x0004)            ;Transmit FIFO Priority
CAN_MCR_RFLM                EQU (0x0008)            ;Receive FIFO Locked Mode
CAN_MCR_NART                EQU (0x0010)            ;No Automatic Retransmission
CAN_MCR_AWUM                EQU (0x0020)            ;Automatic Wakeup Mode
CAN_MCR_ABOM                EQU (0x0040)            ;Automatic Bus-Off Management
CAN_MCR_TTCM                EQU (0x0080)            ;Time Triggered Communication Mode
CAN_MCR_RESET               EQU (0x8000)            ;bxCAN software master reset

; *******************  Bit definition for CAN_MSR register  *******************
CAN_MSR_INAK                EQU (0x0001)            ;Initialization Acknowledge
CAN_MSR_SLAK                EQU (0x0002)            ;Sleep Acknowledge
CAN_MSR_ERRI                EQU (0x0004)            ;Error Interrupt
CAN_MSR_WKUI                EQU (0x0008)            ;Wakeup Interrupt
CAN_MSR_SLAKI               EQU (0x0010)            ;Sleep Acknowledge Interrupt
CAN_MSR_TXM                 EQU (0x0100)            ;Transmit Mode
CAN_MSR_RXM                 EQU (0x0200)            ;Receive Mode
CAN_MSR_SAMP                EQU (0x0400)            ;Last Sample Point
CAN_MSR_RX                  EQU (0x0800)            ;CAN Rx Signal

; *******************  Bit definition for CAN_TSR register  *******************
CAN_TSR_RQCP0               EQU (0x00000001)        ;Request Completed Mailbox0
CAN_TSR_TXOK0               EQU (0x00000002)        ;Transmission OK of Mailbox0
CAN_TSR_ALST0               EQU (0x00000004)        ;Arbitration Lost for Mailbox0
CAN_TSR_TERR0               EQU (0x00000008)        ;Transmission Error of Mailbox0
CAN_TSR_ABRQ0               EQU (0x00000080)        ;Abort Request for Mailbox0
CAN_TSR_RQCP1               EQU (0x00000100)        ;Request Completed Mailbox1
CAN_TSR_TXOK1               EQU (0x00000200)        ;Transmission OK of Mailbox1
CAN_TSR_ALST1               EQU (0x00000400)        ;Arbitration Lost for Mailbox1
CAN_TSR_TERR1               EQU (0x00000800)        ;Transmission Error of Mailbox1
CAN_TSR_ABRQ1               EQU (0x00008000)        ;Abort Request for Mailbox 1
CAN_TSR_RQCP2               EQU (0x00010000)        ;Request Completed Mailbox2
CAN_TSR_TXOK2               EQU (0x00020000)        ;Transmission OK of Mailbox 2
CAN_TSR_ALST2               EQU (0x00040000)        ;Arbitration Lost for mailbox 2
CAN_TSR_TERR2               EQU (0x00080000)        ;Transmission Error of Mailbox 2
CAN_TSR_ABRQ2               EQU (0x00800000)        ;Abort Request for Mailbox 2
CAN_TSR_CODE                EQU (0x03000000)        ;Mailbox Code

CAN_TSR_TME                 EQU (0x1C000000)        ;TME[2:0] bits
CAN_TSR_TME0                EQU (0x04000000)        ;Transmit Mailbox 0 Empty
CAN_TSR_TME1                EQU (0x08000000)        ;Transmit Mailbox 1 Empty
CAN_TSR_TME2                EQU (0x10000000)        ;Transmit Mailbox 2 Empty

CAN_TSR_LOW                 EQU (0xE0000000)        ;LOW[2:0] bits
CAN_TSR_LOW0                EQU (0x20000000)        ;Lowest Priority Flag for Mailbox 0
CAN_TSR_LOW1                EQU (0x40000000)        ;Lowest Priority Flag for Mailbox 1
CAN_TSR_LOW2                EQU (0x80000000)        ;Lowest Priority Flag for Mailbox 2

; *******************  Bit definition for CAN_RF0R register  ******************
CAN_RF0R_FMP0               EQU (0x03)               ;FIFO 0 Message Pending
CAN_RF0R_FULL0              EQU (0x08)               ;FIFO 0 Full
CAN_RF0R_FOVR0              EQU (0x10)               ;FIFO 0 Overrun
CAN_RF0R_RFOM0              EQU (0x20)               ;Release FIFO 0 Output Mailbox

; *******************  Bit definition for CAN_RF1R register  ******************
CAN_RF1R_FMP1               EQU (0x03)               ;FIFO 1 Message Pending
CAN_RF1R_FULL1              EQU (0x08)               ;FIFO 1 Full
CAN_RF1R_FOVR1              EQU (0x10)               ;FIFO 1 Overrun
CAN_RF1R_RFOM1              EQU (0x20)               ;Release FIFO 1 Output Mailbox

; ********************  Bit definition for CAN_IER register  ******************
CAN_IER_TMEIE               EQU (0x00000001)        ;Transmit Mailbox Empty Interrupt Enable
CAN_IER_FMPIE0              EQU (0x00000002)        ;FIFO Message Pending Interrupt Enable
CAN_IER_FFIE0               EQU (0x00000004)        ;FIFO Full Interrupt Enable
CAN_IER_FOVIE0              EQU (0x00000008)        ;FIFO Overrun Interrupt Enable
CAN_IER_FMPIE1              EQU (0x00000010)        ;FIFO Message Pending Interrupt Enable
CAN_IER_FFIE1               EQU (0x00000020)        ;FIFO Full Interrupt Enable
CAN_IER_FOVIE1              EQU (0x00000040)        ;FIFO Overrun Interrupt Enable
CAN_IER_EWGIE               EQU (0x00000100)        ;Error Warning Interrupt Enable
CAN_IER_EPVIE               EQU (0x00000200)        ;Error Passive Interrupt Enable
CAN_IER_BOFIE               EQU (0x00000400)        ;Bus-Off Interrupt Enable
CAN_IER_LECIE               EQU (0x00000800)        ;Last Error Code Interrupt Enable
CAN_IER_ERRIE               EQU (0x00008000)        ;Error Interrupt Enable
CAN_IER_WKUIE               EQU (0x00010000)        ;Wakeup Interrupt Enable
CAN_IER_SLKIE               EQU (0x00020000)        ;Sleep Interrupt Enable

; ********************  Bit definition for CAN_ESR register  ******************
CAN_ESR_EWGF                EQU (0x00000001)        ;Error Warning Flag
CAN_ESR_EPVF                EQU (0x00000002)        ;Error Passive Flag
CAN_ESR_BOFF                EQU (0x00000004)        ;Bus-Off Flag

CAN_ESR_LEC                 EQU (0x00000070)        ;LEC[2:0] bits (Last Error Code)
CAN_ESR_LEC_0               EQU (0x00000010)        ;Bit 0
CAN_ESR_LEC_1               EQU (0x00000020)        ;Bit 1
CAN_ESR_LEC_2               EQU (0x00000040)        ;Bit 2

CAN_ESR_TEC                 EQU (0x00FF0000)        ;Least significant byte of the 9-bit Transmit Error Counter
CAN_ESR_REC                 EQU (0xFF000000)        ;Receive Error Counter

; *******************  Bit definition for CAN_BTR register  *******************
CAN_BTR_BRP                 EQU (0x000003FF)        ;Baud Rate Prescaler
CAN_BTR_TS1_0               EQU (0x00010000)        ;Time Segment 1 (Bit 0)
CAN_BTR_TS1_1               EQU (0x00020000)        ;Time Segment 1 (Bit 1)
CAN_BTR_TS1_2               EQU (0x00040000)        ;Time Segment 1 (Bit 2)
CAN_BTR_TS1_3               EQU (0x00080000)        ;Time Segment 1 (Bit 3)
CAN_BTR_TS1                 EQU (0x000F0000)        ;Time Segment 1
CAN_BTR_TS2_0               EQU (0x00100000)        ;Time Segment 2 (Bit 0)
CAN_BTR_TS2_1               EQU (0x00200000)        ;Time Segment 2 (Bit 1)
CAN_BTR_TS2_2               EQU (0x00400000)        ;Time Segment 2 (Bit 2)
CAN_BTR_TS2                 EQU (0x00700000)        ;Time Segment 2
CAN_BTR_SJW_0               EQU (0x01000000)        ;Resynchronization Jump Width (Bit 0)
CAN_BTR_SJW_1               EQU (0x02000000)        ;Resynchronization Jump Width (Bit 1)
CAN_BTR_SJW                 EQU (0x03000000)        ;Resynchronization Jump Width
CAN_BTR_LBKM                EQU (0x40000000)        ;Loop Back Mode (Debug)
CAN_BTR_SILM                EQU (0x80000000)        ;Silent Mode

;Mailbox registers
; ******************  Bit definition for CAN_TI0R register  *******************
CAN_TI0R_TXRQ               EQU (0x00000001)        ;Transmit Mailbox Request
CAN_TI0R_RTR                EQU (0x00000002)        ;Remote Transmission Request
CAN_TI0R_IDE                EQU (0x00000004)        ;Identifier Extension
CAN_TI0R_EXID               EQU (0x001FFFF8)        ;Extended Identifier
CAN_TI0R_STID               EQU (0xFFE00000)        ;Standard Identifier or Extended Identifier

; ******************  Bit definition for CAN_TDT0R register  ******************
CAN_TDT0R_DLC               EQU (0x0000000F)        ;Data Length Code
CAN_TDT0R_TGT               EQU (0x00000100)        ;Transmit Global Time
CAN_TDT0R_TIME              EQU (0xFFFF0000)        ;Message Time Stamp

; ******************  Bit definition for CAN_TDL0R register  ******************
CAN_TDL0R_DATA0             EQU (0x000000FF)        ;Data byte 0
CAN_TDL0R_DATA1             EQU (0x0000FF00)        ;Data byte 1
CAN_TDL0R_DATA2             EQU (0x00FF0000)        ;Data byte 2
CAN_TDL0R_DATA3             EQU (0xFF000000)        ;Data byte 3

; ******************  Bit definition for CAN_TDH0R register  ******************
CAN_TDH0R_DATA4             EQU (0x000000FF)        ;Data byte 4
CAN_TDH0R_DATA5             EQU (0x0000FF00)        ;Data byte 5
CAN_TDH0R_DATA6             EQU (0x00FF0000)        ;Data byte 6
CAN_TDH0R_DATA7             EQU (0xFF000000)        ;Data byte 7

; *******************  Bit definition for CAN_TI1R register  ******************
CAN_TI1R_TXRQ               EQU (0x00000001)        ;Transmit Mailbox Request
CAN_TI1R_RTR                EQU (0x00000002)        ;Remote Transmission Request
CAN_TI1R_IDE                EQU (0x00000004)        ;Identifier Extension
CAN_TI1R_EXID               EQU (0x001FFFF8)        ;Extended Identifier
CAN_TI1R_STID               EQU (0xFFE00000)        ;Standard Identifier or Extended Identifier

; *******************  Bit definition for CAN_TDT1R register  *****************
CAN_TDT1R_DLC               EQU (0x0000000F)        ;Data Length Code
CAN_TDT1R_TGT               EQU (0x00000100)        ;Transmit Global Time
CAN_TDT1R_TIME              EQU (0xFFFF0000)        ;Message Time Stamp

; *******************  Bit definition for CAN_TDL1R register  *****************
CAN_TDL1R_DATA0             EQU (0x000000FF)        ;Data byte 0
CAN_TDL1R_DATA1             EQU (0x0000FF00)        ;Data byte 1
CAN_TDL1R_DATA2             EQU (0x00FF0000)        ;Data byte 2
CAN_TDL1R_DATA3             EQU (0xFF000000)        ;Data byte 3

; *******************  Bit definition for CAN_TDH1R register  *****************
CAN_TDH1R_DATA4             EQU (0x000000FF)        ;Data byte 4
CAN_TDH1R_DATA5             EQU (0x0000FF00)        ;Data byte 5
CAN_TDH1R_DATA6             EQU (0x00FF0000)        ;Data byte 6
CAN_TDH1R_DATA7             EQU (0xFF000000)        ;Data byte 7

; *******************  Bit definition for CAN_TI2R register  ******************
CAN_TI2R_TXRQ               EQU (0x00000001)        ;Transmit Mailbox Request
CAN_TI2R_RTR                EQU (0x00000002)        ;Remote Transmission Request
CAN_TI2R_IDE                EQU (0x00000004)        ;Identifier Extension
CAN_TI2R_EXID               EQU (0x001FFFF8)        ;Extended identifier
CAN_TI2R_STID               EQU (0xFFE00000)        ;Standard Identifier or Extended Identifier

; *******************  Bit definition for CAN_TDT2R register  *****************
CAN_TDT2R_DLC               EQU (0x0000000F)        ;Data Length Code
CAN_TDT2R_TGT               EQU (0x00000100)        ;Transmit Global Time
CAN_TDT2R_TIME              EQU (0xFFFF0000)        ;Message Time Stamp

; *******************  Bit definition for CAN_TDL2R register  *****************
CAN_TDL2R_DATA0             EQU (0x000000FF)        ;Data byte 0
CAN_TDL2R_DATA1             EQU (0x0000FF00)        ;Data byte 1
CAN_TDL2R_DATA2             EQU (0x00FF0000)        ;Data byte 2
CAN_TDL2R_DATA3             EQU (0xFF000000)        ;Data byte 3

; *******************  Bit definition for CAN_TDH2R register  *****************
CAN_TDH2R_DATA4             EQU (0x000000FF)        ;Data byte 4
CAN_TDH2R_DATA5             EQU (0x0000FF00)        ;Data byte 5
CAN_TDH2R_DATA6             EQU (0x00FF0000)        ;Data byte 6
CAN_TDH2R_DATA7             EQU (0xFF000000)        ;Data byte 7

; *******************  Bit definition for CAN_RI0R register  ******************
CAN_RI0R_RTR                EQU (0x00000002)        ;Remote Transmission Request
CAN_RI0R_IDE                EQU (0x00000004)        ;Identifier Extension
CAN_RI0R_EXID               EQU (0x001FFFF8)        ;Extended Identifier
CAN_RI0R_STID               EQU (0xFFE00000)        ;Standard Identifier or Extended Identifier

; *******************  Bit definition for CAN_RDT0R register  *****************
CAN_RDT0R_DLC               EQU (0x0000000F)        ;Data Length Code
CAN_RDT0R_FMI               EQU (0x0000FF00)        ;Filter Match Index
CAN_RDT0R_TIME              EQU (0xFFFF0000)        ;Message Time Stamp

; *******************  Bit definition for CAN_RDL0R register  *****************
CAN_RDL0R_DATA0             EQU (0x000000FF)        ;Data byte 0
CAN_RDL0R_DATA1             EQU (0x0000FF00)        ;Data byte 1
CAN_RDL0R_DATA2             EQU (0x00FF0000)        ;Data byte 2
CAN_RDL0R_DATA3             EQU (0xFF000000)        ;Data byte 3

; *******************  Bit definition for CAN_RDH0R register  *****************
CAN_RDH0R_DATA4             EQU (0x000000FF)        ;Data byte 4
CAN_RDH0R_DATA5             EQU (0x0000FF00)        ;Data byte 5
CAN_RDH0R_DATA6             EQU (0x00FF0000)        ;Data byte 6
CAN_RDH0R_DATA7             EQU (0xFF000000)        ;Data byte 7

; *******************  Bit definition for CAN_RI1R register  ******************
CAN_RI1R_RTR                EQU (0x00000002)        ;Remote Transmission Request
CAN_RI1R_IDE                EQU (0x00000004)        ;Identifier Extension
CAN_RI1R_EXID               EQU (0x001FFFF8)        ;Extended identifier
CAN_RI1R_STID               EQU (0xFFE00000)        ;Standard Identifier or Extended Identifier

; *******************  Bit definition for CAN_RDT1R register  *****************
CAN_RDT1R_DLC               EQU (0x0000000F)        ;Data Length Code
CAN_RDT1R_FMI               EQU (0x0000FF00)        ;Filter Match Index
CAN_RDT1R_TIME              EQU (0xFFFF0000)        ;Message Time Stamp

; *******************  Bit definition for CAN_RDL1R register  *****************
CAN_RDL1R_DATA0             EQU (0x000000FF)        ;Data byte 0
CAN_RDL1R_DATA1             EQU (0x0000FF00)        ;Data byte 1
CAN_RDL1R_DATA2             EQU (0x00FF0000)        ;Data byte 2
CAN_RDL1R_DATA3             EQU (0xFF000000)        ;Data byte 3

; *******************  Bit definition for CAN_RDH1R register  *****************
CAN_RDH1R_DATA4             EQU (0x000000FF)        ;Data byte 4
CAN_RDH1R_DATA5             EQU (0x0000FF00)        ;Data byte 5
CAN_RDH1R_DATA6             EQU (0x00FF0000)        ;Data byte 6
CAN_RDH1R_DATA7             EQU (0xFF000000)        ;Data byte 7

;CAN filter registers
; *******************  Bit definition for CAN_FMR register  *******************
CAN_FMR_FINIT               EQU (0x01)               ;Filter Init Mode

; *******************  Bit definition for CAN_FM1R register  ******************
CAN_FM1R_FBM                EQU (0x3FFF)            ;Filter Mode
CAN_FM1R_FBM0               EQU (0x0001)            ;Filter Init Mode bit 0
CAN_FM1R_FBM1               EQU (0x0002)            ;Filter Init Mode bit 1
CAN_FM1R_FBM2               EQU (0x0004)            ;Filter Init Mode bit 2
CAN_FM1R_FBM3               EQU (0x0008)            ;Filter Init Mode bit 3
CAN_FM1R_FBM4               EQU (0x0010)            ;Filter Init Mode bit 4
CAN_FM1R_FBM5               EQU (0x0020)            ;Filter Init Mode bit 5
CAN_FM1R_FBM6               EQU (0x0040)            ;Filter Init Mode bit 6
CAN_FM1R_FBM7               EQU (0x0080)            ;Filter Init Mode bit 7
CAN_FM1R_FBM8               EQU (0x0100)            ;Filter Init Mode bit 8
CAN_FM1R_FBM9               EQU (0x0200)            ;Filter Init Mode bit 9
CAN_FM1R_FBM10              EQU (0x0400)            ;Filter Init Mode bit 10
CAN_FM1R_FBM11              EQU (0x0800)            ;Filter Init Mode bit 11
CAN_FM1R_FBM12              EQU (0x1000)            ;Filter Init Mode bit 12
CAN_FM1R_FBM13              EQU (0x2000)            ;Filter Init Mode bit 13

; *******************  Bit definition for CAN_FS1R register  ******************
CAN_FS1R_FSC                EQU (0x3FFF)            ;Filter Scale Configuration
CAN_FS1R_FSC0               EQU (0x0001)            ;Filter Scale Configuration bit 0
CAN_FS1R_FSC1               EQU (0x0002)            ;Filter Scale Configuration bit 1
CAN_FS1R_FSC2               EQU (0x0004)            ;Filter Scale Configuration bit 2
CAN_FS1R_FSC3               EQU (0x0008)            ;Filter Scale Configuration bit 3
CAN_FS1R_FSC4               EQU (0x0010)            ;Filter Scale Configuration bit 4
CAN_FS1R_FSC5               EQU (0x0020)            ;Filter Scale Configuration bit 5
CAN_FS1R_FSC6               EQU (0x0040)            ;Filter Scale Configuration bit 6
CAN_FS1R_FSC7               EQU (0x0080)            ;Filter Scale Configuration bit 7
CAN_FS1R_FSC8               EQU (0x0100)            ;Filter Scale Configuration bit 8
CAN_FS1R_FSC9               EQU (0x0200)            ;Filter Scale Configuration bit 9
CAN_FS1R_FSC10              EQU (0x0400)            ;Filter Scale Configuration bit 10
CAN_FS1R_FSC11              EQU (0x0800)            ;Filter Scale Configuration bit 11
CAN_FS1R_FSC12              EQU (0x1000)            ;Filter Scale Configuration bit 12
CAN_FS1R_FSC13              EQU (0x2000)            ;Filter Scale Configuration bit 13

; ******************  Bit definition for CAN_FFA1R register  ******************
CAN_FFA1R_FFA               EQU (0x3FFF)            ;Filter FIFO Assignment
CAN_FFA1R_FFA0              EQU (0x0001)            ;Filter FIFO Assignment for Filter 0
CAN_FFA1R_FFA1              EQU (0x0002)            ;Filter FIFO Assignment for Filter 1
CAN_FFA1R_FFA2              EQU (0x0004)            ;Filter FIFO Assignment for Filter 2
CAN_FFA1R_FFA3              EQU (0x0008)            ;Filter FIFO Assignment for Filter 3
CAN_FFA1R_FFA4              EQU (0x0010)            ;Filter FIFO Assignment for Filter 4
CAN_FFA1R_FFA5              EQU (0x0020)            ;Filter FIFO Assignment for Filter 5
CAN_FFA1R_FFA6              EQU (0x0040)            ;Filter FIFO Assignment for Filter 6
CAN_FFA1R_FFA7              EQU (0x0080)            ;Filter FIFO Assignment for Filter 7
CAN_FFA1R_FFA8              EQU (0x0100)            ;Filter FIFO Assignment for Filter 8
CAN_FFA1R_FFA9              EQU (0x0200)            ;Filter FIFO Assignment for Filter 9
CAN_FFA1R_FFA10             EQU (0x0400)            ;Filter FIFO Assignment for Filter 10
CAN_FFA1R_FFA11             EQU (0x0800)            ;Filter FIFO Assignment for Filter 11
CAN_FFA1R_FFA12             EQU (0x1000)            ;Filter FIFO Assignment for Filter 12
CAN_FFA1R_FFA13             EQU (0x2000)            ;Filter FIFO Assignment for Filter 13

; *******************  Bit definition for CAN_FA1R register  ******************
CAN_FA1R_FACT               EQU (0x3FFF)            ;Filter Active
CAN_FA1R_FACT0              EQU (0x0001)            ;Filter 0 Active
CAN_FA1R_FACT1              EQU (0x0002)            ;Filter 1 Active
CAN_FA1R_FACT2              EQU (0x0004)            ;Filter 2 Active
CAN_FA1R_FACT3              EQU (0x0008)            ;Filter 3 Active
CAN_FA1R_FACT4              EQU (0x0010)            ;Filter 4 Active
CAN_FA1R_FACT5              EQU (0x0020)            ;Filter 5 Active
CAN_FA1R_FACT6              EQU (0x0040)            ;Filter 6 Active
CAN_FA1R_FACT7              EQU (0x0080)            ;Filter 7 Active
CAN_FA1R_FACT8              EQU (0x0100)            ;Filter 8 Active
CAN_FA1R_FACT9              EQU (0x0200)            ;Filter 9 Active
CAN_FA1R_FACT10             EQU (0x0400)            ;Filter 10 Active
CAN_FA1R_FACT11             EQU (0x0800)            ;Filter 11 Active
CAN_FA1R_FACT12             EQU (0x1000)            ;Filter 12 Active
CAN_FA1R_FACT13             EQU (0x2000)            ;Filter 13 Active

; *******************  Bit definition for CAN_F0R1 register  ******************
CAN_F0R1_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F0R1_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F0R1_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F0R1_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F0R1_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F0R1_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F0R1_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F0R1_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F0R1_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F0R1_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F0R1_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F0R1_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F0R1_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F0R1_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F0R1_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F0R1_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F0R1_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F0R1_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F0R1_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F0R1_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F0R1_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F0R1_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F0R1_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F0R1_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F0R1_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F0R1_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F0R1_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F0R1_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F0R1_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F0R1_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F0R1_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F0R1_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F1R1 register  ******************
CAN_F1R1_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F1R1_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F1R1_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F1R1_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F1R1_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F1R1_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F1R1_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F1R1_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F1R1_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F1R1_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F1R1_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F1R1_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F1R1_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F1R1_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F1R1_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F1R1_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F1R1_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F1R1_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F1R1_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F1R1_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F1R1_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F1R1_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F1R1_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F1R1_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F1R1_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F1R1_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F1R1_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F1R1_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F1R1_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F1R1_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F1R1_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F1R1_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F2R1 register  ******************
CAN_F2R1_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F2R1_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F2R1_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F2R1_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F2R1_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F2R1_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F2R1_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F2R1_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F2R1_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F2R1_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F2R1_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F2R1_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F2R1_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F2R1_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F2R1_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F2R1_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F2R1_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F2R1_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F2R1_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F2R1_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F2R1_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F2R1_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F2R1_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F2R1_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F2R1_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F2R1_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F2R1_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F2R1_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F2R1_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F2R1_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F2R1_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F2R1_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F3R1 register  ******************
CAN_F3R1_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F3R1_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F3R1_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F3R1_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F3R1_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F3R1_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F3R1_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F3R1_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F3R1_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F3R1_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F3R1_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F3R1_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F3R1_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F3R1_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F3R1_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F3R1_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F3R1_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F3R1_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F3R1_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F3R1_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F3R1_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F3R1_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F3R1_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F3R1_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F3R1_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F3R1_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F3R1_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F3R1_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F3R1_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F3R1_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F3R1_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F3R1_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F4R1 register  ******************
CAN_F4R1_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F4R1_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F4R1_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F4R1_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F4R1_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F4R1_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F4R1_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F4R1_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F4R1_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F4R1_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F4R1_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F4R1_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F4R1_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F4R1_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F4R1_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F4R1_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F4R1_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F4R1_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F4R1_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F4R1_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F4R1_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F4R1_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F4R1_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F4R1_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F4R1_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F4R1_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F4R1_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F4R1_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F4R1_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F4R1_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F4R1_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F4R1_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F5R1 register  ******************
CAN_F5R1_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F5R1_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F5R1_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F5R1_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F5R1_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F5R1_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F5R1_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F5R1_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F5R1_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F5R1_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F5R1_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F5R1_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F5R1_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F5R1_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F5R1_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F5R1_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F5R1_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F5R1_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F5R1_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F5R1_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F5R1_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F5R1_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F5R1_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F5R1_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F5R1_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F5R1_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F5R1_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F5R1_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F5R1_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F5R1_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F5R1_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F5R1_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F6R1 register  ******************
CAN_F6R1_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F6R1_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F6R1_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F6R1_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F6R1_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F6R1_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F6R1_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F6R1_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F6R1_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F6R1_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F6R1_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F6R1_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F6R1_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F6R1_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F6R1_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F6R1_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F6R1_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F6R1_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F6R1_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F6R1_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F6R1_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F6R1_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F6R1_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F6R1_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F6R1_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F6R1_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F6R1_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F6R1_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F6R1_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F6R1_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F6R1_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F6R1_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F7R1 register  ******************
CAN_F7R1_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F7R1_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F7R1_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F7R1_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F7R1_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F7R1_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F7R1_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F7R1_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F7R1_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F7R1_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F7R1_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F7R1_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F7R1_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F7R1_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F7R1_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F7R1_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F7R1_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F7R1_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F7R1_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F7R1_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F7R1_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F7R1_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F7R1_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F7R1_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F7R1_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F7R1_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F7R1_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F7R1_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F7R1_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F7R1_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F7R1_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F7R1_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F8R1 register  ******************
CAN_F8R1_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F8R1_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F8R1_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F8R1_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F8R1_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F8R1_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F8R1_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F8R1_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F8R1_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F8R1_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F8R1_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F8R1_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F8R1_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F8R1_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F8R1_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F8R1_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F8R1_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F8R1_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F8R1_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F8R1_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F8R1_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F8R1_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F8R1_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F8R1_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F8R1_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F8R1_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F8R1_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F8R1_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F8R1_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F8R1_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F8R1_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F8R1_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F9R1 register  ******************
CAN_F9R1_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F9R1_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F9R1_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F9R1_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F9R1_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F9R1_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F9R1_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F9R1_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F9R1_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F9R1_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F9R1_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F9R1_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F9R1_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F9R1_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F9R1_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F9R1_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F9R1_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F9R1_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F9R1_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F9R1_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F9R1_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F9R1_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F9R1_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F9R1_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F9R1_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F9R1_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F9R1_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F9R1_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F9R1_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F9R1_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F9R1_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F9R1_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F10R1 register  *****************
CAN_F10R1_FB0               EQU (0x00000001)        ;Filter bit 0
CAN_F10R1_FB1               EQU (0x00000002)        ;Filter bit 1
CAN_F10R1_FB2               EQU (0x00000004)        ;Filter bit 2
CAN_F10R1_FB3               EQU (0x00000008)        ;Filter bit 3
CAN_F10R1_FB4               EQU (0x00000010)        ;Filter bit 4
CAN_F10R1_FB5               EQU (0x00000020)        ;Filter bit 5
CAN_F10R1_FB6               EQU (0x00000040)        ;Filter bit 6
CAN_F10R1_FB7               EQU (0x00000080)        ;Filter bit 7
CAN_F10R1_FB8               EQU (0x00000100)        ;Filter bit 8
CAN_F10R1_FB9               EQU (0x00000200)        ;Filter bit 9
CAN_F10R1_FB10              EQU (0x00000400)        ;Filter bit 10
CAN_F10R1_FB11              EQU (0x00000800)        ;Filter bit 11
CAN_F10R1_FB12              EQU (0x00001000)        ;Filter bit 12
CAN_F10R1_FB13              EQU (0x00002000)        ;Filter bit 13
CAN_F10R1_FB14              EQU (0x00004000)        ;Filter bit 14
CAN_F10R1_FB15              EQU (0x00008000)        ;Filter bit 15
CAN_F10R1_FB16              EQU (0x00010000)        ;Filter bit 16
CAN_F10R1_FB17              EQU (0x00020000)        ;Filter bit 17
CAN_F10R1_FB18              EQU (0x00040000)        ;Filter bit 18
CAN_F10R1_FB19              EQU (0x00080000)        ;Filter bit 19
CAN_F10R1_FB20              EQU (0x00100000)        ;Filter bit 20
CAN_F10R1_FB21              EQU (0x00200000)        ;Filter bit 21
CAN_F10R1_FB22              EQU (0x00400000)        ;Filter bit 22
CAN_F10R1_FB23              EQU (0x00800000)        ;Filter bit 23
CAN_F10R1_FB24              EQU (0x01000000)        ;Filter bit 24
CAN_F10R1_FB25              EQU (0x02000000)        ;Filter bit 25
CAN_F10R1_FB26              EQU (0x04000000)        ;Filter bit 26
CAN_F10R1_FB27              EQU (0x08000000)        ;Filter bit 27
CAN_F10R1_FB28              EQU (0x10000000)        ;Filter bit 28
CAN_F10R1_FB29              EQU (0x20000000)        ;Filter bit 29
CAN_F10R1_FB30              EQU (0x40000000)        ;Filter bit 30
CAN_F10R1_FB31              EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F11R1 register  *****************
CAN_F11R1_FB0               EQU (0x00000001)        ;Filter bit 0
CAN_F11R1_FB1               EQU (0x00000002)        ;Filter bit 1
CAN_F11R1_FB2               EQU (0x00000004)        ;Filter bit 2
CAN_F11R1_FB3               EQU (0x00000008)        ;Filter bit 3
CAN_F11R1_FB4               EQU (0x00000010)        ;Filter bit 4
CAN_F11R1_FB5               EQU (0x00000020)        ;Filter bit 5
CAN_F11R1_FB6               EQU (0x00000040)        ;Filter bit 6
CAN_F11R1_FB7               EQU (0x00000080)        ;Filter bit 7
CAN_F11R1_FB8               EQU (0x00000100)        ;Filter bit 8
CAN_F11R1_FB9               EQU (0x00000200)        ;Filter bit 9
CAN_F11R1_FB10              EQU (0x00000400)        ;Filter bit 10
CAN_F11R1_FB11              EQU (0x00000800)        ;Filter bit 11
CAN_F11R1_FB12              EQU (0x00001000)        ;Filter bit 12
CAN_F11R1_FB13              EQU (0x00002000)        ;Filter bit 13
CAN_F11R1_FB14              EQU (0x00004000)        ;Filter bit 14
CAN_F11R1_FB15              EQU (0x00008000)        ;Filter bit 15
CAN_F11R1_FB16              EQU (0x00010000)        ;Filter bit 16
CAN_F11R1_FB17              EQU (0x00020000)        ;Filter bit 17
CAN_F11R1_FB18              EQU (0x00040000)        ;Filter bit 18
CAN_F11R1_FB19              EQU (0x00080000)        ;Filter bit 19
CAN_F11R1_FB20              EQU (0x00100000)        ;Filter bit 20
CAN_F11R1_FB21              EQU (0x00200000)        ;Filter bit 21
CAN_F11R1_FB22              EQU (0x00400000)        ;Filter bit 22
CAN_F11R1_FB23              EQU (0x00800000)        ;Filter bit 23
CAN_F11R1_FB24              EQU (0x01000000)        ;Filter bit 24
CAN_F11R1_FB25              EQU (0x02000000)        ;Filter bit 25
CAN_F11R1_FB26              EQU (0x04000000)        ;Filter bit 26
CAN_F11R1_FB27              EQU (0x08000000)        ;Filter bit 27
CAN_F11R1_FB28              EQU (0x10000000)        ;Filter bit 28
CAN_F11R1_FB29              EQU (0x20000000)        ;Filter bit 29
CAN_F11R1_FB30              EQU (0x40000000)        ;Filter bit 30
CAN_F11R1_FB31              EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F12R1 register  *****************
CAN_F12R1_FB0               EQU (0x00000001)        ;Filter bit 0
CAN_F12R1_FB1               EQU (0x00000002)        ;Filter bit 1
CAN_F12R1_FB2               EQU (0x00000004)        ;Filter bit 2
CAN_F12R1_FB3               EQU (0x00000008)        ;Filter bit 3
CAN_F12R1_FB4               EQU (0x00000010)        ;Filter bit 4
CAN_F12R1_FB5               EQU (0x00000020)        ;Filter bit 5
CAN_F12R1_FB6               EQU (0x00000040)        ;Filter bit 6
CAN_F12R1_FB7               EQU (0x00000080)        ;Filter bit 7
CAN_F12R1_FB8               EQU (0x00000100)        ;Filter bit 8
CAN_F12R1_FB9               EQU (0x00000200)        ;Filter bit 9
CAN_F12R1_FB10              EQU (0x00000400)        ;Filter bit 10
CAN_F12R1_FB11              EQU (0x00000800)        ;Filter bit 11
CAN_F12R1_FB12              EQU (0x00001000)        ;Filter bit 12
CAN_F12R1_FB13              EQU (0x00002000)        ;Filter bit 13
CAN_F12R1_FB14              EQU (0x00004000)        ;Filter bit 14
CAN_F12R1_FB15              EQU (0x00008000)        ;Filter bit 15
CAN_F12R1_FB16              EQU (0x00010000)        ;Filter bit 16
CAN_F12R1_FB17              EQU (0x00020000)        ;Filter bit 17
CAN_F12R1_FB18              EQU (0x00040000)        ;Filter bit 18
CAN_F12R1_FB19              EQU (0x00080000)        ;Filter bit 19
CAN_F12R1_FB20              EQU (0x00100000)        ;Filter bit 20
CAN_F12R1_FB21              EQU (0x00200000)        ;Filter bit 21
CAN_F12R1_FB22              EQU (0x00400000)        ;Filter bit 22
CAN_F12R1_FB23              EQU (0x00800000)        ;Filter bit 23
CAN_F12R1_FB24              EQU (0x01000000)        ;Filter bit 24
CAN_F12R1_FB25              EQU (0x02000000)        ;Filter bit 25
CAN_F12R1_FB26              EQU (0x04000000)        ;Filter bit 26
CAN_F12R1_FB27              EQU (0x08000000)        ;Filter bit 27
CAN_F12R1_FB28              EQU (0x10000000)        ;Filter bit 28
CAN_F12R1_FB29              EQU (0x20000000)        ;Filter bit 29
CAN_F12R1_FB30              EQU (0x40000000)        ;Filter bit 30
CAN_F12R1_FB31              EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F13R1 register  *****************
CAN_F13R1_FB0               EQU (0x00000001)        ;Filter bit 0
CAN_F13R1_FB1               EQU (0x00000002)        ;Filter bit 1
CAN_F13R1_FB2               EQU (0x00000004)        ;Filter bit 2
CAN_F13R1_FB3               EQU (0x00000008)        ;Filter bit 3
CAN_F13R1_FB4               EQU (0x00000010)        ;Filter bit 4
CAN_F13R1_FB5               EQU (0x00000020)        ;Filter bit 5
CAN_F13R1_FB6               EQU (0x00000040)        ;Filter bit 6
CAN_F13R1_FB7               EQU (0x00000080)        ;Filter bit 7
CAN_F13R1_FB8               EQU (0x00000100)        ;Filter bit 8
CAN_F13R1_FB9               EQU (0x00000200)        ;Filter bit 9
CAN_F13R1_FB10              EQU (0x00000400)        ;Filter bit 10
CAN_F13R1_FB11              EQU (0x00000800)        ;Filter bit 11
CAN_F13R1_FB12              EQU (0x00001000)        ;Filter bit 12
CAN_F13R1_FB13              EQU (0x00002000)        ;Filter bit 13
CAN_F13R1_FB14              EQU (0x00004000)        ;Filter bit 14
CAN_F13R1_FB15              EQU (0x00008000)        ;Filter bit 15
CAN_F13R1_FB16              EQU (0x00010000)        ;Filter bit 16
CAN_F13R1_FB17              EQU (0x00020000)        ;Filter bit 17
CAN_F13R1_FB18              EQU (0x00040000)        ;Filter bit 18
CAN_F13R1_FB19              EQU (0x00080000)        ;Filter bit 19
CAN_F13R1_FB20              EQU (0x00100000)        ;Filter bit 20
CAN_F13R1_FB21              EQU (0x00200000)        ;Filter bit 21
CAN_F13R1_FB22              EQU (0x00400000)        ;Filter bit 22
CAN_F13R1_FB23              EQU (0x00800000)        ;Filter bit 23
CAN_F13R1_FB24              EQU (0x01000000)        ;Filter bit 24
CAN_F13R1_FB25              EQU (0x02000000)        ;Filter bit 25
CAN_F13R1_FB26              EQU (0x04000000)        ;Filter bit 26
CAN_F13R1_FB27              EQU (0x08000000)        ;Filter bit 27
CAN_F13R1_FB28              EQU (0x10000000)        ;Filter bit 28
CAN_F13R1_FB29              EQU (0x20000000)        ;Filter bit 29
CAN_F13R1_FB30              EQU (0x40000000)        ;Filter bit 30
CAN_F13R1_FB31              EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F0R2 register  ******************
CAN_F0R2_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F0R2_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F0R2_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F0R2_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F0R2_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F0R2_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F0R2_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F0R2_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F0R2_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F0R2_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F0R2_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F0R2_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F0R2_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F0R2_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F0R2_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F0R2_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F0R2_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F0R2_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F0R2_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F0R2_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F0R2_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F0R2_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F0R2_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F0R2_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F0R2_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F0R2_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F0R2_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F0R2_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F0R2_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F0R2_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F0R2_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F0R2_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F1R2 register  ******************
CAN_F1R2_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F1R2_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F1R2_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F1R2_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F1R2_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F1R2_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F1R2_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F1R2_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F1R2_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F1R2_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F1R2_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F1R2_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F1R2_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F1R2_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F1R2_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F1R2_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F1R2_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F1R2_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F1R2_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F1R2_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F1R2_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F1R2_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F1R2_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F1R2_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F1R2_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F1R2_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F1R2_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F1R2_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F1R2_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F1R2_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F1R2_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F1R2_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F2R2 register  ******************
CAN_F2R2_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F2R2_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F2R2_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F2R2_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F2R2_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F2R2_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F2R2_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F2R2_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F2R2_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F2R2_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F2R2_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F2R2_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F2R2_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F2R2_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F2R2_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F2R2_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F2R2_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F2R2_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F2R2_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F2R2_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F2R2_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F2R2_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F2R2_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F2R2_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F2R2_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F2R2_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F2R2_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F2R2_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F2R2_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F2R2_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F2R2_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F2R2_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F3R2 register  ******************
CAN_F3R2_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F3R2_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F3R2_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F3R2_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F3R2_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F3R2_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F3R2_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F3R2_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F3R2_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F3R2_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F3R2_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F3R2_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F3R2_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F3R2_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F3R2_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F3R2_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F3R2_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F3R2_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F3R2_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F3R2_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F3R2_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F3R2_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F3R2_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F3R2_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F3R2_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F3R2_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F3R2_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F3R2_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F3R2_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F3R2_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F3R2_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F3R2_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F4R2 register  ******************
CAN_F4R2_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F4R2_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F4R2_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F4R2_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F4R2_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F4R2_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F4R2_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F4R2_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F4R2_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F4R2_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F4R2_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F4R2_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F4R2_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F4R2_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F4R2_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F4R2_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F4R2_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F4R2_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F4R2_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F4R2_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F4R2_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F4R2_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F4R2_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F4R2_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F4R2_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F4R2_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F4R2_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F4R2_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F4R2_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F4R2_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F4R2_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F4R2_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F5R2 register  ******************
CAN_F5R2_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F5R2_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F5R2_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F5R2_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F5R2_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F5R2_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F5R2_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F5R2_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F5R2_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F5R2_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F5R2_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F5R2_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F5R2_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F5R2_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F5R2_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F5R2_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F5R2_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F5R2_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F5R2_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F5R2_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F5R2_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F5R2_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F5R2_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F5R2_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F5R2_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F5R2_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F5R2_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F5R2_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F5R2_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F5R2_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F5R2_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F5R2_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F6R2 register  ******************
CAN_F6R2_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F6R2_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F6R2_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F6R2_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F6R2_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F6R2_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F6R2_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F6R2_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F6R2_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F6R2_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F6R2_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F6R2_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F6R2_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F6R2_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F6R2_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F6R2_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F6R2_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F6R2_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F6R2_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F6R2_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F6R2_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F6R2_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F6R2_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F6R2_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F6R2_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F6R2_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F6R2_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F6R2_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F6R2_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F6R2_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F6R2_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F6R2_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F7R2 register  ******************
CAN_F7R2_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F7R2_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F7R2_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F7R2_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F7R2_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F7R2_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F7R2_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F7R2_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F7R2_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F7R2_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F7R2_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F7R2_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F7R2_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F7R2_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F7R2_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F7R2_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F7R2_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F7R2_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F7R2_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F7R2_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F7R2_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F7R2_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F7R2_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F7R2_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F7R2_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F7R2_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F7R2_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F7R2_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F7R2_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F7R2_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F7R2_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F7R2_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F8R2 register  ******************
CAN_F8R2_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F8R2_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F8R2_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F8R2_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F8R2_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F8R2_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F8R2_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F8R2_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F8R2_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F8R2_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F8R2_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F8R2_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F8R2_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F8R2_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F8R2_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F8R2_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F8R2_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F8R2_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F8R2_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F8R2_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F8R2_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F8R2_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F8R2_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F8R2_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F8R2_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F8R2_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F8R2_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F8R2_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F8R2_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F8R2_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F8R2_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F8R2_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F9R2 register  ******************
CAN_F9R2_FB0                EQU (0x00000001)        ;Filter bit 0
CAN_F9R2_FB1                EQU (0x00000002)        ;Filter bit 1
CAN_F9R2_FB2                EQU (0x00000004)        ;Filter bit 2
CAN_F9R2_FB3                EQU (0x00000008)        ;Filter bit 3
CAN_F9R2_FB4                EQU (0x00000010)        ;Filter bit 4
CAN_F9R2_FB5                EQU (0x00000020)        ;Filter bit 5
CAN_F9R2_FB6                EQU (0x00000040)        ;Filter bit 6
CAN_F9R2_FB7                EQU (0x00000080)        ;Filter bit 7
CAN_F9R2_FB8                EQU (0x00000100)        ;Filter bit 8
CAN_F9R2_FB9                EQU (0x00000200)        ;Filter bit 9
CAN_F9R2_FB10               EQU (0x00000400)        ;Filter bit 10
CAN_F9R2_FB11               EQU (0x00000800)        ;Filter bit 11
CAN_F9R2_FB12               EQU (0x00001000)        ;Filter bit 12
CAN_F9R2_FB13               EQU (0x00002000)        ;Filter bit 13
CAN_F9R2_FB14               EQU (0x00004000)        ;Filter bit 14
CAN_F9R2_FB15               EQU (0x00008000)        ;Filter bit 15
CAN_F9R2_FB16               EQU (0x00010000)        ;Filter bit 16
CAN_F9R2_FB17               EQU (0x00020000)        ;Filter bit 17
CAN_F9R2_FB18               EQU (0x00040000)        ;Filter bit 18
CAN_F9R2_FB19               EQU (0x00080000)        ;Filter bit 19
CAN_F9R2_FB20               EQU (0x00100000)        ;Filter bit 20
CAN_F9R2_FB21               EQU (0x00200000)        ;Filter bit 21
CAN_F9R2_FB22               EQU (0x00400000)        ;Filter bit 22
CAN_F9R2_FB23               EQU (0x00800000)        ;Filter bit 23
CAN_F9R2_FB24               EQU (0x01000000)        ;Filter bit 24
CAN_F9R2_FB25               EQU (0x02000000)        ;Filter bit 25
CAN_F9R2_FB26               EQU (0x04000000)        ;Filter bit 26
CAN_F9R2_FB27               EQU (0x08000000)        ;Filter bit 27
CAN_F9R2_FB28               EQU (0x10000000)        ;Filter bit 28
CAN_F9R2_FB29               EQU (0x20000000)        ;Filter bit 29
CAN_F9R2_FB30               EQU (0x40000000)        ;Filter bit 30
CAN_F9R2_FB31               EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F10R2 register  *****************
CAN_F10R2_FB0               EQU (0x00000001)        ;Filter bit 0
CAN_F10R2_FB1               EQU (0x00000002)        ;Filter bit 1
CAN_F10R2_FB2               EQU (0x00000004)        ;Filter bit 2
CAN_F10R2_FB3               EQU (0x00000008)        ;Filter bit 3
CAN_F10R2_FB4               EQU (0x00000010)        ;Filter bit 4
CAN_F10R2_FB5               EQU (0x00000020)        ;Filter bit 5
CAN_F10R2_FB6               EQU (0x00000040)        ;Filter bit 6
CAN_F10R2_FB7               EQU (0x00000080)        ;Filter bit 7
CAN_F10R2_FB8               EQU (0x00000100)        ;Filter bit 8
CAN_F10R2_FB9               EQU (0x00000200)        ;Filter bit 9
CAN_F10R2_FB10              EQU (0x00000400)        ;Filter bit 10
CAN_F10R2_FB11              EQU (0x00000800)        ;Filter bit 11
CAN_F10R2_FB12              EQU (0x00001000)        ;Filter bit 12
CAN_F10R2_FB13              EQU (0x00002000)        ;Filter bit 13
CAN_F10R2_FB14              EQU (0x00004000)        ;Filter bit 14
CAN_F10R2_FB15              EQU (0x00008000)        ;Filter bit 15
CAN_F10R2_FB16              EQU (0x00010000)        ;Filter bit 16
CAN_F10R2_FB17              EQU (0x00020000)        ;Filter bit 17
CAN_F10R2_FB18              EQU (0x00040000)        ;Filter bit 18
CAN_F10R2_FB19              EQU (0x00080000)        ;Filter bit 19
CAN_F10R2_FB20              EQU (0x00100000)        ;Filter bit 20
CAN_F10R2_FB21              EQU (0x00200000)        ;Filter bit 21
CAN_F10R2_FB22              EQU (0x00400000)        ;Filter bit 22
CAN_F10R2_FB23              EQU (0x00800000)        ;Filter bit 23
CAN_F10R2_FB24              EQU (0x01000000)        ;Filter bit 24
CAN_F10R2_FB25              EQU (0x02000000)        ;Filter bit 25
CAN_F10R2_FB26              EQU (0x04000000)        ;Filter bit 26
CAN_F10R2_FB27              EQU (0x08000000)        ;Filter bit 27
CAN_F10R2_FB28              EQU (0x10000000)        ;Filter bit 28
CAN_F10R2_FB29              EQU (0x20000000)        ;Filter bit 29
CAN_F10R2_FB30              EQU (0x40000000)        ;Filter bit 30
CAN_F10R2_FB31              EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F11R2 register  *****************
CAN_F11R2_FB0               EQU (0x00000001)        ;Filter bit 0
CAN_F11R2_FB1               EQU (0x00000002)        ;Filter bit 1
CAN_F11R2_FB2               EQU (0x00000004)        ;Filter bit 2
CAN_F11R2_FB3               EQU (0x00000008)        ;Filter bit 3
CAN_F11R2_FB4               EQU (0x00000010)        ;Filter bit 4
CAN_F11R2_FB5               EQU (0x00000020)        ;Filter bit 5
CAN_F11R2_FB6               EQU (0x00000040)        ;Filter bit 6
CAN_F11R2_FB7               EQU (0x00000080)        ;Filter bit 7
CAN_F11R2_FB8               EQU (0x00000100)        ;Filter bit 8
CAN_F11R2_FB9               EQU (0x00000200)        ;Filter bit 9
CAN_F11R2_FB10              EQU (0x00000400)        ;Filter bit 10
CAN_F11R2_FB11              EQU (0x00000800)        ;Filter bit 11
CAN_F11R2_FB12              EQU (0x00001000)        ;Filter bit 12
CAN_F11R2_FB13              EQU (0x00002000)        ;Filter bit 13
CAN_F11R2_FB14              EQU (0x00004000)        ;Filter bit 14
CAN_F11R2_FB15              EQU (0x00008000)        ;Filter bit 15
CAN_F11R2_FB16              EQU (0x00010000)        ;Filter bit 16
CAN_F11R2_FB17              EQU (0x00020000)        ;Filter bit 17
CAN_F11R2_FB18              EQU (0x00040000)        ;Filter bit 18
CAN_F11R2_FB19              EQU (0x00080000)        ;Filter bit 19
CAN_F11R2_FB20              EQU (0x00100000)        ;Filter bit 20
CAN_F11R2_FB21              EQU (0x00200000)        ;Filter bit 21
CAN_F11R2_FB22              EQU (0x00400000)        ;Filter bit 22
CAN_F11R2_FB23              EQU (0x00800000)        ;Filter bit 23
CAN_F11R2_FB24              EQU (0x01000000)        ;Filter bit 24
CAN_F11R2_FB25              EQU (0x02000000)        ;Filter bit 25
CAN_F11R2_FB26              EQU (0x04000000)        ;Filter bit 26
CAN_F11R2_FB27              EQU (0x08000000)        ;Filter bit 27
CAN_F11R2_FB28              EQU (0x10000000)        ;Filter bit 28
CAN_F11R2_FB29              EQU (0x20000000)        ;Filter bit 29
CAN_F11R2_FB30              EQU (0x40000000)        ;Filter bit 30
CAN_F11R2_FB31              EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F12R2 register  *****************
CAN_F12R2_FB0               EQU (0x00000001)        ;Filter bit 0
CAN_F12R2_FB1               EQU (0x00000002)        ;Filter bit 1
CAN_F12R2_FB2               EQU (0x00000004)        ;Filter bit 2
CAN_F12R2_FB3               EQU (0x00000008)        ;Filter bit 3
CAN_F12R2_FB4               EQU (0x00000010)        ;Filter bit 4
CAN_F12R2_FB5               EQU (0x00000020)        ;Filter bit 5
CAN_F12R2_FB6               EQU (0x00000040)        ;Filter bit 6
CAN_F12R2_FB7               EQU (0x00000080)        ;Filter bit 7
CAN_F12R2_FB8               EQU (0x00000100)        ;Filter bit 8
CAN_F12R2_FB9               EQU (0x00000200)        ;Filter bit 9
CAN_F12R2_FB10              EQU (0x00000400)        ;Filter bit 10
CAN_F12R2_FB11              EQU (0x00000800)        ;Filter bit 11
CAN_F12R2_FB12              EQU (0x00001000)        ;Filter bit 12
CAN_F12R2_FB13              EQU (0x00002000)        ;Filter bit 13
CAN_F12R2_FB14              EQU (0x00004000)        ;Filter bit 14
CAN_F12R2_FB15              EQU (0x00008000)        ;Filter bit 15
CAN_F12R2_FB16              EQU (0x00010000)        ;Filter bit 16
CAN_F12R2_FB17              EQU (0x00020000)        ;Filter bit 17
CAN_F12R2_FB18              EQU (0x00040000)        ;Filter bit 18
CAN_F12R2_FB19              EQU (0x00080000)        ;Filter bit 19
CAN_F12R2_FB20              EQU (0x00100000)        ;Filter bit 20
CAN_F12R2_FB21              EQU (0x00200000)        ;Filter bit 21
CAN_F12R2_FB22              EQU (0x00400000)        ;Filter bit 22
CAN_F12R2_FB23              EQU (0x00800000)        ;Filter bit 23
CAN_F12R2_FB24              EQU (0x01000000)        ;Filter bit 24
CAN_F12R2_FB25              EQU (0x02000000)        ;Filter bit 25
CAN_F12R2_FB26              EQU (0x04000000)        ;Filter bit 26
CAN_F12R2_FB27              EQU (0x08000000)        ;Filter bit 27
CAN_F12R2_FB28              EQU (0x10000000)        ;Filter bit 28
CAN_F12R2_FB29              EQU (0x20000000)        ;Filter bit 29
CAN_F12R2_FB30              EQU (0x40000000)        ;Filter bit 30
CAN_F12R2_FB31              EQU (0x80000000)        ;Filter bit 31

; *******************  Bit definition for CAN_F13R2 register  *****************
CAN_F13R2_FB0               EQU (0x00000001)        ;Filter bit 0
CAN_F13R2_FB1               EQU (0x00000002)        ;Filter bit 1
CAN_F13R2_FB2               EQU (0x00000004)        ;Filter bit 2
CAN_F13R2_FB3               EQU (0x00000008)        ;Filter bit 3
CAN_F13R2_FB4               EQU (0x00000010)        ;Filter bit 4
CAN_F13R2_FB5               EQU (0x00000020)        ;Filter bit 5
CAN_F13R2_FB6               EQU (0x00000040)        ;Filter bit 6
CAN_F13R2_FB7               EQU (0x00000080)        ;Filter bit 7
CAN_F13R2_FB8               EQU (0x00000100)        ;Filter bit 8
CAN_F13R2_FB9               EQU (0x00000200)        ;Filter bit 9
CAN_F13R2_FB10              EQU (0x00000400)        ;Filter bit 10
CAN_F13R2_FB11              EQU (0x00000800)        ;Filter bit 11
CAN_F13R2_FB12              EQU (0x00001000)        ;Filter bit 12
CAN_F13R2_FB13              EQU (0x00002000)        ;Filter bit 13
CAN_F13R2_FB14              EQU (0x00004000)        ;Filter bit 14
CAN_F13R2_FB15              EQU (0x00008000)        ;Filter bit 15
CAN_F13R2_FB16              EQU (0x00010000)        ;Filter bit 16
CAN_F13R2_FB17              EQU (0x00020000)        ;Filter bit 17
CAN_F13R2_FB18              EQU (0x00040000)        ;Filter bit 18
CAN_F13R2_FB19              EQU (0x00080000)        ;Filter bit 19
CAN_F13R2_FB20              EQU (0x00100000)        ;Filter bit 20
CAN_F13R2_FB21              EQU (0x00200000)        ;Filter bit 21
CAN_F13R2_FB22              EQU (0x00400000)        ;Filter bit 22
CAN_F13R2_FB23              EQU (0x00800000)        ;Filter bit 23
CAN_F13R2_FB24              EQU (0x01000000)        ;Filter bit 24
CAN_F13R2_FB25              EQU (0x02000000)        ;Filter bit 25
CAN_F13R2_FB26              EQU (0x04000000)        ;Filter bit 26
CAN_F13R2_FB27              EQU (0x08000000)        ;Filter bit 27
CAN_F13R2_FB28              EQU (0x10000000)        ;Filter bit 28
CAN_F13R2_FB29              EQU (0x20000000)        ;Filter bit 29
CAN_F13R2_FB30              EQU (0x40000000)        ;Filter bit 30
CAN_F13R2_FB31              EQU (0x80000000)        ;Filter bit 31

; *****************************************************************************
;
;                          CRC calculation unit
;
; *****************************************************************************
; *******************  Bit definition for CRC_DR register  ********************
CRC_DR_DR                   EQU (0xFFFFFFFF) ; Data register bits

; *******************  Bit definition for CRC_IDR register  *******************
CRC_IDR_IDR                 EQU (0xFF)        ; General-purpose 8-bit data register bits

; ********************  Bit definition for CRC_CR register  *******************
CRC_CR_RESET                EQU (0x00000001) ; RESET the CRC computation unit bit
CRC_CR_POLYSIZE             EQU (0x00000018) ; Polynomial size bits
CRC_CR_POLYSIZE_0           EQU (0x00000008) ; Polynomial size bit 0
CRC_CR_POLYSIZE_1           EQU (0x00000010) ; Polynomial size bit 1
CRC_CR_REV_IN               EQU (0x00000060) ; REV_IN Reverse Input Data bits
CRC_CR_REV_IN_0             EQU (0x00000020) ; Bit 0
CRC_CR_REV_IN_1             EQU (0x00000040) ; Bit 1
CRC_CR_REV_OUT              EQU (0x00000080) ; REV_OUT Reverse Output Data bits

; *******************  Bit definition for CRC_INIT register  ******************
CRC_INIT_INIT               EQU (0xFFFFFFFF) ; Initial CRC value bits

; *******************  Bit definition for CRC_POL register  *******************
CRC_POL_POL                 EQU (0xFFFFFFFF) ; Coefficients of the polynomial

; *****************************************************************************
;
;                      Digital to Analog Converter
;
; *****************************************************************************
; ********************  Bit definition for DAC_CR register  *******************
DAC_CR_EN1                  EQU (0x00000001)        ;DAC channel1 enable
DAC_CR_TEN1                 EQU (0x00000004)        ;DAC channel1 Trigger enable

DAC_CR_TSEL1                EQU (0x00000038)        ;TSEL1[2:0] (DAC channel1 Trigger selection)
DAC_CR_TSEL1_0              EQU (0x00000008)        ;Bit 0
DAC_CR_TSEL1_1              EQU (0x00000010)        ;Bit 1
DAC_CR_TSEL1_2              EQU (0x00000020)        ;Bit 2

DAC_CR_WAVE1                EQU (0x000000C0)        ;WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable)
DAC_CR_WAVE1_0              EQU (0x00000040)        ;Bit 0
DAC_CR_WAVE1_1              EQU (0x00000080)        ;Bit 1

DAC_CR_MAMP1                EQU (0x00000F00)        ;MAMP1[3:0] (DAC channel1 Mask/Amplitude selector)
DAC_CR_MAMP1_0              EQU (0x00000100)        ;Bit 0
DAC_CR_MAMP1_1              EQU (0x00000200)        ;Bit 1
DAC_CR_MAMP1_2              EQU (0x00000400)        ;Bit 2
DAC_CR_MAMP1_3              EQU (0x00000800)        ;Bit 3

DAC_CR_DMAEN1               EQU (0x00001000)        ;DAC channel1 DMA enable
DAC_CR_DMAUDRIE1            EQU (0x00002000)        ;DAC channel 1 DMA underrun interrupt enable  >
DAC_CR_CEN1                 EQU (0x00004000)        ;DAC channel 1 calibration enable >

DAC_CR_EN2                  EQU (0x00010000)        ;DAC channel2 enable
DAC_CR_TEN2                 EQU (0x00040000)        ;DAC channel2 Trigger enable

DAC_CR_TSEL2                EQU (0x00380000)        ;TSEL2[2:0] (DAC channel2 Trigger selection)
DAC_CR_TSEL2_0              EQU (0x00080000)        ;Bit 0
DAC_CR_TSEL2_1              EQU (0x00100000)        ;Bit 1
DAC_CR_TSEL2_2              EQU (0x00200000)        ;Bit 2

DAC_CR_WAVE2                EQU (0x00C00000)        ;WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable)
DAC_CR_WAVE2_0              EQU (0x00400000)        ;Bit 0
DAC_CR_WAVE2_1              EQU (0x00800000)        ;Bit 1

DAC_CR_MAMP2                EQU (0x0F000000)        ;MAMP2[3:0] (DAC channel2 Mask/Amplitude selector)
DAC_CR_MAMP2_0              EQU (0x01000000)        ;Bit 0
DAC_CR_MAMP2_1              EQU (0x02000000)        ;Bit 1
DAC_CR_MAMP2_2              EQU (0x04000000)        ;Bit 2
DAC_CR_MAMP2_3              EQU (0x08000000)        ;Bit 3

DAC_CR_DMAEN2               EQU (0x10000000)        ;DAC channel2 DMA enabled
DAC_CR_DMAUDRIE2            EQU (0x20000000)        ;DAC channel2 DMA underrun interrupt enable  >
DAC_CR_CEN2                 EQU (0x40000000)        ;DAC channel2 calibration enable >

; *****************  Bit definition for DAC_SWTRIGR register  *****************
DAC_SWTRIGR_SWTRIG1         EQU (0x00000001)        ;DAC channel1 software trigger
DAC_SWTRIGR_SWTRIG2         EQU (0x00000002)        ;DAC channel2 software trigger

; *****************  Bit definition for DAC_DHR12R1 register  *****************
DAC_DHR12R1_DACC1DHR        EQU (0x00000FFF)        ;DAC channel1 12-bit Right aligned data

; *****************  Bit definition for DAC_DHR12L1 register  *****************
DAC_DHR12L1_DACC1DHR        EQU (0x0000FFF0)        ;DAC channel1 12-bit Left aligned data

; ******************  Bit definition for DAC_DHR8R1 register  *****************
DAC_DHR8R1_DACC1DHR         EQU (0x000000FF)        ;DAC channel1 8-bit Right aligned data

; *****************  Bit definition for DAC_DHR12R2 register  *****************
DAC_DHR12R2_DACC2DHR        EQU (0x00000FFF)        ;DAC channel2 12-bit Right aligned data

; *****************  Bit definition for DAC_DHR12L2 register  *****************
DAC_DHR12L2_DACC2DHR        EQU (0x0000FFF0)        ;DAC channel2 12-bit Left aligned data

; ******************  Bit definition for DAC_DHR8R2 register  *****************
DAC_DHR8R2_DACC2DHR         EQU (0x000000FF)        ;DAC channel2 8-bit Right aligned data

; *****************  Bit definition for DAC_DHR12RD register  *****************
DAC_DHR12RD_DACC1DHR        EQU (0x00000FFF)        ;DAC channel1 12-bit Right aligned data
DAC_DHR12RD_DACC2DHR        EQU (0x0FFF0000)        ;DAC channel2 12-bit Right aligned data

; *****************  Bit definition for DAC_DHR12LD register  *****************
DAC_DHR12LD_DACC1DHR        EQU (0x0000FFF0)        ;DAC channel1 12-bit Left aligned data
DAC_DHR12LD_DACC2DHR        EQU (0xFFF00000)        ;DAC channel2 12-bit Left aligned data

; ******************  Bit definition for DAC_DHR8RD register  *****************
DAC_DHR8RD_DACC1DHR         EQU (0x000000FF)        ;DAC channel1 8-bit Right aligned data
DAC_DHR8RD_DACC2DHR         EQU (0x0000FF00)        ;DAC channel2 8-bit Right aligned data

; *******************  Bit definition for DAC_DOR1 register  ******************
DAC_DOR1_DACC1DOR           EQU (0x00000FFF)        ;DAC channel1 data output

; *******************  Bit definition for DAC_DOR2 register  ******************
DAC_DOR2_DACC2DOR           EQU (0x00000FFF)        ;DAC channel2 data output

; ********************  Bit definition for DAC_SR register  *******************
DAC_SR_DMAUDR1              EQU (0x00002000)        ;DAC channel1 DMA underrun flag
DAC_SR_CAL_FLAG1            EQU (0x00004000)        ;DAC channel1 calibration offset status
DAC_SR_BWST1                EQU (0x20008000)        ;DAC channel1 busy writing sample time flag

DAC_SR_DMAUDR2              EQU (0x20000000)        ;DAC channel2 DMA underrun flag
DAC_SR_CAL_FLAG2            EQU (0x40000000)        ;DAC channel2 calibration offset status
DAC_SR_BWST2                EQU (0x80000000)        ;DAC channel2 busy writing sample time flag

; *******************  Bit definition for DAC_CCR register  *******************
DAC_CCR_OTRIM1              EQU (0x0000001F)        ;DAC channel1 offset trimming value
DAC_CCR_OTRIM2              EQU (0x001F0000)        ;DAC channel2 offset trimming value

; *******************  Bit definition for DAC_MCR register  ******************
DAC_MCR_MODE1                EQU (0x00000007)        ;MODE1[2:0] (DAC channel1 mode)
DAC_MCR_MODE1_0              EQU (0x00000001)        ;Bit 0
DAC_MCR_MODE1_1              EQU (0x00000002)        ;Bit 1
DAC_MCR_MODE1_2              EQU (0x00000004)        ;Bit 2

DAC_MCR_MODE2                EQU (0x00070000)        ;MODE2[2:0] (DAC channel2 mode)
DAC_MCR_MODE2_0              EQU (0x00010000)        ;Bit 0
DAC_MCR_MODE2_1              EQU (0x00020000)        ;Bit 1
DAC_MCR_MODE2_2              EQU (0x00040000)        ;Bit 2

; ******************  Bit definition for DAC_SHSR1 register  *****************
DAC_SHSR1_TSAMPLE1           EQU (0x000003FF)        ;DAC channel1 sample time

; ******************  Bit definition for DAC_SHSR2 register  *****************
DAC_SHSR2_TSAMPLE2           EQU (0x000003FF)        ;DAC channel2 sample time

; ******************  Bit definition for DAC_SHHR register  *****************
DAC_SHHR_THOLD1              EQU (0x000003FF)        ;DAC channel1 hold time
DAC_SHHR_THOLD2              EQU (0x03FF0000)        ;DAC channel2 hold time

; ******************  Bit definition for DAC_SHRR register  *****************
DAC_SHRR_TREFRESH1           EQU (0x000000FF)        ;DAC channel1 refresh time
DAC_SHRR_TREFRESH2           EQU (0x00FF0000)        ;DAC channel2 refresh time


; *****************************************************************************
;
;                 Digital Filter for Sigma Delta Modulators
;
; *****************************************************************************

; ****************   DFSDM channel configuration registers  *******************

; ***************  Bit definition for DFSDM_CHCFGR1 register  *****************
DFSDM_CHCFGR1_DFSDMEN        EQU (0x80000000)            ; Global enable for DFSDM interface
DFSDM_CHCFGR1_CKOUTSRC       EQU (0x40000000)            ; Output serial clock source selection
DFSDM_CHCFGR1_CKOUTDIV       EQU (0x00FF0000)            ; CKOUTDIV[7:0] output serial clock divider
DFSDM_CHCFGR1_DATPACK        EQU (0x0000C000)            ; DATPACK[1:0] Data packing mode
DFSDM_CHCFGR1_DATPACK_1      EQU (0x00008000)            ; Data packing mode, Bit 1
DFSDM_CHCFGR1_DATPACK_0      EQU (0x00004000)            ; Data packing mode, Bit 0
DFSDM_CHCFGR1_DATMPX         EQU (0x00003000)            ; DATMPX[1:0] Input data multiplexer for channel y
DFSDM_CHCFGR1_DATMPX_1       EQU (0x00002000)            ; Input data multiplexer for channel y, Bit 1
DFSDM_CHCFGR1_DATMPX_0       EQU (0x00001000)            ; Input data multiplexer for channel y, Bit 0
DFSDM_CHCFGR1_CHINSEL        EQU (0x00000100)            ; Serial inputs selection for channel y
DFSDM_CHCFGR1_CHEN           EQU (0x00000080)            ; Channel y enable
DFSDM_CHCFGR1_CKABEN         EQU (0x00000040)            ; Clock absence detector enable on channel y
DFSDM_CHCFGR1_SCDEN          EQU (0x00000020)            ; Short circuit detector enable on channel y
DFSDM_CHCFGR1_SPICKSEL       EQU (0x0000000C)            ; SPICKSEL[1:0] SPI clock select for channel y
DFSDM_CHCFGR1_SPICKSEL_1     EQU (0x00000008)            ; SPI clock select for channel y, Bit 1
DFSDM_CHCFGR1_SPICKSEL_0     EQU (0x00000004)            ; SPI clock select for channel y, Bit 0
DFSDM_CHCFGR1_SITP           EQU (0x00000003)            ; SITP[1:0] Serial interface type for channel y
DFSDM_CHCFGR1_SITP_1         EQU (0x00000002)            ; Serial interface type for channel y, Bit 1
DFSDM_CHCFGR1_SITP_0         EQU (0x00000001)            ; Serial interface type for channel y, Bit 0

; ***************  Bit definition for DFSDM_CHCFGR2 register  *****************
DFSDM_CHCFGR2_OFFSET         EQU (0xFFFFFF00)            ; OFFSET[23:0] 24-bit calibration offset for channel y
DFSDM_CHCFGR2_DTRBS          EQU (0x000000F8)            ; DTRBS[4:0] Data right bit-shift for channel y

; ******************  Bit definition for DFSDM_AWSCDR register ****************
DFSDM_AWSCDR_AWFORD          EQU (0x00C00000)            ; AWFORD[1:0] Analog watchdog Sinc filter order on channel y
DFSDM_AWSCDR_AWFORD_1        EQU (0x00800000)            ; Analog watchdog Sinc filter order on channel y, Bit 1
DFSDM_AWSCDR_AWFORD_0        EQU (0x00400000)            ; Analog watchdog Sinc filter order on channel y, Bit 0
DFSDM_AWSCDR_AWFOSR          EQU (0x001F0000)            ; AWFOSR[4:0] Analog watchdog filter oversampling ratio on channel y
DFSDM_AWSCDR_BKSCD           EQU (0x0000F000)            ; BKSCD[3:0] Break signal assignment for short circuit detector on channel y
DFSDM_AWSCDR_SCDT            EQU (0x000000FF)            ; SCDT[7:0] Short circuit detector threshold for channel y

; ****************  Bit definition for DFSDM_CHWDATR register ******************
DFSDM_AWSCDR_WDATA           EQU (0x0000FFFF)            ; WDATA[15:0] Input channel y watchdog data

; ****************  Bit definition for DFSDM_CHDATINR register ****************
DFSDM_AWSCDR_INDAT0           EQU (0x0000FFFF)            ; INDAT0[31:16] Input data for channel y or channel (y+1)
DFSDM_AWSCDR_INDAT1           EQU (0xFFFF0000)            ; INDAT0[15:0] Input data for channel y

; ************************   DFSDM module registers  ***************************

; ********************  Bit definition for DFSDM_CR1 register ******************
DFSDM_CR1_AWFSEL             EQU (0x40000000)            ; Analog watchdog fast mode select
DFSDM_CR1_FAST               EQU (0x20000000)            ; Fast conversion mode selection
DFSDM_CR1_RCH                EQU (0x07000000)            ; RCH[2:0] Regular channel selection
DFSDM_CR1_RDMAEN             EQU (0x00200000)            ; DMA channel enabled to read data for the regular conversion
DFSDM_CR1_RSYNC              EQU (0x00080000)            ; Launch regular conversion synchronously with DFSDMx
DFSDM_CR1_RCONT              EQU (0x00040000)            ; Continuous mode selection for regular conversions
DFSDM_CR1_RSWSTART           EQU (0x00020000)            ; Software start of a conversion on the regular channel
DFSDM_CR1_JEXTEN             EQU (0x00006000)            ; JEXTEN[1:0] Trigger enable and trigger edge selection for injected conversions
DFSDM_CR1_JEXTEN_1           EQU (0x00004000)            ; Trigger enable and trigger edge selection for injected conversions, Bit 1
DFSDM_CR1_JEXTEN_0           EQU (0x00002000)            ; Trigger enable and trigger edge selection for injected conversions, Bit 0
DFSDM_CR1_JEXTSEL            EQU (0x00000700)            ; JEXTSEL[2:0]Trigger signal selection for launching injected conversions
DFSDM_CR1_JEXTSEL_2          EQU (0x00000400)            ; Trigger signal selection for launching injected conversions, Bit 2
DFSDM_CR1_JEXTSEL_1          EQU (0x00000200)            ; Trigger signal selection for launching injected conversions, Bit 1
DFSDM_CR1_JEXTSEL_0          EQU (0x00000100)            ; Trigger signal selection for launching injected conversions, Bit 0
DFSDM_CR1_JDMAEN             EQU (0x00000020)            ; DMA channel enabled to read data for the injected channel group
DFSDM_CR1_JSCAN              EQU (0x00000010)            ; Scanning conversion in continuous mode selection for injected conversions
DFSDM_CR1_JSYNC              EQU (0x00000008)            ; Launch an injected conversion synchronously with DFSDMx JSWSTART trigger
DFSDM_CR1_JSWSTART           EQU (0x00000002)            ; Start the conversion of the injected group of channels
DFSDM_CR1_DFEN               EQU (0x00000001)            ; DFSDM enable

; ********************  Bit definition for DFSDM_CR2 register ******************
DFSDM_CR2_AWDCH              EQU (0x00FF0000)            ; AWDCH[7:0] Analog watchdog channel selection
DFSDM_CR2_EXCH               EQU (0x0000FF00)            ; EXCH[7:0] Extreme detector channel selection
DFSDM_CR2_CKABIE             EQU (0x00000040)            ; Clock absence interrupt enable
DFSDM_CR2_SCDIE              EQU (0x00000020)            ; Short circuit detector interrupt enable
DFSDM_CR2_AWDIE              EQU (0x00000010)            ; Analog watchdog interrupt enable
DFSDM_CR2_ROVRIE             EQU (0x00000008)            ; Regular data overrun interrupt enable
DFSDM_CR2_JOVRIE             EQU (0x00000004)            ; Injected data overrun interrupt enable
DFSDM_CR2_REOCIE             EQU (0x00000002)            ; Regular end of conversion interrupt enable
DFSDM_CR2_JEOCIE             EQU (0x00000001)            ; Injected end of conversion interrupt enable

; ********************  Bit definition for DFSDM_ISR register ******************
DFSDM_ISR_SCDF               EQU (0xFF000000)            ; SCDF[7:0] Short circuit detector flag
DFSDM_ISR_CKABF              EQU (0x00FF0000)            ; CKABF[7:0] Clock absence flag
DFSDM_ISR_RCIP               EQU (0x00004000)            ; Regular conversion in progress status
DFSDM_ISR_JCIP               EQU (0x00002000)            ; Injected conversion in progress status
DFSDM_ISR_AWDF               EQU (0x00000010)            ; Analog watchdog
DFSDM_ISR_ROVRF              EQU (0x00000008)            ; Regular conversion overrun flag
DFSDM_ISR_JOVRF              EQU (0x00000004)            ; Injected conversion overrun flag
DFSDM_ISR_REOCF              EQU (0x00000002)            ; End of regular conversion flag
DFSDM_ISR_JEOCF              EQU (0x00000001)            ; End of injected conversion flag

; ********************  Bit definition for DFSDM_ICR register ******************
DFSDM_ICR_CLRSCSDF           EQU (0xFF000000)            ; CLRSCSDF[7:0] Clear the short circuit detector flag
DFSDM_ICR_CLRCKABF           EQU (0x00FF0000)            ; CLRCKABF[7:0] Clear the clock absence flag
DFSDM_ICR_CLRROVRF           EQU (0x00000008)            ; Clear the regular conversion overrun flag
DFSDM_ICR_CLRJOVRF           EQU (0x00000004)            ; Clear the injected conversion overrun flag

; *******************  Bit definition for DFSDM_JCHGR register *****************
DFSDM_JCHGR_JCHG             EQU (0x000000FF)            ; JCHG[7:0] Injected channel group selection

; ********************  Bit definition for DFSDM_FCR register ******************
DFSDM_FCR_FORD               EQU (0xE0000000)            ; FORD[2:0] Sinc filter order
DFSDM_FCR_FORD_2             EQU (0x80000000)            ; Sinc filter order, Bit 2
DFSDM_FCR_FORD_1             EQU (0x40000000)            ; Sinc filter order, Bit 1
DFSDM_FCR_FORD_0             EQU (0x20000000)            ; Sinc filter order, Bit 0
DFSDM_FCR_FOSR               EQU (0x03FF0000)            ; FOSR[9:0] Sinc filter oversampling ratio (decimation rate)
DFSDM_FCR_IOSR               EQU (0x000000FF)            ; IOSR[7:0] Integrator oversampling ratio (averaging length)

; ******************  Bit definition for DFSDM_JDATAR register ****************
DFSDM_JDATAR_JDATA           EQU (0xFFFFFF00)            ; JDATA[23:0] Injected group conversion data
DFSDM_JDATAR_JDATACH         EQU (0x00000007)            ; JDATACH[2:0] Injected channel most recently converted

; ******************  Bit definition for DFSDM_RDATAR register ****************
DFSDM_RDATAR_RDATA           EQU (0xFFFFFF00)            ; RDATA[23:0] Regular channel conversion data
DFSDM_RDATAR_RPEND           EQU (0x00000010)            ; RPEND Regular channel pending data
DFSDM_RDATAR_RDATACH         EQU (0x00000007)            ; RDATACH[2:0] Regular channel most recently converted

; ******************  Bit definition for DFSDM_AWHTR register *****************
DFSDM_AWHTR_AWHT            EQU (0xFFFFFF00)             ; AWHT[23:0] Analog watchdog high threshold
DFSDM_AWHTR_BKAWH           EQU (0x0000000F)             ; BKAWH[3:0] Break signal assignment to analog watchdog high threshold event

; ******************  Bit definition for DFSDM_AWLTR register *****************
DFSDM_AWLTR_AWLT            EQU (0xFFFFFF00)             ; AWHT[23:0] Analog watchdog low threshold
DFSDM_AWLTR_BKAWL           EQU (0x0000000F)             ; BKAWL[3:0] Break signal assignment to analog watchdog low threshold event

; ******************  Bit definition for DFSDM_AWSR register *****************
DFSDM_AWSR_AWHTF            EQU (0x0000FF00)             ; AWHTF[15:8] Analog watchdog high threshold error on given channels
DFSDM_AWSR_AWLTF            EQU (0x000000FF)             ; AWLTF[7:0] Analog watchdog low threshold error on given channels

; ******************  Bit definition for DFSDM_AWCFR) register ****************
DFSDM_AWCFR_CLRAWHTF        EQU (0x0000FF00)             ; CLRAWHTF[15:8] Clear the Analog watchdog high threshold flag
DFSDM_AWCFR_CLRAWLTF        EQU (0x000000FF)             ; CLRAWLTF[7:0] Clear the Analog watchdog low threshold flag

; ******************  Bit definition for DFSDM_EXMAX register *****************
DFSDM_EXMAX_EXMAX           EQU (0xFFFFFF00)             ; EXMAX[23:0] Extreme detector maximum value
DFSDM_EXMAX_EXMAXCH         EQU (0x00000007)             ; EXMAXCH[2:0] Extreme detector maximum data channel

; ******************  Bit definition for DFSDM_EXMIN register *****************
DFSDM_EXMIN_EXMIN           EQU (0xFFFFFF00)             ; EXMIN[23:0] Extreme detector minimum value
DFSDM_EXMIN_EXMINCH         EQU (0x00000007)             ; EXMINCH[2:0] Extreme detector minimum data channel

; ******************  Bit definition for DFSDM_EXMIN register *****************
DFSDM_CNVTIMR_CNVCNT        EQU (0xFFFFFFF0)             ; CNVCNT[27:0]: 28-bit timer counting conversion time

; *****************************************************************************
;
;                           DMA Controller (DMA)
;
; *****************************************************************************

; *******************  Bit definition for DMA_ISR register  *******************
DMA_ISR_GIF1                EQU (0x00000001)        ; Channel 1 Global interrupt flag
DMA_ISR_TCIF1               EQU (0x00000002)        ; Channel 1 Transfer Complete flag
DMA_ISR_HTIF1               EQU (0x00000004)        ; Channel 1 Half Transfer flag
DMA_ISR_TEIF1               EQU (0x00000008)        ; Channel 1 Transfer Error flag
DMA_ISR_GIF2                EQU (0x00000010)        ; Channel 2 Global interrupt flag
DMA_ISR_TCIF2               EQU (0x00000020)        ; Channel 2 Transfer Complete flag
DMA_ISR_HTIF2               EQU (0x00000040)        ; Channel 2 Half Transfer flag
DMA_ISR_TEIF2               EQU (0x00000080)        ; Channel 2 Transfer Error flag
DMA_ISR_GIF3                EQU (0x00000100)        ; Channel 3 Global interrupt flag
DMA_ISR_TCIF3               EQU (0x00000200)        ; Channel 3 Transfer Complete flag
DMA_ISR_HTIF3               EQU (0x00000400)        ; Channel 3 Half Transfer flag
DMA_ISR_TEIF3               EQU (0x00000800)        ; Channel 3 Transfer Error flag
DMA_ISR_GIF4                EQU (0x00001000)        ; Channel 4 Global interrupt flag
DMA_ISR_TCIF4               EQU (0x00002000)        ; Channel 4 Transfer Complete flag
DMA_ISR_HTIF4               EQU (0x00004000)        ; Channel 4 Half Transfer flag
DMA_ISR_TEIF4               EQU (0x00008000)        ; Channel 4 Transfer Error flag
DMA_ISR_GIF5                EQU (0x00010000)        ; Channel 5 Global interrupt flag
DMA_ISR_TCIF5               EQU (0x00020000)        ; Channel 5 Transfer Complete flag
DMA_ISR_HTIF5               EQU (0x00040000)        ; Channel 5 Half Transfer flag
DMA_ISR_TEIF5               EQU (0x00080000)        ; Channel 5 Transfer Error flag
DMA_ISR_GIF6                EQU (0x00100000)        ; Channel 6 Global interrupt flag
DMA_ISR_TCIF6               EQU (0x00200000)        ; Channel 6 Transfer Complete flag
DMA_ISR_HTIF6               EQU (0x00400000)        ; Channel 6 Half Transfer flag
DMA_ISR_TEIF6               EQU (0x00800000)        ; Channel 6 Transfer Error flag
DMA_ISR_GIF7                EQU (0x01000000)        ; Channel 7 Global interrupt flag
DMA_ISR_TCIF7               EQU (0x02000000)        ; Channel 7 Transfer Complete flag
DMA_ISR_HTIF7               EQU (0x04000000)        ; Channel 7 Half Transfer flag
DMA_ISR_TEIF7               EQU (0x08000000)        ; Channel 7 Transfer Error flag

; *******************  Bit definition for DMA_IFCR register  ******************
DMA_IFCR_CGIF1              EQU (0x00000001)        ; Channel 1 Global interrupt clearr
DMA_IFCR_CTCIF1             EQU (0x00000002)        ; Channel 1 Transfer Complete clear
DMA_IFCR_CHTIF1             EQU (0x00000004)        ; Channel 1 Half Transfer clear
DMA_IFCR_CTEIF1             EQU (0x00000008)        ; Channel 1 Transfer Error clear
DMA_IFCR_CGIF2              EQU (0x00000010)        ; Channel 2 Global interrupt clear
DMA_IFCR_CTCIF2             EQU (0x00000020)        ; Channel 2 Transfer Complete clear
DMA_IFCR_CHTIF2             EQU (0x00000040)        ; Channel 2 Half Transfer clear
DMA_IFCR_CTEIF2             EQU (0x00000080)        ; Channel 2 Transfer Error clear
DMA_IFCR_CGIF3              EQU (0x00000100)        ; Channel 3 Global interrupt clear
DMA_IFCR_CTCIF3             EQU (0x00000200)        ; Channel 3 Transfer Complete clear
DMA_IFCR_CHTIF3             EQU (0x00000400)        ; Channel 3 Half Transfer clear
DMA_IFCR_CTEIF3             EQU (0x00000800)        ; Channel 3 Transfer Error clear
DMA_IFCR_CGIF4              EQU (0x00001000)        ; Channel 4 Global interrupt clear
DMA_IFCR_CTCIF4             EQU (0x00002000)        ; Channel 4 Transfer Complete clear
DMA_IFCR_CHTIF4             EQU (0x00004000)        ; Channel 4 Half Transfer clear
DMA_IFCR_CTEIF4             EQU (0x00008000)        ; Channel 4 Transfer Error clear
DMA_IFCR_CGIF5              EQU (0x00010000)        ; Channel 5 Global interrupt clear
DMA_IFCR_CTCIF5             EQU (0x00020000)        ; Channel 5 Transfer Complete clear
DMA_IFCR_CHTIF5             EQU (0x00040000)        ; Channel 5 Half Transfer clear
DMA_IFCR_CTEIF5             EQU (0x00080000)        ; Channel 5 Transfer Error clear
DMA_IFCR_CGIF6              EQU (0x00100000)        ; Channel 6 Global interrupt clear
DMA_IFCR_CTCIF6             EQU (0x00200000)        ; Channel 6 Transfer Complete clear
DMA_IFCR_CHTIF6             EQU (0x00400000)        ; Channel 6 Half Transfer clear
DMA_IFCR_CTEIF6             EQU (0x00800000)        ; Channel 6 Transfer Error clear
DMA_IFCR_CGIF7              EQU (0x01000000)        ; Channel 7 Global interrupt clear
DMA_IFCR_CTCIF7             EQU (0x02000000)        ; Channel 7 Transfer Complete clear
DMA_IFCR_CHTIF7             EQU (0x04000000)        ; Channel 7 Half Transfer clear
DMA_IFCR_CTEIF7             EQU (0x08000000)        ; Channel 7 Transfer Error clear

; *******************  Bit definition for DMA_CCR register  *******************
DMA_CCR_EN                  EQU (0x00000001)        ; Channel enable
DMA_CCR_TCIE                EQU (0x00000002)        ; Transfer complete interrupt enable
DMA_CCR_HTIE                EQU (0x00000004)        ; Half Transfer interrupt enable
DMA_CCR_TEIE                EQU (0x00000008)        ; Transfer error interrupt enable
DMA_CCR_DIR                 EQU (0x00000010)        ; Data transfer direction
DMA_CCR_CIRC                EQU (0x00000020)        ; Circular mode
DMA_CCR_PINC                EQU (0x00000040)        ; Peripheral increment mode
DMA_CCR_MINC                EQU (0x00000080)        ; Memory increment mode

DMA_CCR_PSIZE               EQU (0x00000300)        ; PSIZE[1:0] bits (Peripheral size)
DMA_CCR_PSIZE_0             EQU (0x00000100)        ; Bit 0
DMA_CCR_PSIZE_1             EQU (0x00000200)        ; Bit 1

DMA_CCR_MSIZE               EQU (0x00000C00)        ; MSIZE[1:0] bits (Memory size)
DMA_CCR_MSIZE_0             EQU (0x00000400)        ; Bit 0
DMA_CCR_MSIZE_1             EQU (0x00000800)        ; Bit 1

DMA_CCR_PL                  EQU (0x00003000)        ; PL[1:0] bits(Channel Priority level)
DMA_CCR_PL_0                EQU (0x00001000)        ; Bit 0
DMA_CCR_PL_1                EQU (0x00002000)        ; Bit 1

DMA_CCR_MEM2MEM             EQU (0x00004000)        ; Memory to memory mode

; ******************  Bit definition for DMA_CNDTR register  ******************
DMA_CNDTR_NDT               EQU (0x0000FFFF)        ; Number of data to Transfer

; ******************  Bit definition for DMA_CPAR register  *******************
DMA_CPAR_PA                 EQU (0xFFFFFFFF)        ; Peripheral Address

; ******************  Bit definition for DMA_CMAR register  *******************
DMA_CMAR_MA                 EQU (0xFFFFFFFF)        ; Memory Address


; *******************  Bit definition for DMA_CSELR register  ******************
DMA_CSELR_C1S                  EQU (0x0000000F)          ; Channel 1 Selection
DMA_CSELR_C2S                  EQU (0x000000F0)          ; Channel 2 Selection
DMA_CSELR_C3S                  EQU (0x00000F00)          ; Channel 3 Selection
DMA_CSELR_C4S                  EQU (0x0000F000)          ; Channel 4 Selection
DMA_CSELR_C5S                  EQU (0x000F0000)          ; Channel 5 Selection
DMA_CSELR_C6S                  EQU (0x00F00000)          ; Channel 6 Selection
DMA_CSELR_C7S                  EQU (0x0F000000)          ; Channel 7 Selection


; *****************************************************************************
;
;                    External Interrupt/Event Controller
;
; *****************************************************************************
; *******************  Bit definition for EXTI_IMR1 register  *****************
EXTI_IMR1_IM0               EQU (0x00000001)        ; Interrupt Mask on line 0
EXTI_IMR1_IM1               EQU (0x00000002)        ; Interrupt Mask on line 1
EXTI_IMR1_IM2               EQU (0x00000004)        ; Interrupt Mask on line 2
EXTI_IMR1_IM3               EQU (0x00000008)        ; Interrupt Mask on line 3
EXTI_IMR1_IM4               EQU (0x00000010)        ; Interrupt Mask on line 4
EXTI_IMR1_IM5               EQU (0x00000020)        ; Interrupt Mask on line 5
EXTI_IMR1_IM6               EQU (0x00000040)        ; Interrupt Mask on line 6
EXTI_IMR1_IM7               EQU (0x00000080)        ; Interrupt Mask on line 7
EXTI_IMR1_IM8               EQU (0x00000100)        ; Interrupt Mask on line 8
EXTI_IMR1_IM9               EQU (0x00000200)        ; Interrupt Mask on line 9
EXTI_IMR1_IM10              EQU (0x00000400)        ; Interrupt Mask on line 10
EXTI_IMR1_IM11              EQU (0x00000800)        ; Interrupt Mask on line 11
EXTI_IMR1_IM12              EQU (0x00001000)        ; Interrupt Mask on line 12
EXTI_IMR1_IM13              EQU (0x00002000)        ; Interrupt Mask on line 13
EXTI_IMR1_IM14              EQU (0x00004000)        ; Interrupt Mask on line 14
EXTI_IMR1_IM15              EQU (0x00008000)        ; Interrupt Mask on line 15
EXTI_IMR1_IM16              EQU (0x00010000)        ; Interrupt Mask on line 16
EXTI_IMR1_IM17              EQU (0x00020000)        ; Interrupt Mask on line 17
EXTI_IMR1_IM18              EQU (0x00040000)        ; Interrupt Mask on line 18
EXTI_IMR1_IM19              EQU (0x00080000)        ; Interrupt Mask on line 19
EXTI_IMR1_IM20              EQU (0x00100000)        ; Interrupt Mask on line 20
EXTI_IMR1_IM21              EQU (0x00200000)        ; Interrupt Mask on line 21
EXTI_IMR1_IM22              EQU (0x00400000)        ; Interrupt Mask on line 22
EXTI_IMR1_IM23              EQU (0x00800000)        ; Interrupt Mask on line 23
EXTI_IMR1_IM24              EQU (0x01000000)        ; Interrupt Mask on line 24
EXTI_IMR1_IM25              EQU (0x02000000)        ; Interrupt Mask on line 25
EXTI_IMR1_IM26              EQU (0x04000000)        ; Interrupt Mask on line 26
EXTI_IMR1_IM27              EQU (0x08000000)        ; Interrupt Mask on line 27
EXTI_IMR1_IM28              EQU (0x10000000)        ; Interrupt Mask on line 28
EXTI_IMR1_IM29              EQU (0x20000000)        ; Interrupt Mask on line 29
EXTI_IMR1_IM30              EQU (0x40000000)        ; Interrupt Mask on line 30
EXTI_IMR1_IM31              EQU (0x80000000)        ; Interrupt Mask on line 31

; *******************  Bit definition for EXTI_EMR1 register  *****************
EXTI_EMR1_EM0               EQU (0x00000001)        ; Event Mask on line 0
EXTI_EMR1_EM1               EQU (0x00000002)        ; Event Mask on line 1
EXTI_EMR1_EM2               EQU (0x00000004)        ; Event Mask on line 2
EXTI_EMR1_EM3               EQU (0x00000008)        ; Event Mask on line 3
EXTI_EMR1_EM4               EQU (0x00000010)        ; Event Mask on line 4
EXTI_EMR1_EM5               EQU (0x00000020)        ; Event Mask on line 5
EXTI_EMR1_EM6               EQU (0x00000040)        ; Event Mask on line 6
EXTI_EMR1_EM7               EQU (0x00000080)        ; Event Mask on line 7
EXTI_EMR1_EM8               EQU (0x00000100)        ; Event Mask on line 8
EXTI_EMR1_EM9               EQU (0x00000200)        ; Event Mask on line 9
EXTI_EMR1_EM10              EQU (0x00000400)        ; Event Mask on line 10
EXTI_EMR1_EM11              EQU (0x00000800)        ; Event Mask on line 11
EXTI_EMR1_EM12              EQU (0x00001000)        ; Event Mask on line 12
EXTI_EMR1_EM13              EQU (0x00002000)        ; Event Mask on line 13
EXTI_EMR1_EM14              EQU (0x00004000)        ; Event Mask on line 14
EXTI_EMR1_EM15              EQU (0x00008000)        ; Event Mask on line 15
EXTI_EMR1_EM16              EQU (0x00010000)        ; Event Mask on line 16
EXTI_EMR1_EM17              EQU (0x00020000)        ; Event Mask on line 17
EXTI_EMR1_EM18              EQU (0x00040000)        ; Event Mask on line 18
EXTI_EMR1_EM19              EQU (0x00080000)        ; Event Mask on line 19
EXTI_EMR1_EM20              EQU (0x00100000)        ; Event Mask on line 20
EXTI_EMR1_EM21              EQU (0x00200000)        ; Event Mask on line 21
EXTI_EMR1_EM22              EQU (0x00400000)        ; Event Mask on line 22
EXTI_EMR1_EM23              EQU (0x00800000)        ; Event Mask on line 23
EXTI_EMR1_EM24              EQU (0x01000000)        ; Event Mask on line 24
EXTI_EMR1_EM25              EQU (0x02000000)        ; Event Mask on line 25
EXTI_EMR1_EM26              EQU (0x04000000)        ; Event Mask on line 26
EXTI_EMR1_EM27              EQU (0x08000000)        ; Event Mask on line 27
EXTI_EMR1_EM28              EQU (0x10000000)        ; Event Mask on line 28
EXTI_EMR1_EM29              EQU (0x20000000)        ; Event Mask on line 29
EXTI_EMR1_EM30              EQU (0x40000000)        ; Event Mask on line 30
EXTI_EMR1_EM31              EQU (0x80000000)        ; Event Mask on line 31

; ******************  Bit definition for EXTI_RTSR1 register  *****************
EXTI_RTSR1_RT0              EQU (0x00000001)        ; Rising trigger event configuration bit of line 0
EXTI_RTSR1_RT1              EQU (0x00000002)        ; Rising trigger event configuration bit of line 1
EXTI_RTSR1_RT2              EQU (0x00000004)        ; Rising trigger event configuration bit of line 2
EXTI_RTSR1_RT3              EQU (0x00000008)        ; Rising trigger event configuration bit of line 3
EXTI_RTSR1_RT4              EQU (0x00000010)        ; Rising trigger event configuration bit of line 4
EXTI_RTSR1_RT5              EQU (0x00000020)        ; Rising trigger event configuration bit of line 5
EXTI_RTSR1_RT6              EQU (0x00000040)        ; Rising trigger event configuration bit of line 6
EXTI_RTSR1_RT7              EQU (0x00000080)        ; Rising trigger event configuration bit of line 7
EXTI_RTSR1_RT8              EQU (0x00000100)        ; Rising trigger event configuration bit of line 8
EXTI_RTSR1_RT9              EQU (0x00000200)        ; Rising trigger event configuration bit of line 9
EXTI_RTSR1_RT10             EQU (0x00000400)        ; Rising trigger event configuration bit of line 10
EXTI_RTSR1_RT11             EQU (0x00000800)        ; Rising trigger event configuration bit of line 11
EXTI_RTSR1_RT12             EQU (0x00001000)        ; Rising trigger event configuration bit of line 12
EXTI_RTSR1_RT13             EQU (0x00002000)        ; Rising trigger event configuration bit of line 13
EXTI_RTSR1_RT14             EQU (0x00004000)        ; Rising trigger event configuration bit of line 14
EXTI_RTSR1_RT15             EQU (0x00008000)        ; Rising trigger event configuration bit of line 15
EXTI_RTSR1_RT16             EQU (0x00010000)        ; Rising trigger event configuration bit of line 16
EXTI_RTSR1_RT18             EQU (0x00040000)        ; Rising trigger event configuration bit of line 18
EXTI_RTSR1_RT19             EQU (0x00080000)        ; Rising trigger event configuration bit of line 19
EXTI_RTSR1_RT20             EQU (0x00100000)        ; Rising trigger event configuration bit of line 20
EXTI_RTSR1_RT21             EQU (0x00200000)        ; Rising trigger event configuration bit of line 21
EXTI_RTSR1_RT22             EQU (0x00400000)        ; Rising trigger event configuration bit of line 22

; ******************  Bit definition for EXTI_FTSR1 register  *****************
EXTI_FTSR1_FT0              EQU (0x00000001)        ; Falling trigger event configuration bit of line 0
EXTI_FTSR1_FT1              EQU (0x00000002)        ; Falling trigger event configuration bit of line 1
EXTI_FTSR1_FT2              EQU (0x00000004)        ; Falling trigger event configuration bit of line 2
EXTI_FTSR1_FT3              EQU (0x00000008)        ; Falling trigger event configuration bit of line 3
EXTI_FTSR1_FT4              EQU (0x00000010)        ; Falling trigger event configuration bit of line 4
EXTI_FTSR1_FT5              EQU (0x00000020)        ; Falling trigger event configuration bit of line 5
EXTI_FTSR1_FT6              EQU (0x00000040)        ; Falling trigger event configuration bit of line 6
EXTI_FTSR1_FT7              EQU (0x00000080)        ; Falling trigger event configuration bit of line 7
EXTI_FTSR1_FT8              EQU (0x00000100)        ; Falling trigger event configuration bit of line 8
EXTI_FTSR1_FT9              EQU (0x00000200)        ; Falling trigger event configuration bit of line 9
EXTI_FTSR1_FT10             EQU (0x00000400)        ; Falling trigger event configuration bit of line 10
EXTI_FTSR1_FT11             EQU (0x00000800)        ; Falling trigger event configuration bit of line 11
EXTI_FTSR1_FT12             EQU (0x00001000)        ; Falling trigger event configuration bit of line 12
EXTI_FTSR1_FT13             EQU (0x00002000)        ; Falling trigger event configuration bit of line 13
EXTI_FTSR1_FT14             EQU (0x00004000)        ; Falling trigger event configuration bit of line 14
EXTI_FTSR1_FT15             EQU (0x00008000)        ; Falling trigger event configuration bit of line 15
EXTI_FTSR1_FT16             EQU (0x00010000)        ; Falling trigger event configuration bit of line 16
EXTI_FTSR1_FT18             EQU (0x00040000)        ; Falling trigger event configuration bit of line 18
EXTI_FTSR1_FT19             EQU (0x00080000)        ; Falling trigger event configuration bit of line 19
EXTI_FTSR1_FT20             EQU (0x00100000)        ; Falling trigger event configuration bit of line 20
EXTI_FTSR1_FT21             EQU (0x00200000)        ; Falling trigger event configuration bit of line 21
EXTI_FTSR1_FT22             EQU (0x00400000)        ; Falling trigger event configuration bit of line 22

; ******************  Bit definition for EXTI_SWIER1 register  ****************
EXTI_SWIER1_SWI0            EQU (0x00000001)        ; Software Interrupt on line 0
EXTI_SWIER1_SWI1            EQU (0x00000002)        ; Software Interrupt on line 1
EXTI_SWIER1_SWI2            EQU (0x00000004)        ; Software Interrupt on line 2
EXTI_SWIER1_SWI3            EQU (0x00000008)        ; Software Interrupt on line 3
EXTI_SWIER1_SWI4            EQU (0x00000010)        ; Software Interrupt on line 4
EXTI_SWIER1_SWI5            EQU (0x00000020)        ; Software Interrupt on line 5
EXTI_SWIER1_SWI6            EQU (0x00000040)        ; Software Interrupt on line 6
EXTI_SWIER1_SWI7            EQU (0x00000080)        ; Software Interrupt on line 7
EXTI_SWIER1_SWI8            EQU (0x00000100)        ; Software Interrupt on line 8
EXTI_SWIER1_SWI9            EQU (0x00000200)        ; Software Interrupt on line 9
EXTI_SWIER1_SWI10           EQU (0x00000400)        ; Software Interrupt on line 10
EXTI_SWIER1_SWI11           EQU (0x00000800)        ; Software Interrupt on line 11
EXTI_SWIER1_SWI12           EQU (0x00001000)        ; Software Interrupt on line 12
EXTI_SWIER1_SWI13           EQU (0x00002000)        ; Software Interrupt on line 13
EXTI_SWIER1_SWI14           EQU (0x00004000)        ; Software Interrupt on line 14
EXTI_SWIER1_SWI15           EQU (0x00008000)        ; Software Interrupt on line 15
EXTI_SWIER1_SWI16           EQU (0x00010000)        ; Software Interrupt on line 16
EXTI_SWIER1_SWI18           EQU (0x00040000)        ; Software Interrupt on line 18
EXTI_SWIER1_SWI19           EQU (0x00080000)        ; Software Interrupt on line 19
EXTI_SWIER1_SWI20           EQU (0x00100000)        ; Software Interrupt on line 20
EXTI_SWIER1_SWI21           EQU (0x00200000)        ; Software Interrupt on line 21
EXTI_SWIER1_SWI22           EQU (0x00400000)        ; Software Interrupt on line 22

; *******************  Bit definition for EXTI_PR1 register  ******************
EXTI_PR1_PIF0               EQU (0x00000001)        ; Pending bit for line 0
EXTI_PR1_PIF1               EQU (0x00000002)        ; Pending bit for line 1
EXTI_PR1_PIF2               EQU (0x00000004)        ; Pending bit for line 2
EXTI_PR1_PIF3               EQU (0x00000008)        ; Pending bit for line 3
EXTI_PR1_PIF4               EQU (0x00000010)        ; Pending bit for line 4
EXTI_PR1_PIF5               EQU (0x00000020)        ; Pending bit for line 5
EXTI_PR1_PIF6               EQU (0x00000040)        ; Pending bit for line 6
EXTI_PR1_PIF7               EQU (0x00000080)        ; Pending bit for line 7
EXTI_PR1_PIF8               EQU (0x00000100)        ; Pending bit for line 8
EXTI_PR1_PIF9               EQU (0x00000200)        ; Pending bit for line 9
EXTI_PR1_PIF10              EQU (0x00000400)        ; Pending bit for line 10
EXTI_PR1_PIF11              EQU (0x00000800)        ; Pending bit for line 11
EXTI_PR1_PIF12              EQU (0x00001000)        ; Pending bit for line 12
EXTI_PR1_PIF13              EQU (0x00002000)        ; Pending bit for line 13
EXTI_PR1_PIF14              EQU (0x00004000)        ; Pending bit for line 14
EXTI_PR1_PIF15              EQU (0x00008000)        ; Pending bit for line 15
EXTI_PR1_PIF16              EQU (0x00010000)        ; Pending bit for line 16
EXTI_PR1_PIF18              EQU (0x00040000)        ; Pending bit for line 18
EXTI_PR1_PIF19              EQU (0x00080000)        ; Pending bit for line 19
EXTI_PR1_PIF20              EQU (0x00100000)        ; Pending bit for line 20
EXTI_PR1_PIF21              EQU (0x00200000)        ; Pending bit for line 21
EXTI_PR1_PIF22              EQU (0x00400000)        ; Pending bit for line 22

; *******************  Bit definition for EXTI_IMR2 register  *****************
EXTI_IMR2_IM32              EQU (0x00000001)        ; Interrupt Mask on line 32
EXTI_IMR2_IM33              EQU (0x00000002)        ; Interrupt Mask on line 33
EXTI_IMR2_IM34              EQU (0x00000004)        ; Interrupt Mask on line 34
EXTI_IMR2_IM35              EQU (0x00000008)        ; Interrupt Mask on line 35
EXTI_IMR2_IM36              EQU (0x00000010)        ; Interrupt Mask on line 36
EXTI_IMR2_IM37              EQU (0x00000020)        ; Interrupt Mask on line 37
EXTI_IMR2_IM38              EQU (0x00000040)        ; Interrupt Mask on line 38
EXTI_IMR2_IM39              EQU (0x00000080)        ; Interrupt Mask on line 39

; *******************  Bit definition for EXTI_EMR2 register  *****************
EXTI_EMR2_EM32              EQU (0x00000001)        ; Event Mask on line 32
EXTI_EMR2_EM33              EQU (0x00000002)        ; Event Mask on line 33
EXTI_EMR2_EM34              EQU (0x00000004)        ; Event Mask on line 34
EXTI_EMR2_EM35              EQU (0x00000008)        ; Event Mask on line 35
EXTI_EMR2_EM36              EQU (0x00000010)        ; Event Mask on line 36
EXTI_EMR2_EM37              EQU (0x00000020)        ; Event Mask on line 37
EXTI_EMR2_EM38              EQU (0x00000040)        ; Event Mask on line 38
EXTI_EMR2_EM39              EQU (0x00000080)        ; Event Mask on line 39

; ******************  Bit definition for EXTI_RTSR2 register  *****************
EXTI_RTSR2_RT35             EQU (0x00000008)        ; Rising trigger event configuration bit of line 35
EXTI_RTSR2_RT36             EQU (0x00000010)        ; Rising trigger event configuration bit of line 36
EXTI_RTSR2_RT37             EQU (0x00000020)        ; Rising trigger event configuration bit of line 37
EXTI_RTSR2_RT38             EQU (0x00000040)        ; Rising trigger event configuration bit of line 38

; ******************  Bit definition for EXTI_FTSR2 register  *****************
EXTI_FTSR2_FT35             EQU (0x00000008)        ; Falling trigger event configuration bit of line 35
EXTI_FTSR2_FT36             EQU (0x00000010)        ; Falling trigger event configuration bit of line 36
EXTI_FTSR2_FT37             EQU (0x00000020)        ; Falling trigger event configuration bit of line 37
EXTI_FTSR2_FT38             EQU (0x00000040)        ; Falling trigger event configuration bit of line 38

; ******************  Bit definition for EXTI_SWIER2 register  ****************
EXTI_SWIER2_SWI35           EQU (0x00000008)        ; Software Interrupt on line 35
EXTI_SWIER2_SWI36           EQU (0x00000010)        ; Software Interrupt on line 36
EXTI_SWIER2_SWI37           EQU (0x00000020)        ; Software Interrupt on line 37
EXTI_SWIER2_SWI38           EQU (0x00000040)        ; Software Interrupt on line 38

; *******************  Bit definition for EXTI_PR2 register  ******************
EXTI_PR2_PIF35               EQU (0x00000008)        ; Pending bit for line 35
EXTI_PR2_PIF36               EQU (0x00000010)        ; Pending bit for line 36
EXTI_PR2_PIF37               EQU (0x00000020)        ; Pending bit for line 37
EXTI_PR2_PIF38               EQU (0x00000040)        ; Pending bit for line 38


; *****************************************************************************
;
;                                    FLASH
;
; *****************************************************************************
; *******************  Bits definition for FLASH_ACR register  ****************
FLASH_ACR_LATENCY            EQU (0x00000007)
FLASH_ACR_LATENCY_0WS        EQU (0x00000000)
FLASH_ACR_LATENCY_1WS        EQU (0x00000001)
FLASH_ACR_LATENCY_2WS        EQU (0x00000002)
FLASH_ACR_LATENCY_3WS        EQU (0x00000003)
FLASH_ACR_LATENCY_4WS        EQU (0x00000004)
FLASH_ACR_PRFTEN             EQU (0x00000100)
FLASH_ACR_ICEN               EQU (0x00000200)
FLASH_ACR_DCEN               EQU (0x00000400)
FLASH_ACR_ICRST              EQU (0x00000800)
FLASH_ACR_DCRST              EQU (0x00001000)
FLASH_ACR_RUN_PD             EQU (0x00002000)   ; Flash power down mode during run
FLASH_ACR_SLEEP_PD           EQU (0x00004000)   ; Flash power down mode during sleep

; *******************  Bits definition for FLASH_SR register  *****************
FLASH_SR_EOP                 EQU (0x00000001)
FLASH_SR_OPERR               EQU (0x00000002)
FLASH_SR_PROGERR             EQU (0x00000008)
FLASH_SR_WRPERR              EQU (0x00000010)
FLASH_SR_PGAERR              EQU (0x00000020)
FLASH_SR_SIZERR              EQU (0x00000040)
FLASH_SR_PGSERR              EQU (0x00000080)
FLASH_SR_MISERR              EQU (0x00000100)
FLASH_SR_FASTERR             EQU (0x00000200)
FLASH_SR_RDERR               EQU (0x00004000)
FLASH_SR_OPTVERR             EQU (0x00008000)
FLASH_SR_BSY                 EQU (0x00010000)

; *******************  Bits definition for FLASH_CR register  *****************
FLASH_CR_PG                  EQU (0x00000001)
FLASH_CR_PER                 EQU (0x00000002)
FLASH_CR_MER1                EQU (0x00000004)
FLASH_CR_PNB                 EQU (0x000007F8)
FLASH_CR_BKER                EQU (0x00000800)
FLASH_CR_MER2                EQU (0x00008000)
FLASH_CR_STRT                EQU (0x00010000)
FLASH_CR_OPTSTRT             EQU (0x00020000)
FLASH_CR_FSTPG               EQU (0x00040000)
FLASH_CR_EOPIE               EQU (0x01000000)
FLASH_CR_ERRIE               EQU (0x02000000)
FLASH_CR_RDERRIE             EQU (0x04000000)
FLASH_CR_OBL_LAUNCH          EQU (0x08000000)
FLASH_CR_OPTLOCK             EQU (0x40000000)
FLASH_CR_LOCK                EQU (0x80000000)

; *******************  Bits definition for FLASH_ECCR register  **************
FLASH_ECCR_ADDR_ECC         EQU (0x0007FFFF)
FLASH_ECCR_BK_ECC           EQU (0x00080000)
FLASH_ECCR_SYSF_ECC         EQU (0x00100000)
FLASH_ECCR_ECCIE            EQU (0x01000000)
FLASH_ECCR_ECCC             EQU (0x40000000)
FLASH_ECCR_ECCD             EQU (0x80000000)

; *******************  Bits definition for FLASH_OPTR register  **************
FLASH_OPTR_RDP              EQU (0x000000FF)
FLASH_OPTR_BOR_LEV          EQU (0x00000700)
FLASH_OPTR_BOR_LEV_0        EQU (0x00000000)
FLASH_OPTR_BOR_LEV_1        EQU (0x00000100)
FLASH_OPTR_BOR_LEV_2        EQU (0x00000200)
FLASH_OPTR_BOR_LEV_3        EQU (0x00000300)
FLASH_OPTR_BOR_LEV_4        EQU (0x00000400)
FLASH_OPTR_nRST_STOP        EQU (0x00001000)
FLASH_OPTR_nRST_STDBY       EQU (0x00002000)
FLASH_OPTR_nRST_SHDW        EQU (0x00004000)
FLASH_OPTR_IWDG_SW          EQU (0x00010000)
FLASH_OPTR_IWDG_STOP        EQU (0x00020000)
FLASH_OPTR_IWDG_STDBY       EQU (0x00040000)
FLASH_OPTR_WWDG_SW          EQU (0x00080000)
FLASH_OPTR_BFB2             EQU (0x00100000)
FLASH_OPTR_DUALBANK         EQU (0x00200000)
FLASH_OPTR_nBOOT1           EQU (0x00800000)
FLASH_OPTR_SRAM2_PE         EQU (0x01000000)
FLASH_OPTR_SRAM2_RST        EQU (0x02000000)

; ******************  Bits definition for FLASH_PCROP1SR register  *********
FLASH_PCROP1SR_PCROP1_STRT  EQU (0x0000FFFF)

; ******************  Bits definition for FLASH_PCROP1ER register  **********
FLASH_PCROP1ER_PCROP1_END   EQU (0x0000FFFF)
FLASH_PCROP1ER_PCROP_RDP    EQU (0x80000000)

; ******************  Bits definition for FLASH_WRP1AR register  **************
FLASH_WRP1AR_WRP1A_STRT     EQU (0x000000FF)
FLASH_WRP1AR_WRP1A_END      EQU (0x00FF0000)

; ******************  Bits definition for FLASH_WRPB1R register  **************
FLASH_WRP1BR_WRP1B_STRT     EQU (0x000000FF)
FLASH_WRP1BR_WRP1B_END      EQU (0x00FF0000)

; ******************  Bits definition for FLASH_PCROP2SR register  *********
FLASH_PCROP2SR_PCROP2_STRT  EQU (0x0000FFFF)

; ******************  Bits definition for FLASH_PCROP2ER register  **********
FLASH_PCROP2ER_PCROP2_END  EQU (0x0000FFFF)

; ******************  Bits definition for FLASH_WRP2AR register  **************
FLASH_WRP2AR_WRP2A_STRT     EQU (0x000000FF)
FLASH_WRP2AR_WRP2A_END      EQU (0x00FF0000)

; ******************  Bits definition for FLASH_WRP2BR register  **************
FLASH_WRP2BR_WRP2B_STRT     EQU (0x000000FF)
FLASH_WRP2BR_WRP2B_END      EQU (0x00FF0000)


; *****************************************************************************
;
;                          Flexible Memory Controller
;
; *****************************************************************************
; ******************  Bit definition for FMC_BCR1 register  ******************
FMC_BCR1_CCLKEN            EQU (0x00100000)        ;Continous clock enable
FMC_BCR1_WFDIS             EQU (0x00200000)        ;Write FIFO Disable

; ******************  Bit definition for FMC_BCRx registers (xEQU 1..4)  ********
FMC_BCRx_MBKEN             EQU (0x00000001)        ;Memory bank enable bit
FMC_BCRx_MUXEN             EQU (0x00000002)        ;Address/data multiplexing enable bit

FMC_BCRx_MTYP              EQU (0x0000000C)        ;MTYP[1:0] bits (Memory type)
FMC_BCRx_MTYP_0            EQU (0x00000004)        ;Bit 0
FMC_BCRx_MTYP_1            EQU (0x00000008)        ;Bit 1

FMC_BCRx_MWID              EQU (0x00000030)        ;MWID[1:0] bits (Memory data bus width)
FMC_BCRx_MWID_0            EQU (0x00000010)        ;Bit 0
FMC_BCRx_MWID_1            EQU (0x00000020)        ;Bit 1

FMC_BCRx_FACCEN            EQU (0x00000040)        ;Flash access enable
FMC_BCRx_BURSTEN           EQU (0x00000100)        ;Burst enable bit
FMC_BCRx_WAITPOL           EQU (0x00000200)        ;Wait signal polarity bit
FMC_BCRx_WAITCFG           EQU (0x00000800)        ;Wait timing configuration
FMC_BCRx_WREN              EQU (0x00001000)        ;Write enable bit
FMC_BCRx_WAITEN            EQU (0x00002000)        ;Wait enable bit
FMC_BCRx_EXTMOD            EQU (0x00004000)        ;Extended mode enable
FMC_BCRx_ASYNCWAIT         EQU (0x00008000)        ;Asynchronous wait

FMC_BCRx_CPSIZE            EQU (0x00070000)        ;CRAM page size
FMC_BCRx_CPSIZE_0          EQU (0x00010000)        ;Bit 0
FMC_BCRx_CPSIZE_1          EQU (0x00020000)        ;Bit 1
FMC_BCRx_CPSIZE_2          EQU (0x00040000)        ;Bit 1

FMC_BCRx_CBURSTRW          EQU (0x00080000)        ;Write burst enable

; ******************  Bit definition for FMC_BTRx registers (xEQU 1..4)  ********
FMC_BTRx_ADDSET            EQU (0x0000000F)        ;ADDSET[3:0] bits (Address setup phase duration)
FMC_BTRx_ADDSET_0          EQU (0x00000001)        ;Bit 0
FMC_BTRx_ADDSET_1          EQU (0x00000002)        ;Bit 1
FMC_BTRx_ADDSET_2          EQU (0x00000004)        ;Bit 2
FMC_BTRx_ADDSET_3          EQU (0x00000008)        ;Bit 3

FMC_BTRx_ADDHLD            EQU (0x000000F0)        ;ADDHLD[3:0] bits (Address-hold phase duration)
FMC_BTRx_ADDHLD_0          EQU (0x00000010)        ;Bit 0
FMC_BTRx_ADDHLD_1          EQU (0x00000020)        ;Bit 1
FMC_BTRx_ADDHLD_2          EQU (0x00000040)        ;Bit 2
FMC_BTRx_ADDHLD_3          EQU (0x00000080)        ;Bit 3

FMC_BTRx_DATAST            EQU (0x0000FF00)        ;DATAST [3:0] bits (Data-phase duration)
FMC_BTRx_DATAST_0          EQU (0x00000100)        ;Bit 0
FMC_BTRx_DATAST_1          EQU (0x00000200)        ;Bit 1
FMC_BTRx_DATAST_2          EQU (0x00000400)        ;Bit 2
FMC_BTRx_DATAST_3          EQU (0x00000800)        ;Bit 3
FMC_BTRx_DATAST_4          EQU (0x00001000)        ;Bit 4
FMC_BTRx_DATAST_5          EQU (0x00002000)        ;Bit 5
FMC_BTRx_DATAST_6          EQU (0x00004000)        ;Bit 6
FMC_BTRx_DATAST_7          EQU (0x00008000)        ;Bit 7

FMC_BTRx_BUSTURN           EQU (0x000F0000)        ;BUSTURN[3:0] bits (Bus turnaround phase duration)
FMC_BTRx_BUSTURN_0         EQU (0x00010000)        ;Bit 0
FMC_BTRx_BUSTURN_1         EQU (0x00020000)        ;Bit 1
FMC_BTRx_BUSTURN_2         EQU (0x00040000)        ;Bit 2
FMC_BTRx_BUSTURN_3         EQU (0x00080000)        ;Bit 3

FMC_BTRx_CLKDIV            EQU (0x00F00000)        ;CLKDIV[3:0] bits (Clock divide ratio)
FMC_BTRx_CLKDIV_0          EQU (0x00100000)        ;Bit 0
FMC_BTRx_CLKDIV_1          EQU (0x00200000)        ;Bit 1
FMC_BTRx_CLKDIV_2          EQU (0x00400000)        ;Bit 2
FMC_BTRx_CLKDIV_3          EQU (0x00800000)        ;Bit 3

FMC_BTRx_DATLAT            EQU (0x0F000000)        ;DATLA[3:0] bits (Data latency)
FMC_BTRx_DATLAT_0          EQU (0x01000000)        ;Bit 0
FMC_BTRx_DATLAT_1          EQU (0x02000000)        ;Bit 1
FMC_BTRx_DATLAT_2          EQU (0x04000000)        ;Bit 2
FMC_BTRx_DATLAT_3          EQU (0x08000000)        ;Bit 3

FMC_BTRx_ACCMOD            EQU (0x30000000)        ;ACCMOD[1:0] bits (Access mode)
FMC_BTRx_ACCMOD_0          EQU (0x10000000)        ;Bit 0
FMC_BTRx_ACCMOD_1          EQU (0x20000000)        ;Bit 1

; ******************  Bit definition for FMC_BWTRx registers (xEQU 1..4)  ********
FMC_BWTRx_ADDSET           EQU (0x0000000F)        ;ADDSET[3:0] bits (Address setup phase duration)
FMC_BWTRx_ADDSET_0         EQU (0x00000001)        ;Bit 0
FMC_BWTRx_ADDSET_1         EQU (0x00000002)        ;Bit 1
FMC_BWTRx_ADDSET_2         EQU (0x00000004)        ;Bit 2
FMC_BWTRx_ADDSET_3         EQU (0x00000008)        ;Bit 3

FMC_BWTRx_ADDHLD           EQU (0x000000F0)        ;ADDHLD[3:0] bits (Address-hold phase duration)
FMC_BWTRx_ADDHLD_0         EQU (0x00000010)        ;Bit 0
FMC_BWTRx_ADDHLD_1         EQU (0x00000020)        ;Bit 1
FMC_BWTRx_ADDHLD_2         EQU (0x00000040)        ;Bit 2
FMC_BWTRx_ADDHLD_3         EQU (0x00000080)        ;Bit 3

FMC_BWTRx_DATAST           EQU (0x0000FF00)        ;DATAST [3:0] bits (Data-phase duration)
FMC_BWTRx_DATAST_0         EQU (0x00000100)        ;Bit 0
FMC_BWTRx_DATAST_1         EQU (0x00000200)        ;Bit 1
FMC_BWTRx_DATAST_2         EQU (0x00000400)        ;Bit 2
FMC_BWTRx_DATAST_3         EQU (0x00000800)        ;Bit 3
FMC_BWTRx_DATAST_4         EQU (0x00001000)        ;Bit 4
FMC_BWTRx_DATAST_5         EQU (0x00002000)        ;Bit 5
FMC_BWTRx_DATAST_6         EQU (0x00004000)        ;Bit 6
FMC_BWTRx_DATAST_7         EQU (0x00008000)        ;Bit 7

FMC_BWTRx_ACCMOD           EQU (0x30000000)        ;ACCMOD[1:0] bits (Access mode)
FMC_BWTRx_ACCMOD_0         EQU (0x10000000)        ;Bit 0
FMC_BWTRx_ACCMOD_1         EQU (0x20000000)        ;Bit 1

; ******************  Bit definition for FMC_PCR register  *******************
FMC_PCR_PWAITEN            EQU (0x00000002)        ;Wait feature enable bit
FMC_PCR_PBKEN              EQU (0x00000004)        ;NAND Flash memory bank enable bit
FMC_PCR_PTYP               EQU (0x00000008)        ;Memory type

FMC_PCR_PWID               EQU (0x00000030)        ;PWID[1:0] bits (NAND Flash databus width)
FMC_PCR_PWID_0             EQU (0x00000010)        ;Bit 0
FMC_PCR_PWID_1             EQU (0x00000020)        ;Bit 1

FMC_PCR_ECCEN              EQU (0x00000040)        ;ECC computation logic enable bit

FMC_PCR_TCLR               EQU (0x00001E00)        ;TCLR[3:0] bits (CLE to RE delay)
FMC_PCR_TCLR_0             EQU (0x00000200)        ;Bit 0
FMC_PCR_TCLR_1             EQU (0x00000400)        ;Bit 1
FMC_PCR_TCLR_2             EQU (0x00000800)        ;Bit 2
FMC_PCR_TCLR_3             EQU (0x00001000)        ;Bit 3

FMC_PCR_TAR                EQU (0x0001E000)        ;TAR[3:0] bits (ALE to RE delay)
FMC_PCR_TAR_0              EQU (0x00002000)        ;Bit 0
FMC_PCR_TAR_1              EQU (0x00004000)        ;Bit 1
FMC_PCR_TAR_2              EQU (0x00008000)        ;Bit 2
FMC_PCR_TAR_3              EQU (0x00010000)        ;Bit 3

FMC_PCR_ECCPS              EQU (0x000E0000)        ;ECCPS[1:0] bits (ECC page size)
FMC_PCR_ECCPS_0            EQU (0x00020000)        ;Bit 0
FMC_PCR_ECCPS_1            EQU (0x00040000)        ;Bit 1
FMC_PCR_ECCPS_2            EQU (0x00080000)        ;Bit 2

; *******************  Bit definition for FMC_SR register  *******************
FMC_SR_IRS                 EQU (0x00000001)        ;Interrupt Rising Edge status
FMC_SR_ILS                 EQU (0x00000002)        ;Interrupt Level status
FMC_SR_IFS                 EQU (0x00000004)        ;Interrupt Falling Edge status
FMC_SR_IREN                EQU (0x00000008)        ;Interrupt Rising Edge detection Enable bit
FMC_SR_ILEN                EQU (0x00000010)        ;Interrupt Level detection Enable bit
FMC_SR_IFEN                EQU (0x00000020)        ;Interrupt Falling Edge detection Enable bit
FMC_SR_FEMPT               EQU (0x00000040)        ;FIFO empty

; ******************  Bit definition for FMC_PMEM register  *****************
FMC_PMEM_MEMSET            EQU (0x000000FF)        ;MEMSET[7:0] bits (Common memory setup time)
FMC_PMEM_MEMSET_0          EQU (0x00000001)        ;Bit 0
FMC_PMEM_MEMSET_1          EQU (0x00000002)        ;Bit 1
FMC_PMEM_MEMSET_2          EQU (0x00000004)        ;Bit 2
FMC_PMEM_MEMSET_3          EQU (0x00000008)        ;Bit 3
FMC_PMEM_MEMSET_4          EQU (0x00000010)        ;Bit 4
FMC_PMEM_MEMSET_5          EQU (0x00000020)        ;Bit 5
FMC_PMEM_MEMSET_6          EQU (0x00000040)        ;Bit 6
FMC_PMEM_MEMSET_7          EQU (0x00000080)        ;Bit 7

FMC_PMEM_MEMWAIT           EQU (0x0000FF00)        ;MEMWAIT[7:0] bits (Common memory wait time)
FMC_PMEM_MEMWAIT_0         EQU (0x00000100)        ;Bit 0
FMC_PMEM_MEMWAIT_1         EQU (0x00000200)        ;Bit 1
FMC_PMEM_MEMWAIT_2         EQU (0x00000400)        ;Bit 2
FMC_PMEM_MEMWAIT_3         EQU (0x00000800)        ;Bit 3
FMC_PMEM_MEMWAIT_4         EQU (0x00001000)        ;Bit 4
FMC_PMEM_MEMWAIT_5         EQU (0x00002000)        ;Bit 5
FMC_PMEM_MEMWAIT_6         EQU (0x00004000)        ;Bit 6
FMC_PMEM_MEMWAIT_7         EQU (0x00008000)        ;Bit 7

FMC_PMEM_MEMHOLD           EQU (0x00FF0000)        ;MEMHOLD[7:0] bits (Common memory hold time)
FMC_PMEM_MEMHOLD_0         EQU (0x00010000)        ;Bit 0
FMC_PMEM_MEMHOLD_1         EQU (0x00020000)        ;Bit 1
FMC_PMEM_MEMHOLD_2         EQU (0x00040000)        ;Bit 2
FMC_PMEM_MEMHOLD_3         EQU (0x00080000)        ;Bit 3
FMC_PMEM_MEMHOLD_4         EQU (0x00100000)        ;Bit 4
FMC_PMEM_MEMHOLD_5         EQU (0x00200000)        ;Bit 5
FMC_PMEM_MEMHOLD_6         EQU (0x00400000)        ;Bit 6
FMC_PMEM_MEMHOLD_7         EQU (0x00800000)        ;Bit 7

FMC_PMEM_MEMHIZ            EQU (0xFF000000)        ;MEMHIZ[7:0] bits (Common memory databus HiZ time)
FMC_PMEM_MEMHIZ_0          EQU (0x01000000)        ;Bit 0
FMC_PMEM_MEMHIZ_1          EQU (0x02000000)        ;Bit 1
FMC_PMEM_MEMHIZ_2          EQU (0x04000000)        ;Bit 2
FMC_PMEM_MEMHIZ_3          EQU (0x08000000)        ;Bit 3
FMC_PMEM_MEMHIZ_4          EQU (0x10000000)        ;Bit 4
FMC_PMEM_MEMHIZ_5          EQU (0x20000000)        ;Bit 5
FMC_PMEM_MEMHIZ_6          EQU (0x40000000)        ;Bit 6
FMC_PMEM_MEMHIZ_7          EQU (0x80000000)        ;Bit 7

; ******************  Bit definition for FMC_PATT register  ******************
FMC_PATT_ATTSET            EQU (0x000000FF)        ;ATTSET[7:0] bits (Attribute memory setup time)
FMC_PATT_ATTSET_0          EQU (0x00000001)        ;Bit 0
FMC_PATT_ATTSET_1          EQU (0x00000002)        ;Bit 1
FMC_PATT_ATTSET_2          EQU (0x00000004)        ;Bit 2
FMC_PATT_ATTSET_3          EQU (0x00000008)        ;Bit 3
FMC_PATT_ATTSET_4          EQU (0x00000010)        ;Bit 4
FMC_PATT_ATTSET_5          EQU (0x00000020)        ;Bit 5
FMC_PATT_ATTSET_6          EQU (0x00000040)        ;Bit 6
FMC_PATT_ATTSET_7          EQU (0x00000080)        ;Bit 7

FMC_PATT_ATTWAIT           EQU (0x0000FF00)        ;ATTWAIT[7:0] bits (Attribute memory wait time)
FMC_PATT_ATTWAIT_0         EQU (0x00000100)        ;Bit 0
FMC_PATT_ATTWAIT_1         EQU (0x00000200)        ;Bit 1
FMC_PATT_ATTWAIT_2         EQU (0x00000400)        ;Bit 2
FMC_PATT_ATTWAIT_3         EQU (0x00000800)        ;Bit 3
FMC_PATT_ATTWAIT_4         EQU (0x00001000)        ;Bit 4
FMC_PATT_ATTWAIT_5         EQU (0x00002000)        ;Bit 5
FMC_PATT_ATTWAIT_6         EQU (0x00004000)        ;Bit 6
FMC_PATT_ATTWAIT_7         EQU (0x00008000)        ;Bit 7

FMC_PATT_ATTHOLD           EQU (0x00FF0000)        ;ATTHOLD[7:0] bits (Attribute memory hold time)
FMC_PATT_ATTHOLD_0         EQU (0x00010000)        ;Bit 0
FMC_PATT_ATTHOLD_1         EQU (0x00020000)        ;Bit 1
FMC_PATT_ATTHOLD_2         EQU (0x00040000)        ;Bit 2
FMC_PATT_ATTHOLD_3         EQU (0x00080000)        ;Bit 3
FMC_PATT_ATTHOLD_4         EQU (0x00100000)        ;Bit 4
FMC_PATT_ATTHOLD_5         EQU (0x00200000)        ;Bit 5
FMC_PATT_ATTHOLD_6         EQU (0x00400000)        ;Bit 6
FMC_PATT_ATTHOLD_7         EQU (0x00800000)        ;Bit 7

FMC_PATT_ATTHIZ            EQU (0xFF000000)        ;ATTHIZ[7:0] bits (Attribute memory databus HiZ time)
FMC_PATT_ATTHIZ_0          EQU (0x01000000)        ;Bit 0
FMC_PATT_ATTHIZ_1          EQU (0x02000000)        ;Bit 1
FMC_PATT_ATTHIZ_2          EQU (0x04000000)        ;Bit 2
FMC_PATT_ATTHIZ_3          EQU (0x08000000)        ;Bit 3
FMC_PATT_ATTHIZ_4          EQU (0x10000000)        ;Bit 4
FMC_PATT_ATTHIZ_5          EQU (0x20000000)        ;Bit 5
FMC_PATT_ATTHIZ_6          EQU (0x40000000)        ;Bit 6
FMC_PATT_ATTHIZ_7            EQU (0x80000000)        ;Bit 7

; ******************  Bit definition for FMC_ECCR register  ******************
FMC_ECCR_ECC               EQU (0xFFFFFFFF)        ;ECC result

; *****************************************************************************
;
;                            General Purpose I/O
;
; *****************************************************************************
; ******************  Bits definition for GPIO_MODER register  ****************
GPIO_MODER_MODER0           EQU (0x00000003)
GPIO_MODER_MODER0_0         EQU (0x00000001)
GPIO_MODER_MODER0_1         EQU (0x00000002)
GPIO_MODER_MODER1           EQU (0x0000000C)
GPIO_MODER_MODER1_0         EQU (0x00000004)
GPIO_MODER_MODER1_1         EQU (0x00000008)
GPIO_MODER_MODER2           EQU (0x00000030)
GPIO_MODER_MODER2_0         EQU (0x00000010)
GPIO_MODER_MODER2_1         EQU (0x00000020)
GPIO_MODER_MODER3           EQU (0x000000C0)
GPIO_MODER_MODER3_0         EQU (0x00000040)
GPIO_MODER_MODER3_1         EQU (0x00000080)
GPIO_MODER_MODER4           EQU (0x00000300)
GPIO_MODER_MODER4_0         EQU (0x00000100)
GPIO_MODER_MODER4_1         EQU (0x00000200)
GPIO_MODER_MODER5           EQU (0x00000C00)
GPIO_MODER_MODER5_0         EQU (0x00000400)
GPIO_MODER_MODER5_1         EQU (0x00000800)
GPIO_MODER_MODER6           EQU (0x00003000)
GPIO_MODER_MODER6_0         EQU (0x00001000)
GPIO_MODER_MODER6_1         EQU (0x00002000)
GPIO_MODER_MODER7           EQU (0x0000C000)
GPIO_MODER_MODER7_0         EQU (0x00004000)
GPIO_MODER_MODER7_1         EQU (0x00008000)
GPIO_MODER_MODER8           EQU (0x00030000)
GPIO_MODER_MODER8_0         EQU (0x00010000)
GPIO_MODER_MODER8_1         EQU (0x00020000)
GPIO_MODER_MODER9           EQU (0x000C0000)
GPIO_MODER_MODER9_0         EQU (0x00040000)
GPIO_MODER_MODER9_1         EQU (0x00080000)
GPIO_MODER_MODER10          EQU (0x00300000)
GPIO_MODER_MODER10_0        EQU (0x00100000)
GPIO_MODER_MODER10_1        EQU (0x00200000)
GPIO_MODER_MODER11          EQU (0x00C00000)
GPIO_MODER_MODER11_0        EQU (0x00400000)
GPIO_MODER_MODER11_1        EQU (0x00800000)
GPIO_MODER_MODER12          EQU (0x03000000)
GPIO_MODER_MODER12_0        EQU (0x01000000)
GPIO_MODER_MODER12_1        EQU (0x02000000)
GPIO_MODER_MODER13          EQU (0x0C000000)
GPIO_MODER_MODER13_0        EQU (0x04000000)
GPIO_MODER_MODER13_1        EQU (0x08000000)
GPIO_MODER_MODER14          EQU (0x30000000)
GPIO_MODER_MODER14_0        EQU (0x10000000)
GPIO_MODER_MODER14_1        EQU (0x20000000)
GPIO_MODER_MODER15          EQU (0xC0000000)
GPIO_MODER_MODER15_0        EQU (0x40000000)
GPIO_MODER_MODER15_1        EQU (0x80000000)

; ******************  Bits definition for GPIO_OTYPER register  ***************
GPIO_OTYPER_OT_0            EQU (0x00000001)
GPIO_OTYPER_OT_1            EQU (0x00000002)
GPIO_OTYPER_OT_2            EQU (0x00000004)
GPIO_OTYPER_OT_3            EQU (0x00000008)
GPIO_OTYPER_OT_4            EQU (0x00000010)
GPIO_OTYPER_OT_5            EQU (0x00000020)
GPIO_OTYPER_OT_6            EQU (0x00000040)
GPIO_OTYPER_OT_7            EQU (0x00000080)
GPIO_OTYPER_OT_8            EQU (0x00000100)
GPIO_OTYPER_OT_9            EQU (0x00000200)
GPIO_OTYPER_OT_10           EQU (0x00000400)
GPIO_OTYPER_OT_11           EQU (0x00000800)
GPIO_OTYPER_OT_12           EQU (0x00001000)
GPIO_OTYPER_OT_13           EQU (0x00002000)
GPIO_OTYPER_OT_14           EQU (0x00004000)
GPIO_OTYPER_OT_15           EQU (0x00008000)

; ******************  Bits definition for GPIO_OSPEEDR register  **************
GPIO_OSPEEDER_OSPEEDR0      EQU (0x00000003)
GPIO_OSPEEDER_OSPEEDR0_0    EQU (0x00000001)
GPIO_OSPEEDER_OSPEEDR0_1    EQU (0x00000002)
GPIO_OSPEEDER_OSPEEDR1      EQU (0x0000000C)
GPIO_OSPEEDER_OSPEEDR1_0    EQU (0x00000004)
GPIO_OSPEEDER_OSPEEDR1_1    EQU (0x00000008)
GPIO_OSPEEDER_OSPEEDR2      EQU (0x00000030)
GPIO_OSPEEDER_OSPEEDR2_0    EQU (0x00000010)
GPIO_OSPEEDER_OSPEEDR2_1    EQU (0x00000020)
GPIO_OSPEEDER_OSPEEDR3      EQU (0x000000C0)
GPIO_OSPEEDER_OSPEEDR3_0    EQU (0x00000040)
GPIO_OSPEEDER_OSPEEDR3_1    EQU (0x00000080)
GPIO_OSPEEDER_OSPEEDR4      EQU (0x00000300)
GPIO_OSPEEDER_OSPEEDR4_0    EQU (0x00000100)
GPIO_OSPEEDER_OSPEEDR4_1    EQU (0x00000200)
GPIO_OSPEEDER_OSPEEDR5      EQU (0x00000C00)
GPIO_OSPEEDER_OSPEEDR5_0    EQU (0x00000400)
GPIO_OSPEEDER_OSPEEDR5_1    EQU (0x00000800)
GPIO_OSPEEDER_OSPEEDR6      EQU (0x00003000)
GPIO_OSPEEDER_OSPEEDR6_0    EQU (0x00001000)
GPIO_OSPEEDER_OSPEEDR6_1    EQU (0x00002000)
GPIO_OSPEEDER_OSPEEDR7      EQU (0x0000C000)
GPIO_OSPEEDER_OSPEEDR7_0    EQU (0x00004000)
GPIO_OSPEEDER_OSPEEDR7_1    EQU (0x00008000)
GPIO_OSPEEDER_OSPEEDR8      EQU (0x00030000)
GPIO_OSPEEDER_OSPEEDR8_0    EQU (0x00010000)
GPIO_OSPEEDER_OSPEEDR8_1    EQU (0x00020000)
GPIO_OSPEEDER_OSPEEDR9      EQU (0x000C0000)
GPIO_OSPEEDER_OSPEEDR9_0    EQU (0x00040000)
GPIO_OSPEEDER_OSPEEDR9_1    EQU (0x00080000)
GPIO_OSPEEDER_OSPEEDR10     EQU (0x00300000)
GPIO_OSPEEDER_OSPEEDR10_0   EQU (0x00100000)
GPIO_OSPEEDER_OSPEEDR10_1   EQU (0x00200000)
GPIO_OSPEEDER_OSPEEDR11     EQU (0x00C00000)
GPIO_OSPEEDER_OSPEEDR11_0   EQU (0x00400000)
GPIO_OSPEEDER_OSPEEDR11_1   EQU (0x00800000)
GPIO_OSPEEDER_OSPEEDR12     EQU (0x03000000)
GPIO_OSPEEDER_OSPEEDR12_0   EQU (0x01000000)
GPIO_OSPEEDER_OSPEEDR12_1   EQU (0x02000000)
GPIO_OSPEEDER_OSPEEDR13     EQU (0x0C000000)
GPIO_OSPEEDER_OSPEEDR13_0   EQU (0x04000000)
GPIO_OSPEEDER_OSPEEDR13_1   EQU (0x08000000)
GPIO_OSPEEDER_OSPEEDR14     EQU (0x30000000)
GPIO_OSPEEDER_OSPEEDR14_0   EQU (0x10000000)
GPIO_OSPEEDER_OSPEEDR14_1   EQU (0x20000000)
GPIO_OSPEEDER_OSPEEDR15     EQU (0xC0000000)
GPIO_OSPEEDER_OSPEEDR15_0   EQU (0x40000000)
GPIO_OSPEEDER_OSPEEDR15_1   EQU (0x80000000)

; ******************  Bits definition for GPIO_PUPDR register  ****************
GPIO_PUPDR_PUPDR0           EQU (0x00000003)
GPIO_PUPDR_PUPDR0_0         EQU (0x00000001)
GPIO_PUPDR_PUPDR0_1         EQU (0x00000002)
GPIO_PUPDR_PUPDR1           EQU (0x0000000C)
GPIO_PUPDR_PUPDR1_0         EQU (0x00000004)
GPIO_PUPDR_PUPDR1_1         EQU (0x00000008)
GPIO_PUPDR_PUPDR2           EQU (0x00000030)
GPIO_PUPDR_PUPDR2_0         EQU (0x00000010)
GPIO_PUPDR_PUPDR2_1         EQU (0x00000020)
GPIO_PUPDR_PUPDR3           EQU (0x000000C0)
GPIO_PUPDR_PUPDR3_0         EQU (0x00000040)
GPIO_PUPDR_PUPDR3_1         EQU (0x00000080)
GPIO_PUPDR_PUPDR4           EQU (0x00000300)
GPIO_PUPDR_PUPDR4_0         EQU (0x00000100)
GPIO_PUPDR_PUPDR4_1         EQU (0x00000200)
GPIO_PUPDR_PUPDR5           EQU (0x00000C00)
GPIO_PUPDR_PUPDR5_0         EQU (0x00000400)
GPIO_PUPDR_PUPDR5_1         EQU (0x00000800)
GPIO_PUPDR_PUPDR6           EQU (0x00003000)
GPIO_PUPDR_PUPDR6_0         EQU (0x00001000)
GPIO_PUPDR_PUPDR6_1         EQU (0x00002000)
GPIO_PUPDR_PUPDR7           EQU (0x0000C000)
GPIO_PUPDR_PUPDR7_0         EQU (0x00004000)
GPIO_PUPDR_PUPDR7_1         EQU (0x00008000)
GPIO_PUPDR_PUPDR8           EQU (0x00030000)
GPIO_PUPDR_PUPDR8_0         EQU (0x00010000)
GPIO_PUPDR_PUPDR8_1         EQU (0x00020000)
GPIO_PUPDR_PUPDR9           EQU (0x000C0000)
GPIO_PUPDR_PUPDR9_0         EQU (0x00040000)
GPIO_PUPDR_PUPDR9_1         EQU (0x00080000)
GPIO_PUPDR_PUPDR10          EQU (0x00300000)
GPIO_PUPDR_PUPDR10_0        EQU (0x00100000)
GPIO_PUPDR_PUPDR10_1        EQU (0x00200000)
GPIO_PUPDR_PUPDR11          EQU (0x00C00000)
GPIO_PUPDR_PUPDR11_0        EQU (0x00400000)
GPIO_PUPDR_PUPDR11_1        EQU (0x00800000)
GPIO_PUPDR_PUPDR12          EQU (0x03000000)
GPIO_PUPDR_PUPDR12_0        EQU (0x01000000)
GPIO_PUPDR_PUPDR12_1        EQU (0x02000000)
GPIO_PUPDR_PUPDR13          EQU (0x0C000000)
GPIO_PUPDR_PUPDR13_0        EQU (0x04000000)
GPIO_PUPDR_PUPDR13_1        EQU (0x08000000)
GPIO_PUPDR_PUPDR14          EQU (0x30000000)
GPIO_PUPDR_PUPDR14_0        EQU (0x10000000)
GPIO_PUPDR_PUPDR14_1        EQU (0x20000000)
GPIO_PUPDR_PUPDR15          EQU (0xC0000000)
GPIO_PUPDR_PUPDR15_0        EQU (0x40000000)
GPIO_PUPDR_PUPDR15_1        EQU (0x80000000)

; ******************  Bits definition for GPIO_IDR register  ******************
GPIO_IDR_IDR_0              EQU (0x00000001)
GPIO_IDR_IDR_1              EQU (0x00000002)
GPIO_IDR_IDR_2              EQU (0x00000004)
GPIO_IDR_IDR_3              EQU (0x00000008)
GPIO_IDR_IDR_4              EQU (0x00000010)
GPIO_IDR_IDR_5              EQU (0x00000020)
GPIO_IDR_IDR_6              EQU (0x00000040)
GPIO_IDR_IDR_7              EQU (0x00000080)
GPIO_IDR_IDR_8              EQU (0x00000100)
GPIO_IDR_IDR_9              EQU (0x00000200)
GPIO_IDR_IDR_10             EQU (0x00000400)
GPIO_IDR_IDR_11             EQU (0x00000800)
GPIO_IDR_IDR_12             EQU (0x00001000)
GPIO_IDR_IDR_13             EQU (0x00002000)
GPIO_IDR_IDR_14             EQU (0x00004000)
GPIO_IDR_IDR_15             EQU (0x00008000)

; Old GPIO_IDR register bits definition, maintained for legacy purpose
GPIO_OTYPER_IDR_0           EQU GPIO_IDR_IDR_0
GPIO_OTYPER_IDR_1           EQU GPIO_IDR_IDR_1
GPIO_OTYPER_IDR_2           EQU GPIO_IDR_IDR_2
GPIO_OTYPER_IDR_3           EQU GPIO_IDR_IDR_3
GPIO_OTYPER_IDR_4           EQU GPIO_IDR_IDR_4
GPIO_OTYPER_IDR_5           EQU GPIO_IDR_IDR_5
GPIO_OTYPER_IDR_6           EQU GPIO_IDR_IDR_6
GPIO_OTYPER_IDR_7           EQU GPIO_IDR_IDR_7
GPIO_OTYPER_IDR_8           EQU GPIO_IDR_IDR_8
GPIO_OTYPER_IDR_9           EQU GPIO_IDR_IDR_9
GPIO_OTYPER_IDR_10          EQU GPIO_IDR_IDR_10
GPIO_OTYPER_IDR_11          EQU GPIO_IDR_IDR_11
GPIO_OTYPER_IDR_12          EQU GPIO_IDR_IDR_12
GPIO_OTYPER_IDR_13          EQU GPIO_IDR_IDR_13
GPIO_OTYPER_IDR_14          EQU GPIO_IDR_IDR_14
GPIO_OTYPER_IDR_15          EQU GPIO_IDR_IDR_15

; ******************  Bits definition for GPIO_ODR register  ******************
GPIO_ODR_ODR_0              EQU (0x00000001)
GPIO_ODR_ODR_1              EQU (0x00000002)
GPIO_ODR_ODR_2              EQU (0x00000004)
GPIO_ODR_ODR_3              EQU (0x00000008)
GPIO_ODR_ODR_4              EQU (0x00000010)
GPIO_ODR_ODR_5              EQU (0x00000020)
GPIO_ODR_ODR_6              EQU (0x00000040)
GPIO_ODR_ODR_7              EQU (0x00000080)
GPIO_ODR_ODR_8              EQU (0x00000100)
GPIO_ODR_ODR_9              EQU (0x00000200)
GPIO_ODR_ODR_10             EQU (0x00000400)
GPIO_ODR_ODR_11             EQU (0x00000800)
GPIO_ODR_ODR_12             EQU (0x00001000)
GPIO_ODR_ODR_13             EQU (0x00002000)
GPIO_ODR_ODR_14             EQU (0x00004000)
GPIO_ODR_ODR_15             EQU (0x00008000)

; Old GPIO_ODR register bits definition, maintained for legacy purpose
GPIO_OTYPER_ODR_0           EQU GPIO_ODR_ODR_0
GPIO_OTYPER_ODR_1           EQU GPIO_ODR_ODR_1
GPIO_OTYPER_ODR_2           EQU GPIO_ODR_ODR_2
GPIO_OTYPER_ODR_3           EQU GPIO_ODR_ODR_3
GPIO_OTYPER_ODR_4           EQU GPIO_ODR_ODR_4
GPIO_OTYPER_ODR_5           EQU GPIO_ODR_ODR_5
GPIO_OTYPER_ODR_6           EQU GPIO_ODR_ODR_6
GPIO_OTYPER_ODR_7           EQU GPIO_ODR_ODR_7
GPIO_OTYPER_ODR_8           EQU GPIO_ODR_ODR_8
GPIO_OTYPER_ODR_9           EQU GPIO_ODR_ODR_9
GPIO_OTYPER_ODR_10          EQU GPIO_ODR_ODR_10
GPIO_OTYPER_ODR_11          EQU GPIO_ODR_ODR_11
GPIO_OTYPER_ODR_12          EQU GPIO_ODR_ODR_12
GPIO_OTYPER_ODR_13          EQU GPIO_ODR_ODR_13
GPIO_OTYPER_ODR_14          EQU GPIO_ODR_ODR_14
GPIO_OTYPER_ODR_15          EQU GPIO_ODR_ODR_15

; ******************  Bits definition for GPIO_BSRR register  *****************
GPIO_BSRR_BS_0              EQU (0x00000001)
GPIO_BSRR_BS_1              EQU (0x00000002)
GPIO_BSRR_BS_2              EQU (0x00000004)
GPIO_BSRR_BS_3              EQU (0x00000008)
GPIO_BSRR_BS_4              EQU (0x00000010)
GPIO_BSRR_BS_5              EQU (0x00000020)
GPIO_BSRR_BS_6              EQU (0x00000040)
GPIO_BSRR_BS_7              EQU (0x00000080)
GPIO_BSRR_BS_8              EQU (0x00000100)
GPIO_BSRR_BS_9              EQU (0x00000200)
GPIO_BSRR_BS_10             EQU (0x00000400)
GPIO_BSRR_BS_11             EQU (0x00000800)
GPIO_BSRR_BS_12             EQU (0x00001000)
GPIO_BSRR_BS_13             EQU (0x00002000)
GPIO_BSRR_BS_14             EQU (0x00004000)
GPIO_BSRR_BS_15             EQU (0x00008000)
GPIO_BSRR_BR_0              EQU (0x00010000)
GPIO_BSRR_BR_1              EQU (0x00020000)
GPIO_BSRR_BR_2              EQU (0x00040000)
GPIO_BSRR_BR_3              EQU (0x00080000)
GPIO_BSRR_BR_4              EQU (0x00100000)
GPIO_BSRR_BR_5              EQU (0x00200000)
GPIO_BSRR_BR_6              EQU (0x00400000)
GPIO_BSRR_BR_7              EQU (0x00800000)
GPIO_BSRR_BR_8              EQU (0x01000000)
GPIO_BSRR_BR_9              EQU (0x02000000)
GPIO_BSRR_BR_10             EQU (0x04000000)
GPIO_BSRR_BR_11             EQU (0x08000000)
GPIO_BSRR_BR_12             EQU (0x10000000)
GPIO_BSRR_BR_13             EQU (0x20000000)
GPIO_BSRR_BR_14             EQU (0x40000000)
GPIO_BSRR_BR_15             EQU (0x80000000)

; ******************  Bits definition for GPIO_BRR register  *****************
GPIO_BRR_BR_0               EQU (0x00000001)
GPIO_BRR_BR_1               EQU (0x00000002)
GPIO_BRR_BR_2               EQU (0x00000004)
GPIO_BRR_BR_3               EQU (0x00000008)
GPIO_BRR_BR_4               EQU (0x00000010)
GPIO_BRR_BR_5               EQU (0x00000020)
GPIO_BRR_BR_6               EQU (0x00000040)
GPIO_BRR_BR_7               EQU (0x00000080)
GPIO_BRR_BR_8               EQU (0x00000100)
GPIO_BRR_BR_9               EQU (0x00000200)
GPIO_BRR_BR_10              EQU (0x00000400)
GPIO_BRR_BR_11              EQU (0x00000800)
GPIO_BRR_BR_12              EQU (0x00001000)
GPIO_BRR_BR_13              EQU (0x00002000)
GPIO_BRR_BR_14              EQU (0x00004000)
GPIO_BRR_BR_15              EQU (0x00008000)

; ****************** Bit definition for GPIO_LCKR register ********************
GPIO_LCKR_LCK0              EQU (0x00000001)
GPIO_LCKR_LCK1              EQU (0x00000002)
GPIO_LCKR_LCK2              EQU (0x00000004)
GPIO_LCKR_LCK3              EQU (0x00000008)
GPIO_LCKR_LCK4              EQU (0x00000010)
GPIO_LCKR_LCK5              EQU (0x00000020)
GPIO_LCKR_LCK6              EQU (0x00000040)
GPIO_LCKR_LCK7              EQU (0x00000080)
GPIO_LCKR_LCK8              EQU (0x00000100)
GPIO_LCKR_LCK9              EQU (0x00000200)
GPIO_LCKR_LCK10             EQU (0x00000400)
GPIO_LCKR_LCK11             EQU (0x00000800)
GPIO_LCKR_LCK12             EQU (0x00001000)
GPIO_LCKR_LCK13             EQU (0x00002000)
GPIO_LCKR_LCK14             EQU (0x00004000)
GPIO_LCKR_LCK15             EQU (0x00008000)
GPIO_LCKR_LCKK              EQU (0x00010000)

; ****************** Bit definition for GPIO_AFRL register  *******************
GPIO_AFRL_AFRL0              EQU (0x0000000F)
GPIO_AFRL_AFRL1              EQU (0x000000F0)
GPIO_AFRL_AFRL2              EQU (0x00000F00)
GPIO_AFRL_AFRL3              EQU (0x0000F000)
GPIO_AFRL_AFRL4              EQU (0x000F0000)
GPIO_AFRL_AFRL5              EQU (0x00F00000)
GPIO_AFRL_AFRL6              EQU (0x0F000000)
GPIO_AFRL_AFRL7              EQU (0xF0000000)

; ****************** Bit definition for GPIO_AFRH register  *******************
GPIO_AFRH_AFRH0              EQU (0x0000000F)
GPIO_AFRH_AFRH1              EQU (0x000000F0)
GPIO_AFRH_AFRH2              EQU (0x00000F00)
GPIO_AFRH_AFRH3              EQU (0x0000F000)
GPIO_AFRH_AFRH4              EQU (0x000F0000)
GPIO_AFRH_AFRH5              EQU (0x00F00000)
GPIO_AFRH_AFRH6              EQU (0x0F000000)
GPIO_AFRH_AFRH7              EQU (0xF0000000)

; ******************  Bits definition for GPIO_ASCR register  ******************
GPIO_ASCR_EN_0               EQU (0x00000001)
GPIO_ASCR_EN_1               EQU (0x00000002)
GPIO_ASCR_EN_2               EQU (0x00000004)
GPIO_ASCR_EN_3               EQU (0x00000008)
GPIO_ASCR_EN_4               EQU (0x00000010)
GPIO_ASCR_EN_5               EQU (0x00000020)
GPIO_ASCR_EN_6               EQU (0x00000040)
GPIO_ASCR_EN_7               EQU (0x00000080)
GPIO_ASCR_EN_8               EQU (0x00000100)
GPIO_ASCR_EN_9               EQU (0x00000200)
GPIO_ASCR_EN_10              EQU (0x00000400)
GPIO_ASCR_EN_11              EQU (0x00000800)
GPIO_ASCR_EN_12              EQU (0x00001000)
GPIO_ASCR_EN_13              EQU (0x00002000)
GPIO_ASCR_EN_14              EQU (0x00004000)
GPIO_ASCR_EN_15              EQU (0x00008000)

; *****************************************************************************
;
;                      Inter-integrated Circuit Interface (I2C)
;
; *****************************************************************************
; *******************  Bit definition for I2C_CR1 register  ******************
I2C_CR1_PE                  EQU (0x00000001)        ; Peripheral enable
I2C_CR1_TXIE                EQU (0x00000002)        ; TX interrupt enable
I2C_CR1_RXIE                EQU (0x00000004)        ; RX interrupt enable
I2C_CR1_ADDRIE              EQU (0x00000008)        ; Address match interrupt enable
I2C_CR1_NACKIE              EQU (0x00000010)        ; NACK received interrupt enable
I2C_CR1_STOPIE              EQU (0x00000020)        ; STOP detection interrupt enable
I2C_CR1_TCIE                EQU (0x00000040)        ; Transfer complete interrupt enable
I2C_CR1_ERRIE               EQU (0x00000080)        ; Errors interrupt enable
I2C_CR1_DNF                 EQU (0x00000F00)        ; Digital noise filter
I2C_CR1_ANFOFF              EQU (0x00001000)        ; Analog noise filter OFF
I2C_CR1_SWRST               EQU (0x00002000)        ; Software reset
I2C_CR1_TXDMAEN             EQU (0x00004000)        ; DMA transmission requests enable
I2C_CR1_RXDMAEN             EQU (0x00008000)        ; DMA reception requests enable
I2C_CR1_SBC                 EQU (0x00010000)        ; Slave byte control
I2C_CR1_NOSTRETCH           EQU (0x00020000)        ; Clock stretching disable
I2C_CR1_WUPEN               EQU (0x00040000)        ; Wakeup from STOP enable
I2C_CR1_GCEN                EQU (0x00080000)        ; General call enable
I2C_CR1_SMBHEN              EQU (0x00100000)        ; SMBus host address enable
I2C_CR1_SMBDEN              EQU (0x00200000)        ; SMBus device default address enable
I2C_CR1_ALERTEN             EQU (0x00400000)        ; SMBus alert enable
I2C_CR1_PECEN               EQU (0x00800000)        ; PEC enable

; ******************  Bit definition for I2C_CR2 register  *******************
I2C_CR2_SADD                EQU (0x000003FF)        ; Slave address (master mode)
I2C_CR2_RD_WRN              EQU (0x00000400)        ; Transfer direction (master mode)
I2C_CR2_ADD10               EQU (0x00000800)        ; 10-bit addressing mode (master mode)
I2C_CR2_HEAD10R             EQU (0x00001000)        ; 10-bit address header only read direction (master mode)
I2C_CR2_START               EQU (0x00002000)        ; START generation
I2C_CR2_STOP                EQU (0x00004000)        ; STOP generation (master mode)
I2C_CR2_NACK                EQU (0x00008000)        ; NACK generation (slave mode)
I2C_CR2_NBYTES              EQU (0x00FF0000)        ; Number of bytes
I2C_CR2_RELOAD              EQU (0x01000000)        ; NBYTES reload mode
I2C_CR2_AUTOEND             EQU (0x02000000)        ; Automatic end mode (master mode)
I2C_CR2_PECBYTE             EQU (0x04000000)        ; Packet error checking byte

; *******************  Bit definition for I2C_OAR1 register  *****************
I2C_OAR1_OA1                EQU (0x000003FF)        ; Interface own address 1
I2C_OAR1_OA1MODE            EQU (0x00000400)        ; Own address 1 10-bit mode
I2C_OAR1_OA1EN              EQU (0x00008000)        ; Own address 1 enable

; *******************  Bit definition for I2C_OAR2 register  *****************
I2C_OAR2_OA2                EQU (0x000000FE)        ; Interface own address 2
I2C_OAR2_OA2MSK             EQU (0x00000700)        ; Own address 2 masks
I2C_OAR2_OA2NOMASK          EQU (0x00000000)        ; No mask
I2C_OAR2_OA2MASK01          EQU (0x00000100)        ; OA2[1] is masked, Only OA2[7:2] are compared
I2C_OAR2_OA2MASK02          EQU (0x00000200)        ; OA2[2:1] is masked, Only OA2[7:3] are compared
I2C_OAR2_OA2MASK03          EQU (0x00000300)        ; OA2[3:1] is masked, Only OA2[7:4] are compared
I2C_OAR2_OA2MASK04          EQU (0x00000400)        ; OA2[4:1] is masked, Only OA2[7:5] are compared
I2C_OAR2_OA2MASK05          EQU (0x00000500)        ; OA2[5:1] is masked, Only OA2[7:6] are compared
I2C_OAR2_OA2MASK06          EQU (0x00000600)        ; OA2[6:1] is masked, Only OA2[7] are compared
I2C_OAR2_OA2MASK07          EQU (0x00000700)        ; OA2[7:1] is masked, No comparison is done
I2C_OAR2_OA2EN              EQU (0x00008000)        ; Own address 2 enable

; *******************  Bit definition for I2C_TIMINGR register ******************
I2C_TIMINGR_SCLL            EQU (0x000000FF)        ; SCL low period (master mode)
I2C_TIMINGR_SCLH            EQU (0x0000FF00)        ; SCL high period (master mode)
I2C_TIMINGR_SDADEL          EQU (0x000F0000)        ; Data hold time
I2C_TIMINGR_SCLDEL          EQU (0x00F00000)        ; Data setup time
I2C_TIMINGR_PRESC           EQU (0xF0000000)        ; Timings prescaler

; ******************* Bit definition for I2C_TIMEOUTR register ******************
I2C_TIMEOUTR_TIMEOUTA       EQU (0x00000FFF)        ; Bus timeout A
I2C_TIMEOUTR_TIDLE          EQU (0x00001000)        ; Idle clock timeout detection
I2C_TIMEOUTR_TIMOUTEN       EQU (0x00008000)        ; Clock timeout enable
I2C_TIMEOUTR_TIMEOUTB       EQU (0x0FFF0000)        ; Bus timeout B
I2C_TIMEOUTR_TEXTEN         EQU (0x80000000)        ; Extended clock timeout enable

; ******************  Bit definition for I2C_ISR register  ********************
I2C_ISR_TXE                 EQU (0x00000001)        ; Transmit data register empty
I2C_ISR_TXIS                EQU (0x00000002)        ; Transmit interrupt status
I2C_ISR_RXNE                EQU (0x00000004)        ; Receive data register not empty
I2C_ISR_ADDR                EQU (0x00000008)        ; Address matched (slave mode)
I2C_ISR_NACKF               EQU (0x00000010)        ; NACK received flag
I2C_ISR_STOPF               EQU (0x00000020)        ; STOP detection flag
I2C_ISR_TC                  EQU (0x00000040)        ; Transfer complete (master mode)
I2C_ISR_TCR                 EQU (0x00000080)        ; Transfer complete reload
I2C_ISR_BERR                EQU (0x00000100)        ; Bus error
I2C_ISR_ARLO                EQU (0x00000200)        ; Arbitration lost
I2C_ISR_OVR                 EQU (0x00000400)        ; Overrun/Underrun
I2C_ISR_PECERR              EQU (0x00000800)        ; PEC error in reception
I2C_ISR_TIMEOUT             EQU (0x00001000)        ; Timeout or Tlow detection flag
I2C_ISR_ALERT               EQU (0x00002000)        ; SMBus alert
I2C_ISR_BUSY                EQU (0x00008000)        ; Bus busy
I2C_ISR_DIR                 EQU (0x00010000)        ; Transfer direction (slave mode)
I2C_ISR_ADDCODE             EQU (0x00FE0000)        ; Address match code (slave mode)

; ******************  Bit definition for I2C_ICR register  ********************
I2C_ICR_ADDRCF              EQU (0x00000008)        ; Address matched clear flag
I2C_ICR_NACKCF              EQU (0x00000010)        ; NACK clear flag
I2C_ICR_STOPCF              EQU (0x00000020)        ; STOP detection clear flag
I2C_ICR_BERRCF              EQU (0x00000100)        ; Bus error clear flag
I2C_ICR_ARLOCF              EQU (0x00000200)        ; Arbitration lost clear flag
I2C_ICR_OVRCF               EQU (0x00000400)        ; Overrun/Underrun clear flag
I2C_ICR_PECCF               EQU (0x00000800)        ; PAC error clear flag
I2C_ICR_TIMOUTCF            EQU (0x00001000)        ; Timeout clear flag
I2C_ICR_ALERTCF             EQU (0x00002000)        ; Alert clear flag

; ******************  Bit definition for I2C_PECR register  ********************
I2C_PECR_PEC                EQU (0x000000FF)        ; PEC register

; ******************  Bit definition for I2C_RXDR register  ********************
I2C_RXDR_RXDATA             EQU (0x000000FF)        ; 8-bit receive data

; ******************  Bit definition for I2C_TXDR register  ********************
I2C_TXDR_TXDATA             EQU (0x000000FF)        ; 8-bit transmit data

; *****************************************************************************
;
;                           Independent WATCHDOG
;
; *****************************************************************************
; *******************  Bit definition for IWDG_KR register  *******************
IWDG_KR_KEY                 EQU (0x0000FFFF)        ;Key value (write only, read 0000h)

; *******************  Bit definition for IWDG_PR register  *******************
IWDG_PR_PR                  EQU (0x00000007)        ;PR[2:0] (Prescaler divider)
IWDG_PR_PR_0                EQU (0x00000001)        ;Bit 0
IWDG_PR_PR_1                EQU (0x00000002)        ;Bit 1
IWDG_PR_PR_2                EQU (0x00000004)        ;Bit 2

; *******************  Bit definition for IWDG_RLR register  ******************
IWDG_RLR_RL                 EQU (0x00000FFF)        ;Watchdog counter reload value

; *******************  Bit definition for IWDG_SR register  *******************
IWDG_SR_PVU                 EQU (0x00000001)        ; Watchdog prescaler value update
IWDG_SR_RVU                 EQU (0x00000002)        ; Watchdog counter reload value update
IWDG_SR_WVU                 EQU (0x00000004)        ; Watchdog counter window value update

; *******************  Bit definition for IWDG_KR register  *******************
IWDG_WINR_WIN               EQU (0x00000FFF)        ; Watchdog counter window value

; *****************************************************************************
;
;                                     Firewall
;
; *****************************************************************************

; *******Bit definition for CSSA;CSL;NVDSSA;NVDSL;VDSSA;VDSL;LSSA;LSL register
FW_CSSA_ADD           EQU (0x00FFFF00)        ; Code Segment Start Address
FW_CSL_LENG           EQU (0x003FFF00)        ; Code Segment Length
FW_NVDSSA_ADD         EQU (0x00FFFF00)        ; Non Volatile Dat Segment Start Address
FW_NVDSL_LENG         EQU (0x003FFF00)        ; Non Volatile Data Segment Length
FW_VDSSA_ADD          EQU (0x0001FFC0)        ; Volatile Data Segment Start Address
FW_VDSL_LENG          EQU (0x0001FFC0)        ; Volatile Data Segment Length
FW_LSSA_ADD           EQU (0x0007FF80)        ; Library Segment Start Address
FW_LSL_LENG           EQU (0x0007FF80)        ; Library Segment Length

; **************************Bit definition for CR register ********************
FW_CR_FPA             EQU (0x00000001)         ; Firewall Pre Arm
FW_CR_VDS             EQU (0x00000002)         ; Volatile Data Sharing
FW_CR_VDE             EQU (0x00000004)         ; Volatile Data Execution

; *****************************************************************************
;
;                             Power Control
;
; *****************************************************************************

; ********************  Bit definition for PWR_CR1 register  *******************

PWR_CR1_LPR                 EQU (0x00004000)     ; Regulator low-power mode
PWR_CR1_VOS                 EQU (0x00000600)     ; VOS[1:0] bits (Regulator voltage scaling output selection)
PWR_CR1_VOS_0               EQU (0x00000200)     ; Bit 0
PWR_CR1_VOS_1               EQU (0x00000400)     ; Bit 1
PWR_CR1_DBP                 EQU (0x00000100)     ; Disable Back-up domain Protection
PWR_CR1_LPMS                EQU (0x00000007)     ; Low-power mode selection field
PWR_CR1_LPMS_STOP1MR        EQU (0x00000000)     ; Stop 1 mode with Main Regulator
PWR_CR1_LPMS_STOP1LPR       EQU (0x00000001)     ; Stop 1 mode with Low-Power Regulator
PWR_CR1_LPMS_STOP2          EQU (0x00000002)     ; Stop 2 mode
PWR_CR1_LPMS_STANDBY        EQU (0x00000003)     ; Stand-by mode
PWR_CR1_LPMS_SHUTDOWN       EQU (0x00000004)     ; Shut-down mode


; ********************  Bit definition for PWR_CR2 register  *******************
PWR_CR2_USV                 EQU (0x00000400)     ; VDD USB Supply Valid
PWR_CR2_IOSV                EQU (0x00000200)     ; VDD IO2 independent I/Os Supply Valid
; PVME  Peripheral Voltage Monitor Enable
PWR_CR2_PVME                EQU (0x000000F0)     ; PVM bits field
PWR_CR2_PVME4               EQU (0x00000080)     ; PVM 4 Enable
PWR_CR2_PVME3               EQU (0x00000040)     ; PVM 3 Enable
PWR_CR2_PVME2               EQU (0x00000020)     ; PVM 2 Enable
PWR_CR2_PVME1               EQU (0x00000010)     ; PVM 1 Enable
; PVD level configuration
PWR_CR2_PLS                 EQU (0x0000000E)     ; PVD level selection
PWR_CR2_PLS_LEV0            EQU (0x00000000)     ; PVD level 0
PWR_CR2_PLS_LEV1            EQU (0x00000002)     ; PVD level 1
PWR_CR2_PLS_LEV2            EQU (0x00000004)     ; PVD level 2
PWR_CR2_PLS_LEV3            EQU (0x00000006)     ; PVD level 3
PWR_CR2_PLS_LEV4            EQU (0x00000008)     ; PVD level 4
PWR_CR2_PLS_LEV5            EQU (0x0000000A)     ; PVD level 5
PWR_CR2_PLS_LEV6            EQU (0x0000000C)     ; PVD level 6
PWR_CR2_PLS_LEV7            EQU (0x0000000E)     ; PVD level 7
PWR_CR2_PVDE                EQU (0x00000001)     ; Power Voltage Detector Enable

; ********************  Bit definition for PWR_CR3 register  *******************
PWR_CR3_EIWF                EQU (0x00008000)      ; Enable Internal Wake-up line
PWR_CR3_APC                 EQU (0x00000400)      ; Apply pull-up and pull-down configuration
PWR_CR3_RRS                 EQU (0x00000100)      ; SRAM2 Retention in Stand-by mode
PWR_CR3_EWUP5               EQU (0x00000010)      ; Enable Wake-Up Pin 5
PWR_CR3_EWUP4               EQU (0x00000008)      ; Enable Wake-Up Pin 4
PWR_CR3_EWUP3               EQU (0x00000004)      ; Enable Wake-Up Pin 3
PWR_CR3_EWUP2               EQU (0x00000002)      ; Enable Wake-Up Pin 2
PWR_CR3_EWUP1               EQU (0x00000001)      ; Enable Wake-Up Pin 1
PWR_CR3_EWUP                EQU (0x0000001F)      ; Enable Wake-Up Pins

; ********************  Bit definition for PWR_CR4 register  *******************
PWR_CR4_VBRS                EQU (0x00000200)      ; VBAT Battery charging Resistor Selection
PWR_CR4_VBE                 EQU (0x00000100)      ; VBAT Battery charging Enable
PWR_CR4_WP5                 EQU (0x00000010)      ; Wake-Up Pin 5 polarity
PWR_CR4_WP4                 EQU (0x00000008)      ; Wake-Up Pin 4 polarity
PWR_CR4_WP3                 EQU (0x00000004)      ; Wake-Up Pin 3 polarity
PWR_CR4_WP2                 EQU (0x00000002)      ; Wake-Up Pin 2 polarity
PWR_CR4_WP1                 EQU (0x00000001)      ; Wake-Up Pin 1 polarity

; ********************  Bit definition for PWR_SR1 register  *******************
PWR_SR1_WUFI               EQU (0x00008000)     ; Wake-Up Flag Internal
PWR_SR1_SBF                EQU (0x00000100)     ; Stand-By Flag
PWR_SR1_WUF                EQU (0x0000001F)     ; Wake-up Flags
PWR_SR1_WUF5               EQU (0x00000010)     ; Wake-up Flag 5
PWR_SR1_WUF4               EQU (0x00000008)     ; Wake-up Flag 4
PWR_SR1_WUF3               EQU (0x00000004)     ; Wake-up Flag 3
PWR_SR1_WUF2               EQU (0x00000002)     ; Wake-up Flag 2
PWR_SR1_WUF1               EQU (0x00000001)     ; Wake-up Flag 1

; ********************  Bit definition for PWR_SR2 register  *******************
PWR_SR2_PVMO4              EQU (0x00008000)     ; Peripheral Voltage Monitoring Output 4
PWR_SR2_PVMO3              EQU (0x00004000)     ; Peripheral Voltage Monitoring Output 3
PWR_SR2_PVMO2              EQU (0x00002000)     ; Peripheral Voltage Monitoring Output 2
PWR_SR2_PVMO1              EQU (0x00001000)     ; Peripheral Voltage Monitoring Output 1
PWR_SR2_PVDO               EQU (0x00000800)     ; Power Voltage Detector Output
PWR_SR2_VOSF               EQU (0x00000400)     ; Voltage Scaling Flag
PWR_SR2_REGLPF             EQU (0x00000200)     ; Low-power Regulator Flag
PWR_SR2_REGLPS             EQU (0x00000100)     ; Low-power Regulator Started

; ********************  Bit definition for PWR_SCR register  *******************
PWR_SCR_CSBF               EQU (0x00000100)      ; Clear Stand-By Flag
PWR_SCR_CWUF               EQU (0x0000001F)      ; Clear Wake-up Flags
PWR_SCR_CWUF5              EQU (0x00000010)      ; Clear Wake-up Flag 5
PWR_SCR_CWUF4              EQU (0x00000008)      ; Clear Wake-up Flag 4
PWR_SCR_CWUF3              EQU (0x00000004)      ; Clear Wake-up Flag 3
PWR_SCR_CWUF2              EQU (0x00000002)      ; Clear Wake-up Flag 2
PWR_SCR_CWUF1              EQU (0x00000001)      ; Clear Wake-up Flag 1

; ********************  Bit definition for PWR_PUCRA register  *******************
PWR_PUCRA_PA15             EQU (0x00008000)      ; Port PA15 Pull-Up set
PWR_PUCRA_PA13             EQU (0x00002000)      ; Port PA13 Pull-Up set
PWR_PUCRA_PA12             EQU (0x00001000)      ; Port PA12 Pull-Up set
PWR_PUCRA_PA11             EQU (0x00000800)      ; Port PA11 Pull-Up set
PWR_PUCRA_PA10             EQU (0x00000400)      ; Port PA10 Pull-Up set
PWR_PUCRA_PA9              EQU (0x00000200)      ; Port PA9 Pull-Up set
PWR_PUCRA_PA8              EQU (0x00000100)      ; Port PA8 Pull-Up set
PWR_PUCRA_PA7              EQU (0x00000080)      ; Port PA7 Pull-Up set
PWR_PUCRA_PA6              EQU (0x00000040)      ; Port PA6 Pull-Up set
PWR_PUCRA_PA5              EQU (0x00000020)      ; Port PA5 Pull-Up set
PWR_PUCRA_PA4              EQU (0x00000010)      ; Port PA4 Pull-Up set
PWR_PUCRA_PA3              EQU (0x00000008)      ; Port PA3 Pull-Up set
PWR_PUCRA_PA2              EQU (0x00000004)      ; Port PA2 Pull-Up set
PWR_PUCRA_PA1              EQU (0x00000002)      ; Port PA1 Pull-Up set
PWR_PUCRA_PA0              EQU (0x00000001)      ; Port PA0 Pull-Up set

; ********************  Bit definition for PWR_PDCRA register  *******************
PWR_PDCRA_PA14             EQU (0x00004000)      ; Port PA14 Pull-Down set
PWR_PDCRA_PA12             EQU (0x00001000)      ; Port PA12 Pull-Down set
PWR_PDCRA_PA11             EQU (0x00000800)      ; Port PA11 Pull-Down set
PWR_PDCRA_PA10             EQU (0x00000400)      ; Port PA10 Pull-Down set
PWR_PDCRA_PA9              EQU (0x00000200)      ; Port PA9 Pull-Down set
PWR_PDCRA_PA8              EQU (0x00000100)      ; Port PA8 Pull-Down set
PWR_PDCRA_PA7              EQU (0x00000080)      ; Port PA7 Pull-Down set
PWR_PDCRA_PA6              EQU (0x00000040)      ; Port PA6 Pull-Down set
PWR_PDCRA_PA5              EQU (0x00000020)      ; Port PA5 Pull-Down set
PWR_PDCRA_PA4              EQU (0x00000010)      ; Port PA4 Pull-Down set
PWR_PDCRA_PA3              EQU (0x00000008)      ; Port PA3 Pull-Down set
PWR_PDCRA_PA2              EQU (0x00000004)      ; Port PA2 Pull-Down set
PWR_PDCRA_PA1              EQU (0x00000002)      ; Port PA1 Pull-Down set
PWR_PDCRA_PA0              EQU (0x00000001)      ; Port PA0 Pull-Down set

; ********************  Bit definition for PWR_PUCRB register  *******************
PWR_PUCRB_PB15             EQU (0x00008000)      ; Port PB15 Pull-Up set
PWR_PUCRB_PB14             EQU (0x00004000)      ; Port PB14 Pull-Up set
PWR_PUCRB_PB13             EQU (0x00002000)      ; Port PB13 Pull-Up set
PWR_PUCRB_PB12             EQU (0x00001000)      ; Port PB12 Pull-Up set
PWR_PUCRB_PB11             EQU (0x00000800)      ; Port PB11 Pull-Up set
PWR_PUCRB_PB10             EQU (0x00000400)      ; Port PB10 Pull-Up set
PWR_PUCRB_PB9              EQU (0x00000200)      ; Port PB9 Pull-Up set
PWR_PUCRB_PB8              EQU (0x00000100)      ; Port PB8 Pull-Up set
PWR_PUCRB_PB7              EQU (0x00000080)      ; Port PB7 Pull-Up set
PWR_PUCRB_PB6              EQU (0x00000040)      ; Port PB6 Pull-Up set
PWR_PUCRB_PB5              EQU (0x00000020)      ; Port PB5 Pull-Up set
PWR_PUCRB_PB4              EQU (0x00000010)      ; Port PB4 Pull-Up set
PWR_PUCRB_PB3              EQU (0x00000008)      ; Port PB3 Pull-Up set
PWR_PUCRB_PB2              EQU (0x00000004)      ; Port PB2 Pull-Up set
PWR_PUCRB_PB1              EQU (0x00000002)      ; Port PB1 Pull-Up set
PWR_PUCRB_PB0              EQU (0x00000001)      ; Port PB0 Pull-Up set

; ********************  Bit definition for PWR_PDCRB register  *******************
PWR_PDCRB_PB15             EQU (0x00008000)      ; Port PB15 Pull-Down set
PWR_PDCRB_PB14             EQU (0x00004000)      ; Port PB14 Pull-Down set
PWR_PDCRB_PB13             EQU (0x00002000)      ; Port PB13 Pull-Down set
PWR_PDCRB_PB12             EQU (0x00001000)      ; Port PB12 Pull-Down set
PWR_PDCRB_PB11             EQU (0x00000800)      ; Port PB11 Pull-Down set
PWR_PDCRB_PB10             EQU (0x00000400)      ; Port PB10 Pull-Down set
PWR_PDCRB_PB9              EQU (0x00000200)      ; Port PB9 Pull-Down set
PWR_PDCRB_PB8              EQU (0x00000100)      ; Port PB8 Pull-Down set
PWR_PDCRB_PB7              EQU (0x00000080)      ; Port PB7 Pull-Down set
PWR_PDCRB_PB6              EQU (0x00000040)      ; Port PB6 Pull-Down set
PWR_PDCRB_PB5              EQU (0x00000020)      ; Port PB5 Pull-Down set
PWR_PDCRB_PB3              EQU (0x00000008)      ; Port PB3 Pull-Down set
PWR_PDCRB_PB2              EQU (0x00000004)      ; Port PB2 Pull-Down set
PWR_PDCRB_PB1              EQU (0x00000002)      ; Port PB1 Pull-Down set
PWR_PDCRB_PB0              EQU (0x00000001)      ; Port PB0 Pull-Down set

; ********************  Bit definition for PWR_PUCRC register  *******************
PWR_PUCRC_PC15             EQU (0x00008000)      ; Port PC15 Pull-Up set
PWR_PUCRC_PC14             EQU (0x00004000)      ; Port PC14 Pull-Up set
PWR_PUCRC_PC13             EQU (0x00002000)      ; Port PC13 Pull-Up set
PWR_PUCRC_PC12             EQU (0x00001000)      ; Port PC12 Pull-Up set
PWR_PUCRC_PC11             EQU (0x00000800)      ; Port PC11 Pull-Up set
PWR_PUCRC_PC10             EQU (0x00000400)      ; Port PC10 Pull-Up set
PWR_PUCRC_PC9              EQU (0x00000200)      ; Port PC9 Pull-Up set
PWR_PUCRC_PC8              EQU (0x00000100)      ; Port PC8 Pull-Up set
PWR_PUCRC_PC7              EQU (0x00000080)      ; Port PC7 Pull-Up set
PWR_PUCRC_PC6              EQU (0x00000040)      ; Port PC6 Pull-Up set
PWR_PUCRC_PC5              EQU (0x00000020)      ; Port PC5 Pull-Up set
PWR_PUCRC_PC4              EQU (0x00000010)      ; Port PC4 Pull-Up set
PWR_PUCRC_PC3              EQU (0x00000008)      ; Port PC3 Pull-Up set
PWR_PUCRC_PC2              EQU (0x00000004)      ; Port PC2 Pull-Up set
PWR_PUCRC_PC1              EQU (0x00000002)      ; Port PC1 Pull-Up set
PWR_PUCRC_PC0              EQU (0x00000001)      ; Port PC0 Pull-Up set

; ********************  Bit definition for PWR_PDCRC register  *******************
PWR_PDCRC_PC15             EQU (0x00008000)      ; Port PC15 Pull-Down set
PWR_PDCRC_PC14             EQU (0x00004000)      ; Port PC14 Pull-Down set
PWR_PDCRC_PC13             EQU (0x00002000)      ; Port PC13 Pull-Down set
PWR_PDCRC_PC12             EQU (0x00001000)      ; Port PC12 Pull-Down set
PWR_PDCRC_PC11             EQU (0x00000800)      ; Port PC11 Pull-Down set
PWR_PDCRC_PC10             EQU (0x00000400)      ; Port PC10 Pull-Down set
PWR_PDCRC_PC9              EQU (0x00000200)      ; Port PC9 Pull-Down set
PWR_PDCRC_PC8              EQU (0x00000100)      ; Port PC8 Pull-Down set
PWR_PDCRC_PC7              EQU (0x00000080)      ; Port PC7 Pull-Down set
PWR_PDCRC_PC6              EQU (0x00000040)      ; Port PC6 Pull-Down set
PWR_PDCRC_PC5              EQU (0x00000020)      ; Port PC5 Pull-Down set
PWR_PDCRC_PC4              EQU (0x00000010)      ; Port PC4 Pull-Down set
PWR_PDCRC_PC3              EQU (0x00000008)      ; Port PC3 Pull-Down set
PWR_PDCRC_PC2              EQU (0x00000004)      ; Port PC2 Pull-Down set
PWR_PDCRC_PC1              EQU (0x00000002)      ; Port PC1 Pull-Down set
PWR_PDCRC_PC0              EQU (0x00000001)      ; Port PC0 Pull-Down set

; ********************  Bit definition for PWR_PUCRD register  *******************
PWR_PUCRD_PD15             EQU (0x00008000)      ; Port PD15 Pull-Up set
PWR_PUCRD_PD14             EQU (0x00004000)      ; Port PD14 Pull-Up set
PWR_PUCRD_PD13             EQU (0x00002000)      ; Port PD13 Pull-Up set
PWR_PUCRD_PD12             EQU (0x00001000)      ; Port PD12 Pull-Up set
PWR_PUCRD_PD11             EQU (0x00000800)      ; Port PD11 Pull-Up set
PWR_PUCRD_PD10             EQU (0x00000400)      ; Port PD10 Pull-Up set
PWR_PUCRD_PD9              EQU (0x00000200)      ; Port PD9 Pull-Up set
PWR_PUCRD_PD8              EQU (0x00000100)      ; Port PD8 Pull-Up set
PWR_PUCRD_PD7              EQU (0x00000080)      ; Port PD7 Pull-Up set
PWR_PUCRD_PD6              EQU (0x00000040)      ; Port PD6 Pull-Up set
PWR_PUCRD_PD5              EQU (0x00000020)      ; Port PD5 Pull-Up set
PWR_PUCRD_PD4              EQU (0x00000010)      ; Port PD4 Pull-Up set
PWR_PUCRD_PD3              EQU (0x00000008)      ; Port PD3 Pull-Up set
PWR_PUCRD_PD2              EQU (0x00000004)      ; Port PD2 Pull-Up set
PWR_PUCRD_PD1              EQU (0x00000002)      ; Port PD1 Pull-Up set
PWR_PUCRD_PD0              EQU (0x00000001)      ; Port PD0 Pull-Up set

; ********************  Bit definition for PWR_PDCRD register  *******************
PWR_PDCRD_PD15             EQU (0x00008000)      ; Port PD15 Pull-Down set
PWR_PDCRD_PD14             EQU (0x00004000)      ; Port PD14 Pull-Down set
PWR_PDCRD_PD13             EQU (0x00002000)      ; Port PD13 Pull-Down set
PWR_PDCRD_PD12             EQU (0x00001000)      ; Port PD12 Pull-Down set
PWR_PDCRD_PD11             EQU (0x00000800)      ; Port PD11 Pull-Down set
PWR_PDCRD_PD10             EQU (0x00000400)      ; Port PD10 Pull-Down set
PWR_PDCRD_PD9              EQU (0x00000200)      ; Port PD9 Pull-Down set
PWR_PDCRD_PD8              EQU (0x00000100)      ; Port PD8 Pull-Down set
PWR_PDCRD_PD7              EQU (0x00000080)      ; Port PD7 Pull-Down set
PWR_PDCRD_PD6              EQU (0x00000040)      ; Port PD6 Pull-Down set
PWR_PDCRD_PD5              EQU (0x00000020)      ; Port PD5 Pull-Down set
PWR_PDCRD_PD4              EQU (0x00000010)      ; Port PD4 Pull-Down set
PWR_PDCRD_PD3              EQU (0x00000008)      ; Port PD3 Pull-Down set
PWR_PDCRD_PD2              EQU (0x00000004)      ; Port PD2 Pull-Down set
PWR_PDCRD_PD1              EQU (0x00000002)      ; Port PD1 Pull-Down set
PWR_PDCRD_PD0              EQU (0x00000001)      ; Port PD0 Pull-Down set

; ********************  Bit definition for PWR_PUCRE register  *******************
PWR_PUCRE_PE15             EQU (0x00008000)      ; Port PE15 Pull-Up set
PWR_PUCRE_PE14             EQU (0x00004000)      ; Port PE14 Pull-Up set
PWR_PUCRE_PE13             EQU (0x00002000)      ; Port PE13 Pull-Up set
PWR_PUCRE_PE12             EQU (0x00001000)      ; Port PE12 Pull-Up set
PWR_PUCRE_PE11             EQU (0x00000800)      ; Port PE11 Pull-Up set
PWR_PUCRE_PE10             EQU (0x00000400)      ; Port PE10 Pull-Up set
PWR_PUCRE_PE9              EQU (0x00000200)      ; Port PE9 Pull-Up set
PWR_PUCRE_PE8              EQU (0x00000100)      ; Port PE8 Pull-Up set
PWR_PUCRE_PE7              EQU (0x00000080)      ; Port PE7 Pull-Up set
PWR_PUCRE_PE6              EQU (0x00000040)      ; Port PE6 Pull-Up set
PWR_PUCRE_PE5              EQU (0x00000020)      ; Port PE5 Pull-Up set
PWR_PUCRE_PE4              EQU (0x00000010)      ; Port PE4 Pull-Up set
PWR_PUCRE_PE3              EQU (0x00000008)      ; Port PE3 Pull-Up set
PWR_PUCRE_PE2              EQU (0x00000004)      ; Port PE2 Pull-Up set
PWR_PUCRE_PE1              EQU (0x00000002)      ; Port PE1 Pull-Up set
PWR_PUCRE_PE0              EQU (0x00000001)      ; Port PE0 Pull-Up set

; ********************  Bit definition for PWR_PDCRE register  *******************
PWR_PDCRE_PE15             EQU (0x00008000)      ; Port PE15 Pull-Down set
PWR_PDCRE_PE14             EQU (0x00004000)      ; Port PE14 Pull-Down set
PWR_PDCRE_PE13             EQU (0x00002000)      ; Port PE13 Pull-Down set
PWR_PDCRE_PE12             EQU (0x00001000)      ; Port PE12 Pull-Down set
PWR_PDCRE_PE11             EQU (0x00000800)      ; Port PE11 Pull-Down set
PWR_PDCRE_PE10             EQU (0x00000400)      ; Port PE10 Pull-Down set
PWR_PDCRE_PE9              EQU (0x00000200)      ; Port PE9 Pull-Down set
PWR_PDCRE_PE8              EQU (0x00000100)      ; Port PE8 Pull-Down set
PWR_PDCRE_PE7              EQU (0x00000080)      ; Port PE7 Pull-Down set
PWR_PDCRE_PE6              EQU (0x00000040)      ; Port PE6 Pull-Down set
PWR_PDCRE_PE5              EQU (0x00000020)      ; Port PE5 Pull-Down set
PWR_PDCRE_PE4              EQU (0x00000010)      ; Port PE4 Pull-Down set
PWR_PDCRE_PE3              EQU (0x00000008)      ; Port PE3 Pull-Down set
PWR_PDCRE_PE2              EQU (0x00000004)      ; Port PE2 Pull-Down set
PWR_PDCRE_PE1              EQU (0x00000002)      ; Port PE1 Pull-Down set
PWR_PDCRE_PE0              EQU (0x00000001)      ; Port PE0 Pull-Down set

; ********************  Bit definition for PWR_PUCRF register  *******************
PWR_PUCRF_PF15             EQU (0x00008000)      ; Port PF15 Pull-Up set
PWR_PUCRF_PF14             EQU (0x00004000)      ; Port PF14 Pull-Up set
PWR_PUCRF_PF13             EQU (0x00002000)      ; Port PF13 Pull-Up set
PWR_PUCRF_PF12             EQU (0x00001000)      ; Port PF12 Pull-Up set
PWR_PUCRF_PF11             EQU (0x00000800)      ; Port PF11 Pull-Up set
PWR_PUCRF_PF10             EQU (0x00000400)      ; Port PF10 Pull-Up set
PWR_PUCRF_PF9              EQU (0x00000200)      ; Port PF9 Pull-Up set
PWR_PUCRF_PF8              EQU (0x00000100)      ; Port PF8 Pull-Up set
PWR_PUCRF_PF7              EQU (0x00000080)      ; Port PF7 Pull-Up set
PWR_PUCRF_PF6              EQU (0x00000040)      ; Port PF6 Pull-Up set
PWR_PUCRF_PF5              EQU (0x00000020)      ; Port PF5 Pull-Up set
PWR_PUCRF_PF4              EQU (0x00000010)      ; Port PF4 Pull-Up set
PWR_PUCRF_PF3              EQU (0x00000008)      ; Port PF3 Pull-Up set
PWR_PUCRF_PF2              EQU (0x00000004)      ; Port PF2 Pull-Up set
PWR_PUCRF_PF1              EQU (0x00000002)      ; Port PF1 Pull-Up set
PWR_PUCRF_PF0              EQU (0x00000001)      ; Port PF0 Pull-Up set

; ********************  Bit definition for PWR_PDCRF register  *******************
PWR_PDCRF_PF15             EQU (0x00008000)      ; Port PF15 Pull-Down set
PWR_PDCRF_PF14             EQU (0x00004000)      ; Port PF14 Pull-Down set
PWR_PDCRF_PF13             EQU (0x00002000)      ; Port PF13 Pull-Down set
PWR_PDCRF_PF12             EQU (0x00001000)      ; Port PF12 Pull-Down set
PWR_PDCRF_PF11             EQU (0x00000800)      ; Port PF11 Pull-Down set
PWR_PDCRF_PF10             EQU (0x00000400)      ; Port PF10 Pull-Down set
PWR_PDCRF_PF9              EQU (0x00000200)      ; Port PF9 Pull-Down set
PWR_PDCRF_PF8              EQU (0x00000100)      ; Port PF8 Pull-Down set
PWR_PDCRF_PF7              EQU (0x00000080)      ; Port PF7 Pull-Down set
PWR_PDCRF_PF6              EQU (0x00000040)      ; Port PF6 Pull-Down set
PWR_PDCRF_PF5              EQU (0x00000020)      ; Port PF5 Pull-Down set
PWR_PDCRF_PF4              EQU (0x00000010)      ; Port PF4 Pull-Down set
PWR_PDCRF_PF3              EQU (0x00000008)      ; Port PF3 Pull-Down set
PWR_PDCRF_PF2              EQU (0x00000004)      ; Port PF2 Pull-Down set
PWR_PDCRF_PF1              EQU (0x00000002)      ; Port PF1 Pull-Down set
PWR_PDCRF_PF0              EQU (0x00000001)      ; Port PF0 Pull-Down set

; ********************  Bit definition for PWR_PUCRG register  *******************
PWR_PUCRG_PG15             EQU (0x00008000)      ; Port PG15 Pull-Up set
PWR_PUCRG_PG14             EQU (0x00004000)      ; Port PG14 Pull-Up set
PWR_PUCRG_PG13             EQU (0x00002000)      ; Port PG13 Pull-Up set
PWR_PUCRG_PG12             EQU (0x00001000)      ; Port PG12 Pull-Up set
PWR_PUCRG_PG11             EQU (0x00000800)      ; Port PG11 Pull-Up set
PWR_PUCRG_PG10             EQU (0x00000400)      ; Port PG10 Pull-Up set
PWR_PUCRG_PG9              EQU (0x00000200)      ; Port PG9 Pull-Up set
PWR_PUCRG_PG8              EQU (0x00000100)      ; Port PG8 Pull-Up set
PWR_PUCRG_PG7              EQU (0x00000080)      ; Port PG7 Pull-Up set
PWR_PUCRG_PG6              EQU (0x00000040)      ; Port PG6 Pull-Up set
PWR_PUCRG_PG5              EQU (0x00000020)      ; Port PG5 Pull-Up set
PWR_PUCRG_PG4              EQU (0x00000010)      ; Port PG4 Pull-Up set
PWR_PUCRG_PG3              EQU (0x00000008)      ; Port PG3 Pull-Up set
PWR_PUCRG_PG2              EQU (0x00000004)      ; Port PG2 Pull-Up set
PWR_PUCRG_PG1              EQU (0x00000002)      ; Port PG1 Pull-Up set
PWR_PUCRG_PG0              EQU (0x00000001)      ; Port PG0 Pull-Up set

; ********************  Bit definition for PWR_PDCRG register  *******************
PWR_PDCRG_PG15             EQU (0x00008000)      ; Port PG15 Pull-Down set
PWR_PDCRG_PG14             EQU (0x00004000)      ; Port PG14 Pull-Down set
PWR_PDCRG_PG13             EQU (0x00002000)      ; Port PG13 Pull-Down set
PWR_PDCRG_PG12             EQU (0x00001000)      ; Port PG12 Pull-Down set
PWR_PDCRG_PG11             EQU (0x00000800)      ; Port PG11 Pull-Down set
PWR_PDCRG_PG10             EQU (0x00000400)      ; Port PG10 Pull-Down set
PWR_PDCRG_PG9              EQU (0x00000200)      ; Port PG9 Pull-Down set
PWR_PDCRG_PG8              EQU (0x00000100)      ; Port PG8 Pull-Down set
PWR_PDCRG_PG7              EQU (0x00000080)      ; Port PG7 Pull-Down set
PWR_PDCRG_PG6              EQU (0x00000040)      ; Port PG6 Pull-Down set
PWR_PDCRG_PG5              EQU (0x00000020)      ; Port PG5 Pull-Down set
PWR_PDCRG_PG4              EQU (0x00000010)      ; Port PG4 Pull-Down set
PWR_PDCRG_PG3              EQU (0x00000008)      ; Port PG3 Pull-Down set
PWR_PDCRG_PG2              EQU (0x00000004)      ; Port PG2 Pull-Down set
PWR_PDCRG_PG1              EQU (0x00000002)      ; Port PG1 Pull-Down set
PWR_PDCRG_PG0              EQU (0x00000001)      ; Port PG0 Pull-Down set

; ********************  Bit definition for PWR_PUCRH register  *******************
PWR_PUCRH_PH1              EQU (0x00000002)      ; Port PH1 Pull-Up set
PWR_PUCRH_PH0              EQU (0x00000001)      ; Port PH0 Pull-Up set

; ********************  Bit definition for PWR_PDCRH register  *******************
PWR_PDCRH_PH1              EQU (0x00000002)      ; Port PH1 Pull-Down set
PWR_PDCRH_PH0              EQU (0x00000001)      ; Port PH0 Pull-Down set


; *****************************************************************************
;
;                         Reset and Clock Control
;
; *****************************************************************************
; ********************  Bit definition for RCC_CR register  *******************
RCC_CR_MSION                EQU (0x00000001)      ; Internal Multi Speed oscillator (MSI) clock enable
RCC_CR_MSIRDY               EQU (0x00000002)      ; Internal Multi Speed oscillator (MSI) clock ready flag
RCC_CR_MSIPLLEN             EQU (0x00000004)      ; Internal Multi Speed oscillator (MSI) PLL enable
RCC_CR_MSIRGSEL             EQU (0x00000008)      ; Internal Multi Speed oscillator (MSI) range selection

; MSIRANGE configuration : 12 frequency ranges available
RCC_CR_MSIRANGE             EQU (0x000000F0)       ; Internal Multi Speed oscillator (MSI) clock Range
RCC_CR_MSIRANGE_0           EQU (0x00000000)       ; Internal Multi Speed oscillator (MSI) clock Range 100 KHz
RCC_CR_MSIRANGE_1           EQU (0x00000010)       ; Internal Multi Speed oscillator (MSI) clock Range 200 KHz
RCC_CR_MSIRANGE_2           EQU (0x00000020)       ; Internal Multi Speed oscillator (MSI) clock Range 400 KHz
RCC_CR_MSIRANGE_3           EQU (0x00000030)       ; Internal Multi Speed oscillator (MSI) clock Range 800 KHz
RCC_CR_MSIRANGE_4           EQU (0x00000040)       ; Internal Multi Speed oscillator (MSI) clock Range 1 MHz
RCC_CR_MSIRANGE_5           EQU (0x00000050)       ; Internal Multi Speed oscillator (MSI) clock Range 2 MHz
RCC_CR_MSIRANGE_6           EQU (0x00000060)       ; Internal Multi Speed oscillator (MSI) clock Range 4  MHz
RCC_CR_MSIRANGE_7           EQU (0x00000070)       ; Internal Multi Speed oscillator (MSI) clock Range 8 KHz
RCC_CR_MSIRANGE_8           EQU (0x00000080)       ; Internal Multi Speed oscillator (MSI) clock Range 16 MHz
RCC_CR_MSIRANGE_9           EQU (0x00000090)       ; Internal Multi Speed oscillator (MSI) clock Range 24 MHz
RCC_CR_MSIRANGE_10          EQU (0x000000A0)       ; Internal Multi Speed oscillator (MSI) clock Range 32 MHz
RCC_CR_MSIRANGE_11          EQU (0x000000B0)       ; Internal Multi Speed oscillator (MSI) clock Range 48  MHz

RCC_CR_HSION                EQU (0x00000100)       ; Internal High Speed oscillator (HSI16) clock enable
RCC_CR_HSIKERON             EQU (0x00000200)       ; Internal High Speed oscillator (HSI16) clock enable for some IPs Kernel
RCC_CR_HSIRDY               EQU (0x00000400)       ; Internal High Speed oscillator (HSI16) clock ready flag
RCC_CR_HSIASFS              EQU (0x00000800)       ; HSI16 Automatic Start from Stop

RCC_CR_HSEON                EQU (0x00010000)       ; External High Speed oscillator (HSE) clock enable
RCC_CR_HSERDY               EQU (0x00020000)       ; External High Speed oscillator (HSE) clock ready
RCC_CR_HSEBYP               EQU (0x00040000)       ; External High Speed oscillator (HSE) clock bypass
RCC_CR_CSSON                EQU (0x00080000)       ; HSE Clock Security System enable

RCC_CR_PLLON                EQU (0x01000000)       ; System PLL clock enable
RCC_CR_PLLRDY               EQU (0x02000000)       ; System PLL clock ready
RCC_CR_PLLSAI1ON            EQU (0x04000000)       ; SAI1 PLL enable
RCC_CR_PLLSAI1RDY           EQU (0x08000000)       ; SAI1 PLL ready
RCC_CR_PLLSAI2ON            EQU (0x10000000)       ; SAI2 PLL enable
RCC_CR_PLLSAI2RDY           EQU (0x20000000)       ; SAI2 PLL ready

; ********************  Bit definition for RCC_ICSCR register  **************
; MSICAL configuration
RCC_ICSCR_MSICAL            EQU (0x000000FF)       ; MSICAL[7:0] bits
RCC_ICSCR_MSICAL_0          EQU (0x00000001)       ;Bit 0
RCC_ICSCR_MSICAL_1          EQU (0x00000002)       ;Bit 1
RCC_ICSCR_MSICAL_2          EQU (0x00000004)       ;Bit 2
RCC_ICSCR_MSICAL_3          EQU (0x00000008)       ;Bit 3
RCC_ICSCR_MSICAL_4          EQU (0x00000010)       ;Bit 4
RCC_ICSCR_MSICAL_5          EQU (0x00000020)       ;Bit 5
RCC_ICSCR_MSICAL_6          EQU (0x00000040)       ;Bit 6
RCC_ICSCR_MSICAL_7          EQU (0x00000080)       ;Bit 7

; MSITRIM configuration
RCC_ICSCR_MSITRIM           EQU (0x0000FF00)       ; MSITRIM[7:0] bits
RCC_ICSCR_MSITRIM_0         EQU (0x00000100)       ;Bit 0
RCC_ICSCR_MSITRIM_1         EQU (0x00000200)       ;Bit 1
RCC_ICSCR_MSITRIM_2         EQU (0x00000400)       ;Bit 2
RCC_ICSCR_MSITRIM_3         EQU (0x00000800)       ;Bit 3
RCC_ICSCR_MSITRIM_4         EQU (0x00001000)       ;Bit 4
RCC_ICSCR_MSITRIM_5         EQU (0x00002000)       ;Bit 5
RCC_ICSCR_MSITRIM_6         EQU (0x00004000)       ;Bit 6
RCC_ICSCR_MSITRIM_7         EQU (0x00008000)       ;Bit 7

; HSICAL configuration
RCC_ICSCR_HSICAL            EQU (0x00FF0000)       ; HSICAL[7:0] bits
RCC_ICSCR_HSICAL_0          EQU (0x00010000)        ;Bit 0
RCC_ICSCR_HSICAL_1          EQU (0x00020000)        ;Bit 1
RCC_ICSCR_HSICAL_2          EQU (0x00040000)        ;Bit 2
RCC_ICSCR_HSICAL_3          EQU (0x00080000)        ;Bit 3
RCC_ICSCR_HSICAL_4          EQU (0x00100000)        ;Bit 4
RCC_ICSCR_HSICAL_5          EQU (0x00200000)        ;Bit 5
RCC_ICSCR_HSICAL_6          EQU (0x00400000)        ;Bit 6
RCC_ICSCR_HSICAL_7          EQU (0x00800000)        ;Bit 7

; HSITRIM configuration
RCC_ICSCR_HSITRIM           EQU (0x1F000000)       ; HSITRIM[4:0] bits
RCC_ICSCR_HSITRIM_0         EQU (0x01000000)        ;Bit 0
RCC_ICSCR_HSITRIM_1         EQU (0x02000000)        ;Bit 1
RCC_ICSCR_HSITRIM_2         EQU (0x04000000)        ;Bit 2
RCC_ICSCR_HSITRIM_3         EQU (0x08000000)        ;Bit 3
RCC_ICSCR_HSITRIM_4         EQU (0x10000000)        ;Bit 4

; ********************  Bit definition for RCC_CFGR register  *****************
; SW configuration
RCC_CFGR_SW                 EQU (0x00000003)      ; SW[1:0] bits (System clock Switch)
RCC_CFGR_SW_0               EQU (0x00000001)      ;Bit 0
RCC_CFGR_SW_1               EQU (0x00000002)      ;Bit 1

RCC_CFGR_SW_MSI             EQU (0x00000000)      ; MSI oscillator selection as system clock
RCC_CFGR_SW_HSI             EQU (0x00000001)      ; HSI16 oscillator selection as system clock
RCC_CFGR_SW_HSE             EQU (0x00000002)      ; HSE oscillator selection as system clock
RCC_CFGR_SW_PLL             EQU (0x00000003)      ; PLL selection as system clock

; SWS configuration
RCC_CFGR_SWS                EQU (0x0000000C)      ; SWS[1:0] bits (System Clock Switch Status)
RCC_CFGR_SWS_0              EQU (0x00000004)      ;Bit 0
RCC_CFGR_SWS_1              EQU (0x00000008)      ;Bit 1

RCC_CFGR_SWS_MSI            EQU (0x00000000)      ; MSI oscillator used as system clock
RCC_CFGR_SWS_HSI            EQU (0x00000004)      ; HSI16 oscillator used as system clock
RCC_CFGR_SWS_HSE            EQU (0x00000008)      ; HSE oscillator used as system clock
RCC_CFGR_SWS_PLL            EQU (0x0000000C)      ; PLL used as system clock

; HPRE configuration
RCC_CFGR_HPRE               EQU (0x000000F0)      ; HPRE[3:0] bits (AHB prescaler)
RCC_CFGR_HPRE_0             EQU (0x00000010)      ;Bit 0
RCC_CFGR_HPRE_1             EQU (0x00000020)      ;Bit 1
RCC_CFGR_HPRE_2             EQU (0x00000040)      ;Bit 2
RCC_CFGR_HPRE_3             EQU (0x00000080)      ;Bit 3

RCC_CFGR_HPRE_DIV1          EQU (0x00000000)      ; SYSCLK not divided
RCC_CFGR_HPRE_DIV2          EQU (0x00000080)      ; SYSCLK divided by 2
RCC_CFGR_HPRE_DIV4          EQU (0x00000090)      ; SYSCLK divided by 4
RCC_CFGR_HPRE_DIV8          EQU (0x000000A0)      ; SYSCLK divided by 8
RCC_CFGR_HPRE_DIV16         EQU (0x000000B0)      ; SYSCLK divided by 16
RCC_CFGR_HPRE_DIV64         EQU (0x000000C0)      ; SYSCLK divided by 64
RCC_CFGR_HPRE_DIV128        EQU (0x000000D0)      ; SYSCLK divided by 128
RCC_CFGR_HPRE_DIV256        EQU (0x000000E0)      ; SYSCLK divided by 256
RCC_CFGR_HPRE_DIV512        EQU (0x000000F0)      ; SYSCLK divided by 512

; PPRE1 configuration
RCC_CFGR_PPRE1              EQU (0x00000700)      ; PRE1[2:0] bits (APB2 prescaler)
RCC_CFGR_PPRE1_0            EQU (0x00000100)      ;Bit 0
RCC_CFGR_PPRE1_1            EQU (0x00000200)      ;Bit 1
RCC_CFGR_PPRE1_2            EQU (0x00000400)      ;Bit 2

RCC_CFGR_PPRE1_DIV1         EQU (0x00000000)      ; HCLK not divided
RCC_CFGR_PPRE1_DIV2         EQU (0x00000400)      ; HCLK divided by 2
RCC_CFGR_PPRE1_DIV4         EQU (0x00000500)      ; HCLK divided by 4
RCC_CFGR_PPRE1_DIV8         EQU (0x00000600)      ; HCLK divided by 8
RCC_CFGR_PPRE1_DIV16        EQU (0x00000700)      ; HCLK divided by 16

; PPRE2 configuration
RCC_CFGR_PPRE2              EQU (0x00003800)      ; PRE2[2:0] bits (APB2 prescaler)
RCC_CFGR_PPRE2_0            EQU (0x00000800)      ;Bit 0
RCC_CFGR_PPRE2_1            EQU (0x00001000)      ;Bit 1
RCC_CFGR_PPRE2_2            EQU (0x00002000)      ;Bit 2

RCC_CFGR_PPRE2_DIV1         EQU (0x00000000)      ; HCLK not divided
RCC_CFGR_PPRE2_DIV2         EQU (0x00002000)      ; HCLK divided by 2
RCC_CFGR_PPRE2_DIV4         EQU (0x00002800)      ; HCLK divided by 4
RCC_CFGR_PPRE2_DIV8         EQU (0x00003000)      ; HCLK divided by 8
RCC_CFGR_PPRE2_DIV16        EQU (0x00003800)      ; HCLK divided by 16

RCC_CFGR_STOPWUCK           EQU (0x00008000)      ; Wake Up from stop and CSS backup clock selection

; MCOSEL configuration
RCC_CFGR_MCOSEL             EQU (0x07000000)      ; MCOSEL [2:0] bits (Clock output selection)
RCC_CFGR_MCOSEL_0           EQU (0x01000000)      ;Bit 0
RCC_CFGR_MCOSEL_1           EQU (0x02000000)      ;Bit 1
RCC_CFGR_MCOSEL_2           EQU (0x04000000)      ;Bit 2

RCC_CFGR_MCO_PRE            EQU (0x70000000)      ; MCO prescaler
RCC_CFGR_MCO_PRE_1          EQU (0x00000000)      ; MCO is divided by 1
RCC_CFGR_MCO_PRE_2          EQU (0x10000000)      ; MCO is divided by 2
RCC_CFGR_MCO_PRE_4          EQU (0x20000000)      ; MCO is divided by 4
RCC_CFGR_MCO_PRE_8          EQU (0x30000000)      ; MCO is divided by 8
RCC_CFGR_MCO_PRE_16         EQU (0x40000000)      ; MCO is divided by 16

; ********************  Bit definition for RCC_PLLCFGR register  **************
RCC_PLLCFGR_PLLSRC          EQU (0x00000003)

RCC_PLLCFGR_PLLSRC_MSI      EQU (0x00000001)      ; MSI oscillator source clock selected
RCC_PLLCFGR_PLLSRC_HSI      EQU (0x00000002)      ; HSI16 oscillator source clock selected
RCC_PLLCFGR_PLLSRC_HSE      EQU (0x00000003)      ; HSE oscillator source clock selected

RCC_PLLCFGR_PLLM            EQU (0x00000070)
RCC_PLLCFGR_PLLM_0          EQU (0x00000010)
RCC_PLLCFGR_PLLM_1          EQU (0x00000020)
RCC_PLLCFGR_PLLM_2          EQU (0x00000040)

RCC_PLLCFGR_PLLN            EQU (0x00007F00)
RCC_PLLCFGR_PLLN_0          EQU (0x00000100)
RCC_PLLCFGR_PLLN_1          EQU (0x00000200)
RCC_PLLCFGR_PLLN_2          EQU (0x00000400)
RCC_PLLCFGR_PLLN_3          EQU (0x00000800)
RCC_PLLCFGR_PLLN_4          EQU (0x00001000)
RCC_PLLCFGR_PLLN_5          EQU (0x00002000)
RCC_PLLCFGR_PLLN_6          EQU (0x00004000)

RCC_PLLCFGR_PLLPEN          EQU (0x00010000)
RCC_PLLCFGR_PLLP            EQU (0x00020000)
RCC_PLLCFGR_PLLQEN          EQU (0x00100000)

RCC_PLLCFGR_PLLQ            EQU (0x00600000)
RCC_PLLCFGR_PLLQ_0          EQU (0x00200000)
RCC_PLLCFGR_PLLQ_1          EQU (0x00400000)

RCC_PLLCFGR_PLLREN          EQU (0x01000000)
RCC_PLLCFGR_PLLR            EQU (0x06000000)
RCC_PLLCFGR_PLLR_0          EQU (0x02000000)
RCC_PLLCFGR_PLLR_1          EQU (0x04000000)

; ********************  Bit definition for RCC_PLLSAI1CFGR register  ***********
RCC_PLLSAI1CFGR_PLLSAI1N    EQU (0x00007F00)
RCC_PLLSAI1CFGR_PLLSAI1N_0  EQU (0x00000100)
RCC_PLLSAI1CFGR_PLLSAI1N_1  EQU (0x00000200)
RCC_PLLSAI1CFGR_PLLSAI1N_2  EQU (0x00000400)
RCC_PLLSAI1CFGR_PLLSAI1N_3  EQU (0x00000800)
RCC_PLLSAI1CFGR_PLLSAI1N_4  EQU (0x00001000)
RCC_PLLSAI1CFGR_PLLSAI1N_5  EQU (0x00002000)
RCC_PLLSAI1CFGR_PLLSAI1N_6  EQU (0x00004000)

RCC_PLLSAI1CFGR_PLLSAI1PEN  EQU (0x00010000)
RCC_PLLSAI1CFGR_PLLSAI1P    EQU (0x00020000)

RCC_PLLSAI1CFGR_PLLSAI1QEN  EQU (0x00100000)
RCC_PLLSAI1CFGR_PLLSAI1Q    EQU (0x00600000)
RCC_PLLSAI1CFGR_PLLSAI1Q_0  EQU (0x00200000)
RCC_PLLSAI1CFGR_PLLSAI1Q_1  EQU (0x00400000)

RCC_PLLSAI1CFGR_PLLSAI1REN  EQU (0x01000000)
RCC_PLLSAI1CFGR_PLLSAI1R    EQU (0x06000000)
RCC_PLLSAI1CFGR_PLLSAI1R_0  EQU (0x02000000)
RCC_PLLSAI1CFGR_PLLSAI1R_1  EQU (0x04000000)

; ********************  Bit definition for RCC_PLLSAI2CFGR register  ***********
RCC_PLLSAI2CFGR_PLLSAI2N    EQU (0x00007F00)
RCC_PLLSAI2CFGR_PLLSAI2N_0  EQU (0x00000100)
RCC_PLLSAI2CFGR_PLLSAI2N_1  EQU (0x00000200)
RCC_PLLSAI2CFGR_PLLSAI2N_2  EQU (0x00000400)
RCC_PLLSAI2CFGR_PLLSAI2N_3  EQU (0x00000800)
RCC_PLLSAI2CFGR_PLLSAI2N_4  EQU (0x00001000)
RCC_PLLSAI2CFGR_PLLSAI2N_5  EQU (0x00002000)
RCC_PLLSAI2CFGR_PLLSAI2N_6  EQU (0x00004000)

RCC_PLLSAI2CFGR_PLLSAI2PEN  EQU (0x00010000)
RCC_PLLSAI2CFGR_PLLSAI2P    EQU (0x00020000)

RCC_PLLSAI2CFGR_PLLSAI2REN  EQU (0x01000000)
RCC_PLLSAI2CFGR_PLLSAI2R    EQU (0x06000000)
RCC_PLLSAI2CFGR_PLLSAI2R_0  EQU (0x02000000)
RCC_PLLSAI2CFGR_PLLSAI2R_1  EQU (0x04000000)

; ********************  Bit definition for RCC_CIER register  *****************
RCC_CIER_LSIRDYIE           EQU (0x00000001)
RCC_CIER_LSERDYIE           EQU (0x00000002)
RCC_CIER_MSIRDYIE           EQU (0x00000004)
RCC_CIER_HSIRDYIE           EQU (0x00000008)
RCC_CIER_HSERDYIE           EQU (0x00000010)
RCC_CIER_PLLRDYIE           EQU (0x00000020)
RCC_CIER_PLLSAI1RDYIE       EQU (0x00000040)
RCC_CIER_PLLSAI2RDYIE       EQU (0x00000080)
RCC_CIER_LSECSSIE           EQU (0x00000200)

; ********************  Bit definition for RCC_CIFR register  *****************
RCC_CIFR_LSIRDYF            EQU (0x00000001)
RCC_CIFR_LSERDYF            EQU (0x00000002)
RCC_CIFR_MSIRDYF            EQU (0x00000004)
RCC_CIFR_HSIRDYF            EQU (0x00000008)
RCC_CIFR_HSERDYF            EQU (0x00000010)
RCC_CIFR_PLLRDYF            EQU (0x00000020)
RCC_CIFR_PLLSAI1RDYF        EQU (0x00000040)
RCC_CIFR_PLLSAI2RDYF        EQU (0x00000080)
RCC_CIFR_CSSF               EQU (0x00000100)
RCC_CIFR_LSECSSF            EQU (0x00000200)

; ********************  Bit definition for RCC_CICR register  *****************
RCC_CICR_LSIRDYC            EQU (0x00000001)
RCC_CICR_LSERDYC            EQU (0x00000002)
RCC_CICR_MSIRDYC            EQU (0x00000004)
RCC_CICR_HSIRDYC            EQU (0x00000008)
RCC_CICR_HSERDYC            EQU (0x00000010)
RCC_CICR_PLLRDYC            EQU (0x00000020)
RCC_CICR_PLLSAI1RDYC        EQU (0x00000040)
RCC_CICR_PLLSAI2RDYC        EQU (0x00000080)
RCC_CICR_CSSC               EQU (0x00000100)
RCC_CICR_LSECSSC            EQU (0x00000200)

; ********************  Bit definition for RCC_AHB1RSTR register  *************
RCC_AHB1RSTR_DMA1RST        EQU (0x00000001)
RCC_AHB1RSTR_DMA2RST        EQU (0x00000002)
RCC_AHB1RSTR_FLASHRST       EQU (0x00000100)
RCC_AHB1RSTR_CRCRST         EQU (0x00001000)
RCC_AHB1RSTR_TSCRST         EQU (0x00010000)

; ********************  Bit definition for RCC_AHB2RSTR register  *************
RCC_AHB2RSTR_GPIOARST       EQU (0x00000001)
RCC_AHB2RSTR_GPIOBRST       EQU (0x00000002)
RCC_AHB2RSTR_GPIOCRST       EQU (0x00000004)
RCC_AHB2RSTR_GPIODRST       EQU (0x00000008)
RCC_AHB2RSTR_GPIOERST       EQU (0x00000010)
RCC_AHB2RSTR_GPIOFRST       EQU (0x00000020)
RCC_AHB2RSTR_GPIOGRST       EQU (0x00000040)
RCC_AHB2RSTR_GPIOHRST       EQU (0x00000080)
RCC_AHB2RSTR_OTGFSRST       EQU (0x00001000)
RCC_AHB2RSTR_ADCRST         EQU (0x00002000)
RCC_AHB2RSTR_RNGRST         EQU (0x00040000)

; ********************  Bit definition for RCC_AHB3RSTR register  *************
RCC_AHB3RSTR_FMCRST         EQU (0x00000001)
RCC_AHB3RSTR_QSPIRST        EQU (0x00000100)

; ********************  Bit definition for RCC_APB1RSTR1 register  *************
RCC_APB1RSTR1_TIM2RST       EQU (0x00000001)
RCC_APB1RSTR1_TIM3RST       EQU (0x00000002)
RCC_APB1RSTR1_TIM4RST       EQU (0x00000004)
RCC_APB1RSTR1_TIM5RST       EQU (0x00000008)
RCC_APB1RSTR1_TIM6RST       EQU (0x00000010)
RCC_APB1RSTR1_TIM7RST       EQU (0x00000020)
RCC_APB1RSTR1_LCDRST        EQU (0x00000200)
RCC_APB1RSTR1_SPI2RST       EQU (0x00004000)
RCC_APB1RSTR1_SPI3RST       EQU (0x00008000)
RCC_APB1RSTR1_USART2RST     EQU (0x00020000)
RCC_APB1RSTR1_USART3RST     EQU (0x00040000)
RCC_APB1RSTR1_UART4RST      EQU (0x00080000)
RCC_APB1RSTR1_UART5RST      EQU (0x00100000)
RCC_APB1RSTR1_I2C1RST       EQU (0x00200000)
RCC_APB1RSTR1_I2C2RST       EQU (0x00400000)
RCC_APB1RSTR1_I2C3RST       EQU (0x00800000)
RCC_APB1RSTR1_CAN1RST       EQU (0x02000000)
RCC_APB1RSTR1_PWRRST        EQU (0x10000000)
RCC_APB1RSTR1_DAC1RST       EQU (0x20000000)
RCC_APB1RSTR1_OPAMPRST      EQU (0x40000000)
RCC_APB1RSTR1_LPTIM1RST     EQU (0x80000000)

; ********************  Bit definition for RCC_APB1RSTR2 register  *************
RCC_APB1RSTR2_LPUART1RST    EQU (0x00000001)
RCC_APB1RSTR2_SWPMI1RST     EQU (0x00000004)
RCC_APB1RSTR2_LPTIM2RST     EQU (0x00000020)

; ********************  Bit definition for RCC_APB2RSTR register  *************
RCC_APB2RSTR_SYSCFGRST      EQU (0x00000001)
RCC_APB2RSTR_SDMMC1RST      EQU (0x00000400)
RCC_APB2RSTR_TIM1RST        EQU (0x00000800)
RCC_APB2RSTR_SPI1RST        EQU (0x00001000)
RCC_APB2RSTR_TIM8RST        EQU (0x00002000)
RCC_APB2RSTR_USART1RST      EQU (0x00004000)
RCC_APB2RSTR_TIM15RST       EQU (0x00010000)
RCC_APB2RSTR_TIM16RST       EQU (0x00020000)
RCC_APB2RSTR_TIM17RST       EQU (0x00040000)
RCC_APB2RSTR_SAI1RST        EQU (0x00200000)
RCC_APB2RSTR_SAI2RST        EQU (0x00400000)
RCC_APB2RSTR_DFSDMRST       EQU (0x01000000)

; ********************  Bit definition for RCC_AHB1ENR register  **************
RCC_AHB1ENR_DMA1EN          EQU (0x00000001)
RCC_AHB1ENR_DMA2EN          EQU (0x00000002)
RCC_AHB1ENR_FLASHEN         EQU (0x00000100)
RCC_AHB1ENR_CRCEN           EQU (0x00001000)
RCC_AHB1ENR_TSCEN           EQU (0x00010000)

; ********************  Bit definition for RCC_AHB2ENR register  **************
RCC_AHB2ENR_GPIOAEN         EQU (0x00000001)
RCC_AHB2ENR_GPIOBEN         EQU (0x00000002)
RCC_AHB2ENR_GPIOCEN         EQU (0x00000004)
RCC_AHB2ENR_GPIODEN         EQU (0x00000008)
RCC_AHB2ENR_GPIOEEN         EQU (0x00000010)
RCC_AHB2ENR_GPIOFEN         EQU (0x00000020)
RCC_AHB2ENR_GPIOGEN         EQU (0x00000040)
RCC_AHB2ENR_GPIOHEN         EQU (0x00000080)
RCC_AHB2ENR_OTGFSEN         EQU (0x00001000)
RCC_AHB2ENR_ADCEN           EQU (0x00002000)
RCC_AHB2ENR_RNGEN           EQU (0x00040000)

; ********************  Bit definition for RCC_AHB3ENR register  **************
RCC_AHB3ENR_FMCEN           EQU (0x00000001)
RCC_AHB3ENR_QSPIEN          EQU (0x00000100)

; ********************  Bit definition for RCC_APB1ENR1 register  **************
RCC_APB1ENR1_TIM2EN         EQU (0x00000001)
RCC_APB1ENR1_TIM3EN         EQU (0x00000002)
RCC_APB1ENR1_TIM4EN         EQU (0x00000004)
RCC_APB1ENR1_TIM5EN         EQU (0x00000008)
RCC_APB1ENR1_TIM6EN         EQU (0x00000010)
RCC_APB1ENR1_TIM7EN         EQU (0x00000020)
RCC_APB1ENR1_LCDEN          EQU (0x00000200)
RCC_APB1ENR1_WWDGEN         EQU (0x00000800)
RCC_APB1ENR1_SPI2EN         EQU (0x00004000)
RCC_APB1ENR1_SPI3EN         EQU (0x00008000)
RCC_APB1ENR1_USART2EN       EQU (0x00020000)
RCC_APB1ENR1_USART3EN       EQU (0x00040000)
RCC_APB1ENR1_UART4EN        EQU (0x00080000)
RCC_APB1ENR1_UART5EN        EQU (0x00100000)
RCC_APB1ENR1_I2C1EN         EQU (0x00200000)
RCC_APB1ENR1_I2C2EN         EQU (0x00400000)
RCC_APB1ENR1_I2C3EN         EQU (0x00800000)
RCC_APB1ENR1_CAN1EN         EQU (0x02000000)
RCC_APB1ENR1_PWREN          EQU (0x10000000)
RCC_APB1ENR1_DAC1EN         EQU (0x20000000)
RCC_APB1ENR1_OPAMPEN        EQU (0x40000000)
RCC_APB1ENR1_LPTIM1EN       EQU (0x80000000)

; ********************  Bit definition for RCC_APB1RSTR2 register  *************
RCC_APB1ENR2_LPUART1EN      EQU (0x00000001)
RCC_APB1ENR2_SWPMI1EN       EQU (0x00000004)
RCC_APB1ENR2_LPTIM2EN       EQU (0x00000020)

; ********************  Bit definition for RCC_APB2ENR register  **************
RCC_APB2ENR_SYSCFGEN        EQU (0x00000001)
RCC_APB2ENR_FWEN            EQU (0x00000080)
RCC_APB2ENR_SDMMC1EN        EQU (0x00000400)
RCC_APB2ENR_TIM1EN          EQU (0x00000800)
RCC_APB2ENR_SPI1EN          EQU (0x00001000)
RCC_APB2ENR_TIM8EN          EQU (0x00002000)
RCC_APB2ENR_USART1EN        EQU (0x00004000)
RCC_APB2ENR_TIM15EN         EQU (0x00010000)
RCC_APB2ENR_TIM16EN         EQU (0x00020000)
RCC_APB2ENR_TIM17EN         EQU (0x00040000)
RCC_APB2ENR_SAI1EN          EQU (0x00200000)
RCC_APB2ENR_SAI2EN          EQU (0x00400000)
RCC_APB2ENR_DFSDMEN         EQU (0x01000000)

; ********************  Bit definition for RCC_AHB1SMENR register  **************
RCC_AHB1SMENR_DMA1SMEN      EQU (0x00000001)
RCC_AHB1SMENR_DMA2SMEN      EQU (0x00000002)
RCC_AHB1SMENR_FLASHSMEN     EQU (0x00000100)
RCC_AHB1SMENR_SRAM1SMEN     EQU (0x00000200)
RCC_AHB1SMENR_CRCSMEN       EQU (0x00001000)
RCC_AHB1SMENR_TSCSMEN       EQU (0x00010000)

; ********************  Bit definition for RCC_AHB2SMENR register  ************
RCC_AHB2SMENR_GPIOASMEN     EQU (0x00000001)
RCC_AHB2SMENR_GPIOBSMEN     EQU (0x00000002)
RCC_AHB2SMENR_GPIOCSMEN     EQU (0x00000004)
RCC_AHB2SMENR_GPIODSMEN     EQU (0x00000008)
RCC_AHB2SMENR_GPIOESMEN     EQU (0x00000010)
RCC_AHB2SMENR_GPIOFSMEN     EQU (0x00000020)
RCC_AHB2SMENR_GPIOGSMEN     EQU (0x00000040)
RCC_AHB2SMENR_GPIOHSMEN     EQU (0x00000080)
RCC_AHB2SMENR_SRAM2SMEN     EQU (0x00000200)
RCC_AHB2SMENR_OTGFSSMEN     EQU (0x00001000)
RCC_AHB2SMENR_ADCSMEN       EQU (0x00002000)
RCC_AHB2SMENR_RNGSMEN       EQU (0x00040000)

; ********************  Bit definition for RCC_AHB3SMENR register  ************
RCC_AHB3SMENR_FMCSMEN       EQU (0x00000001)
RCC_AHB3SMENR_QSPISMEN      EQU (0x00000100)

; ********************  Bit definition for RCC_APB1SMENR1 register  ************
RCC_APB1SMENR1_TIM2SMEN     EQU (0x00000001)
RCC_APB1SMENR1_TIM3SMEN     EQU (0x00000002)
RCC_APB1SMENR1_TIM4SMEN     EQU (0x00000004)
RCC_APB1SMENR1_TIM5SMEN     EQU (0x00000008)
RCC_APB1SMENR1_TIM6SMEN     EQU (0x00000010)
RCC_APB1SMENR1_TIM7SMEN     EQU (0x00000020)
RCC_APB1SMENR1_LCDSMEN      EQU (0x00000200)
RCC_APB1SMENR1_WWDGSMEN     EQU (0x00000800)
RCC_APB1SMENR1_SPI2SMEN     EQU (0x00004000)
RCC_APB1SMENR1_SPI3SMEN     EQU (0x00008000)
RCC_APB1SMENR1_USART2SMEN   EQU (0x00020000)
RCC_APB1SMENR1_USART3SMEN   EQU (0x00040000)
RCC_APB1SMENR1_UART4SMEN    EQU (0x00080000)
RCC_APB1SMENR1_UART5SMEN    EQU (0x00100000)
RCC_APB1SMENR1_I2C1SMEN     EQU (0x00200000)
RCC_APB1SMENR1_I2C2SMEN     EQU (0x00400000)
RCC_APB1SMENR1_I2C3SMEN     EQU (0x00800000)
RCC_APB1SMENR1_CAN1SMEN     EQU (0x02000000)
RCC_APB1SMENR1_PWRSMEN      EQU (0x10000000)
RCC_APB1SMENR1_DAC1SMEN     EQU (0x20000000)
RCC_APB1SMENR1_OPAMPSMEN    EQU (0x40000000)
RCC_APB1SMENR1_LPTIM1SMEN   EQU (0x80000000)

; ********************  Bit definition for RCC_APB1SMENR2 register  ************
RCC_APB1SMENR2_LPUART1SMEN  EQU (0x00000001)
RCC_APB1SMENR2_SWPMI1SMEN   EQU (0x00000004)
RCC_APB1SMENR2_LPTIM2SMEN   EQU (0x00000020)

; ********************  Bit definition for RCC_APB2SMENR register  ************
RCC_APB2SMENR_SYSCFGSMEN    EQU (0x00000001)
RCC_APB2SMENR_SDMMC1SMEN    EQU (0x00000400)
RCC_APB2SMENR_TIM1SMEN      EQU (0x00000800)
RCC_APB2SMENR_SPI1SMEN      EQU (0x00001000)
RCC_APB2SMENR_TIM8SMEN      EQU (0x00002000)
RCC_APB2SMENR_USART1SMEN    EQU (0x00004000)
RCC_APB2SMENR_TIM15SMEN     EQU (0x00010000)
RCC_APB2SMENR_TIM16SMEN     EQU (0x00020000)
RCC_APB2SMENR_TIM17SMEN     EQU (0x00040000)
RCC_APB2SMENR_SAI1SMEN      EQU (0x00200000)
RCC_APB2SMENR_SAI2SMEN      EQU (0x00400000)
RCC_APB2SMENR_DFSDMSMEN     EQU (0x01000000)

; ********************  Bit definition for RCC_CCIPR register  *****************
RCC_CCIPR_USART1SEL         EQU (0x00000003)
RCC_CCIPR_USART1SEL_0       EQU (0x00000001)
RCC_CCIPR_USART1SEL_1       EQU (0x00000002)

RCC_CCIPR_USART2SEL         EQU (0x0000000C)
RCC_CCIPR_USART2SEL_0       EQU (0x00000004)
RCC_CCIPR_USART2SEL_1       EQU (0x00000008)

RCC_CCIPR_USART3SEL         EQU (0x00000030)
RCC_CCIPR_USART3SEL_0       EQU (0x00000010)
RCC_CCIPR_USART3SEL_1       EQU (0x00000020)

RCC_CCIPR_UART4SEL          EQU (0x000000C0)
RCC_CCIPR_UART4SEL_0        EQU (0x00000040)
RCC_CCIPR_UART4SEL_1        EQU (0x00000080)

RCC_CCIPR_UART5SEL          EQU (0x00000300)
RCC_CCIPR_UART5SEL_0        EQU (0x00000100)
RCC_CCIPR_UART5SEL_1        EQU (0x00000200)

RCC_CCIPR_LPUART1SEL        EQU (0x00000C00)
RCC_CCIPR_LPUART1SEL_0      EQU (0x00000400)
RCC_CCIPR_LPUART1SEL_1      EQU (0x00000800)

RCC_CCIPR_I2C1SEL           EQU (0x00003000)
RCC_CCIPR_I2C1SEL_0         EQU (0x00001000)
RCC_CCIPR_I2C1SEL_1         EQU (0x00002000)

RCC_CCIPR_I2C2SEL           EQU (0x0000C000)
RCC_CCIPR_I2C2SEL_0         EQU (0x00004000)
RCC_CCIPR_I2C2SEL_1         EQU (0x00008000)

RCC_CCIPR_I2C3SEL           EQU (0x00030000)
RCC_CCIPR_I2C3SEL_0         EQU (0x00010000)
RCC_CCIPR_I2C3SEL_1         EQU (0x00020000)

RCC_CCIPR_LPTIM1SEL         EQU (0x000C0000)
RCC_CCIPR_LPTIM1SEL_0       EQU (0x00040000)
RCC_CCIPR_LPTIM1SEL_1       EQU (0x00080000)

RCC_CCIPR_LPTIM2SEL         EQU (0x00300000)
RCC_CCIPR_LPTIM2SEL_0       EQU (0x00100000)
RCC_CCIPR_LPTIM2SEL_1       EQU (0x00200000)

RCC_CCIPR_SAI1SEL           EQU (0x00C00000)
RCC_CCIPR_SAI1SEL_0         EQU (0x00400000)
RCC_CCIPR_SAI1SEL_1         EQU (0x00800000)

RCC_CCIPR_SAI2SEL           EQU (0x03000000)
RCC_CCIPR_SAI2SEL_0         EQU (0x01000000)
RCC_CCIPR_SAI2SEL_1         EQU (0x02000000)

RCC_CCIPR_CLK48SEL          EQU (0x0C000000)
RCC_CCIPR_CLK48SEL_0        EQU (0x04000000)
RCC_CCIPR_CLK48SEL_1        EQU (0x08000000)

RCC_CCIPR_ADCSEL            EQU (0x30000000)
RCC_CCIPR_ADCSEL_0          EQU (0x10000000)
RCC_CCIPR_ADCSEL_1          EQU (0x20000000)

RCC_CCIPR_SWPMI1SEL         EQU (0x40000000)
RCC_CCIPR_DFSDMSEL          EQU (0x80000000)

; ********************  Bit definition for RCC_BDCR register  *****************
RCC_BDCR_LSEON              EQU (0x00000001)
RCC_BDCR_LSERDY             EQU (0x00000002)
RCC_BDCR_LSEBYP             EQU (0x00000004)

RCC_BDCR_LSEDRV             EQU (0x00000018)
RCC_BDCR_LSEDRV_0           EQU (0x00000008)
RCC_BDCR_LSEDRV_1           EQU (0x00000010)

RCC_BDCR_LSECSSON           EQU (0x00000020)
RCC_BDCR_LSECSSD            EQU (0x00000040)

RCC_BDCR_RTCSEL             EQU (0x00000300)
RCC_BDCR_RTCSEL_0           EQU (0x00000100)
RCC_BDCR_RTCSEL_1           EQU (0x00000200)

RCC_BDCR_RTCEN              EQU (0x00008000)
RCC_BDCR_BDRST              EQU (0x00010000)
RCC_BDCR_LSCOEN             EQU (0x01000000)
RCC_BDCR_LSCOSEL            EQU (0x02000000)

; ********************  Bit definition for RCC_CSR register  ******************
RCC_CSR_LSION               EQU (0x00000001)
RCC_CSR_LSIRDY              EQU (0x00000002)

RCC_CSR_MSISRANGE           EQU (0x00000F00)
RCC_CSR_MSISRANGE_1         EQU (0x00000400)      ; MSI frequency 1MHZ
RCC_CSR_MSISRANGE_2         EQU (0x00000500)      ; MSI frequency 2MHZ
RCC_CSR_MSISRANGE_4         EQU (0x00000600)      ; The default frequency 4MHZ
RCC_CSR_MSISRANGE_8         EQU (0x00000700)      ; MSI frequency 8MHZ

RCC_CSR_RMVF                EQU (0x00800000)
RCC_CSR_FWRSTF              EQU (0x01000000)
RCC_CSR_OBLRSTF             EQU (0x02000000)
RCC_CSR_PINRSTF             EQU (0x04000000)
RCC_CSR_BORRSTF             EQU (0x08000000)
RCC_CSR_SFTRSTF             EQU (0x10000000)
RCC_CSR_IWDGRSTF            EQU (0x20000000)
RCC_CSR_WWDGRSTF            EQU (0x40000000)
RCC_CSR_LPWRRSTF            EQU (0x80000000)



; *****************************************************************************
;
;                                    RNG
;
; *****************************************************************************
; ********************  Bits definition for RNG_CR register  ******************
RNG_CR_RNGEN                 EQU (0x00000004)
RNG_CR_IE     EQU (0x00000008)

; ********************  Bits definition for RNG_SR register  ******************
RNG_SR_DRDY                  EQU (0x00000001)
RNG_SR_CECS                  EQU (0x00000002)
RNG_SR_SECS                  EQU (0x00000004)
RNG_SR_CEIS                  EQU (0x00000020)
RNG_SR_SEIS                  EQU (0x00000040)

; *****************************************************************************
;
;                           Real-Time Clock (RTC)
;
; *****************************************************************************
; ********************  Bits definition for RTC_TR register  ******************
RTC_TR_PM     EQU (0x00400000)
RTC_TR_HT     EQU (0x00300000)
RTC_TR_HT_0                  EQU (0x00100000)
RTC_TR_HT_1                  EQU (0x00200000)
RTC_TR_HU     EQU (0x000F0000)
RTC_TR_HU_0                  EQU (0x00010000)
RTC_TR_HU_1                  EQU (0x00020000)
RTC_TR_HU_2                  EQU (0x00040000)
RTC_TR_HU_3                  EQU (0x00080000)
RTC_TR_MNT                   EQU (0x00007000)
RTC_TR_MNT_0                 EQU (0x00001000)
RTC_TR_MNT_1                 EQU (0x00002000)
RTC_TR_MNT_2                 EQU (0x00004000)
RTC_TR_MNU                   EQU (0x00000F00)
RTC_TR_MNU_0                 EQU (0x00000100)
RTC_TR_MNU_1                 EQU (0x00000200)
RTC_TR_MNU_2                 EQU (0x00000400)
RTC_TR_MNU_3                 EQU (0x00000800)
RTC_TR_ST     EQU (0x00000070)
RTC_TR_ST_0                  EQU (0x00000010)
RTC_TR_ST_1                  EQU (0x00000020)
RTC_TR_ST_2                  EQU (0x00000040)
RTC_TR_SU     EQU (0x0000000F)
RTC_TR_SU_0                  EQU (0x00000001)
RTC_TR_SU_1                  EQU (0x00000002)
RTC_TR_SU_2                  EQU (0x00000004)
RTC_TR_SU_3                  EQU (0x00000008)

; ********************  Bits definition for RTC_DR register  ******************
RTC_DR_YT     EQU (0x00F00000)
RTC_DR_YT_0                  EQU (0x00100000)
RTC_DR_YT_1                  EQU (0x00200000)
RTC_DR_YT_2                  EQU (0x00400000)
RTC_DR_YT_3                  EQU (0x00800000)
RTC_DR_YU     EQU (0x000F0000)
RTC_DR_YU_0                  EQU (0x00010000)
RTC_DR_YU_1                  EQU (0x00020000)
RTC_DR_YU_2                  EQU (0x00040000)
RTC_DR_YU_3                  EQU (0x00080000)
RTC_DR_WDU                   EQU (0x0000E000)
RTC_DR_WDU_0                 EQU (0x00002000)
RTC_DR_WDU_1                 EQU (0x00004000)
RTC_DR_WDU_2                 EQU (0x00008000)
RTC_DR_MT     EQU (0x00001000)
RTC_DR_MU     EQU (0x00000F00)
RTC_DR_MU_0                  EQU (0x00000100)
RTC_DR_MU_1                  EQU (0x00000200)
RTC_DR_MU_2                  EQU (0x00000400)
RTC_DR_MU_3                  EQU (0x00000800)
RTC_DR_DT     EQU (0x00000030)
RTC_DR_DT_0                  EQU (0x00000010)
RTC_DR_DT_1                  EQU (0x00000020)
RTC_DR_DU     EQU (0x0000000F)
RTC_DR_DU_0                  EQU (0x00000001)
RTC_DR_DU_1                  EQU (0x00000002)
RTC_DR_DU_2                  EQU (0x00000004)
RTC_DR_DU_3                  EQU (0x00000008)

; ********************  Bits definition for RTC_CR register  ******************
RTC_CR_ITSE                  EQU (0x01000000)
RTC_CR_COE                   EQU (0x00800000)
RTC_CR_OSEL                  EQU (0x00600000)
RTC_CR_OSEL_0                EQU (0x00200000)
RTC_CR_OSEL_1                EQU (0x00400000)
RTC_CR_POL                   EQU (0x00100000)
RTC_CR_COSEL                 EQU (0x00080000)
RTC_CR_BCK                   EQU (0x00040000)
RTC_CR_SUB1H                 EQU (0x00020000)
RTC_CR_ADD1H                 EQU (0x00010000)
RTC_CR_TSIE                  EQU (0x00008000)
RTC_CR_WUTIE                 EQU (0x00004000)
RTC_CR_ALRBIE                EQU (0x00002000)
RTC_CR_ALRAIE                EQU (0x00001000)
RTC_CR_TSE                   EQU (0x00000800)
RTC_CR_WUTE                  EQU (0x00000400)
RTC_CR_ALRBE                 EQU (0x00000200)
RTC_CR_ALRAE                 EQU (0x00000100)
RTC_CR_FMT                   EQU (0x00000040)
RTC_CR_BYPSHAD               EQU (0x00000020)
RTC_CR_REFCKON               EQU (0x00000010)
RTC_CR_TSEDGE                EQU (0x00000008)
RTC_CR_WUCKSEL               EQU (0x00000007)
RTC_CR_WUCKSEL_0             EQU (0x00000001)
RTC_CR_WUCKSEL_1             EQU (0x00000002)
RTC_CR_WUCKSEL_2             EQU (0x00000004)

; ********************  Bits definition for RTC_ISR register  *****************
RTC_ISR_ITSF                 EQU (0x00020000)
RTC_ISR_RECALPF              EQU (0x00010000)
RTC_ISR_TAMP3F               EQU (0x00008000)
RTC_ISR_TAMP2F               EQU (0x00004000)
RTC_ISR_TAMP1F               EQU (0x00002000)
RTC_ISR_TSOVF                EQU (0x00001000)
RTC_ISR_TSF                  EQU (0x00000800)
RTC_ISR_WUTF                 EQU (0x00000400)
RTC_ISR_ALRBF                EQU (0x00000200)
RTC_ISR_ALRAF                EQU (0x00000100)
RTC_ISR_INIT                 EQU (0x00000080)
RTC_ISR_INITF                EQU (0x00000040)
RTC_ISR_RSF                  EQU (0x00000020)
RTC_ISR_INITS                EQU (0x00000010)
RTC_ISR_SHPF                 EQU (0x00000008)
RTC_ISR_WUTWF                EQU (0x00000004)
RTC_ISR_ALRBWF               EQU (0x00000002)
RTC_ISR_ALRAWF               EQU (0x00000001)

; ********************  Bits definition for RTC_PRER register  ****************
RTC_PRER_PREDIV_A            EQU (0x007F0000)
RTC_PRER_PREDIV_S            EQU (0x00007FFF)

; ********************  Bits definition for RTC_WUTR register  ****************
RTC_WUTR_WUT                 EQU (0x0000FFFF)

; ********************  Bits definition for RTC_ALRMAR register  **************
RTC_ALRMAR_MSK4              EQU (0x80000000)
RTC_ALRMAR_WDSEL             EQU (0x40000000)
RTC_ALRMAR_DT                EQU (0x30000000)
RTC_ALRMAR_DT_0              EQU (0x10000000)
RTC_ALRMAR_DT_1              EQU (0x20000000)
RTC_ALRMAR_DU                EQU (0x0F000000)
RTC_ALRMAR_DU_0              EQU (0x01000000)
RTC_ALRMAR_DU_1              EQU (0x02000000)
RTC_ALRMAR_DU_2              EQU (0x04000000)
RTC_ALRMAR_DU_3              EQU (0x08000000)
RTC_ALRMAR_MSK3              EQU (0x00800000)
RTC_ALRMAR_PM                EQU (0x00400000)
RTC_ALRMAR_HT                EQU (0x00300000)
RTC_ALRMAR_HT_0              EQU (0x00100000)
RTC_ALRMAR_HT_1              EQU (0x00200000)
RTC_ALRMAR_HU                EQU (0x000F0000)
RTC_ALRMAR_HU_0              EQU (0x00010000)
RTC_ALRMAR_HU_1              EQU (0x00020000)
RTC_ALRMAR_HU_2              EQU (0x00040000)
RTC_ALRMAR_HU_3              EQU (0x00080000)
RTC_ALRMAR_MSK2              EQU (0x00008000)
RTC_ALRMAR_MNT               EQU (0x00007000)
RTC_ALRMAR_MNT_0             EQU (0x00001000)
RTC_ALRMAR_MNT_1             EQU (0x00002000)
RTC_ALRMAR_MNT_2             EQU (0x00004000)
RTC_ALRMAR_MNU               EQU (0x00000F00)
RTC_ALRMAR_MNU_0             EQU (0x00000100)
RTC_ALRMAR_MNU_1             EQU (0x00000200)
RTC_ALRMAR_MNU_2             EQU (0x00000400)
RTC_ALRMAR_MNU_3             EQU (0x00000800)
RTC_ALRMAR_MSK1              EQU (0x00000080)
RTC_ALRMAR_ST                EQU (0x00000070)
RTC_ALRMAR_ST_0              EQU (0x00000010)
RTC_ALRMAR_ST_1              EQU (0x00000020)
RTC_ALRMAR_ST_2              EQU (0x00000040)
RTC_ALRMAR_SU                EQU (0x0000000F)
RTC_ALRMAR_SU_0              EQU (0x00000001)
RTC_ALRMAR_SU_1              EQU (0x00000002)
RTC_ALRMAR_SU_2              EQU (0x00000004)
RTC_ALRMAR_SU_3              EQU (0x00000008)

; ********************  Bits definition for RTC_ALRMBR register  **************
RTC_ALRMBR_MSK4              EQU (0x80000000)
RTC_ALRMBR_WDSEL             EQU (0x40000000)
RTC_ALRMBR_DT                EQU (0x30000000)
RTC_ALRMBR_DT_0              EQU (0x10000000)
RTC_ALRMBR_DT_1              EQU (0x20000000)
RTC_ALRMBR_DU                EQU (0x0F000000)
RTC_ALRMBR_DU_0              EQU (0x01000000)
RTC_ALRMBR_DU_1              EQU (0x02000000)
RTC_ALRMBR_DU_2              EQU (0x04000000)
RTC_ALRMBR_DU_3              EQU (0x08000000)
RTC_ALRMBR_MSK3              EQU (0x00800000)
RTC_ALRMBR_PM                EQU (0x00400000)
RTC_ALRMBR_HT                EQU (0x00300000)
RTC_ALRMBR_HT_0              EQU (0x00100000)
RTC_ALRMBR_HT_1              EQU (0x00200000)
RTC_ALRMBR_HU                EQU (0x000F0000)
RTC_ALRMBR_HU_0              EQU (0x00010000)
RTC_ALRMBR_HU_1              EQU (0x00020000)
RTC_ALRMBR_HU_2              EQU (0x00040000)
RTC_ALRMBR_HU_3              EQU (0x00080000)
RTC_ALRMBR_MSK2              EQU (0x00008000)
RTC_ALRMBR_MNT               EQU (0x00007000)
RTC_ALRMBR_MNT_0             EQU (0x00001000)
RTC_ALRMBR_MNT_1             EQU (0x00002000)
RTC_ALRMBR_MNT_2             EQU (0x00004000)
RTC_ALRMBR_MNU               EQU (0x00000F00)
RTC_ALRMBR_MNU_0             EQU (0x00000100)
RTC_ALRMBR_MNU_1             EQU (0x00000200)
RTC_ALRMBR_MNU_2             EQU (0x00000400)
RTC_ALRMBR_MNU_3             EQU (0x00000800)
RTC_ALRMBR_MSK1              EQU (0x00000080)
RTC_ALRMBR_ST                EQU (0x00000070)
RTC_ALRMBR_ST_0              EQU (0x00000010)
RTC_ALRMBR_ST_1              EQU (0x00000020)
RTC_ALRMBR_ST_2              EQU (0x00000040)
RTC_ALRMBR_SU                EQU (0x0000000F)
RTC_ALRMBR_SU_0              EQU (0x00000001)
RTC_ALRMBR_SU_1              EQU (0x00000002)
RTC_ALRMBR_SU_2              EQU (0x00000004)
RTC_ALRMBR_SU_3              EQU (0x00000008)

; ********************  Bits definition for RTC_WPR register  *****************
RTC_WPR_KEY                  EQU (0x000000FF)

; ********************  Bits definition for RTC_SSR register  *****************
RTC_SSR_SS                   EQU (0x0000FFFF)

; ********************  Bits definition for RTC_SHIFTR register  **************
RTC_SHIFTR_SUBFS             EQU (0x00007FFF)
RTC_SHIFTR_ADD1S             EQU (0x80000000)

; ********************  Bits definition for RTC_TSTR register  ****************
RTC_TSTR_PM                  EQU (0x00400000)
RTC_TSTR_HT                  EQU (0x00300000)
RTC_TSTR_HT_0                EQU (0x00100000)
RTC_TSTR_HT_1                EQU (0x00200000)
RTC_TSTR_HU                  EQU (0x000F0000)
RTC_TSTR_HU_0                EQU (0x00010000)
RTC_TSTR_HU_1                EQU (0x00020000)
RTC_TSTR_HU_2                EQU (0x00040000)
RTC_TSTR_HU_3                EQU (0x00080000)
RTC_TSTR_MNT                 EQU (0x00007000)
RTC_TSTR_MNT_0               EQU (0x00001000)
RTC_TSTR_MNT_1               EQU (0x00002000)
RTC_TSTR_MNT_2               EQU (0x00004000)
RTC_TSTR_MNU                 EQU (0x00000F00)
RTC_TSTR_MNU_0               EQU (0x00000100)
RTC_TSTR_MNU_1               EQU (0x00000200)
RTC_TSTR_MNU_2               EQU (0x00000400)
RTC_TSTR_MNU_3               EQU (0x00000800)
RTC_TSTR_ST                  EQU (0x00000070)
RTC_TSTR_ST_0                EQU (0x00000010)
RTC_TSTR_ST_1                EQU (0x00000020)
RTC_TSTR_ST_2                EQU (0x00000040)
RTC_TSTR_SU                  EQU (0x0000000F)
RTC_TSTR_SU_0                EQU (0x00000001)
RTC_TSTR_SU_1                EQU (0x00000002)
RTC_TSTR_SU_2                EQU (0x00000004)
RTC_TSTR_SU_3                EQU (0x00000008)

; ********************  Bits definition for RTC_TSDR register  ****************
RTC_TSDR_WDU                 EQU (0x0000E000)
RTC_TSDR_WDU_0               EQU (0x00002000)
RTC_TSDR_WDU_1               EQU (0x00004000)
RTC_TSDR_WDU_2               EQU (0x00008000)
RTC_TSDR_MT                  EQU (0x00001000)
RTC_TSDR_MU                  EQU (0x00000F00)
RTC_TSDR_MU_0                EQU (0x00000100)
RTC_TSDR_MU_1                EQU (0x00000200)
RTC_TSDR_MU_2                EQU (0x00000400)
RTC_TSDR_MU_3                EQU (0x00000800)
RTC_TSDR_DT                  EQU (0x00000030)
RTC_TSDR_DT_0                EQU (0x00000010)
RTC_TSDR_DT_1                EQU (0x00000020)
RTC_TSDR_DU                  EQU (0x0000000F)
RTC_TSDR_DU_0                EQU (0x00000001)
RTC_TSDR_DU_1                EQU (0x00000002)
RTC_TSDR_DU_2                EQU (0x00000004)
RTC_TSDR_DU_3                EQU (0x00000008)

; ********************  Bits definition for RTC_TSSSR register  ***************
RTC_TSSSR_SS                 EQU (0x0000FFFF)

; ********************  Bits definition for RTC_CAL register  ****************
RTC_CALR_CALP                EQU (0x00008000)
RTC_CALR_CALW8               EQU (0x00004000)
RTC_CALR_CALW16              EQU (0x00002000)
RTC_CALR_CALM                EQU (0x000001FF)
RTC_CALR_CALM_0              EQU (0x00000001)
RTC_CALR_CALM_1              EQU (0x00000002)
RTC_CALR_CALM_2              EQU (0x00000004)
RTC_CALR_CALM_3              EQU (0x00000008)
RTC_CALR_CALM_4              EQU (0x00000010)
RTC_CALR_CALM_5              EQU (0x00000020)
RTC_CALR_CALM_6              EQU (0x00000040)
RTC_CALR_CALM_7              EQU (0x00000080)
RTC_CALR_CALM_8              EQU (0x00000100)

; ********************  Bits definition for RTC_TAMPCR register  **************
RTC_TAMPCR_TAMP3MF           EQU (0x01000000)
RTC_TAMPCR_TAMP3NOERASE      EQU (0x00800000)
RTC_TAMPCR_TAMP3IE           EQU (0x00400000)
RTC_TAMPCR_TAMP2MF           EQU (0x00200000)
RTC_TAMPCR_TAMP2NOERASE      EQU (0x00100000)
RTC_TAMPCR_TAMP2IE           EQU (0x00080000)
RTC_TAMPCR_TAMP1MF           EQU (0x00040000)
RTC_TAMPCR_TAMP1NOERASE      EQU (0x00020000)
RTC_TAMPCR_TAMP1IE           EQU (0x00010000)
RTC_TAMPCR_TAMPPUDIS         EQU (0x00008000)
RTC_TAMPCR_TAMPPRCH          EQU (0x00006000)
RTC_TAMPCR_TAMPPRCH_0        EQU (0x00002000)
RTC_TAMPCR_TAMPPRCH_1        EQU (0x00004000)
RTC_TAMPCR_TAMPFLT           EQU (0x00001800)
RTC_TAMPCR_TAMPFLT_0         EQU (0x00000800)
RTC_TAMPCR_TAMPFLT_1         EQU (0x00001000)
RTC_TAMPCR_TAMPFREQ          EQU (0x00000700)
RTC_TAMPCR_TAMPFREQ_0        EQU (0x00000100)
RTC_TAMPCR_TAMPFREQ_1        EQU (0x00000200)
RTC_TAMPCR_TAMPFREQ_2        EQU (0x00000400)
RTC_TAMPCR_TAMPTS            EQU (0x00000080)
RTC_TAMPCR_TAMP3TRG          EQU (0x00000040)
RTC_TAMPCR_TAMP3E            EQU (0x00000020)
RTC_TAMPCR_TAMP2TRG          EQU (0x00000010)
RTC_TAMPCR_TAMP2E            EQU (0x00000008)
RTC_TAMPCR_TAMPIE            EQU (0x00000004)
RTC_TAMPCR_TAMP1TRG          EQU (0x00000002)
RTC_TAMPCR_TAMP1E            EQU (0x00000001)

; ********************  Bits definition for RTC_ALRMASSR register  ************
RTC_ALRMASSR_MASKSS          EQU (0x0F000000)
RTC_ALRMASSR_MASKSS_0        EQU (0x01000000)
RTC_ALRMASSR_MASKSS_1        EQU (0x02000000)
RTC_ALRMASSR_MASKSS_2        EQU (0x04000000)
RTC_ALRMASSR_MASKSS_3        EQU (0x08000000)
RTC_ALRMASSR_SS              EQU (0x00007FFF)

; ********************  Bits definition for RTC_ALRMBSSR register  ************
RTC_ALRMBSSR_MASKSS          EQU (0x0F000000)
RTC_ALRMBSSR_MASKSS_0        EQU (0x01000000)
RTC_ALRMBSSR_MASKSS_1        EQU (0x02000000)
RTC_ALRMBSSR_MASKSS_2        EQU (0x04000000)
RTC_ALRMBSSR_MASKSS_3        EQU (0x08000000)
RTC_ALRMBSSR_SS              EQU (0x00007FFF)

; ********************  Bits definition for RTC_0R register  ******************
RTC_OR_OUT_RMP              EQU (0x00000002)
RTC_OR_ALARMOUTTYPE         EQU (0x00000001)


; ********************  Bits definition for RTC_BKP0R register  ***************
RTC_BKP0R_MASK     EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP1R register  ***************
RTC_BKP1R_MASK     EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP2R register  ***************
RTC_BKP2R_MASK     EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP3R register  ***************
RTC_BKP3R_MASK     EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP4R register  ***************
RTC_BKP4R_MASK     EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP5R register  ***************
RTC_BKP5R_MASK     EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP6R register  ***************
RTC_BKP6R_MASK     EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP7R register  ***************
RTC_BKP7R_MASK     EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP8R register  ***************
RTC_BKP8R_MASK     EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP9R register  ***************
RTC_BKP9R_MASK     EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP10R register  **************
RTC_BKP10R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP11R register  **************
RTC_BKP11R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP12R register  **************
RTC_BKP12R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP13R register  **************
RTC_BKP13R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP14R register  **************
RTC_BKP14R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP15R register  **************
RTC_BKP15R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP16R register  **************
RTC_BKP16R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP17R register  **************
RTC_BKP17R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP18R register  **************
RTC_BKP18R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP19R register  **************
RTC_BKP19R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP20R register  **************
RTC_BKP20R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP21R register  **************
RTC_BKP21R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP22R register  **************
RTC_BKP22R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP23R register  **************
RTC_BKP23R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP24R register  **************
RTC_BKP24R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP25R register  **************
RTC_BKP25R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP26R register  **************
RTC_BKP26R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP27R register  **************
RTC_BKP27R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP28R register  **************
RTC_BKP28R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP29R register  **************
RTC_BKP29R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP30R register  **************
RTC_BKP30R_MASK                   EQU (0xFFFFFFFF)

; ********************  Bits definition for RTC_BKP31R register  **************
RTC_BKP31R_MASK                   EQU (0xFFFFFFFF)

; ******************** Number of backup registers *****************************
RTC_BKP_NUMBER               EQU (0x00000020)

; *****************************************************************************
;
;                          Serial Audio Interface
;
; *****************************************************************************
; ********************  Bit definition for SAI_GCR register  ******************
SAI_GCR_SYNCIN          EQU (0x00000003)        ;SYNCIN[1:0] bits (Synchronization Inputs)
SAI_GCR_SYNCIN_0        EQU (0x00000001)        ;Bit 0
SAI_GCR_SYNCIN_1        EQU (0x00000002)        ;Bit 1

SAI_GCR_SYNCOUT         EQU (0x00000030)        ;SYNCOUT[1:0] bits (Synchronization Outputs)
SAI_GCR_SYNCOUT_0       EQU (0x00000010)        ;Bit 0
SAI_GCR_SYNCOUT_1       EQU (0x00000020)        ;Bit 1

; *******************  Bit definition for SAI_xCR1 register  ******************
SAI_xCR1_MODE            EQU (0x00000003)        ;MODE[1:0] bits (Audio Block Mode)
SAI_xCR1_MODE_0          EQU (0x00000001)        ;Bit 0
SAI_xCR1_MODE_1          EQU (0x00000002)        ;Bit 1

SAI_xCR1_PRTCFG          EQU (0x0000000C)        ;PRTCFG[1:0] bits (Protocol Configuration)
SAI_xCR1_PRTCFG_0        EQU (0x00000004)        ;Bit 0
SAI_xCR1_PRTCFG_1        EQU (0x00000008)        ;Bit 1

SAI_xCR1_DS              EQU (0x000000E0)        ;DS[1:0] bits (Data Size)
SAI_xCR1_DS_0            EQU (0x00000020)        ;Bit 0
SAI_xCR1_DS_1            EQU (0x00000040)        ;Bit 1
SAI_xCR1_DS_2            EQU (0x00000080)        ;Bit 2

SAI_xCR1_LSBFIRST        EQU (0x00000100)        ;LSB First Configuration
SAI_xCR1_CKSTR           EQU (0x00000200)        ;ClocK STRobing edge

SAI_xCR1_SYNCEN          EQU (0x00000C00)        ;SYNCEN[1:0](SYNChronization ENable)
SAI_xCR1_SYNCEN_0        EQU (0x00000400)        ;Bit 0
SAI_xCR1_SYNCEN_1        EQU (0x00000800)        ;Bit 1

SAI_xCR1_MONO            EQU (0x00001000)        ;Mono mode
SAI_xCR1_OUTDRIV         EQU (0x00002000)        ;Output Drive
SAI_xCR1_SAIEN           EQU (0x00010000)        ;Audio Block enable
SAI_xCR1_DMAEN           EQU (0x00020000)        ;DMA enable
SAI_xCR1_NODIV           EQU (0x00080000)        ;No Divider Configuration

SAI_xCR1_MCKDIV          EQU (0x00F00000)        ;MCKDIV[3:0] (Master ClocK Divider)
SAI_xCR1_MCKDIV_0        EQU (0x00100000)        ;Bit 0
SAI_xCR1_MCKDIV_1        EQU (0x00200000)        ;Bit 1
SAI_xCR1_MCKDIV_2        EQU (0x00400000)        ;Bit 2
SAI_xCR1_MCKDIV_3        EQU (0x00800000)        ;Bit 3

; *******************  Bit definition for SAI_xCR2 register  ******************
SAI_xCR2_FTH             EQU (0x00000007)        ;FTH[2:0](Fifo THreshold)
SAI_xCR2_FTH_0           EQU (0x00000001)        ;Bit 0
SAI_xCR2_FTH_1           EQU (0x00000002)        ;Bit 1
SAI_xCR2_FTH_2           EQU (0x00000004)        ;Bit 2

SAI_xCR2_FFLUSH          EQU (0x00000008)        ;Fifo FLUSH
SAI_xCR2_TRIS            EQU (0x00000010)        ;TRIState Management on data line
SAI_xCR2_MUTE            EQU (0x00000020)        ;Mute mode
SAI_xCR2_MUTEVAL         EQU (0x00000040)        ;Muate value


SAI_xCR2_MUTECNT         EQU (0x00001F80)        ;MUTECNT[5:0] (MUTE counter)
SAI_xCR2_MUTECNT_0       EQU (0x00000080)        ;Bit 0
SAI_xCR2_MUTECNT_1       EQU (0x00000100)        ;Bit 1
SAI_xCR2_MUTECNT_2       EQU (0x00000200)        ;Bit 2
SAI_xCR2_MUTECNT_3       EQU (0x00000400)        ;Bit 3
SAI_xCR2_MUTECNT_4       EQU (0x00000800)        ;Bit 4
SAI_xCR2_MUTECNT_5       EQU (0x00001000)        ;Bit 5

SAI_xCR2_CPL             EQU (0x00002000)        ;CPL mode
SAI_xCR2_COMP            EQU (0x0000C000)        ;COMP[1:0] (Companding mode)
SAI_xCR2_COMP_0          EQU (0x00004000)        ;Bit 0
SAI_xCR2_COMP_1          EQU (0x00008000)        ;Bit 1


; ******************  Bit definition for SAI_xFRCR register  ******************
SAI_xFRCR_FRL            EQU (0x000000FF)        ;FRL[7:0](Frame length)
SAI_xFRCR_FRL_0          EQU (0x00000001)        ;Bit 0
SAI_xFRCR_FRL_1          EQU (0x00000002)        ;Bit 1
SAI_xFRCR_FRL_2          EQU (0x00000004)        ;Bit 2
SAI_xFRCR_FRL_3          EQU (0x00000008)        ;Bit 3
SAI_xFRCR_FRL_4          EQU (0x00000010)        ;Bit 4
SAI_xFRCR_FRL_5          EQU (0x00000020)        ;Bit 5
SAI_xFRCR_FRL_6          EQU (0x00000040)        ;Bit 6
SAI_xFRCR_FRL_7          EQU (0x00000080)        ;Bit 7

SAI_xFRCR_FSALL          EQU (0x00007F00)        ;FRL[6:0] (Frame synchronization active level length)
SAI_xFRCR_FSALL_0        EQU (0x00000100)        ;Bit 0
SAI_xFRCR_FSALL_1        EQU (0x00000200)        ;Bit 1
SAI_xFRCR_FSALL_2        EQU (0x00000400)        ;Bit 2
SAI_xFRCR_FSALL_3        EQU (0x00000800)        ;Bit 3
SAI_xFRCR_FSALL_4        EQU (0x00001000)        ;Bit 4
SAI_xFRCR_FSALL_5        EQU (0x00002000)        ;Bit 5
SAI_xFRCR_FSALL_6        EQU (0x00004000)        ;Bit 6

SAI_xFRCR_FSDEF          EQU (0x00010000)        ; Frame Synchronization Definition
SAI_xFRCR_FSPO           EQU (0x00020000)        ;Frame Synchronization POLarity
SAI_xFRCR_FSOFF          EQU (0x00040000)        ;Frame Synchronization OFFset

; ******************  Bit definition for SAI_xSLOTR register  ******************
SAI_xSLOTR_FBOFF         EQU (0x0000001F)        ;FRL[4:0](First Bit Offset)
SAI_xSLOTR_FBOFF_0       EQU (0x00000001)        ;Bit 0
SAI_xSLOTR_FBOFF_1       EQU (0x00000002)        ;Bit 1
SAI_xSLOTR_FBOFF_2       EQU (0x00000004)        ;Bit 2
SAI_xSLOTR_FBOFF_3       EQU (0x00000008)        ;Bit 3
SAI_xSLOTR_FBOFF_4       EQU (0x00000010)        ;Bit 4

SAI_xSLOTR_SLOTSZ        EQU (0x000000C0)        ;SLOTSZ[1:0] (Slot size)
SAI_xSLOTR_SLOTSZ_0      EQU (0x00000040)        ;Bit 0
SAI_xSLOTR_SLOTSZ_1      EQU (0x00000080)        ;Bit 1

SAI_xSLOTR_NBSLOT        EQU (0x00000F00)        ;NBSLOT[3:0] (Number of Slot in audio Frame)
SAI_xSLOTR_NBSLOT_0      EQU (0x00000100)        ;Bit 0
SAI_xSLOTR_NBSLOT_1      EQU (0x00000200)        ;Bit 1
SAI_xSLOTR_NBSLOT_2      EQU (0x00000400)        ;Bit 2
SAI_xSLOTR_NBSLOT_3      EQU (0x00000800)        ;Bit 3

SAI_xSLOTR_SLOTEN        EQU (0xFFFF0000)        ;SLOTEN[15:0] (Slot Enable)

; *******************  Bit definition for SAI_xIMR register  ******************
SAI_xIMR_OVRUDRIE        EQU (0x00000001)        ;Overrun underrun interrupt enable
SAI_xIMR_MUTEDETIE       EQU (0x00000002)        ;Mute detection interrupt enable
SAI_xIMR_WCKCFGIE        EQU (0x00000004)        ;Wrong Clock Configuration interrupt enable
SAI_xIMR_FREQIE          EQU (0x00000008)        ;FIFO request interrupt enable
SAI_xIMR_CNRDYIE         EQU (0x00000010)        ;Codec not ready interrupt enable
SAI_xIMR_AFSDETIE        EQU (0x00000020)        ;Anticipated frame synchronization detection interrupt enable
SAI_xIMR_LFSDETIE        EQU (0x00000040)        ;Late frame synchronization detection interrupt enable

; ********************  Bit definition for SAI_xSR register  ******************
SAI_xSR_OVRUDR           EQU (0x00000001)         ;Overrun underrun
SAI_xSR_MUTEDET          EQU (0x00000002)         ;Mute detection
SAI_xSR_WCKCFG           EQU (0x00000004)         ;Wrong Clock Configuration
SAI_xSR_FREQ             EQU (0x00000008)         ;FIFO request
SAI_xSR_CNRDY            EQU (0x00000010)         ;Codec not ready
SAI_xSR_AFSDET           EQU (0x00000020)         ;Anticipated frame synchronization detection
SAI_xSR_LFSDET           EQU (0x00000040)         ;Late frame synchronization detection

SAI_xSR_FLVL             EQU (0x00070000)         ;FLVL[2:0] (FIFO Level Threshold)
SAI_xSR_FLVL_0           EQU (0x00010000)         ;Bit 0
SAI_xSR_FLVL_1           EQU (0x00020000)         ;Bit 1
SAI_xSR_FLVL_2           EQU (0x00030000)         ;Bit 2

; ******************  Bit definition for SAI_xCLRFR register  *****************
SAI_xCLRFR_COVRUDR       EQU (0x00000001)        ;Clear Overrun underrun
SAI_xCLRFR_CMUTEDET      EQU (0x00000002)        ;Clear Mute detection
SAI_xCLRFR_CWCKCFG       EQU (0x00000004)        ;Clear Wrong Clock Configuration
SAI_xCLRFR_CFREQ         EQU (0x00000008)        ;Clear FIFO request
SAI_xCLRFR_CCNRDY        EQU (0x00000010)        ;Clear Codec not ready
SAI_xCLRFR_CAFSDET       EQU (0x00000020)        ;Clear Anticipated frame synchronization detection
SAI_xCLRFR_CLFSDET       EQU (0x00000040)        ;Clear Late frame synchronization detection

; ******************  Bit definition for SAI_xDR register  *****************
SAI_xDR_DATA             EQU (0xFFFFFFFF)

; *****************************************************************************
;
;                          LCD Controller (LCD)
;
; *****************************************************************************

; *******************  Bit definition for LCD_CR register  ********************
LCD_CR_LCDEN       EQU (0x00000001)     ; LCD Enable Bit
LCD_CR_VSEL        EQU (0x00000002)     ; Voltage source selector Bit

LCD_CR_DUTY        EQU (0x0000001C)     ; DUTY[2:0] bits (Duty selector)
LCD_CR_DUTY_0      EQU (0x00000004)     ; Duty selector Bit 0
LCD_CR_DUTY_1      EQU (0x00000008)     ; Duty selector Bit 1
LCD_CR_DUTY_2      EQU (0x00000010)     ; Duty selector Bit 2

LCD_CR_BIAS        EQU (0x00000060)     ; BIAS[1:0] bits (Bias selector)
LCD_CR_BIAS_0      EQU (0x00000020)     ; Bias selector Bit 0
LCD_CR_BIAS_1      EQU (0x00000040)     ; Bias selector Bit 1

LCD_CR_MUX_SEG     EQU (0x00000080)     ; Mux Segment Enable Bit
LCD_CR_BUFEN       EQU (0x00000100)     ; Voltage output buffer enable

; *******************  Bit definition for LCD_FCR register  *******************
LCD_FCR_HD         EQU (0x00000001)     ; High Drive Enable Bit
LCD_FCR_SOFIE      EQU (0x00000002)     ; Start of Frame Interrupt Enable Bit
LCD_FCR_UDDIE      EQU (0x00000008)     ; Update Display Done Interrupt Enable Bit

LCD_FCR_PON        EQU (0x00000070)     ; PON[2:0] bits (Pulse ON Duration)
LCD_FCR_PON_0      EQU (0x00000010)     ; Bit 0
LCD_FCR_PON_1      EQU (0x00000020)     ; Bit 1
LCD_FCR_PON_2      EQU (0x00000040)     ; Bit 2

LCD_FCR_DEAD       EQU (0x00000380)     ; DEAD[2:0] bits (DEAD Time)
LCD_FCR_DEAD_0     EQU (0x00000080)     ; Bit 0
LCD_FCR_DEAD_1     EQU (0x00000100)     ; Bit 1
LCD_FCR_DEAD_2     EQU (0x00000200)     ; Bit 2

LCD_FCR_CC         EQU (0x00001C00)     ; CC[2:0] bits (Contrast Control)
LCD_FCR_CC_0       EQU (0x00000400)     ; Bit 0
LCD_FCR_CC_1       EQU (0x00000800)     ; Bit 1
LCD_FCR_CC_2       EQU (0x00001000)     ; Bit 2

LCD_FCR_BLINKF     EQU (0x0000E000)     ; BLINKF[2:0] bits (Blink Frequency)
LCD_FCR_BLINKF_0   EQU (0x00002000)     ; Bit 0
LCD_FCR_BLINKF_1   EQU (0x00004000)     ; Bit 1
LCD_FCR_BLINKF_2   EQU (0x00008000)     ; Bit 2

LCD_FCR_BLINK      EQU (0x00030000)     ; BLINK[1:0] bits (Blink Enable)
LCD_FCR_BLINK_0    EQU (0x00010000)     ; Bit 0
LCD_FCR_BLINK_1    EQU (0x00020000)     ; Bit 1

LCD_FCR_DIV        EQU (0x003C0000)     ; DIV[3:0] bits (Divider)
LCD_FCR_PS         EQU (0x03C00000)     ; PS[3:0] bits (Prescaler)

; *******************  Bit definition for LCD_SR register  ********************
LCD_SR_ENS         EQU (0x00000001)     ; LCD Enabled Bit
LCD_SR_SOF         EQU (0x00000002)     ; Start Of Frame Flag Bit
LCD_SR_UDR         EQU (0x00000004)     ; Update Display Request Bit
LCD_SR_UDD         EQU (0x00000008)     ; Update Display Done Flag Bit
LCD_SR_RDY         EQU (0x00000010)     ; Ready Flag Bit
LCD_SR_FCRSR       EQU (0x00000020)     ; LCD FCR Register Synchronization Flag Bit

; *******************  Bit definition for LCD_CLR register  *******************
LCD_CLR_SOFC       EQU (0x00000002)     ; Start Of Frame Flag Clear Bit
LCD_CLR_UDDC       EQU (0x00000008)     ; Update Display Done Flag Clear Bit

; *******************  Bit definition for LCD_RAM register  *******************
LCD_RAM_SEGMENT_DATA       EQU (0xFFFFFFFF)     ; Segment Data Bits

; *****************************************************************************
;
;                           SDMMC Interface
;
; *****************************************************************************
; ******************  Bit definition for SDMMC_POWER register  *****************
SDMMC_POWER_PWRCTRL          EQU (0x03)               ;PWRCTRL[1:0] bits (Power supply control bits)
SDMMC_POWER_PWRCTRL_0        EQU (0x01)               ;Bit 0
SDMMC_POWER_PWRCTRL_1        EQU (0x02)               ;Bit 1

; ******************  Bit definition for SDMMC_CLKCR register  *****************
SDMMC_CLKCR_CLKDIV           EQU (0x00FF)            ;Clock divide factor
SDMMC_CLKCR_CLKEN            EQU (0x0100)            ;Clock enable bit
SDMMC_CLKCR_PWRSAV           EQU (0x0200)            ;Power saving configuration bit
SDMMC_CLKCR_BYPASS           EQU (0x0400)            ;Clock divider bypass enable bit

SDMMC_CLKCR_WIDBUS           EQU (0x1800)            ;WIDBUS[1:0] bits (Wide bus mode enable bit)
SDMMC_CLKCR_WIDBUS_0         EQU (0x0800)            ;Bit 0
SDMMC_CLKCR_WIDBUS_1         EQU (0x1000)            ;Bit 1

SDMMC_CLKCR_NEGEDGE          EQU (0x2000)            ;SDMMC_CK dephasing selection bit
SDMMC_CLKCR_HWFC_EN          EQU (0x4000)            ;HW Flow Control enable

; *******************  Bit definition for SDMMC_ARG register  ******************
SDMMC_ARG_CMDARG             EQU (0xFFFFFFFF)            ;Command argument

; *******************  Bit definition for SDMMC_CMD register  ******************
SDMMC_CMD_CMDINDEX           EQU (0x003F)            ;Command Index

SDMMC_CMD_WAITRESP           EQU (0x00C0)            ;WAITRESP[1:0] bits (Wait for response bits)
SDMMC_CMD_WAITRESP_0         EQU (0x0040)            ; Bit 0
SDMMC_CMD_WAITRESP_1         EQU (0x0080)            ; Bit 1

SDMMC_CMD_WAITINT            EQU (0x0100)            ;CPSM Waits for Interrupt Request
SDMMC_CMD_WAITPEND           EQU (0x0200)            ;CPSM Waits for ends of data transfer (CmdPend internal signal)
SDMMC_CMD_CPSMEN             EQU (0x0400)            ;Command path state machine (CPSM) Enable bit
SDMMC_CMD_SDIOSUSPEND        EQU (0x0800)            ;SD I/O suspend command

; *****************  Bit definition for SDMMC_RESPCMD register  ****************
SDMMC_RESPCMD_RESPCMD        EQU (0x3F)               ;Response command index

; ******************  Bit definition for SDMMC_RESP0 register  *****************
SDMMC_RESP0_CARDSTATUS0      EQU (0xFFFFFFFF)        ;Card Status

; ******************  Bit definition for SDMMC_RESP1 register  *****************
SDMMC_RESP1_CARDSTATUS1      EQU (0xFFFFFFFF)        ;Card Status

; ******************  Bit definition for SDMMC_RESP2 register  *****************
SDMMC_RESP2_CARDSTATUS2      EQU (0xFFFFFFFF)        ;Card Status

; ******************  Bit definition for SDMMC_RESP3 register  *****************
SDMMC_RESP3_CARDSTATUS3      EQU (0xFFFFFFFF)        ;Card Status

; ******************  Bit definition for SDMMC_RESP4 register  *****************
SDMMC_RESP4_CARDSTATUS4      EQU (0xFFFFFFFF)        ;Card Status

; ******************  Bit definition for SDMMC_DTIMER register  ****************
SDMMC_DTIMER_DATATIME        EQU (0xFFFFFFFF)        ;Data timeout period.

; ******************  Bit definition for SDMMC_DLEN register  ******************
SDMMC_DLEN_DATALENGTH        EQU (0x01FFFFFF)        ;Data length value

; ******************  Bit definition for SDMMC_DCTRL register  *****************
SDMMC_DCTRL_DTEN             EQU (0x0001)            ;Data transfer enabled bit
SDMMC_DCTRL_DTDIR            EQU (0x0002)            ;Data transfer direction selection
SDMMC_DCTRL_DTMODE           EQU (0x0004)            ;Data transfer mode selection
SDMMC_DCTRL_DMAEN            EQU (0x0008)            ;DMA enabled bit

SDMMC_DCTRL_DBLOCKSIZE       EQU (0x00F0)            ;DBLOCKSIZE[3:0] bits (Data block size)
SDMMC_DCTRL_DBLOCKSIZE_0     EQU (0x0010)            ;Bit 0
SDMMC_DCTRL_DBLOCKSIZE_1     EQU (0x0020)            ;Bit 1
SDMMC_DCTRL_DBLOCKSIZE_2     EQU (0x0040)            ;Bit 2
SDMMC_DCTRL_DBLOCKSIZE_3     EQU (0x0080)            ;Bit 3

SDMMC_DCTRL_RWSTART          EQU (0x0100)            ;Read wait start
SDMMC_DCTRL_RWSTOP           EQU (0x0200)            ;Read wait stop
SDMMC_DCTRL_RWMOD            EQU (0x0400)            ;Read wait mode
SDMMC_DCTRL_SDIOEN           EQU (0x0800)            ;SD I/O enable functions

; ******************  Bit definition for SDMMC_DCOUNT register  ****************
SDMMC_DCOUNT_DATACOUNT       EQU (0x01FFFFFF)        ;Data count value

; ******************  Bit definition for SDMMC_STA register  *******************
SDMMC_STA_CCRCFAIL           EQU (0x00000001)        ;Command response received (CRC check failed)
SDMMC_STA_DCRCFAIL           EQU (0x00000002)        ;Data block sent/received (CRC check failed)
SDMMC_STA_CTIMEOUT           EQU (0x00000004)        ;Command response timeout
SDMMC_STA_DTIMEOUT           EQU (0x00000008)        ;Data timeout
SDMMC_STA_TXUNDERR           EQU (0x00000010)        ;Transmit FIFO underrun error
SDMMC_STA_RXOVERR            EQU (0x00000020)        ;Received FIFO overrun error
SDMMC_STA_CMDREND            EQU (0x00000040)        ;Command response received (CRC check passed)
SDMMC_STA_CMDSENT            EQU (0x00000080)        ;Command sent (no response required)
SDMMC_STA_DATAEND            EQU (0x00000100)        ;Data end (data counter, SDIDCOUNT, is zero)
SDMMC_STA_STBITERR           EQU (0x00000200)        ;Start bit not detected on all data signals in wide bus mode
SDMMC_STA_DBCKEND            EQU (0x00000400)        ;Data block sent/received (CRC check passed)
SDMMC_STA_CMDACT             EQU (0x00000800)        ;Command transfer in progress
SDMMC_STA_TXACT              EQU (0x00001000)        ;Data transmit in progress
SDMMC_STA_RXACT              EQU (0x00002000)        ;Data receive in progress
SDMMC_STA_TXFIFOHE           EQU (0x00004000)        ;Transmit FIFO Half Empty: at least 8 words can be written into the FIFO
SDMMC_STA_RXFIFOHF           EQU (0x00008000)        ;Receive FIFO Half Full: there are at least 8 words in the FIFO
SDMMC_STA_TXFIFOF            EQU (0x00010000)        ;Transmit FIFO full
SDMMC_STA_RXFIFOF            EQU (0x00020000)        ;Receive FIFO full
SDMMC_STA_TXFIFOE            EQU (0x00040000)        ;Transmit FIFO empty
SDMMC_STA_RXFIFOE            EQU (0x00080000)        ;Receive FIFO empty
SDMMC_STA_TXDAVL             EQU (0x00100000)        ;Data available in transmit FIFO
SDMMC_STA_RXDAVL             EQU (0x00200000)        ;Data available in receive FIFO
SDMMC_STA_SDIOIT             EQU (0x00400000)        ;SDIO interrupt received

; *******************  Bit definition for SDMMC_ICR register  ******************
SDMMC_ICR_CCRCFAILC          EQU (0x00000001)        ;CCRCFAIL flag clear bit
SDMMC_ICR_DCRCFAILC          EQU (0x00000002)        ;DCRCFAIL flag clear bit
SDMMC_ICR_CTIMEOUTC          EQU (0x00000004)        ;CTIMEOUT flag clear bit
SDMMC_ICR_DTIMEOUTC          EQU (0x00000008)        ;DTIMEOUT flag clear bit
SDMMC_ICR_TXUNDERRC          EQU (0x00000010)        ;TXUNDERR flag clear bit
SDMMC_ICR_RXOVERRC           EQU (0x00000020)        ;RXOVERR flag clear bit
SDMMC_ICR_CMDRENDC           EQU (0x00000040)        ;CMDREND flag clear bit
SDMMC_ICR_CMDSENTC           EQU (0x00000080)        ;CMDSENT flag clear bit
SDMMC_ICR_DATAENDC           EQU (0x00000100)        ;DATAEND flag clear bit
SDMMC_ICR_STBITERRC          EQU (0x00000200)        ;STBITERR flag clear bit
SDMMC_ICR_DBCKENDC           EQU (0x00000400)        ;DBCKEND flag clear bit
SDMMC_ICR_SDIOITC            EQU (0x00400000)        ;SDIOIT flag clear bit

; ******************  Bit definition for SDMMC_MASK register  ******************
SDMMC_MASK_CCRCFAILIE        EQU (0x00000001)        ;Command CRC Fail Interrupt Enable
SDMMC_MASK_DCRCFAILIE        EQU (0x00000002)        ;Data CRC Fail Interrupt Enable
SDMMC_MASK_CTIMEOUTIE        EQU (0x00000004)        ;Command TimeOut Interrupt Enable
SDMMC_MASK_DTIMEOUTIE        EQU (0x00000008)        ;Data TimeOut Interrupt Enable
SDMMC_MASK_TXUNDERRIE        EQU (0x00000010)        ;Tx FIFO UnderRun Error Interrupt Enable
SDMMC_MASK_RXOVERRIE         EQU (0x00000020)        ;Rx FIFO OverRun Error Interrupt Enable
SDMMC_MASK_CMDRENDIE         EQU (0x00000040)        ;Command Response Received Interrupt Enable
SDMMC_MASK_CMDSENTIE         EQU (0x00000080)        ;Command Sent Interrupt Enable
SDMMC_MASK_DATAENDIE         EQU (0x00000100)        ;Data End Interrupt Enable
SDMMC_MASK_DBCKENDIE         EQU (0x00000400)        ;Data Block End Interrupt Enable
SDMMC_MASK_CMDACTIE          EQU (0x00000800)        ;CCommand Acting Interrupt Enable
SDMMC_MASK_TXACTIE           EQU (0x00001000)        ;Data Transmit Acting Interrupt Enable
SDMMC_MASK_RXACTIE           EQU (0x00002000)        ;Data receive acting interrupt enabled
SDMMC_MASK_TXFIFOHEIE        EQU (0x00004000)        ;Tx FIFO Half Empty interrupt Enable
SDMMC_MASK_RXFIFOHFIE        EQU (0x00008000)        ;Rx FIFO Half Full interrupt Enable
SDMMC_MASK_TXFIFOFIE         EQU (0x00010000)        ;Tx FIFO Full interrupt Enable
SDMMC_MASK_RXFIFOFIE         EQU (0x00020000)        ;Rx FIFO Full interrupt Enable
SDMMC_MASK_TXFIFOEIE         EQU (0x00040000)        ;Tx FIFO Empty interrupt Enable
SDMMC_MASK_RXFIFOEIE         EQU (0x00080000)        ;Rx FIFO Empty interrupt Enable
SDMMC_MASK_TXDAVLIE          EQU (0x00100000)        ;Data available in Tx FIFO interrupt Enable
SDMMC_MASK_RXDAVLIE          EQU (0x00200000)        ;Data available in Rx FIFO interrupt Enable
SDMMC_MASK_SDIOITIE          EQU (0x00400000)        ;SDIO Mode Interrupt Received interrupt Enable

; *****************  Bit definition for SDMMC_FIFOCNT register  ****************
SDMMC_FIFOCNT_FIFOCOUNT      EQU (0x00FFFFFF)        ;Remaining number of words to be written to or read from the FIFO

; ******************  Bit definition for SDMMC_FIFO register  ******************
SDMMC_FIFO_FIFODATA          EQU (0xFFFFFFFF)        ;Receive and transmit FIFO data

; *****************************************************************************
;
;                        Serial Peripheral Interface (SPI)
;
; *****************************************************************************
; *******************  Bit definition for SPI_CR1 register  *******************
SPI_CR1_CPHA                EQU (0x00000001)            ;Clock Phase
SPI_CR1_CPOL                EQU (0x00000002)            ;Clock Polarity
SPI_CR1_MSTR                EQU (0x00000004)            ;Master Selection

SPI_CR1_BR                  EQU (0x00000038)            ;BR[2:0] bits (Baud Rate Control)
SPI_CR1_BR_0                EQU (0x00000008)            ;Bit 0
SPI_CR1_BR_1                EQU (0x00000010)            ;Bit 1
SPI_CR1_BR_2                EQU (0x00000020)            ;Bit 2

SPI_CR1_SPE                 EQU (0x00000040)            ;SPI Enable
SPI_CR1_LSBFIRST            EQU (0x00000080)            ;Frame Format
SPI_CR1_SSI                 EQU (0x00000100)            ;Internal slave select
SPI_CR1_SSM                 EQU (0x00000200)            ;Software slave management
SPI_CR1_RXONLY              EQU (0x00000400)            ;Receive only
SPI_CR1_CRCL                EQU (0x00000800)            ; CRC Length
SPI_CR1_CRCNEXT             EQU (0x00001000)            ;Transmit CRC next
SPI_CR1_CRCEN               EQU (0x00002000)            ;Hardware CRC calculation enable
SPI_CR1_BIDIOE              EQU (0x00004000)            ;Output enable in bidirectional mode
SPI_CR1_BIDIMODE            EQU (0x00008000)            ;Bidirectional data mode enable

; *******************  Bit definition for SPI_CR2 register  *******************
SPI_CR2_RXDMAEN             EQU (0x00000001)            ; Rx Buffer DMA Enable
SPI_CR2_TXDMAEN             EQU (0x00000002)            ; Tx Buffer DMA Enable
SPI_CR2_SSOE                EQU (0x00000004)            ; SS Output Enable
SPI_CR2_NSSP                EQU (0x00000008)            ; NSS pulse management Enable
SPI_CR2_FRF                 EQU (0x00000010)            ; Frame Format Enable
SPI_CR2_ERRIE               EQU (0x00000020)            ; Error Interrupt Enable
SPI_CR2_RXNEIE              EQU (0x00000040)            ; RX buffer Not Empty Interrupt Enable
SPI_CR2_TXEIE               EQU (0x00000080)            ; Tx buffer Empty Interrupt Enable
SPI_CR2_DS                  EQU (0x00000F00)            ; DS[3:0] Data Size
SPI_CR2_DS_0                EQU (0x00000100)            ; Bit 0
SPI_CR2_DS_1                EQU (0x00000200)            ; Bit 1
SPI_CR2_DS_2                EQU (0x00000400)            ; Bit 2
SPI_CR2_DS_3                EQU (0x00000800)            ; Bit 3
SPI_CR2_FRXTH               EQU (0x00001000)            ; FIFO reception Threshold
SPI_CR2_LDMARX              EQU (0x00002000)            ; Last DMA transfer for reception
SPI_CR2_LDMATX              EQU (0x00004000)            ; Last DMA transfer for transmission

; ********************  Bit definition for SPI_SR register  *******************
SPI_SR_RXNE                 EQU (0x00000001)            ; Receive buffer Not Empty
SPI_SR_TXE                  EQU (0x00000002)            ; Transmit buffer Empty
SPI_SR_CHSIDE               EQU (0x00000004)            ; Channel side
SPI_SR_UDR                  EQU (0x00000008)            ; Underrun flag
SPI_SR_CRCERR               EQU (0x00000010)            ; CRC Error flag
SPI_SR_MODF                 EQU (0x00000020)            ; Mode fault
SPI_SR_OVR                  EQU (0x00000040)            ; Overrun flag
SPI_SR_BSY                  EQU (0x00000080)            ; Busy flag
SPI_SR_FRE                  EQU (0x00000100)            ; TI frame format error
SPI_SR_FRLVL                EQU (0x00000600)            ; FIFO Reception Level
SPI_SR_FRLVL_0              EQU (0x00000200)            ; Bit 0
SPI_SR_FRLVL_1              EQU (0x00000400)            ; Bit 1
SPI_SR_FTLVL                EQU (0x00001800)            ; FIFO Transmission Level
SPI_SR_FTLVL_0              EQU (0x00000800)            ; Bit 0
SPI_SR_FTLVL_1              EQU (0x00001000)            ; Bit 1

; ********************  Bit definition for SPI_DR register  *******************
SPI_DR_DR                   EQU (0x0000FFFF)            ;Data Register

; *******************  Bit definition for SPI_CRCPR register  *****************
SPI_CRCPR_CRCPOLY           EQU (0x0000FFFF)            ;CRC polynomial register

; ******************  Bit definition for SPI_RXCRCR register  *****************
SPI_RXCRCR_RXCRC            EQU (0x0000FFFF)            ;Rx CRC Register

; ******************  Bit definition for SPI_TXCRCR register  *****************
SPI_TXCRCR_TXCRC            EQU (0x0000FFFF)            ;Tx CRC Register

; *****************************************************************************
;
;                                    QUADSPI
;
; *****************************************************************************
; *****************  Bit definition for QUADSPI_CR register  ******************
QUADSPI_CR_EN                   EQU (0x00000001)            ; Enable
QUADSPI_CR_ABORT                EQU (0x00000002)            ; Abort request
QUADSPI_CR_DMAEN                EQU (0x00000004)            ; DMA Enable
QUADSPI_CR_TCEN                 EQU (0x00000008)            ; Timeout Counter Enable
QUADSPI_CR_SSHIFT               EQU (0x00000010)            ; Sample Shift
QUADSPI_CR_FTHRES               EQU (0x00000F00)            ; FTHRES[3:0] FIFO Level
QUADSPI_CR_TEIE                 EQU (0x00010000)            ; Transfer Error Interrupt Enable
QUADSPI_CR_TCIE                 EQU (0x00020000)            ; Transfer Complete Interrupt Enable
QUADSPI_CR_FTIE                 EQU (0x00040000)            ; FIFO Threshold Interrupt Enable
QUADSPI_CR_SMIE                 EQU (0x00080000)            ; Status Match Interrupt Enable
QUADSPI_CR_TOIE                 EQU (0x00100000)            ; TimeOut Interrupt Enable
QUADSPI_CR_APMS                 EQU (0x00400000)            ; Automatic Polling Mode Stop
QUADSPI_CR_PMM                  EQU (0x00800000)            ; Polling Match Mode
QUADSPI_CR_PRESCALER            EQU (0xFF000000)            ; PRESCALER[7:0] Clock prescaler

; *****************  Bit definition for QUADSPI_DCR register  *****************
QUADSPI_DCR_CKMODE              EQU (0x00000001)            ; Mode 0 / Mode 3
QUADSPI_DCR_CSHT                EQU (0x00000700)            ; CSHT[2:0]: ChipSelect High Time
QUADSPI_DCR_CSHT_0              EQU (0x00000100)            ; Bit 0
QUADSPI_DCR_CSHT_1              EQU (0x00000200)            ; Bit 1
QUADSPI_DCR_CSHT_2              EQU (0x00000400)            ; Bit 2
QUADSPI_DCR_FSIZE               EQU (0x001F0000)            ; FSIZE[4:0]: Flash Size

; ******************  Bit definition for QUADSPI_SR register  ******************
QUADSPI_SR_TEF                  EQU (0x00000001)             ; Transfer Error Flag
QUADSPI_SR_TCF                  EQU (0x00000002)             ; Transfer Complete Flag
QUADSPI_SR_FTF                  EQU (0x00000004)             ; FIFO Threshlod Flag
QUADSPI_SR_SMF                  EQU (0x00000008)             ; Status Match Flag
QUADSPI_SR_TOF                  EQU (0x00000010)             ; Timeout Flag
QUADSPI_SR_BUSY                 EQU (0x00000020)             ; Busy
QUADSPI_SR_FLEVEL               EQU (0x00001F00)             ; FIFO Threshlod Flag

; ******************  Bit definition for QUADSPI_FCR register  *****************
QUADSPI_FCR_CTEF                EQU (0x00000001)             ; Clear Transfer Error Flag
QUADSPI_FCR_CTCF                EQU (0x00000002)             ; Clear Transfer Complete Flag
QUADSPI_FCR_CSMF                EQU (0x00000008)             ; Clear Status Match Flag
QUADSPI_FCR_CTOF                EQU (0x00000010)             ; Clear Timeout Flag

; ******************  Bit definition for QUADSPI_DLR register  *****************
QUADSPI_DLR_DL                EQU (0xFFFFFFFF)               ; DL[31:0]: Data Length

; ******************  Bit definition for QUADSPI_CCR register  *****************
QUADSPI_CCR_INSTRUCTION          EQU (0x000000FF)            ; INSTRUCTION[7:0]: Instruction
QUADSPI_CCR_IMODE                EQU (0x00000300)            ; IMODE[1:0]: Instruction Mode
QUADSPI_CCR_IMODE_0              EQU (0x00000100)            ; Bit 0
QUADSPI_CCR_IMODE_1              EQU (0x00000200)            ; Bit 1
QUADSPI_CCR_ADMODE               EQU (0x00000C00)            ; ADMODE[1:0]: Address Mode
QUADSPI_CCR_ADMODE_0             EQU (0x00000400)            ; Bit 0
QUADSPI_CCR_ADMODE_1             EQU (0x00000800)            ; Bit 1
QUADSPI_CCR_ADSIZE               EQU (0x00003000)            ; ADSIZE[1:0]: Address Size
QUADSPI_CCR_ADSIZE_0             EQU (0x00001000)            ; Bit 0
QUADSPI_CCR_ADSIZE_1             EQU (0x00002000)            ; Bit 1
QUADSPI_CCR_ABMODE               EQU (0x0000C000)            ; ABMODE[1:0]: Alternate Bytes Mode
QUADSPI_CCR_ABMODE_0             EQU (0x00004000)            ; Bit 0
QUADSPI_CCR_ABMODE_1             EQU (0x00008000)            ; Bit 1
QUADSPI_CCR_ABSIZE               EQU (0x00030000)            ; ABSIZE[1:0]: Instruction Mode
QUADSPI_CCR_ABSIZE_0             EQU (0x00010000)            ; Bit 0
QUADSPI_CCR_ABSIZE_1             EQU (0x00020000)            ; Bit 1
QUADSPI_CCR_DCYC                 EQU (0x007C0000)            ; DCYC[4:0]: Dummy Cycles
QUADSPI_CCR_DMODE                EQU (0x03000000)            ; DMODE[1:0]: Data Mode
QUADSPI_CCR_DMODE_0              EQU (0x01000000)            ; Bit 0
QUADSPI_CCR_DMODE_1              EQU (0x02000000)            ; Bit 1
QUADSPI_CCR_FMODE                EQU (0x0C000000)            ; FMODE[1:0]: Functional Mode
QUADSPI_CCR_FMODE_0              EQU (0x04000000)            ; Bit 0
QUADSPI_CCR_FMODE_1              EQU (0x08000000)            ; Bit 1
QUADSPI_CCR_SIOO                 EQU (0x10000000)            ; SIOO: Send Instruction Only Once Mode
QUADSPI_CCR_DDRM                 EQU (0x80000000)            ; DDRM: Double Data Rate Mode

; ******************  Bit definition for QUADSPI_AR register  ******************
QUADSPI_AR_ADDRESS               EQU (0xFFFFFFFF)            ; ADDRESS[31:0]: Address

; ******************  Bit definition for QUADSPI_ABR register  *****************
QUADSPI_ABR_ALTERNATE            EQU (0xFFFFFFFF)            ; ALTERNATE[31:0]: Alternate Bytes

; ******************  Bit definition for QUADSPI_DR register  ******************
QUADSPI_DR_DATA                  EQU (0xFFFFFFFF)            ; DATA[31:0]: Data

; ******************  Bit definition for QUADSPI_PSMKR register  ***************
QUADSPI_PSMKR_MASK               EQU (0xFFFFFFFF)            ; MASK[31:0]: Status Mask

; ******************  Bit definition for QUADSPI_PSMAR register  ***************
QUADSPI_PSMAR_MATCH              EQU (0xFFFFFFFF)            ; MATCH[31:0]: Status Match

; ******************  Bit definition for QUADSPI_PIR register  ****************
QUADSPI_PIR_INTERVAL             EQU (0x0000FFFF)            ; INTERVAL[15:0]: Polling Interval

; ******************  Bit definition for QUADSPI_LPTR register  ****************
QUADSPI_LPTR_TIMEOUT             EQU (0x0000FFFF)            ; TIMEOUT[15:0]: Timeout period

; *****************************************************************************
;
;                                 SYSCFG
;
; *****************************************************************************
; ******************  Bit definition for SYSCFG_MEMRMP register  **************
SYSCFG_MEMRMP_MEM_MODE  EQU (0x00000007) ; SYSCFG_Memory Remap Config
SYSCFG_MEMRMP_MEM_MODE_0        EQU (0x00000001)
SYSCFG_MEMRMP_MEM_MODE_1        EQU (0x00000002)
SYSCFG_MEMRMP_MEM_MODE_2        EQU (0x00000004)

SYSCFG_MEMRMP_FB_MODE   EQU (0x00000100) ; Flash Bank mode selection


; ******************  Bit definition for SYSCFG_CFGR1 register  *****************
SYSCFG_CFGR1_FWDIS          EQU (0x00000001) ; FIREWALL access enable
SYSCFG_CFGR1_BOOSTEN        EQU (0x00000100) ; I/O analog switch voltage booster enable
SYSCFG_CFGR1_I2C_PB6_FMP    EQU (0x00010000) ; I2C PB6 Fast mode plus
SYSCFG_CFGR1_I2C_PB7_FMP    EQU (0x00020000) ; I2C PB7 Fast mode plus
SYSCFG_CFGR1_I2C_PB8_FMP    EQU (0x00040000) ; I2C PB8 Fast mode plus
SYSCFG_CFGR1_I2C_PB9_FMP    EQU (0x00080000) ; I2C PB9 Fast mode plus
SYSCFG_CFGR1_I2C1_FMP       EQU (0x00100000) ; I2C1 Fast mode plus
SYSCFG_CFGR1_I2C2_FMP       EQU (0x00200000) ; I2C2 Fast mode plus
SYSCFG_CFGR1_I2C3_FMP       EQU (0x00400000) ; I2C3 Fast mode plus
SYSCFG_CFGR1_FPU_IE_0       EQU (0x04000000) ;  Invalid operation Interrupt enable
SYSCFG_CFGR1_FPU_IE_1       EQU (0x08000000) ;  Divide-by-zero Interrupt enable
SYSCFG_CFGR1_FPU_IE_2       EQU (0x10000000) ;  Underflow Interrupt enable
SYSCFG_CFGR1_FPU_IE_3       EQU (0x20000000) ;  Overflow Interrupt enable
SYSCFG_CFGR1_FPU_IE_4       EQU (0x40000000) ;  Input denormal Interrupt enable
SYSCFG_CFGR1_FPU_IE_5       EQU (0x80000000) ;  Inexact Interrupt enable (interrupt disabled at reset)

; *****************  Bit definition for SYSCFG_EXTICR1 register  **************
SYSCFG_EXTICR1_EXTI0    EQU (0x00000007) ;EXTI 0 configuration
SYSCFG_EXTICR1_EXTI1    EQU (0x00000070) ;EXTI 1 configuration
SYSCFG_EXTICR1_EXTI2    EQU (0x00000700) ;EXTI 2 configuration
SYSCFG_EXTICR1_EXTI3    EQU (0x00007000) ;EXTI 3 configuration
;   EXTI0 configuration

SYSCFG_EXTICR1_EXTI0_PA EQU (0x00000000) ;PA[0] pin
SYSCFG_EXTICR1_EXTI0_PB EQU (0x00000001) ;PB[0] pin
SYSCFG_EXTICR1_EXTI0_PC EQU (0x00000002) ;PC[0] pin
SYSCFG_EXTICR1_EXTI0_PD EQU (0x00000003) ;PD[0] pin
SYSCFG_EXTICR1_EXTI0_PE EQU (0x00000004) ;PE[0] pin
SYSCFG_EXTICR1_EXTI0_PF EQU (0x00000005) ;PF[0] pin
SYSCFG_EXTICR1_EXTI0_PG EQU (0x00000006) ;PG[0] pin
SYSCFG_EXTICR1_EXTI0_PH EQU (0x00000007) ;PH[0] pin


;   EXTI1 configuration

SYSCFG_EXTICR1_EXTI1_PA EQU (0x00000000) ;PA[1] pin
SYSCFG_EXTICR1_EXTI1_PB EQU (0x00000010) ;PB[1] pin
SYSCFG_EXTICR1_EXTI1_PC EQU (0x00000020) ;PC[1] pin
SYSCFG_EXTICR1_EXTI1_PD EQU (0x00000030) ;PD[1] pin
SYSCFG_EXTICR1_EXTI1_PE EQU (0x00000040) ;PE[1] pin
SYSCFG_EXTICR1_EXTI1_PF EQU (0x00000050) ;PF[1] pin
SYSCFG_EXTICR1_EXTI1_PG EQU (0x00000060) ;PG[1] pin
SYSCFG_EXTICR1_EXTI1_PH EQU (0x00000070) ;PH[1] pin

;   EXTI2 configuration

SYSCFG_EXTICR1_EXTI2_PA EQU (0x00000000) ;PA[2] pin
SYSCFG_EXTICR1_EXTI2_PB EQU (0x00000100) ;PB[2] pin
SYSCFG_EXTICR1_EXTI2_PC EQU (0x00000200) ;PC[2] pin
SYSCFG_EXTICR1_EXTI2_PD EQU (0x00000300) ;PD[2] pin
SYSCFG_EXTICR1_EXTI2_PE EQU (0x00000400) ;PE[2] pin
SYSCFG_EXTICR1_EXTI2_PF EQU (0x00000500) ;PF[2] pin
SYSCFG_EXTICR1_EXTI2_PG EQU (0x00000600) ;PG[2] pin


;   EXTI3 configuration

SYSCFG_EXTICR1_EXTI3_PA EQU (0x00000000) ;PA[3] pin
SYSCFG_EXTICR1_EXTI3_PB EQU (0x00001000) ;PB[3] pin
SYSCFG_EXTICR1_EXTI3_PC EQU (0x00002000) ;PC[3] pin
SYSCFG_EXTICR1_EXTI3_PD EQU (0x00003000) ;PD[3] pin
SYSCFG_EXTICR1_EXTI3_PE EQU (0x00004000) ;PE[3] pin
SYSCFG_EXTICR1_EXTI3_PF EQU (0x00005000) ;PF[3] pin
SYSCFG_EXTICR1_EXTI3_PG EQU (0x00006000) ;PG[3] pin


; *****************  Bit definition for SYSCFG_EXTICR2 register  **************
SYSCFG_EXTICR2_EXTI4    EQU (0x00000007) ;EXTI 4 configuration
SYSCFG_EXTICR2_EXTI5    EQU (0x00000070) ;EXTI 5 configuration
SYSCFG_EXTICR2_EXTI6    EQU (0x00000700) ;EXTI 6 configuration
SYSCFG_EXTICR2_EXTI7    EQU (0x00007000) ;EXTI 7 configuration
;   EXTI4 configuration

SYSCFG_EXTICR2_EXTI4_PA EQU (0x00000000) ;PA[4] pin
SYSCFG_EXTICR2_EXTI4_PB EQU (0x00000001) ;PB[4] pin
SYSCFG_EXTICR2_EXTI4_PC EQU (0x00000002) ;PC[4] pin
SYSCFG_EXTICR2_EXTI4_PD EQU (0x00000003) ;PD[4] pin
SYSCFG_EXTICR2_EXTI4_PE EQU (0x00000004) ;PE[4] pin
SYSCFG_EXTICR2_EXTI4_PF EQU (0x00000005) ;PF[4] pin
SYSCFG_EXTICR2_EXTI4_PG EQU (0x00000006) ;PG[4] pin

;   EXTI5 configuration

SYSCFG_EXTICR2_EXTI5_PA EQU (0x00000000) ;PA[5] pin
SYSCFG_EXTICR2_EXTI5_PB EQU (0x00000010) ;PB[5] pin
SYSCFG_EXTICR2_EXTI5_PC EQU (0x00000020) ;PC[5] pin
SYSCFG_EXTICR2_EXTI5_PD EQU (0x00000030) ;PD[5] pin
SYSCFG_EXTICR2_EXTI5_PE EQU (0x00000040) ;PE[5] pin
SYSCFG_EXTICR2_EXTI5_PF EQU (0x00000050) ;PF[5] pin
SYSCFG_EXTICR2_EXTI5_PG EQU (0x00000060) ;PG[5] pin

;   EXTI6 configuration

SYSCFG_EXTICR2_EXTI6_PA EQU (0x00000000) ;PA[6] pin
SYSCFG_EXTICR2_EXTI6_PB EQU (0x00000100) ;PB[6] pin
SYSCFG_EXTICR2_EXTI6_PC EQU (0x00000200) ;PC[6] pin
SYSCFG_EXTICR2_EXTI6_PD EQU (0x00000300) ;PD[6] pin
SYSCFG_EXTICR2_EXTI6_PE EQU (0x00000400) ;PE[6] pin
SYSCFG_EXTICR2_EXTI6_PF EQU (0x00000500) ;PF[6] pin
SYSCFG_EXTICR2_EXTI6_PG EQU (0x00000600) ;PG[6] pin

;   EXTI7 configuration

SYSCFG_EXTICR2_EXTI7_PA EQU (0x00000000) ;PA[7] pin
SYSCFG_EXTICR2_EXTI7_PB EQU (0x00001000) ;PB[7] pin
SYSCFG_EXTICR2_EXTI7_PC EQU (0x00002000) ;PC[7] pin
SYSCFG_EXTICR2_EXTI7_PD EQU (0x00003000) ;PD[7] pin
SYSCFG_EXTICR2_EXTI7_PE EQU (0x00004000) ;PE[7] pin
SYSCFG_EXTICR2_EXTI7_PF EQU (0x00005000) ;PF[7] pin
SYSCFG_EXTICR2_EXTI7_PG EQU (0x00006000) ;PG[7] pin


; *****************  Bit definition for SYSCFG_EXTICR3 register  **************
SYSCFG_EXTICR3_EXTI8    EQU (0x00000007) ;EXTI 8 configuration
SYSCFG_EXTICR3_EXTI9    EQU (0x00000070) ;EXTI 9 configuration
SYSCFG_EXTICR3_EXTI10   EQU (0x00000700) ;EXTI 10 configuration
SYSCFG_EXTICR3_EXTI11   EQU (0x00007000) ;EXTI 11 configuration

;   EXTI8 configuration

SYSCFG_EXTICR3_EXTI8_PA EQU (0x00000000) ;PA[8] pin
SYSCFG_EXTICR3_EXTI8_PB EQU (0x00000001) ;PB[8] pin
SYSCFG_EXTICR3_EXTI8_PC EQU (0x00000002) ;PC[8] pin
SYSCFG_EXTICR3_EXTI8_PD EQU (0x00000003) ;PD[8] pin
SYSCFG_EXTICR3_EXTI8_PE EQU (0x00000004) ;PE[8] pin
SYSCFG_EXTICR3_EXTI8_PF EQU (0x00000005) ;PF[8] pin
SYSCFG_EXTICR3_EXTI8_PG EQU (0x00000006) ;PG[8] pin

;   EXTI9 configuration

SYSCFG_EXTICR3_EXTI9_PA EQU (0x00000000) ;PA[9] pin
SYSCFG_EXTICR3_EXTI9_PB EQU (0x00000010) ;PB[9] pin
SYSCFG_EXTICR3_EXTI9_PC EQU (0x00000020) ;PC[9] pin
SYSCFG_EXTICR3_EXTI9_PD EQU (0x00000030) ;PD[9] pin
SYSCFG_EXTICR3_EXTI9_PE EQU (0x00000040) ;PE[9] pin
SYSCFG_EXTICR3_EXTI9_PF EQU (0x00000050) ;PF[9] pin
SYSCFG_EXTICR3_EXTI9_PG EQU (0x00000060) ;PG[9] pin

;   EXTI10 configuration

SYSCFG_EXTICR3_EXTI10_PA        EQU (0x00000000) ;PA[10] pin
SYSCFG_EXTICR3_EXTI10_PB        EQU (0x00000100) ;PB[10] pin
SYSCFG_EXTICR3_EXTI10_PC        EQU (0x00000200) ;PC[10] pin
SYSCFG_EXTICR3_EXTI10_PD        EQU (0x00000300) ;PD[10] pin
SYSCFG_EXTICR3_EXTI10_PE        EQU (0x00000400) ;PE[10] pin
SYSCFG_EXTICR3_EXTI10_PF        EQU (0x00000500) ;PF[10] pin
SYSCFG_EXTICR3_EXTI10_PG        EQU (0x00000600) ;PG[10] pin

;   EXTI11 configuration

SYSCFG_EXTICR3_EXTI11_PA        EQU (0x00000000) ;PA[11] pin
SYSCFG_EXTICR3_EXTI11_PB        EQU (0x00001000) ;PB[11] pin
SYSCFG_EXTICR3_EXTI11_PC        EQU (0x00002000) ;PC[11] pin
SYSCFG_EXTICR3_EXTI11_PD        EQU (0x00003000) ;PD[11] pin
SYSCFG_EXTICR3_EXTI11_PE        EQU (0x00004000) ;PE[11] pin
SYSCFG_EXTICR3_EXTI11_PF        EQU (0x00005000) ;PF[11] pin
SYSCFG_EXTICR3_EXTI11_PG        EQU (0x00006000) ;PG[11] pin

; *****************  Bit definition for SYSCFG_EXTICR4 register  **************
SYSCFG_EXTICR4_EXTI12   EQU (0x00000007) ;EXTI 12 configuration
SYSCFG_EXTICR4_EXTI13   EQU (0x00000070) ;EXTI 13 configuration
SYSCFG_EXTICR4_EXTI14   EQU (0x00000700) ;EXTI 14 configuration
SYSCFG_EXTICR4_EXTI15   EQU (0x00007000) ;EXTI 15 configuration
;   EXTI12 configuration

SYSCFG_EXTICR4_EXTI12_PA        EQU (0x00000000) ;PA[12] pin
SYSCFG_EXTICR4_EXTI12_PB        EQU (0x00000001) ;PB[12] pin
SYSCFG_EXTICR4_EXTI12_PC        EQU (0x00000002) ;PC[12] pin
SYSCFG_EXTICR4_EXTI12_PD        EQU (0x00000003) ;PD[12] pin
SYSCFG_EXTICR4_EXTI12_PE        EQU (0x00000004) ;PE[12] pin
SYSCFG_EXTICR4_EXTI12_PF        EQU (0x00000005) ;PF[12] pin
SYSCFG_EXTICR4_EXTI12_PG        EQU (0x00000006) ;PG[12] pin

;   EXTI13 configuration

SYSCFG_EXTICR4_EXTI13_PA        EQU (0x00000000) ;PA[13] pin
SYSCFG_EXTICR4_EXTI13_PB        EQU (0x00000010) ;PB[13] pin
SYSCFG_EXTICR4_EXTI13_PC        EQU (0x00000020) ;PC[13] pin
SYSCFG_EXTICR4_EXTI13_PD        EQU (0x00000030) ;PD[13] pin
SYSCFG_EXTICR4_EXTI13_PE        EQU (0x00000040) ;PE[13] pin
SYSCFG_EXTICR4_EXTI13_PF        EQU (0x00000050) ;PF[13] pin
SYSCFG_EXTICR4_EXTI13_PG        EQU (0x00000060) ;PG[13] pin

;   EXTI14 configuration

SYSCFG_EXTICR4_EXTI14_PA        EQU (0x00000000) ;PA[14] pin
SYSCFG_EXTICR4_EXTI14_PB        EQU (0x00000100) ;PB[14] pin
SYSCFG_EXTICR4_EXTI14_PC        EQU (0x00000200) ;PC[14] pin
SYSCFG_EXTICR4_EXTI14_PD        EQU (0x00000300) ;PD[14] pin
SYSCFG_EXTICR4_EXTI14_PE        EQU (0x00000400) ;PE[14] pin
SYSCFG_EXTICR4_EXTI14_PF        EQU (0x00000500) ;PF[14] pin
SYSCFG_EXTICR4_EXTI14_PG        EQU (0x00000600) ;PG[14] pin

;   EXTI15 configuration

SYSCFG_EXTICR4_EXTI15_PA        EQU (0x00000000) ;PA[15] pin
SYSCFG_EXTICR4_EXTI15_PB        EQU (0x00001000) ;PB[15] pin
SYSCFG_EXTICR4_EXTI15_PC        EQU (0x00002000) ;PC[15] pin
SYSCFG_EXTICR4_EXTI15_PD        EQU (0x00003000) ;PD[15] pin
SYSCFG_EXTICR4_EXTI15_PE        EQU (0x00004000) ;PE[15] pin
SYSCFG_EXTICR4_EXTI15_PF        EQU (0x00005000) ;PF[15] pin
SYSCFG_EXTICR4_EXTI15_PG        EQU (0x00006000) ;PG[15] pin

; ******************  Bit definition for SYSCFG_SCSR register  ***************
SYSCFG_SCSR_SRAM2ER     EQU (0x00000001) ; SRAM2 Erase Request
SYSCFG_SCSR_SRAM2BSY    EQU (0x00000002) ; SRAM2 Erase Ongoing

; ******************  Bit definition for SYSCFG_CFGR2 register  ***************
SYSCFG_CFGR2_CLL        EQU (0x00000001) ; Core Lockup Lock
SYSCFG_CFGR2_SPL        EQU (0x00000002) ; SRAM Parity Lock
SYSCFG_CFGR2_PVDL       EQU (0x00000004) ;  PVD Lock
SYSCFG_CFGR2_ECCL       EQU (0x00000008) ; ECC Lock
SYSCFG_CFGR2_SPF        EQU (0x00000100) ; SRAM Parity Flag

; ******************  Bit definition for SYSCFG_SWPR register  ***************
SYSCFG_SWPR_PAGE0  EQU (0x00000001) ; SRAM2 Write protection page 0
SYSCFG_SWPR_PAGE1  EQU (0x00000002) ; SRAM2 Write protection page 1
SYSCFG_SWPR_PAGE2  EQU (0x00000004) ; SRAM2 Write protection page 2
SYSCFG_SWPR_PAGE3  EQU (0x00000008) ; SRAM2 Write protection page 3
SYSCFG_SWPR_PAGE4  EQU (0x00000010) ; SRAM2 Write protection page 4
SYSCFG_SWPR_PAGE5  EQU (0x00000020) ; SRAM2 Write protection page 5
SYSCFG_SWPR_PAGE6  EQU (0x00000040) ; SRAM2 Write protection page 6
SYSCFG_SWPR_PAGE7  EQU (0x00000080) ; SRAM2 Write protection page 7
SYSCFG_SWPR_PAGE8  EQU (0x00000100) ; SRAM2 Write protection page 8
SYSCFG_SWPR_PAGE9  EQU (0x00000200) ; SRAM2 Write protection page 9
SYSCFG_SWPR_PAGE10 EQU (0x00000400) ; SRAM2 Write protection page 10
SYSCFG_SWPR_PAGE11 EQU (0x00000800) ; SRAM2 Write protection page 11
SYSCFG_SWPR_PAGE12 EQU (0x00001000) ; SRAM2 Write protection page 12
SYSCFG_SWPR_PAGE13 EQU (0x00002000) ; SRAM2 Write protection page 13
SYSCFG_SWPR_PAGE14 EQU (0x00004000) ; SRAM2 Write protection page 14
SYSCFG_SWPR_PAGE15 EQU (0x00008000) ; SRAM2 Write protection page 15
SYSCFG_SWPR_PAGE16 EQU (0x00010000) ; SRAM2 Write protection page 16
SYSCFG_SWPR_PAGE17 EQU (0x00020000) ; SRAM2 Write protection page 17
SYSCFG_SWPR_PAGE18 EQU (0x00040000) ; SRAM2 Write protection page 18
SYSCFG_SWPR_PAGE19 EQU (0x00080000) ; SRAM2 Write protection page 19
SYSCFG_SWPR_PAGE20 EQU (0x00100000) ; SRAM2 Write protection page 20
SYSCFG_SWPR_PAGE21 EQU (0x00200000) ; SRAM2 Write protection page 21
SYSCFG_SWPR_PAGE22 EQU (0x00400000) ; SRAM2 Write protection page 22
SYSCFG_SWPR_PAGE23 EQU (0x00800000) ; SRAM2 Write protection page 23
SYSCFG_SWPR_PAGE24 EQU (0x01000000) ; SRAM2 Write protection page 24
SYSCFG_SWPR_PAGE25 EQU (0x02000000) ; SRAM2 Write protection page 25
SYSCFG_SWPR_PAGE26 EQU (0x04000000) ; SRAM2 Write protection page 26
SYSCFG_SWPR_PAGE27 EQU (0x08000000) ; SRAM2 Write protection page 27
SYSCFG_SWPR_PAGE28 EQU (0x10000000) ; SRAM2 Write protection page 28
SYSCFG_SWPR_PAGE29 EQU (0x20000000) ; SRAM2 Write protection page 29
SYSCFG_SWPR_PAGE30 EQU (0x40000000) ; SRAM2 Write protection page 30
SYSCFG_SWPR_PAGE31 EQU (0x80000000) ; SRAM2 Write protection page 31

; ******************  Bit definition for SYSCFG_SKR register  ***************
SYSCFG_SKR_KEY     EQU (0x000000FF) ;  SRAM2 write protection key for software erase




; *****************************************************************************
;
;                                    TIM
;
; *****************************************************************************
; *******************  Bit definition for TIM_CR1 register  *******************
TIM_CR1_CEN       EQU (0x00000001)            ;Counter enable
TIM_CR1_UDIS      EQU (0x00000002)            ;Update disable
TIM_CR1_URS       EQU (0x00000004)            ;Update request source
TIM_CR1_OPM       EQU (0x00000008)            ;One pulse mode
TIM_CR1_DIR       EQU (0x00000010)            ;Direction

TIM_CR1_CMS       EQU (0x00000060)            ;CMS[1:0] bits (Center-aligned mode selection)
TIM_CR1_CMS_0     EQU (0x00000020)            ;Bit 0
TIM_CR1_CMS_1     EQU (0x00000040)            ;Bit 1

TIM_CR1_ARPE      EQU (0x00000080)            ;Auto-reload preload enable

TIM_CR1_CKD       EQU (0x00000300)            ;CKD[1:0] bits (clock division)
TIM_CR1_CKD_0     EQU (0x00000100)            ;Bit 0
TIM_CR1_CKD_1     EQU (0x00000200)            ;Bit 1

TIM_CR1_UIFREMAP  EQU (0x00000800)            ;Update interrupt flag remap

; *******************  Bit definition for TIM_CR2 register  *******************
TIM_CR2_CCPC      EQU (0x00000001)            ;Capture/Compare Preloaded Control
TIM_CR2_CCUS      EQU (0x00000004)            ;Capture/Compare Control Update Selection
TIM_CR2_CCDS      EQU (0x00000008)            ;Capture/Compare DMA Selection

TIM_CR2_MMS       EQU (0x00000070)            ;MMS[2:0] bits (Master Mode Selection)
TIM_CR2_MMS_0     EQU (0x00000010)            ;Bit 0
TIM_CR2_MMS_1     EQU (0x00000020)            ;Bit 1
TIM_CR2_MMS_2     EQU (0x00000040)            ;Bit 2

TIM_CR2_TI1S      EQU (0x00000080)            ;TI1 Selection
TIM_CR2_OIS1      EQU (0x00000100)            ;Output Idle state 1 (OC1 output)
TIM_CR2_OIS1N     EQU (0x00000200)            ;Output Idle state 1 (OC1N output)
TIM_CR2_OIS2      EQU (0x00000400)            ;Output Idle state 2 (OC2 output)
TIM_CR2_OIS2N     EQU (0x00000800)            ;Output Idle state 2 (OC2N output)
TIM_CR2_OIS3      EQU (0x00001000)            ;Output Idle state 3 (OC3 output)
TIM_CR2_OIS3N     EQU (0x00002000)            ;Output Idle state 3 (OC3N output)
TIM_CR2_OIS4      EQU (0x00004000)            ;Output Idle state 4 (OC4 output)
TIM_CR2_OIS5      EQU (0x00010000)            ;Output Idle state 5 (OC5 output)
TIM_CR2_OIS6      EQU (0x00040000)            ;Output Idle state 6 (OC6 output)

TIM_CR2_MMS2      EQU (0x00F00000)            ;MMS[2:0] bits (Master Mode Selection)
TIM_CR2_MMS2_0    EQU (0x00100000)            ;Bit 0
TIM_CR2_MMS2_1    EQU (0x00200000)            ;Bit 1
TIM_CR2_MMS2_2    EQU (0x00400000)            ;Bit 2
TIM_CR2_MMS2_3    EQU (0x00800000)            ;Bit 2

; *******************  Bit definition for TIM_SMCR register  ******************
TIM_SMCR_SMS      EQU (0x00010007)            ;SMS[2:0] bits (Slave mode selection)
TIM_SMCR_SMS_0    EQU (0x00000001)            ;Bit 0
TIM_SMCR_SMS_1    EQU (0x00000002)            ;Bit 1
TIM_SMCR_SMS_2    EQU (0x00000004)            ;Bit 2
TIM_SMCR_SMS_3    EQU (0x00010000)            ;Bit 3

TIM_SMCR_OCCS     EQU (0x00000008)            ; OCREF clear selection

TIM_SMCR_TS       EQU (0x00000070)            ;TS[2:0] bits (Trigger selection)
TIM_SMCR_TS_0     EQU (0x00000010)            ;Bit 0
TIM_SMCR_TS_1     EQU (0x00000020)            ;Bit 1
TIM_SMCR_TS_2     EQU (0x00000040)            ;Bit 2

TIM_SMCR_MSM      EQU (0x00000080)            ;Master/slave mode

TIM_SMCR_ETF      EQU (0x00000F00)            ;ETF[3:0] bits (External trigger filter)
TIM_SMCR_ETF_0    EQU (0x00000100)            ;Bit 0
TIM_SMCR_ETF_1    EQU (0x00000200)            ;Bit 1
TIM_SMCR_ETF_2    EQU (0x00000400)            ;Bit 2
TIM_SMCR_ETF_3    EQU (0x00000800)            ;Bit 3

TIM_SMCR_ETPS     EQU (0x00003000)            ;ETPS[1:0] bits (External trigger prescaler)
TIM_SMCR_ETPS_0   EQU (0x00001000)            ;Bit 0
TIM_SMCR_ETPS_1   EQU (0x00002000)            ;Bit 1

TIM_SMCR_ECE      EQU (0x00004000)            ;External clock enable
TIM_SMCR_ETP      EQU (0x00008000)            ;External trigger polarity

; *******************  Bit definition for TIM_DIER register  ******************
TIM_DIER_UIE      EQU (0x00000001)            ;Update interrupt enable
TIM_DIER_CC1IE    EQU (0x00000002)            ;Capture/Compare 1 interrupt enable
TIM_DIER_CC2IE    EQU (0x00000004)            ;Capture/Compare 2 interrupt enable
TIM_DIER_CC3IE    EQU (0x00000008)            ;Capture/Compare 3 interrupt enable
TIM_DIER_CC4IE    EQU (0x00000010)            ;Capture/Compare 4 interrupt enable
TIM_DIER_COMIE    EQU (0x00000020)            ;COM interrupt enable
TIM_DIER_TIE      EQU (0x00000040)            ;Trigger interrupt enable
TIM_DIER_BIE      EQU (0x00000080)            ;Break interrupt enable
TIM_DIER_UDE      EQU (0x00000100)            ;Update DMA request enable
TIM_DIER_CC1DE    EQU (0x00000200)            ;Capture/Compare 1 DMA request enable
TIM_DIER_CC2DE    EQU (0x00000400)            ;Capture/Compare 2 DMA request enable
TIM_DIER_CC3DE    EQU (0x00000800)            ;Capture/Compare 3 DMA request enable
TIM_DIER_CC4DE    EQU (0x00001000)            ;Capture/Compare 4 DMA request enable
TIM_DIER_COMDE    EQU (0x00002000)            ;COM DMA request enable
TIM_DIER_TDE      EQU (0x00004000)            ;Trigger DMA request enable

; ********************  Bit definition for TIM_SR register  *******************
TIM_SR_UIF        EQU (0x00000001)            ;Update interrupt Flag
TIM_SR_CC1IF      EQU (0x00000002)            ;Capture/Compare 1 interrupt Flag
TIM_SR_CC2IF      EQU (0x00000004)            ;Capture/Compare 2 interrupt Flag
TIM_SR_CC3IF      EQU (0x00000008)            ;Capture/Compare 3 interrupt Flag
TIM_SR_CC4IF      EQU (0x00000010)            ;Capture/Compare 4 interrupt Flag
TIM_SR_COMIF      EQU (0x00000020)            ;COM interrupt Flag
TIM_SR_TIF        EQU (0x00000040)            ;Trigger interrupt Flag
TIM_SR_BIF        EQU (0x00000080)            ;Break interrupt Flag
TIM_SR_B2IF       EQU (0x00000100)            ;Break 2 interrupt Flag
TIM_SR_CC1OF      EQU (0x00000200)            ;Capture/Compare 1 Overcapture Flag
TIM_SR_CC2OF      EQU (0x00000400)            ;Capture/Compare 2 Overcapture Flag
TIM_SR_CC3OF      EQU (0x00000800)            ;Capture/Compare 3 Overcapture Flag
TIM_SR_CC4OF      EQU (0x00001000)            ;Capture/Compare 4 Overcapture Flag
TIM_SR_SBIF       EQU (0x00002000)            ;System Break interrupt Flag
TIM_SR_CC5IF      EQU (0x00010000)            ;Capture/Compare 5 interrupt Flag
TIM_SR_CC6IF      EQU (0x00020000)            ;Capture/Compare 6 interrupt Flag


; *******************  Bit definition for TIM_EGR register  *******************
TIM_EGR_UG        EQU (0x00000001)            ;Update Generation
TIM_EGR_CC1G      EQU (0x00000002)            ;Capture/Compare 1 Generation
TIM_EGR_CC2G      EQU (0x00000004)            ;Capture/Compare 2 Generation
TIM_EGR_CC3G      EQU (0x00000008)            ;Capture/Compare 3 Generation
TIM_EGR_CC4G      EQU (0x00000010)            ;Capture/Compare 4 Generation
TIM_EGR_COMG      EQU (0x00000020)            ;Capture/Compare Control Update Generation
TIM_EGR_TG        EQU (0x00000040)            ;Trigger Generation
TIM_EGR_BG        EQU (0x00000080)            ;Break Generation
TIM_EGR_B2G       EQU (0x00000100)            ;Break 2 Generation


; ******************  Bit definition for TIM_CCMR1 register  ******************
TIM_CCMR1_CC1S    EQU (0x00000003)            ;CC1S[1:0] bits (Capture/Compare 1 Selection)
TIM_CCMR1_CC1S_0  EQU (0x00000001)            ;Bit 0
TIM_CCMR1_CC1S_1  EQU (0x00000002)            ;Bit 1

TIM_CCMR1_OC1FE   EQU (0x00000004)            ;Output Compare 1 Fast enable
TIM_CCMR1_OC1PE   EQU (0x00000008)            ;Output Compare 1 Preload enable

TIM_CCMR1_OC1M    EQU (0x00010070)            ;OC1M[2:0] bits (Output Compare 1 Mode)
TIM_CCMR1_OC1M_0  EQU (0x00000010)            ;Bit 0
TIM_CCMR1_OC1M_1  EQU (0x00000020)            ;Bit 1
TIM_CCMR1_OC1M_2  EQU (0x00000040)            ;Bit 2
TIM_CCMR1_OC1M_3  EQU (0x00010000)            ;Bit 3

TIM_CCMR1_OC1CE   EQU (0x00000080)            ;Output Compare 1 Clear Enable

TIM_CCMR1_CC2S    EQU (0x00000300)            ;CC2S[1:0] bits (Capture/Compare 2 Selection)
TIM_CCMR1_CC2S_0  EQU (0x00000100)            ;Bit 0
TIM_CCMR1_CC2S_1  EQU (0x00000200)            ;Bit 1

TIM_CCMR1_OC2FE   EQU (0x00000400)            ;Output Compare 2 Fast enable
TIM_CCMR1_OC2PE   EQU (0x00000800)            ;Output Compare 2 Preload enable

TIM_CCMR1_OC2M    EQU (0x01007000)            ;OC2M[2:0] bits (Output Compare 2 Mode)
TIM_CCMR1_OC2M_0  EQU (0x00001000)            ;Bit 0
TIM_CCMR1_OC2M_1  EQU (0x00002000)            ;Bit 1
TIM_CCMR1_OC2M_2  EQU (0x00004000)            ;Bit 2
TIM_CCMR1_OC2M_3  EQU (0x01000000)            ;Bit 3

TIM_CCMR1_OC2CE   EQU (0x00008000)            ;Output Compare 2 Clear Enable

;----------------------------------------------------------------------------
TIM_CCMR1_IC1PSC  EQU (0x0000000C)            ;IC1PSC[1:0] bits (Input Capture 1 Prescaler)
TIM_CCMR1_IC1PSC_0        EQU (0x00000004)            ;Bit 0
TIM_CCMR1_IC1PSC_1        EQU (0x00000008)            ;Bit 1

TIM_CCMR1_IC1F    EQU (0x000000F0)            ;IC1F[3:0] bits (Input Capture 1 Filter)
TIM_CCMR1_IC1F_0  EQU (0x00000010)            ;Bit 0
TIM_CCMR1_IC1F_1  EQU (0x00000020)            ;Bit 1
TIM_CCMR1_IC1F_2  EQU (0x00000040)            ;Bit 2
TIM_CCMR1_IC1F_3  EQU (0x00000080)            ;Bit 3

TIM_CCMR1_IC2PSC  EQU (0x00000C00)            ;IC2PSC[1:0] bits (Input Capture 2 Prescaler)
TIM_CCMR1_IC2PSC_0        EQU (0x00000400)            ;Bit 0
TIM_CCMR1_IC2PSC_1        EQU (0x00000800)            ;Bit 1

TIM_CCMR1_IC2F    EQU (0x0000F000)            ;IC2F[3:0] bits (Input Capture 2 Filter)
TIM_CCMR1_IC2F_0  EQU (0x00001000)            ;Bit 0
TIM_CCMR1_IC2F_1  EQU (0x00002000)            ;Bit 1
TIM_CCMR1_IC2F_2  EQU (0x00004000)            ;Bit 2
TIM_CCMR1_IC2F_3  EQU (0x00008000)            ;Bit 3

; ******************  Bit definition for TIM_CCMR2 register  ******************
TIM_CCMR2_CC3S    EQU (0x00000003)            ;CC3S[1:0] bits (Capture/Compare 3 Selection)
TIM_CCMR2_CC3S_0  EQU (0x00000001)            ;Bit 0
TIM_CCMR2_CC3S_1  EQU (0x00000002)            ;Bit 1

TIM_CCMR2_OC3FE   EQU (0x00000004)            ;Output Compare 3 Fast enable
TIM_CCMR2_OC3PE   EQU (0x00000008)            ;Output Compare 3 Preload enable

TIM_CCMR2_OC3M    EQU (0x00010070)            ;OC3M[2:0] bits (Output Compare 3 Mode)
TIM_CCMR2_OC3M_0  EQU (0x00000010)            ;Bit 0
TIM_CCMR2_OC3M_1  EQU (0x00000020)            ;Bit 1
TIM_CCMR2_OC3M_2  EQU (0x00000040)            ;Bit 2
TIM_CCMR2_OC3M_3  EQU (0x00010000)            ;Bit 3

TIM_CCMR2_OC3CE   EQU (0x00000080)            ;Output Compare 3 Clear Enable

TIM_CCMR2_CC4S    EQU (0x00000300)            ;CC4S[1:0] bits (Capture/Compare 4 Selection)
TIM_CCMR2_CC4S_0  EQU (0x00000100)            ;Bit 0
TIM_CCMR2_CC4S_1  EQU (0x00000200)            ;Bit 1

TIM_CCMR2_OC4FE   EQU (0x00000400)            ;Output Compare 4 Fast enable
TIM_CCMR2_OC4PE   EQU (0x00000800)            ;Output Compare 4 Preload enable

TIM_CCMR2_OC4M    EQU (0x01007000)            ;OC4M[2:0] bits (Output Compare 4 Mode)
TIM_CCMR2_OC4M_0  EQU (0x00001000)            ;Bit 0
TIM_CCMR2_OC4M_1  EQU (0x00002000)            ;Bit 1
TIM_CCMR2_OC4M_2  EQU (0x00004000)            ;Bit 2
TIM_CCMR2_OC4M_3  EQU (0x01000000)            ;Bit 3

TIM_CCMR2_OC4CE   EQU (0x00008000)            ;Output Compare 4 Clear Enable

;----------------------------------------------------------------------------
TIM_CCMR2_IC3PSC  EQU (0x0000000C)            ;IC3PSC[1:0] bits (Input Capture 3 Prescaler)
TIM_CCMR2_IC3PSC_0        EQU (0x00000004)            ;Bit 0
TIM_CCMR2_IC3PSC_1        EQU (0x00000008)            ;Bit 1

TIM_CCMR2_IC3F    EQU (0x000000F0)            ;IC3F[3:0] bits (Input Capture 3 Filter)
TIM_CCMR2_IC3F_0  EQU (0x00000010)            ;Bit 0
TIM_CCMR2_IC3F_1  EQU (0x00000020)            ;Bit 1
TIM_CCMR2_IC3F_2  EQU (0x00000040)            ;Bit 2
TIM_CCMR2_IC3F_3  EQU (0x00000080)            ;Bit 3

TIM_CCMR2_IC4PSC  EQU (0x00000C00)            ;IC4PSC[1:0] bits (Input Capture 4 Prescaler)
TIM_CCMR2_IC4PSC_0        EQU (0x00000400)            ;Bit 0
TIM_CCMR2_IC4PSC_1        EQU (0x00000800)            ;Bit 1

TIM_CCMR2_IC4F    EQU (0x0000F000)            ;IC4F[3:0] bits (Input Capture 4 Filter)
TIM_CCMR2_IC4F_0  EQU (0x00001000)            ;Bit 0
TIM_CCMR2_IC4F_1  EQU (0x00002000)            ;Bit 1
TIM_CCMR2_IC4F_2  EQU (0x00004000)            ;Bit 2
TIM_CCMR2_IC4F_3  EQU (0x00008000)            ;Bit 3

; ******************  Bit definition for TIM_CCMR3 register  ******************
TIM_CCMR3_OC5FE   EQU (0x00000004)            ;Output Compare 5 Fast enable
TIM_CCMR3_OC5PE   EQU (0x00000008)            ;Output Compare 5 Preload enable

TIM_CCMR3_OC5M    EQU (0x00010070)            ;OC5M[3:0] bits (Output Compare 5 Mode)
TIM_CCMR3_OC5M_0  EQU (0x00000010)            ;Bit 0
TIM_CCMR3_OC5M_1  EQU (0x00000020)            ;Bit 1
TIM_CCMR3_OC5M_2  EQU (0x00000040)            ;Bit 2
TIM_CCMR3_OC5M_3  EQU (0x00010000)            ;Bit 3

TIM_CCMR3_OC5CE   EQU (0x00000080)            ;Output Compare 5 Clear Enable

TIM_CCMR3_OC6FE   EQU (0x00000400)            ;Output Compare 6 Fast enable
TIM_CCMR3_OC6PE   EQU (0x00000800)            ;Output Compare 6 Preload enable

TIM_CCMR3_OC6M    EQU (0x01007000)            ;OC6M[3:0] bits (Output Compare 6 Mode)
TIM_CCMR3_OC6M_0  EQU (0x00001000)            ;Bit 0
TIM_CCMR3_OC6M_1  EQU (0x00002000)            ;Bit 1
TIM_CCMR3_OC6M_2  EQU (0x00004000)            ;Bit 2
TIM_CCMR3_OC6M_3  EQU (0x01000000)            ;Bit 3

TIM_CCMR3_OC6CE   EQU (0x00008000)            ;Output Compare 6 Clear Enable

; *******************  Bit definition for TIM_CCER register  ******************
TIM_CCER_CC1E     EQU (0x00000001)            ;Capture/Compare 1 output enable
TIM_CCER_CC1P     EQU (0x00000002)            ;Capture/Compare 1 output Polarity
TIM_CCER_CC1NE    EQU (0x00000004)            ;Capture/Compare 1 Complementary output enable
TIM_CCER_CC1NP    EQU (0x00000008)            ;Capture/Compare 1 Complementary output Polarity
TIM_CCER_CC2E     EQU (0x00000010)            ;Capture/Compare 2 output enable
TIM_CCER_CC2P     EQU (0x00000020)            ;Capture/Compare 2 output Polarity
TIM_CCER_CC2NE    EQU (0x00000040)            ;Capture/Compare 2 Complementary output enable
TIM_CCER_CC2NP    EQU (0x00000080)            ;Capture/Compare 2 Complementary output Polarity
TIM_CCER_CC3E     EQU (0x00000100)            ;Capture/Compare 3 output enable
TIM_CCER_CC3P     EQU (0x00000200)            ;Capture/Compare 3 output Polarity
TIM_CCER_CC3NE    EQU (0x00000400)            ;Capture/Compare 3 Complementary output enable
TIM_CCER_CC3NP    EQU (0x00000800)            ;Capture/Compare 3 Complementary output Polarity
TIM_CCER_CC4E     EQU (0x00001000)            ;Capture/Compare 4 output enable
TIM_CCER_CC4P     EQU (0x00002000)            ;Capture/Compare 4 output Polarity
TIM_CCER_CC4NP    EQU (0x00008000)            ;Capture/Compare 4 Complementary output Polarity
TIM_CCER_CC5E     EQU (0x00010000)            ;Capture/Compare 5 output enable
TIM_CCER_CC5P     EQU (0x00020000)            ;Capture/Compare 5 output Polarity
TIM_CCER_CC6E     EQU (0x00100000)            ;Capture/Compare 6 output enable
TIM_CCER_CC6P     EQU (0x00200000)            ;Capture/Compare 6 output Polarity

; *******************  Bit definition for TIM_CNT register  *******************
TIM_CNT_CNT       EQU (0xFFFFFFFF)            ;Counter Value
TIM_CNT_UIFCPY    EQU (0x80000000)            ;Update interrupt flag copy (if UIFREMAPEQU 1)

; *******************  Bit definition for TIM_PSC register  *******************
TIM_PSC_PSC       EQU (0x0000FFFF)            ;Prescaler Value

; *******************  Bit definition for TIM_ARR register  *******************
TIM_ARR_ARR       EQU (0xFFFFFFFF)            ;Actual auto-reload Value

; *******************  Bit definition for TIM_RCR register  *******************
TIM_RCR_REP       EQU (0x0000FFFF)            ;Repetition Counter Value

; *******************  Bit definition for TIM_CCR1 register  ******************
TIM_CCR1_CCR1     EQU (0x0000FFFF)            ;Capture/Compare 1 Value

; *******************  Bit definition for TIM_CCR2 register  ******************
TIM_CCR2_CCR2     EQU (0x0000FFFF)            ;Capture/Compare 2 Value

; *******************  Bit definition for TIM_CCR3 register  ******************
TIM_CCR3_CCR3     EQU (0x0000FFFF)            ;Capture/Compare 3 Value

; *******************  Bit definition for TIM_CCR4 register  ******************
TIM_CCR4_CCR4     EQU (0x0000FFFF)            ;Capture/Compare 4 Value

; *******************  Bit definition for TIM_CCR5 register  ******************
TIM_CCR5_CCR5     EQU (0xFFFFFFFF)            ;Capture/Compare 5 Value
TIM_CCR5_GC5C1    EQU (0x20000000)            ;Group Channel 5 and Channel 1
TIM_CCR5_GC5C2    EQU (0x40000000)            ;Group Channel 5 and Channel 2
TIM_CCR5_GC5C3    EQU (0x80000000)            ;Group Channel 5 and Channel 3

; *******************  Bit definition for TIM_CCR6 register  ******************
TIM_CCR6_CCR6     EQU (0x0000FFFF)            ;Capture/Compare 6 Value

; *******************  Bit definition for TIM_BDTR register  ******************
TIM_BDTR_DTG      EQU (0x000000FF)            ;DTG[0:7] bits (Dead-Time Generator set-up)
TIM_BDTR_DTG_0    EQU (0x00000001)            ;Bit 0
TIM_BDTR_DTG_1    EQU (0x00000002)            ;Bit 1
TIM_BDTR_DTG_2    EQU (0x00000004)            ;Bit 2
TIM_BDTR_DTG_3    EQU (0x00000008)            ;Bit 3
TIM_BDTR_DTG_4    EQU (0x00000010)            ;Bit 4
TIM_BDTR_DTG_5    EQU (0x00000020)            ;Bit 5
TIM_BDTR_DTG_6    EQU (0x00000040)            ;Bit 6
TIM_BDTR_DTG_7    EQU (0x00000080)            ;Bit 7

TIM_BDTR_LOCK     EQU (0x00000300)            ;LOCK[1:0] bits (Lock Configuration)
TIM_BDTR_LOCK_0   EQU (0x00000100)            ;Bit 0
TIM_BDTR_LOCK_1   EQU (0x00000200)            ;Bit 1

TIM_BDTR_OSSI     EQU (0x00000400)            ;Off-State Selection for Idle mode
TIM_BDTR_OSSR     EQU (0x00000800)            ;Off-State Selection for Run mode
TIM_BDTR_BKE      EQU (0x00001000)            ;Break enable for Break 1
TIM_BDTR_BKP      EQU (0x00002000)            ;Break Polarity for Break 1
TIM_BDTR_AOE      EQU (0x00004000)            ;Automatic Output enable
TIM_BDTR_MOE      EQU (0x00008000)            ;Main Output enable

TIM_BDTR_BKF      EQU (0x000F0000)            ;Break Filter for Break 1
TIM_BDTR_BK2F     EQU (0x00F00000)            ;Break Filter for Break 2

TIM_BDTR_BK2E     EQU (0x01000000)            ;Break enable for Break 2
TIM_BDTR_BK2P     EQU (0x02000000)            ;Break Polarity for Break 2

; *******************  Bit definition for TIM_DCR register  *******************
TIM_DCR_DBA       EQU (0x0000001F)            ;DBA[4:0] bits (DMA Base Address)
TIM_DCR_DBA_0     EQU (0x00000001)            ;Bit 0
TIM_DCR_DBA_1     EQU (0x00000002)            ;Bit 1
TIM_DCR_DBA_2     EQU (0x00000004)            ;Bit 2
TIM_DCR_DBA_3     EQU (0x00000008)            ;Bit 3
TIM_DCR_DBA_4     EQU (0x00000010)            ;Bit 4

TIM_DCR_DBL       EQU (0x00001F00)            ;DBL[4:0] bits (DMA Burst Length)
TIM_DCR_DBL_0     EQU (0x00000100)            ;Bit 0
TIM_DCR_DBL_1     EQU (0x00000200)            ;Bit 1
TIM_DCR_DBL_2     EQU (0x00000400)            ;Bit 2
TIM_DCR_DBL_3     EQU (0x00000800)            ;Bit 3
TIM_DCR_DBL_4     EQU (0x00001000)            ;Bit 4

; *******************  Bit definition for TIM_DMAR register  ******************
TIM_DMAR_DMAB     EQU (0x0000FFFF)            ;DMA register for burst accesses

; *******************  Bit definition for TIM1_OR1 register  ******************
TIM1_OR1_ETR_ADC1_RMP      EQU (0x00000003)            ;ETR_ADC1_RMP[1:0] bits (TIM1 ETR remap on ADC1)
TIM1_OR1_ETR_ADC1_RMP_0    EQU (0x00000001)            ;Bit 0
TIM1_OR1_ETR_ADC1_RMP_1    EQU (0x00000002)            ;Bit 1

TIM1_OR1_ETR_ADC3_RMP      EQU (0x0000000C)            ;ETR_ADC3_RMP[1:0] bits (TIM1 ETR remap on ADC3)
TIM1_OR1_ETR_ADC3_RMP_0    EQU (0x00000004)            ;Bit 0
TIM1_OR1_ETR_ADC3_RMP_1    EQU (0x00000008)            ;Bit 1

TIM1_OR1_TI1_RMP   EQU (0x00000010)            ;TIM1 Input Capture 1 remap

; *******************  Bit definition for TIM1_OR2 register  ******************
TIM1_OR2_BKINE     EQU (0x00000001)            ;BRK BKIN input enable
TIM1_OR2_BKCMP1E   EQU (0x00000002)            ;BRK COMP1 enable
TIM1_OR2_BKCMP2E   EQU (0x00000004)            ;BRK COMP2 enable
TIM1_OR2_BKDFBK0E  EQU (0x00000100)            ;BRK DFSDM_BREAK[0] enable
TIM1_OR2_BKINP     EQU (0x00000200)            ;BRK BKIN input polarity
TIM1_OR2_BKCMP1P   EQU (0x00000400)            ;BRK COMP1 input polarity
TIM1_OR2_BKCMP2P   EQU (0x00000800)            ;BRK COMP2 input polarity

TIM1_OR2_ETRSEL    EQU (0x0001C000)            ;ETRSEL[2:0] bits (TIM1 ETR source selection)
TIM1_OR2_ETRSEL_0  EQU (0x00004000)            ;Bit 0
TIM1_OR2_ETRSEL_1  EQU (0x00008000)            ;Bit 1
TIM1_OR2_ETRSEL_2  EQU (0x00010000)            ;Bit 2

; *******************  Bit definition for TIM1_OR3 register  ******************
TIM1_OR3_BK2INE    EQU (0x00000001)            ;BRK2 BKIN2 input enable
TIM1_OR3_BK2CMP1E  EQU (0x00000002)            ;BRK2 COMP1 enable
TIM1_OR3_BK2CMP2E  EQU (0x00000004)            ;BRK2 COMP2 enable
TIM1_OR3_BK2DFBK1E EQU (0x00000100)            ;BRK2 DFSDM_BREAK[1] enable
TIM1_OR3_BK2INP    EQU (0x00000200)            ;BRK2 BKIN2 input polarity
TIM1_OR3_BK2CMP1P  EQU (0x00000400)            ;BRK2 COMP1 input polarity
TIM1_OR3_BK2CMP2P  EQU (0x00000800)            ;BRK2 COMP2 input polarity

; *******************  Bit definition for TIM8_OR1 register  ******************
TIM8_OR1_ETR_ADC2_RMP      EQU (0x00000003)            ;ETR_ADC2_RMP[1:0] bits (TIM8 ETR remap on ADC2)
TIM8_OR1_ETR_ADC2_RMP_0    EQU (0x00000001)            ;Bit 0
TIM8_OR1_ETR_ADC2_RMP_1    EQU (0x00000002)            ;Bit 1

TIM8_OR1_ETR_ADC3_RMP      EQU (0x0000000C)            ;ETR_ADC3_RMP[1:0] bits (TIM8 ETR remap on ADC3)
TIM8_OR1_ETR_ADC3_RMP_0    EQU (0x00000004)            ;Bit 0
TIM8_OR1_ETR_ADC3_RMP_1    EQU (0x00000008)            ;Bit 1

TIM8_OR1_TI1_RMP   EQU (0x00000010)            ;TIM8 Input Capture 1 remap

; *******************  Bit definition for TIM8_OR2 register  ******************
TIM8_OR2_BKINE     EQU (0x00000001)            ;BRK BKIN input enable
TIM8_OR2_BKCMP1E   EQU (0x00000002)            ;BRK COMP1 enable
TIM8_OR2_BKCMP2E   EQU (0x00000004)            ;BRK COMP2 enable
TIM8_OR2_BKDFBK2E  EQU (0x00000100)            ;BRK DFSDM_BREAK[2] enable
TIM8_OR2_BKINP     EQU (0x00000200)            ;BRK BKIN input polarity
TIM8_OR2_BKCMP1P   EQU (0x00000400)            ;BRK COMP1 input polarity
TIM8_OR2_BKCMP2P   EQU (0x00000800)            ;BRK COMP2 input polarity

TIM8_OR2_ETRSEL    EQU (0x0001C000)            ;ETRSEL[2:0] bits (TIM8 ETR source selection)
TIM8_OR2_ETRSEL_0  EQU (0x00004000)            ;Bit 0
TIM8_OR2_ETRSEL_1  EQU (0x00008000)            ;Bit 1
TIM8_OR2_ETRSEL_2  EQU (0x00010000)            ;Bit 2

; *******************  Bit definition for TIM8_OR3 register  ******************
TIM8_OR3_BK2INE    EQU (0x00000001)            ;BRK2 BKIN2 input enable
TIM8_OR3_BK2CMP1E  EQU (0x00000002)            ;BRK2 COMP1 enable
TIM8_OR3_BK2CMP2E  EQU (0x00000004)            ;BRK2 COMP2 enable
TIM8_OR3_BK2DFBK3E EQU (0x00000100)            ;BRK2 DFSDM_BREAK[3] enable
TIM8_OR3_BK2INP    EQU (0x00000200)            ;BRK2 BKIN2 input polarity
TIM8_OR3_BK2CMP1P  EQU (0x00000400)            ;BRK2 COMP1 input polarity
TIM8_OR3_BK2CMP2P  EQU (0x00000800)            ;BRK2 COMP2 input polarity

; *******************  Bit definition for TIM2_OR1 register  ******************
TIM2_OR1_ITR1_RMP  EQU (0x00000001)            ;TIM2 Internal trigger 1 remap
TIM2_OR1_ETR1_RMP  EQU (0x00000002)            ;TIM2 External trigger 1 remap

TIM2_OR1_TI4_RMP   EQU (0x0000000C)            ;TI4_RMP[1:0] bits (TIM2 Input Capture 4 remap)
TIM2_OR1_TI4_RMP_0 EQU (0x00000004)            ;Bit 0
TIM2_OR1_TI4_RMP_1 EQU (0x00000008)            ;Bit 1

; *******************  Bit definition for TIM2_OR2 register  ******************
TIM2_OR2_ETRSEL    EQU (0x0001C000)            ;ETRSEL[2:0] bits (TIM2 ETR source selection)
TIM2_OR2_ETRSEL_0  EQU (0x00004000)            ;Bit 0
TIM2_OR2_ETRSEL_1  EQU (0x00008000)            ;Bit 1
TIM2_OR2_ETRSEL_2  EQU (0x00010000)            ;Bit 2

; *******************  Bit definition for TIM3_OR1 register  ******************
TIM3_OR1_TI1_RMP   EQU (0x00000003)            ;TI1_RMP[1:0] bits (TIM3 Input Capture 1 remap)
TIM3_OR1_TI1_RMP_0 EQU (0x00000001)            ;Bit 0
TIM3_OR1_TI1_RMP_1 EQU (0x00000002)            ;Bit 1

; *******************  Bit definition for TIM3_OR2 register  ******************
TIM3_OR2_ETRSEL    EQU (0x0001C000)            ;ETRSEL[2:0] bits (TIM3 ETR source selection)
TIM3_OR2_ETRSEL_0  EQU (0x00004000)            ;Bit 0
TIM3_OR2_ETRSEL_1  EQU (0x00008000)            ;Bit 1
TIM3_OR2_ETRSEL_2  EQU (0x00010000)            ;Bit 2

; *******************  Bit definition for TIM15_OR1 register  *****************
TIM15_OR1_TI1_RMP  EQU (0x00000001)            ;TIM15 Input Capture 1 remap

TIM15_OR1_ENCODER_MODE     EQU (0x00000006)            ;ENCODER_MODE[1:0] bits (TIM15 Encoder mode)
TIM15_OR1_ENCODER_MODE_0   EQU (0x00000002)            ;Bit 0
TIM15_OR1_ENCODER_MODE_1   EQU (0x00000004)            ;Bit 1

; *******************  Bit definition for TIM15_OR2 register  *****************
TIM15_OR2_BKINE    EQU (0x00000001)            ;BRK BKIN input enable
TIM15_OR2_BKCMP1E  EQU (0x00000002)            ;BRK COMP1 enable
TIM15_OR2_BKCMP2E  EQU (0x00000004)            ;BRK COMP2 enable
TIM15_OR2_BKDFBK0E EQU (0x00000100)            ;BRK DFSDM_BREAK[0] enable
TIM15_OR2_BKINP    EQU (0x00000200)            ;BRK BKIN input polarity
TIM15_OR2_BKCMP1P  EQU (0x00000400)            ;BRK COMP1 input polarity
TIM15_OR2_BKCMP2P  EQU (0x00000800)            ;BRK COMP2 input polarity

; *******************  Bit definition for TIM16_OR1 register  *****************
TIM16_OR1_TI1_RMP  EQU (0x00000003)            ;TI1_RMP[1:0] bits (TIM16 Input Capture 1 remap)
TIM16_OR1_TI1_RMP_0        EQU (0x00000001)            ;Bit 0
TIM16_OR1_TI1_RMP_1        EQU (0x00000002)            ;Bit 1

; *******************  Bit definition for TIM16_OR2 register  *****************
TIM16_OR2_BKINE    EQU (0x00000001)            ;BRK BKIN input enable
TIM16_OR2_BKCMP1E  EQU (0x00000002)            ;BRK COMP1 enable
TIM16_OR2_BKCMP2E  EQU (0x00000004)            ;BRK COMP2 enable
TIM16_OR2_BKDFBK1E EQU (0x00000100)            ;BRK DFSDM_BREAK[1] enable
TIM16_OR2_BKINP    EQU (0x00000200)            ;BRK BKIN input polarity
TIM16_OR2_BKCMP1P  EQU (0x00000400)            ;BRK COMP1 input polarity
TIM16_OR2_BKCMP2P  EQU (0x00000800)            ;BRK COMP2 input polarity

; *******************  Bit definition for TIM17_OR1 register  *****************
TIM17_OR1_TI1_RMP  EQU (0x00000003)            ;TI1_RMP[1:0] bits (TIM17 Input Capture 1 remap)
TIM17_OR1_TI1_RMP_0        EQU (0x00000001)            ;Bit 0
TIM17_OR1_TI1_RMP_1        EQU (0x00000002)            ;Bit 1

; *******************  Bit definition for TIM17_OR2 register  *****************
TIM17_OR2_BKINE    EQU (0x00000001)            ;BRK BKIN input enable
TIM17_OR2_BKCMP1E  EQU (0x00000002)            ;BRK COMP1 enable
TIM17_OR2_BKCMP2E  EQU (0x00000004)            ;BRK COMP2 enable
TIM17_OR2_BKDFBK2E EQU (0x00000100)            ;BRK DFSDM_BREAK[2] enable
TIM17_OR2_BKINP    EQU (0x00000200)            ;BRK BKIN input polarity
TIM17_OR2_BKCMP1P  EQU (0x00000400)            ;BRK COMP1 input polarity
TIM17_OR2_BKCMP2P  EQU (0x00000800)            ;BRK COMP2 input polarity

; *****************************************************************************
;
;                         Low Power Timer (LPTTIM)
;
; *****************************************************************************
; ******************  Bit definition for LPTIM_ISR register  ******************
LPTIM_ISR_CMPM                 EQU (0x00000001)            ; Compare match
LPTIM_ISR_ARRM                 EQU (0x00000002)            ; Autoreload match
LPTIM_ISR_EXTTRIG              EQU (0x00000004)            ; External trigger edge event
LPTIM_ISR_CMPOK                EQU (0x00000008)            ; Compare register update OK
LPTIM_ISR_ARROK                EQU (0x00000010)            ; Autoreload register update OK
LPTIM_ISR_UP                   EQU (0x00000020)            ; Counter direction change down to up
LPTIM_ISR_DOWN                 EQU (0x00000040)            ; Counter direction change up to down

; ******************  Bit definition for LPTIM_ICR register  ******************
LPTIM_ICR_CMPMCF               EQU (0x00000001)            ; Compare match Clear Flag
LPTIM_ICR_ARRMCF               EQU (0x00000002)            ; Autoreload match Clear Flag
LPTIM_ICR_EXTTRIGCF            EQU (0x00000004)            ; External trigger edge event Clear Flag
LPTIM_ICR_CMPOKCF              EQU (0x00000008)            ; Compare register update OK Clear Flag
LPTIM_ICR_ARROKCF              EQU (0x00000010)            ; Autoreload register update OK Clear Flag
LPTIM_ICR_UPCF                 EQU (0x00000020)            ; Counter direction change down to up Clear Flag
LPTIM_ICR_DOWNCF               EQU (0x00000040)            ; Counter direction change up to down Clear Flag

; ******************  Bit definition for LPTIM_IER register *******************
LPTIM_IER_CMPMIE               EQU (0x00000001)            ; Compare match Interrupt Enable
LPTIM_IER_ARRMIE               EQU (0x00000002)            ; Autoreload match Interrupt Enable
LPTIM_IER_EXTTRIGIE            EQU (0x00000004)            ; External trigger edge event Interrupt Enable
LPTIM_IER_CMPOKIE              EQU (0x00000008)            ; Compare register update OK Interrupt Enable
LPTIM_IER_ARROKIE              EQU (0x00000010)            ; Autoreload register update OK Interrupt Enable
LPTIM_IER_UPIE                 EQU (0x00000020)            ; Counter direction change down to up Interrupt Enable
LPTIM_IER_DOWNIE               EQU (0x00000040)            ; Counter direction change up to down Interrupt Enable

; ******************  Bit definition for LPTIM_CFGR register ******************
LPTIM_CFGR_CKSEL               EQU (0x00000001)             ; Clock selector

LPTIM_CFGR_CKPOL               EQU (0x00000006)             ; CKPOL[1:0] bits (Clock polarity)
LPTIM_CFGR_CKPOL_0             EQU (0x00000002)             ; Bit 0
LPTIM_CFGR_CKPOL_1             EQU (0x00000004)             ; Bit 1

LPTIM_CFGR_CKFLT               EQU (0x00000018)             ; CKFLT[1:0] bits (Configurable digital filter for external clock)
LPTIM_CFGR_CKFLT_0             EQU (0x00000008)             ; Bit 0
LPTIM_CFGR_CKFLT_1             EQU (0x00000010)             ; Bit 1

LPTIM_CFGR_TRGFLT              EQU (0x000000C0)             ; TRGFLT[1:0] bits (Configurable digital filter for trigger)
LPTIM_CFGR_TRGFLT_0            EQU (0x00000040)             ; Bit 0
LPTIM_CFGR_TRGFLT_1            EQU (0x00000080)             ; Bit 1

LPTIM_CFGR_PRESC               EQU (0x00000E00)             ; PRESC[2:0] bits (Clock prescaler)
LPTIM_CFGR_PRESC_0             EQU (0x00000200)             ; Bit 0
LPTIM_CFGR_PRESC_1             EQU (0x00000400)             ; Bit 1
LPTIM_CFGR_PRESC_2             EQU (0x00000800)             ; Bit 2

LPTIM_CFGR_TRIGSEL             EQU (0x0000E000)             ; TRIGSEL[2:0]] bits (Trigger selector)
LPTIM_CFGR_TRIGSEL_0           EQU (0x00002000)             ; Bit 0
LPTIM_CFGR_TRIGSEL_1           EQU (0x00004000)             ; Bit 1
LPTIM_CFGR_TRIGSEL_2           EQU (0x00008000)             ; Bit 2

LPTIM_CFGR_TRIGEN              EQU (0x00060000)             ; TRIGEN[1:0] bits (Trigger enable and polarity)
LPTIM_CFGR_TRIGEN_0            EQU (0x00020000)             ; Bit 0
LPTIM_CFGR_TRIGEN_1            EQU (0x00040000)             ; Bit 1

LPTIM_CFGR_TIMOUT              EQU (0x00080000)             ; Timout enable
LPTIM_CFGR_WAVE                EQU (0x00100000)             ; Waveform shape
LPTIM_CFGR_WAVPOL              EQU (0x00200000)             ; Waveform shape polarity
LPTIM_CFGR_PRELOAD             EQU (0x00400000)             ; Reg update mode
LPTIM_CFGR_COUNTMODE           EQU (0x00800000)             ; Counter mode enable
LPTIM_CFGR_ENC                 EQU (0x01000000)             ; Encoder mode enable

; ******************  Bit definition for LPTIM_CR register  *******************
LPTIM_CR_ENABLE                EQU (0x00000001)             ; LPTIMer enable
LPTIM_CR_SNGSTRT               EQU (0x00000002)             ; Timer start in single mode
LPTIM_CR_CNTSTRT               EQU (0x00000004)             ; Timer start in continuous mode

; ******************  Bit definition for LPTIM_CMP register  ******************
LPTIM_CMP_CMP                  EQU (0x0000FFFF)             ; Compare register

; ******************  Bit definition for LPTIM_ARR register  ******************
LPTIM_ARR_ARR                  EQU (0x0000FFFF)             ; Auto reload register

; ******************  Bit definition for LPTIM_CNT register  ******************
LPTIM_CNT_CNT                  EQU (0x0000FFFF)             ; Counter register

; ******************  Bit definition for LPTIM_OR register  ******************
LPTIM_OR_OR                   EQU (0x00000003)               ; LPTIMER[1:0] bits (Remap selection)
LPTIM_OR_OR_0                 EQU (0x00000001)               ; Bit 0
LPTIM_OR_OR_1                 EQU (0x00000002)               ; Bit 1

; *****************************************************************************
;
;                      Analog Comparators (COMP)
;
; *****************************************************************************
; **********************  Bit definition for COMPx_CSR register  **************
COMP_CSR_EN            EQU (0x00000001) ; COMPx enable

COMP_CSR_PWRMODE       EQU (0x0000000C) ; COMPx power mode
COMP_CSR_PWRMODE_0     EQU (0x00000004) ; COMPx power mode bit 0
COMP_CSR_PWRMODE_1     EQU (0x00000008) ; COMPx power mode bit 1

COMP_CSR_INMSEL        EQU (0x00000070) ; COMPx inverting input (minus) selection
COMP_CSR_INMSEL_0      EQU (0x00000010) ; COMPx inverting input (minus) selection bit 0
COMP_CSR_INMSEL_1      EQU (0x00000020) ; COMPx inverting input (minus) selection bit 1
COMP_CSR_INMSEL_2      EQU (0x00000040) ; COMPx inverting input (minus) selection bit 2

COMP_CSR_INPSEL        EQU (0x00000080) ; COMPx non inverting input (plus) selection
COMP_CSR_INPSEL_0      EQU (0x00000080) ; COMPx non inverting input (plus) selection bit 0
COMP_CSR_WINMODE       EQU (0x00000200) ; COMPx window mode (only available on COMP2)
COMP_CSR_POLARITY      EQU (0x00008000) ; COMPx output polarity

COMP_CSR_HYST          EQU (0x00030000) ; COMPx hysteresis
COMP_CSR_HYST_0        EQU (0x00010000) ; COMPx hysteresis bit 0
COMP_CSR_HYST_1        EQU (0x00020000) ; COMPx hysteresis bit 1

COMP_CSR_BLANKING      EQU (0x001C0000) ; COMPx blanking source
COMP_CSR_BLANKING_0    EQU (0x00040000) ; COMPx blanking source bit 0
COMP_CSR_BLANKING_1    EQU (0x00080000) ; COMPx blanking source bit 1
COMP_CSR_BLANKING_2    EQU (0x00100000) ; COMPx blanking source bit 2

COMP_CSR_BRGEN         EQU (0x00400000) ; COMPx voltage scaler enable
COMP_CSR_SCALEN        EQU (0x00800000) ; COMPx scaler bridge enable
COMP_CSR_VALUE         EQU (0x40000000) ; COMPx value
COMP_CSR_LOCK          EQU (0x80000000) ; COMPx lock

; *****************************************************************************
;
;                         Operational Amplifier (OPAMP)
;
; *****************************************************************************
; *********************  Bit definition for OPAMPx_CSR register  **************
OPAMP_CSR_OPAMPxEN    EQU (0x00000001) ; OPAMP enable
OPAMP_CSR_OPALPM      EQU (0x00000002) ; Operational amplifier Low Power Mode

OPAMP_CSR_OPAMODE     EQU (0x0000000C) ; Operational amplifier PGA mode
OPAMP_CSR_OPAMODE_0   EQU (0x00000004) ; Bit 0
OPAMP_CSR_OPAMODE_1   EQU (0x00000008) ; Bit 1

OPAMP_CSR_PGGAIN     EQU (0x00000030) ; Operational amplifier Programmable amplifier gain value
OPAMP_CSR_PGGAIN_0   EQU (0x00000010) ; Bit 0
OPAMP_CSR_PGGAIN_1   EQU (0x00000020) ; Bit 1

OPAMP_CSR_VMSEL       EQU (0x00000300) ; Inverting input selection
OPAMP_CSR_VMSEL_0     EQU (0x00000100) ; Bit 0
OPAMP_CSR_VMSEL_1     EQU (0x00000200) ; Bit 1

OPAMP_CSR_VPSEL       EQU (0x00000400) ; Non inverted input selection
OPAMP_CSR_CALON       EQU (0x00001000) ; Calibration mode enable
OPAMP_CSR_CALSEL      EQU (0x00002000) ; Calibration selection
OPAMP_CSR_USERTRIM    EQU (0x00004000) ; User trimming enable
OPAMP_CSR_CALOUT      EQU (0x00008000) ; Operational amplifier1 calibration output

; *********************  Bit definition for OPAMP1_CSR register  **************
OPAMP1_CSR_OPAEN       EQU (0x00000001) ; Operational amplifier1 Enable
OPAMP1_CSR_OPALPM      EQU (0x00000002) ; Operational amplifier1 Low Power Mode

OPAMP1_CSR_OPAMODE     EQU (0x0000000C) ; Operational amplifier1 PGA mode
OPAMP1_CSR_OPAMODE_0   EQU (0x00000004) ; Bit 0
OPAMP1_CSR_OPAMODE_1   EQU (0x00000008) ; Bit 1

OPAMP1_CSR_PGAGAIN     EQU (0x00000030) ; Operational amplifier1 Programmable amplifier gain value
OPAMP1_CSR_PGAGAIN_0   EQU (0x00000010) ; Bit 0
OPAMP1_CSR_PGAGAIN_1   EQU (0x00000020) ; Bit 1

OPAMP1_CSR_VMSEL       EQU (0x00000300) ; Inverting input selection
OPAMP1_CSR_VMSEL_0     EQU (0x00000100) ; Bit 0
OPAMP1_CSR_VMSEL_1     EQU (0x00000200) ; Bit 1

OPAMP1_CSR_VPSEL       EQU (0x00000400) ; Non inverted input selection
OPAMP1_CSR_CALON       EQU (0x00001000) ; Calibration mode enable
OPAMP1_CSR_CALSEL      EQU (0x00002000) ; Calibration selection
OPAMP1_CSR_USERTRIM    EQU (0x00004000) ; User trimming enable
OPAMP1_CSR_CALOUT      EQU (0x00008000) ; Operational amplifier1 calibration output
OPAMP1_CSR_OPARANGE    EQU (0x80000000) ; Operational amplifiers power supply range for stability

; *********************  Bit definition for OPAMP2_CSR register  **************
OPAMP2_CSR_OPAEN       EQU (0x00000001) ; Operational amplifier2 Enable
OPAMP2_CSR_OPALPM      EQU (0x00000002) ; Operational amplifier2 Low Power Mode

OPAMP2_CSR_OPAMODE     EQU (0x0000000C) ; Operational amplifier2 PGA mode
OPAMP2_CSR_OPAMODE_0   EQU (0x00000004) ; Bit 0
OPAMP2_CSR_OPAMODE_1   EQU (0x00000008) ; Bit 1

OPAMP2_CSR_PGAGAIN     EQU (0x00000030) ; Operational amplifier2 Programmable amplifier gain value
OPAMP2_CSR_PGAGAIN_0   EQU (0x00000010) ; Bit 0
OPAMP2_CSR_PGAGAIN_1   EQU (0x00000020) ; Bit 1

OPAMP2_CSR_VMSEL       EQU (0x00000300) ; Inverting input selection
OPAMP2_CSR_VMSEL_0     EQU (0x00000100) ; Bit 0
OPAMP2_CSR_VMSEL_1     EQU (0x00000200) ; Bit 1

OPAMP2_CSR_VPSEL       EQU (0x00000400) ; Non inverted input selection
OPAMP2_CSR_CALON       EQU (0x00001000) ; Calibration mode enable
OPAMP2_CSR_CALSEL      EQU (0x00002000) ; Calibration selection
OPAMP2_CSR_USERTRIM    EQU (0x00004000) ; User trimming enable
OPAMP2_CSR_CALOUT      EQU (0x00008000) ; Operational amplifier2 calibration output

; *******************  Bit definition for OPAMP_OTR register  *****************
OPAMP_OTR_TRIMOFFSETN    EQU (0x0000001F)        ; Trim for NMOS differential pairs
OPAMP_OTR_TRIMOFFSETP    EQU (0x00001F00)        ; Trim for PMOS differential pairs

; *******************  Bit definition for OPAMP1_OTR register  *****************
OPAMP1_OTR_TRIMOFFSETN    EQU (0x0000001F)        ; Trim for NMOS differential pairs
OPAMP1_OTR_TRIMOFFSETP    EQU (0x00001F00)        ; Trim for PMOS differential pairs

; *******************  Bit definition for OPAMP2_OTR register  *****************
OPAMP2_OTR_TRIMOFFSETN    EQU (0x0000001F)        ; Trim for NMOS differential pairs
OPAMP2_OTR_TRIMOFFSETP    EQU (0x00001F00)        ; Trim for PMOS differential pairs

; *******************  Bit definition for OPAMP_LPOTR register  ***************
OPAMP_LPOTR_TRIMLPOFFSETN        EQU (0x0000001F)        ; Trim for NMOS differential pairs
OPAMP_LPOTR_TRIMLPOFFSETP        EQU (0x00001F00)        ; Trim for PMOS differential pairs

; *******************  Bit definition for OPAMP1_LPOTR register  ***************
OPAMP1_LPOTR_TRIMLPOFFSETN        EQU (0x0000001F)        ; Trim for NMOS differential pairs
OPAMP1_LPOTR_TRIMLPOFFSETP        EQU (0x00001F00)        ; Trim for PMOS differential pairs

; *******************  Bit definition for OPAMP2_LPOTR register  ***************
OPAMP2_LPOTR_TRIMLPOFFSETN        EQU (0x0000001F)        ; Trim for NMOS differential pairs
OPAMP2_LPOTR_TRIMLPOFFSETP        EQU (0x00001F00)        ; Trim for PMOS differential pairs

; *****************************************************************************
;
;                          Touch Sensing Controller (TSC)
;
; *****************************************************************************
; *******************  Bit definition for TSC_CR register  ********************
TSC_CR_TSCE                 EQU (0x00000001)            ;Touch sensing controller enable
TSC_CR_START                EQU (0x00000002)            ;Start acquisition
TSC_CR_AM                   EQU (0x00000004)            ;Acquisition mode
TSC_CR_SYNCPOL              EQU (0x00000008)            ;Synchronization pin polarity
TSC_CR_IODEF                EQU (0x00000010)            ;IO default mode

TSC_CR_MCV                  EQU (0x000000E0)            ;MCV[2:0] bits (Max Count Value)
TSC_CR_MCV_0                EQU (0x00000020)            ;Bit 0
TSC_CR_MCV_1                EQU (0x00000040)            ;Bit 1
TSC_CR_MCV_2                EQU (0x00000080)            ;Bit 2

TSC_CR_PGPSC                EQU (0x00007000)            ;PGPSC[2:0] bits (Pulse Generator Prescaler)
TSC_CR_PGPSC_0              EQU (0x00001000)            ;Bit 0
TSC_CR_PGPSC_1              EQU (0x00002000)            ;Bit 1
TSC_CR_PGPSC_2              EQU (0x00004000)            ;Bit 2

TSC_CR_SSPSC                EQU (0x00008000)            ;Spread Spectrum Prescaler
TSC_CR_SSE                  EQU (0x00010000)            ;Spread Spectrum Enable

TSC_CR_SSD                  EQU (0x00FE0000)            ;SSD[6:0] bits (Spread Spectrum Deviation)
TSC_CR_SSD_0                EQU (0x00020000)            ;Bit 0
TSC_CR_SSD_1                EQU (0x00040000)            ;Bit 1
TSC_CR_SSD_2                EQU (0x00080000)            ;Bit 2
TSC_CR_SSD_3                EQU (0x00100000)            ;Bit 3
TSC_CR_SSD_4                EQU (0x00200000)            ;Bit 4
TSC_CR_SSD_5                EQU (0x00400000)            ;Bit 5
TSC_CR_SSD_6                EQU (0x00800000)            ;Bit 6

TSC_CR_CTPL                 EQU (0x0F000000)            ;CTPL[3:0] bits (Charge Transfer pulse low)
TSC_CR_CTPL_0               EQU (0x01000000)            ;Bit 0
TSC_CR_CTPL_1               EQU (0x02000000)            ;Bit 1
TSC_CR_CTPL_2               EQU (0x04000000)            ;Bit 2
TSC_CR_CTPL_3               EQU (0x08000000)            ;Bit 3

TSC_CR_CTPH                 EQU (0xF0000000)            ;CTPH[3:0] bits (Charge Transfer pulse high)
TSC_CR_CTPH_0               EQU (0x10000000)            ;Bit 0
TSC_CR_CTPH_1               EQU (0x20000000)            ;Bit 1
TSC_CR_CTPH_2               EQU (0x40000000)            ;Bit 2
TSC_CR_CTPH_3               EQU (0x80000000)            ;Bit 3

; *******************  Bit definition for TSC_IER register  *******************
TSC_IER_EOAIE               EQU (0x00000001)            ;End of acquisition interrupt enable
TSC_IER_MCEIE               EQU (0x00000002)            ;Max count error interrupt enable

; *******************  Bit definition for TSC_ICR register  *******************
TSC_ICR_EOAIC               EQU (0x00000001)            ;End of acquisition interrupt clear
TSC_ICR_MCEIC               EQU (0x00000002)            ;Max count error interrupt clear

; *******************  Bit definition for TSC_ISR register  *******************
TSC_ISR_EOAF                EQU (0x00000001)            ;End of acquisition flag
TSC_ISR_MCEF                EQU (0x00000002)            ;Max count error flag

; *******************  Bit definition for TSC_IOHCR register  *****************
TSC_IOHCR_G1_IO1            EQU (0x00000001)            ;GROUP1_IO1 schmitt trigger hysteresis mode
TSC_IOHCR_G1_IO2            EQU (0x00000002)            ;GROUP1_IO2 schmitt trigger hysteresis mode
TSC_IOHCR_G1_IO3            EQU (0x00000004)            ;GROUP1_IO3 schmitt trigger hysteresis mode
TSC_IOHCR_G1_IO4            EQU (0x00000008)            ;GROUP1_IO4 schmitt trigger hysteresis mode
TSC_IOHCR_G2_IO1            EQU (0x00000010)            ;GROUP2_IO1 schmitt trigger hysteresis mode
TSC_IOHCR_G2_IO2            EQU (0x00000020)            ;GROUP2_IO2 schmitt trigger hysteresis mode
TSC_IOHCR_G2_IO3            EQU (0x00000040)            ;GROUP2_IO3 schmitt trigger hysteresis mode
TSC_IOHCR_G2_IO4            EQU (0x00000080)            ;GROUP2_IO4 schmitt trigger hysteresis mode
TSC_IOHCR_G3_IO1            EQU (0x00000100)            ;GROUP3_IO1 schmitt trigger hysteresis mode
TSC_IOHCR_G3_IO2            EQU (0x00000200)            ;GROUP3_IO2 schmitt trigger hysteresis mode
TSC_IOHCR_G3_IO3            EQU (0x00000400)            ;GROUP3_IO3 schmitt trigger hysteresis mode
TSC_IOHCR_G3_IO4            EQU (0x00000800)            ;GROUP3_IO4 schmitt trigger hysteresis mode
TSC_IOHCR_G4_IO1            EQU (0x00001000)            ;GROUP4_IO1 schmitt trigger hysteresis mode
TSC_IOHCR_G4_IO2            EQU (0x00002000)            ;GROUP4_IO2 schmitt trigger hysteresis mode
TSC_IOHCR_G4_IO3            EQU (0x00004000)            ;GROUP4_IO3 schmitt trigger hysteresis mode
TSC_IOHCR_G4_IO4            EQU (0x00008000)            ;GROUP4_IO4 schmitt trigger hysteresis mode
TSC_IOHCR_G5_IO1            EQU (0x00010000)            ;GROUP5_IO1 schmitt trigger hysteresis mode
TSC_IOHCR_G5_IO2            EQU (0x00020000)            ;GROUP5_IO2 schmitt trigger hysteresis mode
TSC_IOHCR_G5_IO3            EQU (0x00040000)            ;GROUP5_IO3 schmitt trigger hysteresis mode
TSC_IOHCR_G5_IO4            EQU (0x00080000)            ;GROUP5_IO4 schmitt trigger hysteresis mode
TSC_IOHCR_G6_IO1            EQU (0x00100000)            ;GROUP6_IO1 schmitt trigger hysteresis mode
TSC_IOHCR_G6_IO2            EQU (0x00200000)            ;GROUP6_IO2 schmitt trigger hysteresis mode
TSC_IOHCR_G6_IO3            EQU (0x00400000)            ;GROUP6_IO3 schmitt trigger hysteresis mode
TSC_IOHCR_G6_IO4            EQU (0x00800000)            ;GROUP6_IO4 schmitt trigger hysteresis mode
TSC_IOHCR_G7_IO1            EQU (0x01000000)            ;GROUP7_IO1 schmitt trigger hysteresis mode
TSC_IOHCR_G7_IO2            EQU (0x02000000)            ;GROUP7_IO2 schmitt trigger hysteresis mode
TSC_IOHCR_G7_IO3            EQU (0x04000000)            ;GROUP7_IO3 schmitt trigger hysteresis mode
TSC_IOHCR_G7_IO4            EQU (0x08000000)            ;GROUP7_IO4 schmitt trigger hysteresis mode
TSC_IOHCR_G8_IO1            EQU (0x10000000)            ;GROUP8_IO1 schmitt trigger hysteresis mode
TSC_IOHCR_G8_IO2            EQU (0x20000000)            ;GROUP8_IO2 schmitt trigger hysteresis mode
TSC_IOHCR_G8_IO3            EQU (0x40000000)            ;GROUP8_IO3 schmitt trigger hysteresis mode
TSC_IOHCR_G8_IO4            EQU (0x80000000)            ;GROUP8_IO4 schmitt trigger hysteresis mode

; *******************  Bit definition for TSC_IOASCR register  ****************
TSC_IOASCR_G1_IO1           EQU (0x00000001)            ;GROUP1_IO1 analog switch enable
TSC_IOASCR_G1_IO2           EQU (0x00000002)            ;GROUP1_IO2 analog switch enable
TSC_IOASCR_G1_IO3           EQU (0x00000004)            ;GROUP1_IO3 analog switch enable
TSC_IOASCR_G1_IO4           EQU (0x00000008)            ;GROUP1_IO4 analog switch enable
TSC_IOASCR_G2_IO1           EQU (0x00000010)            ;GROUP2_IO1 analog switch enable
TSC_IOASCR_G2_IO2           EQU (0x00000020)            ;GROUP2_IO2 analog switch enable
TSC_IOASCR_G2_IO3           EQU (0x00000040)            ;GROUP2_IO3 analog switch enable
TSC_IOASCR_G2_IO4           EQU (0x00000080)            ;GROUP2_IO4 analog switch enable
TSC_IOASCR_G3_IO1           EQU (0x00000100)            ;GROUP3_IO1 analog switch enable
TSC_IOASCR_G3_IO2           EQU (0x00000200)            ;GROUP3_IO2 analog switch enable
TSC_IOASCR_G3_IO3           EQU (0x00000400)            ;GROUP3_IO3 analog switch enable
TSC_IOASCR_G3_IO4           EQU (0x00000800)            ;GROUP3_IO4 analog switch enable
TSC_IOASCR_G4_IO1           EQU (0x00001000)            ;GROUP4_IO1 analog switch enable
TSC_IOASCR_G4_IO2           EQU (0x00002000)            ;GROUP4_IO2 analog switch enable
TSC_IOASCR_G4_IO3           EQU (0x00004000)            ;GROUP4_IO3 analog switch enable
TSC_IOASCR_G4_IO4           EQU (0x00008000)            ;GROUP4_IO4 analog switch enable
TSC_IOASCR_G5_IO1           EQU (0x00010000)            ;GROUP5_IO1 analog switch enable
TSC_IOASCR_G5_IO2           EQU (0x00020000)            ;GROUP5_IO2 analog switch enable
TSC_IOASCR_G5_IO3           EQU (0x00040000)            ;GROUP5_IO3 analog switch enable
TSC_IOASCR_G5_IO4           EQU (0x00080000)            ;GROUP5_IO4 analog switch enable
TSC_IOASCR_G6_IO1           EQU (0x00100000)            ;GROUP6_IO1 analog switch enable
TSC_IOASCR_G6_IO2           EQU (0x00200000)            ;GROUP6_IO2 analog switch enable
TSC_IOASCR_G6_IO3           EQU (0x00400000)            ;GROUP6_IO3 analog switch enable
TSC_IOASCR_G6_IO4           EQU (0x00800000)            ;GROUP6_IO4 analog switch enable
TSC_IOASCR_G7_IO1           EQU (0x01000000)            ;GROUP7_IO1 analog switch enable
TSC_IOASCR_G7_IO2           EQU (0x02000000)            ;GROUP7_IO2 analog switch enable
TSC_IOASCR_G7_IO3           EQU (0x04000000)            ;GROUP7_IO3 analog switch enable
TSC_IOASCR_G7_IO4           EQU (0x08000000)            ;GROUP7_IO4 analog switch enable
TSC_IOASCR_G8_IO1           EQU (0x10000000)            ;GROUP8_IO1 analog switch enable
TSC_IOASCR_G8_IO2           EQU (0x20000000)            ;GROUP8_IO2 analog switch enable
TSC_IOASCR_G8_IO3           EQU (0x40000000)            ;GROUP8_IO3 analog switch enable
TSC_IOASCR_G8_IO4           EQU (0x80000000)            ;GROUP8_IO4 analog switch enable

; *******************  Bit definition for TSC_IOSCR register  *****************
TSC_IOSCR_G1_IO1            EQU (0x00000001)            ;GROUP1_IO1 sampling mode
TSC_IOSCR_G1_IO2            EQU (0x00000002)            ;GROUP1_IO2 sampling mode
TSC_IOSCR_G1_IO3            EQU (0x00000004)            ;GROUP1_IO3 sampling mode
TSC_IOSCR_G1_IO4            EQU (0x00000008)            ;GROUP1_IO4 sampling mode
TSC_IOSCR_G2_IO1            EQU (0x00000010)            ;GROUP2_IO1 sampling mode
TSC_IOSCR_G2_IO2            EQU (0x00000020)            ;GROUP2_IO2 sampling mode
TSC_IOSCR_G2_IO3            EQU (0x00000040)            ;GROUP2_IO3 sampling mode
TSC_IOSCR_G2_IO4            EQU (0x00000080)            ;GROUP2_IO4 sampling mode
TSC_IOSCR_G3_IO1            EQU (0x00000100)            ;GROUP3_IO1 sampling mode
TSC_IOSCR_G3_IO2            EQU (0x00000200)            ;GROUP3_IO2 sampling mode
TSC_IOSCR_G3_IO3            EQU (0x00000400)            ;GROUP3_IO3 sampling mode
TSC_IOSCR_G3_IO4            EQU (0x00000800)            ;GROUP3_IO4 sampling mode
TSC_IOSCR_G4_IO1            EQU (0x00001000)            ;GROUP4_IO1 sampling mode
TSC_IOSCR_G4_IO2            EQU (0x00002000)            ;GROUP4_IO2 sampling mode
TSC_IOSCR_G4_IO3            EQU (0x00004000)            ;GROUP4_IO3 sampling mode
TSC_IOSCR_G4_IO4            EQU (0x00008000)            ;GROUP4_IO4 sampling mode
TSC_IOSCR_G5_IO1            EQU (0x00010000)            ;GROUP5_IO1 sampling mode
TSC_IOSCR_G5_IO2            EQU (0x00020000)            ;GROUP5_IO2 sampling mode
TSC_IOSCR_G5_IO3            EQU (0x00040000)            ;GROUP5_IO3 sampling mode
TSC_IOSCR_G5_IO4            EQU (0x00080000)            ;GROUP5_IO4 sampling mode
TSC_IOSCR_G6_IO1            EQU (0x00100000)            ;GROUP6_IO1 sampling mode
TSC_IOSCR_G6_IO2            EQU (0x00200000)            ;GROUP6_IO2 sampling mode
TSC_IOSCR_G6_IO3            EQU (0x00400000)            ;GROUP6_IO3 sampling mode
TSC_IOSCR_G6_IO4            EQU (0x00800000)            ;GROUP6_IO4 sampling mode
TSC_IOSCR_G7_IO1            EQU (0x01000000)            ;GROUP7_IO1 sampling mode
TSC_IOSCR_G7_IO2            EQU (0x02000000)            ;GROUP7_IO2 sampling mode
TSC_IOSCR_G7_IO3            EQU (0x04000000)            ;GROUP7_IO3 sampling mode
TSC_IOSCR_G7_IO4            EQU (0x08000000)            ;GROUP7_IO4 sampling mode
TSC_IOSCR_G8_IO1            EQU (0x10000000)            ;GROUP8_IO1 sampling mode
TSC_IOSCR_G8_IO2            EQU (0x20000000)            ;GROUP8_IO2 sampling mode
TSC_IOSCR_G8_IO3            EQU (0x40000000)            ;GROUP8_IO3 sampling mode
TSC_IOSCR_G8_IO4            EQU (0x80000000)            ;GROUP8_IO4 sampling mode

; *******************  Bit definition for TSC_IOCCR register  *****************
TSC_IOCCR_G1_IO1            EQU (0x00000001)            ;GROUP1_IO1 channel mode
TSC_IOCCR_G1_IO2            EQU (0x00000002)            ;GROUP1_IO2 channel mode
TSC_IOCCR_G1_IO3            EQU (0x00000004)            ;GROUP1_IO3 channel mode
TSC_IOCCR_G1_IO4            EQU (0x00000008)            ;GROUP1_IO4 channel mode
TSC_IOCCR_G2_IO1            EQU (0x00000010)            ;GROUP2_IO1 channel mode
TSC_IOCCR_G2_IO2            EQU (0x00000020)            ;GROUP2_IO2 channel mode
TSC_IOCCR_G2_IO3            EQU (0x00000040)            ;GROUP2_IO3 channel mode
TSC_IOCCR_G2_IO4            EQU (0x00000080)            ;GROUP2_IO4 channel mode
TSC_IOCCR_G3_IO1            EQU (0x00000100)            ;GROUP3_IO1 channel mode
TSC_IOCCR_G3_IO2            EQU (0x00000200)            ;GROUP3_IO2 channel mode
TSC_IOCCR_G3_IO3            EQU (0x00000400)            ;GROUP3_IO3 channel mode
TSC_IOCCR_G3_IO4            EQU (0x00000800)            ;GROUP3_IO4 channel mode
TSC_IOCCR_G4_IO1            EQU (0x00001000)            ;GROUP4_IO1 channel mode
TSC_IOCCR_G4_IO2            EQU (0x00002000)            ;GROUP4_IO2 channel mode
TSC_IOCCR_G4_IO3            EQU (0x00004000)            ;GROUP4_IO3 channel mode
TSC_IOCCR_G4_IO4            EQU (0x00008000)            ;GROUP4_IO4 channel mode
TSC_IOCCR_G5_IO1            EQU (0x00010000)            ;GROUP5_IO1 channel mode
TSC_IOCCR_G5_IO2            EQU (0x00020000)            ;GROUP5_IO2 channel mode
TSC_IOCCR_G5_IO3            EQU (0x00040000)            ;GROUP5_IO3 channel mode
TSC_IOCCR_G5_IO4            EQU (0x00080000)            ;GROUP5_IO4 channel mode
TSC_IOCCR_G6_IO1            EQU (0x00100000)            ;GROUP6_IO1 channel mode
TSC_IOCCR_G6_IO2            EQU (0x00200000)            ;GROUP6_IO2 channel mode
TSC_IOCCR_G6_IO3            EQU (0x00400000)            ;GROUP6_IO3 channel mode
TSC_IOCCR_G6_IO4            EQU (0x00800000)            ;GROUP6_IO4 channel mode
TSC_IOCCR_G7_IO1            EQU (0x01000000)            ;GROUP7_IO1 channel mode
TSC_IOCCR_G7_IO2            EQU (0x02000000)            ;GROUP7_IO2 channel mode
TSC_IOCCR_G7_IO3            EQU (0x04000000)            ;GROUP7_IO3 channel mode
TSC_IOCCR_G7_IO4            EQU (0x08000000)            ;GROUP7_IO4 channel mode
TSC_IOCCR_G8_IO1            EQU (0x10000000)            ;GROUP8_IO1 channel mode
TSC_IOCCR_G8_IO2            EQU (0x20000000)            ;GROUP8_IO2 channel mode
TSC_IOCCR_G8_IO3            EQU (0x40000000)            ;GROUP8_IO3 channel mode
TSC_IOCCR_G8_IO4            EQU (0x80000000)            ;GROUP8_IO4 channel mode

; *******************  Bit definition for TSC_IOGCSR register  ****************
TSC_IOGCSR_G1E              EQU (0x00000001)            ;Analog IO GROUP1 enable
TSC_IOGCSR_G2E              EQU (0x00000002)            ;Analog IO GROUP2 enable
TSC_IOGCSR_G3E              EQU (0x00000004)            ;Analog IO GROUP3 enable
TSC_IOGCSR_G4E              EQU (0x00000008)            ;Analog IO GROUP4 enable
TSC_IOGCSR_G5E              EQU (0x00000010)            ;Analog IO GROUP5 enable
TSC_IOGCSR_G6E              EQU (0x00000020)            ;Analog IO GROUP6 enable
TSC_IOGCSR_G7E              EQU (0x00000040)            ;Analog IO GROUP7 enable
TSC_IOGCSR_G8E              EQU (0x00000080)            ;Analog IO GROUP8 enable
TSC_IOGCSR_G1S              EQU (0x00010000)            ;Analog IO GROUP1 status
TSC_IOGCSR_G2S              EQU (0x00020000)            ;Analog IO GROUP2 status
TSC_IOGCSR_G3S              EQU (0x00040000)            ;Analog IO GROUP3 status
TSC_IOGCSR_G4S              EQU (0x00080000)            ;Analog IO GROUP4 status
TSC_IOGCSR_G5S              EQU (0x00100000)            ;Analog IO GROUP5 status
TSC_IOGCSR_G6S              EQU (0x00200000)            ;Analog IO GROUP6 status
TSC_IOGCSR_G7S              EQU (0x00400000)            ;Analog IO GROUP7 status
TSC_IOGCSR_G8S              EQU (0x00800000)            ;Analog IO GROUP8 status

; *******************  Bit definition for TSC_IOGXCR register  ****************
TSC_IOGXCR_CNT              EQU (0x00003FFF)            ;CNT[13:0] bits (Counter value)

; *****************************************************************************
;
;      Universal Synchronous Asynchronous Receiver Transmitter (USART)
;
; *****************************************************************************
; ******************  Bit definition for USART_CR1 register  ******************
USART_CR1_UE                EQU (0x00000001)            ; USART Enable
USART_CR1_UESM              EQU (0x00000002)            ; USART Enable in STOP Mode
USART_CR1_RE                EQU (0x00000004)            ; Receiver Enable
USART_CR1_TE                EQU (0x00000008)            ; Transmitter Enable
USART_CR1_IDLEIE            EQU (0x00000010)            ; IDLE Interrupt Enable
USART_CR1_RXNEIE            EQU (0x00000020)            ; RXNE Interrupt Enable
USART_CR1_TCIE              EQU (0x00000040)            ; Transmission Complete Interrupt Enable
USART_CR1_TXEIE             EQU (0x00000080)            ; TXE Interrupt Enable
USART_CR1_PEIE              EQU (0x00000100)            ; PE Interrupt Enable
USART_CR1_PS                EQU (0x00000200)            ; Parity Selection
USART_CR1_PCE               EQU (0x00000400)            ; Parity Control Enable
USART_CR1_WAKE              EQU (0x00000800)            ; Receiver Wakeup method
USART_CR1_M                 EQU (0x10001000)            ; Word length
USART_CR1_M0                EQU (0x00001000)            ; Word length - Bit 0
USART_CR1_MME               EQU (0x00002000)            ; Mute Mode Enable
USART_CR1_CMIE              EQU (0x00004000)            ; Character match interrupt enable
USART_CR1_OVER8             EQU (0x00008000)            ; Oversampling by 8-bit or 16-bit mode
USART_CR1_DEDT              EQU (0x001F0000)            ; DEDT[4:0] bits (Driver Enable Deassertion Time)
USART_CR1_DEDT_0            EQU (0x00010000)            ; Bit 0
USART_CR1_DEDT_1            EQU (0x00020000)            ; Bit 1
USART_CR1_DEDT_2            EQU (0x00040000)            ; Bit 2
USART_CR1_DEDT_3            EQU (0x00080000)            ; Bit 3
USART_CR1_DEDT_4            EQU (0x00100000)            ; Bit 4
USART_CR1_DEAT              EQU (0x03E00000)            ; DEAT[4:0] bits (Driver Enable Assertion Time)
USART_CR1_DEAT_0            EQU (0x00200000)            ; Bit 0
USART_CR1_DEAT_1            EQU (0x00400000)            ; Bit 1
USART_CR1_DEAT_2            EQU (0x00800000)            ; Bit 2
USART_CR1_DEAT_3            EQU (0x01000000)            ; Bit 3
USART_CR1_DEAT_4            EQU (0x02000000)            ; Bit 4
USART_CR1_RTOIE             EQU (0x04000000)            ; Receive Time Out interrupt enable
USART_CR1_EOBIE             EQU (0x08000000)            ; End of Block interrupt enable
USART_CR1_M1                EQU (0x10000000)            ; Word length - Bit 1

; ******************  Bit definition for USART_CR2 register  ******************
USART_CR2_ADDM7             EQU (0x00000010)            ; 7-bit or 4-bit Address Detection
USART_CR2_LBDL              EQU (0x00000020)            ; LIN Break Detection Length
USART_CR2_LBDIE             EQU (0x00000040)            ; LIN Break Detection Interrupt Enable
USART_CR2_LBCL              EQU (0x00000100)            ; Last Bit Clock pulse
USART_CR2_CPHA              EQU (0x00000200)            ; Clock Phase
USART_CR2_CPOL              EQU (0x00000400)            ; Clock Polarity
USART_CR2_CLKEN             EQU (0x00000800)            ; Clock Enable
USART_CR2_STOP              EQU (0x00003000)            ; STOP[1:0] bits (STOP bits)
USART_CR2_STOP_0            EQU (0x00001000)            ; Bit 0
USART_CR2_STOP_1            EQU (0x00002000)            ; Bit 1
USART_CR2_LINEN             EQU (0x00004000)            ; LIN mode enable
USART_CR2_SWAP              EQU (0x00008000)            ; SWAP TX/RX pins
USART_CR2_RXINV             EQU (0x00010000)            ; RX pin active level inversion
USART_CR2_TXINV             EQU (0x00020000)            ; TX pin active level inversion
USART_CR2_DATAINV           EQU (0x00040000)            ; Binary data inversion
USART_CR2_MSBFIRST          EQU (0x00080000)            ; Most Significant Bit First
USART_CR2_ABREN             EQU (0x00100000)            ; Auto Baud-Rate Enable
USART_CR2_ABRMODE           EQU (0x00600000)            ; ABRMOD[1:0] bits (Auto Baud-Rate Mode)
USART_CR2_ABRMODE_0         EQU (0x00200000)            ; Bit 0
USART_CR2_ABRMODE_1         EQU (0x00400000)            ; Bit 1
USART_CR2_RTOEN             EQU (0x00800000)            ; Receiver Time-Out enable
USART_CR2_ADD               EQU (0xFF000000)            ; Address of the USART node

; ******************  Bit definition for USART_CR3 register  ******************
USART_CR3_EIE               EQU (0x00000001)            ; Error Interrupt Enable
USART_CR3_IREN              EQU (0x00000002)            ; IrDA mode Enable
USART_CR3_IRLP              EQU (0x00000004)            ; IrDA Low-Power
USART_CR3_HDSEL             EQU (0x00000008)            ; Half-Duplex Selection
USART_CR3_NACK              EQU (0x00000010)            ; SmartCard NACK enable
USART_CR3_SCEN              EQU (0x00000020)            ; SmartCard mode enable
USART_CR3_DMAR              EQU (0x00000040)            ; DMA Enable Receiver
USART_CR3_DMAT              EQU (0x00000080)            ; DMA Enable Transmitter
USART_CR3_RTSE              EQU (0x00000100)            ; RTS Enable
USART_CR3_CTSE              EQU (0x00000200)            ; CTS Enable
USART_CR3_CTSIE             EQU (0x00000400)            ; CTS Interrupt Enable
USART_CR3_ONEBIT            EQU (0x00000800)            ; One sample bit method enable
USART_CR3_OVRDIS            EQU (0x00001000)            ; Overrun Disable
USART_CR3_DDRE              EQU (0x00002000)            ; DMA Disable on Reception Error
USART_CR3_DEM               EQU (0x00004000)            ; Driver Enable Mode
USART_CR3_DEP               EQU (0x00008000)            ; Driver Enable Polarity Selection
USART_CR3_SCARCNT           EQU (0x000E0000)            ; SCARCNT[2:0] bits (SmartCard Auto-Retry Count)
USART_CR3_SCARCNT_0         EQU (0x00020000)            ; Bit 0
USART_CR3_SCARCNT_1         EQU (0x00040000)            ; Bit 1
USART_CR3_SCARCNT_2         EQU (0x00080000)            ; Bit 2
USART_CR3_WUS               EQU (0x00300000)            ; WUS[1:0] bits (Wake UP Interrupt Flag Selection)
USART_CR3_WUS_0             EQU (0x00100000)            ; Bit 0
USART_CR3_WUS_1             EQU (0x00200000)            ; Bit 1
USART_CR3_WUFIE             EQU (0x00400000)            ; Wake Up Interrupt Enable

; ******************  Bit definition for USART_BRR register  ******************
USART_BRR_DIV_FRACTION      EQU (0x000F)                ; Fraction of USARTDIV
USART_BRR_DIV_MANTISSA      EQU (0xFFF0)                ; Mantissa of USARTDIV

; ******************  Bit definition for USART_GTPR register  *****************
USART_GTPR_PSC              EQU (0x000000FF)            ; PSC[7:0] bits (Prescaler value)
USART_GTPR_GT               EQU (0x0000FF00)            ; GT[7:0] bits (Guard time value)


; *******************  Bit definition for USART_RTOR register  ****************
USART_RTOR_RTO              EQU (0x00FFFFFF)            ; Receiver Time Out Value
USART_RTOR_BLEN             EQU (0xFF000000)            ; Block Length

; *******************  Bit definition for USART_RQR register  *****************
USART_RQR_ABRRQ             EQU (0x0001)                ; Auto-Baud Rate Request
USART_RQR_SBKRQ             EQU (0x0002)                ; Send Break Request
USART_RQR_MMRQ              EQU (0x0004)                ; Mute Mode Request
USART_RQR_RXFRQ             EQU (0x0008)                ; Receive Data flush Request
USART_RQR_TXFRQ             EQU (0x0010)                ; Transmit data flush Request

; *******************  Bit definition for USART_ISR register  *****************
USART_ISR_PE                EQU (0x00000001)            ; Parity Error
USART_ISR_FE                EQU (0x00000002)            ; Framing Error
USART_ISR_NE                EQU (0x00000004)            ; Noise detected Flag
USART_ISR_ORE               EQU (0x00000008)            ; OverRun Error
USART_ISR_IDLE              EQU (0x00000010)            ; IDLE line detected
USART_ISR_RXNE              EQU (0x00000020)            ; Read Data Register Not Empty
USART_ISR_TC                EQU (0x00000040)            ; Transmission Complete
USART_ISR_TXE               EQU (0x00000080)            ; Transmit Data Register Empty
USART_ISR_LBDF              EQU (0x00000100)            ; LIN Break Detection Flag
USART_ISR_CTSIF             EQU (0x00000200)            ; CTS interrupt flag
USART_ISR_CTS               EQU (0x00000400)            ; CTS flag
USART_ISR_RTOF              EQU (0x00000800)            ; Receiver Time Out
USART_ISR_EOBF              EQU (0x00001000)            ; End Of Block Flag
USART_ISR_ABRE              EQU (0x00004000)            ; Auto-Baud Rate Error
USART_ISR_ABRF              EQU (0x00008000)            ; Auto-Baud Rate Flag
USART_ISR_BUSY              EQU (0x00010000)            ; Busy Flag
USART_ISR_CMF               EQU (0x00020000)            ; Character Match Flag
USART_ISR_SBKF              EQU (0x00040000)            ; Send Break Flag
USART_ISR_RWU               EQU (0x00080000)            ; Receive Wake Up from mute mode Flag
USART_ISR_WUF               EQU (0x00100000)            ; Wake Up from stop mode Flag
USART_ISR_TEACK             EQU (0x00200000)            ; Transmit Enable Acknowledge Flag
USART_ISR_REACK             EQU (0x00400000)            ; Receive Enable Acknowledge Flag

; *******************  Bit definition for USART_ICR register  *****************
USART_ICR_PECF              EQU (0x00000001)            ; Parity Error Clear Flag
USART_ICR_FECF              EQU (0x00000002)            ; Framing Error Clear Flag
USART_ICR_NCF               EQU (0x00000004)            ; Noise detected Clear Flag
USART_ICR_ORECF             EQU (0x00000008)            ; OverRun Error Clear Flag
USART_ICR_IDLECF            EQU (0x00000010)            ; IDLE line detected Clear Flag
USART_ICR_TCCF              EQU (0x00000040)            ; Transmission Complete Clear Flag
USART_ICR_LBDCF             EQU (0x00000100)            ; LIN Break Detection Clear Flag
USART_ICR_CTSCF             EQU (0x00000200)            ; CTS Interrupt Clear Flag
USART_ICR_RTOCF             EQU (0x00000800)            ; Receiver Time Out Clear Flag
USART_ICR_EOBCF             EQU (0x00001000)            ; End Of Block Clear Flag
USART_ICR_CMCF              EQU (0x00020000)            ; Character Match Clear Flag
USART_ICR_WUCF              EQU (0x00100000)            ; Wake Up from stop mode Clear Flag

; *******************  Bit definition for USART_RDR register  *****************
USART_RDR_RDR               EQU (0x01FF)                ; RDR[8:0] bits (Receive Data value)

; *******************  Bit definition for USART_TDR register  *****************
USART_TDR_TDR               EQU (0x01FF)                ; TDR[8:0] bits (Transmit Data value)

; *****************************************************************************
;
;           Single Wire Protocol Master Interface (SWPMI)
;
; *****************************************************************************

; *******************  Bit definition for SWPMI_CR register   *******************
SWPMI_CR_RXDMA              EQU (0x00000001)        ;Reception DMA enable
SWPMI_CR_TXDMA              EQU (0x00000002)        ;Transmission DMA enable
SWPMI_CR_RXMODE             EQU (0x00000004)        ;Reception buffering mode
SWPMI_CR_TXMODE             EQU (0x00000008)        ;Transmission buffering mode
SWPMI_CR_LPBK               EQU (0x00000010)        ;Loopback mode enable
SWPMI_CR_SWPACT             EQU (0x00000020)        ;Single wire protocol master interface activate
SWPMI_CR_DEACT              EQU (0x00000400)        ;Single wire protocol master interface deactivate

; *******************  Bit definition for SWPMI_BRR register  *******************
SWPMI_BRR_BR                EQU (0x0000003F)        ;BR[5:0] bits (Bitrate prescaler)

; *******************  Bit definition for SWPMI_ISR register  *******************
SWPMI_ISR_RXBFF             EQU (0x00000001)        ;Receive buffer full flag
SWPMI_ISR_TXBEF             EQU (0x00000002)        ;Transmit buffer empty flag
SWPMI_ISR_RXBERF            EQU (0x00000004)        ;Receive CRC error flag
SWPMI_ISR_RXOVRF            EQU (0x00000008)        ;Receive overrun error flag
SWPMI_ISR_TXUNRF            EQU (0x00000010)        ;Transmit underrun error flag
SWPMI_ISR_RXNE              EQU (0x00000020)        ;Receive data register not empty
SWPMI_ISR_TXE               EQU (0x00000040)        ;Transmit data register empty
SWPMI_ISR_TCF               EQU (0x00000080)        ;Transfer complete flag
SWPMI_ISR_SRF               EQU (0x00000100)        ;Slave resume flag
SWPMI_ISR_SUSP              EQU (0x00000200)        ;SUSPEND flag
SWPMI_ISR_DEACTF            EQU (0x00000400)        ;DEACTIVATED flag

; *******************  Bit definition for SWPMI_ICR register  *******************
SWPMI_ICR_CRXBFF            EQU (0x00000001)        ;Clear receive buffer full flag
SWPMI_ICR_CTXBEF            EQU (0x00000002)        ;Clear transmit buffer empty flag
SWPMI_ICR_CRXBERF           EQU (0x00000004)        ;Clear receive CRC error flag
SWPMI_ICR_CRXOVRF           EQU (0x00000008)        ;Clear receive overrun error flag
SWPMI_ICR_CTXUNRF           EQU (0x00000010)        ;Clear transmit underrun error flag
SWPMI_ICR_CTCF              EQU (0x00000080)        ;Clear transfer complete flag
SWPMI_ICR_CSRF              EQU (0x00000100)        ;Clear slave resume flag

; *******************  Bit definition for SWPMI_IER register  *******************
SWPMI_IER_SRIE              EQU (0x00000100)        ;Slave resume interrupt enable
SWPMI_IER_TCIE              EQU (0x00000080)        ;Transmit complete interrupt enable
SWPMI_IER_TIE               EQU (0x00000040)        ;Transmit interrupt enable
SWPMI_IER_RIE               EQU (0x00000020)        ;Receive interrupt enable
SWPMI_IER_TXUNRIE           EQU (0x00000010)        ;Transmit underrun error interrupt enable
SWPMI_IER_RXOVRIE           EQU (0x00000008)        ;Receive overrun error interrupt enable
SWPMI_IER_RXBERIE           EQU (0x00000004)        ;Receive CRC error interrupt enable
SWPMI_IER_TXBEIE            EQU (0x00000002)        ;Transmit buffer empty interrupt enable
SWPMI_IER_RXBFIE            EQU (0x00000001)        ;Receive buffer full interrupt enable

; *******************  Bit definition for SWPMI_RFL register  *******************
SWPMI_RFL_RFL               EQU (0x0000001F)        ;RFL[4:0] bits (Receive Frame length)
SWPMI_RFL_RFL_0_1           EQU (0x00000003)        ;RFL[1:0] bits (number of relevant bytes for the last SWPMI_RDR register read.)

; *******************  Bit definition for SWPMI_TDR register  *******************
SWPMI_TDR_TD                EQU (0xFFFFFFFF)        ;Transmit Data Register

; *******************  Bit definition for SWPMI_RDR register  *******************
SWPMI_RDR_RD                EQU (0xFFFFFFFF)        ;Receive Data Register

; *******************  Bit definition for SWPMI_OR register  *******************
SWPMI_OR_TBYP               EQU (0x00000001)        ;SWP Transceiver Bypass
SWPMI_OR_CLASS              EQU (0x00000002)        ;SWP Voltage Class selection

; *****************************************************************************
;
;                                 VREFBUF
;
; *****************************************************************************
; *******************  Bit definition for VREFBUF_CSR register  ***************
VREFBUF_CSR_ENVR            EQU (0x00000001)        ;Voltage reference buffer enable
VREFBUF_CSR_HIZ             EQU (0x00000002)        ;High impedance mode
VREFBUF_CSR_VRS             EQU (0x00000004)        ;Voltage reference scale
VREFBUF_CSR_VRR             EQU (0x00000008)        ;Voltage reference buffer ready

; *******************  Bit definition for VREFBUF_CCR register  *****************
VREFBUF_CCR_TRIM            EQU (0x0000003F)        ;TRIM[5:0] bits (Trimming code)

; *****************************************************************************
;
;                            Window WATCHDOG
;
; *****************************************************************************
; *******************  Bit definition for WWDG_CR register  *******************
WWDG_CR_T                   EQU (0x0000007F)        ;T[6:0] bits (7-Bit counter (MSB to LSB))
WWDG_CR_T_0                 EQU (0x00000001)        ;Bit 0
WWDG_CR_T_1                 EQU (0x00000002)        ;Bit 1
WWDG_CR_T_2                 EQU (0x00000004)        ;Bit 2
WWDG_CR_T_3                 EQU (0x00000008)        ;Bit 3
WWDG_CR_T_4                 EQU (0x00000010)        ;Bit 4
WWDG_CR_T_5                 EQU (0x00000020)        ;Bit 5
WWDG_CR_T_6                 EQU (0x00000040)        ;Bit 6

WWDG_CR_WDGA                EQU (0x00000080)        ;Activation bit

; *******************  Bit definition for WWDG_CFR register  ******************
WWDG_CFR_W                  EQU (0x0000007F)        ;W[6:0] bits (7-bit window value)
WWDG_CFR_W_0                EQU (0x00000001)        ;Bit 0
WWDG_CFR_W_1                EQU (0x00000002)        ;Bit 1
WWDG_CFR_W_2                EQU (0x00000004)        ;Bit 2
WWDG_CFR_W_3                EQU (0x00000008)        ;Bit 3
WWDG_CFR_W_4                EQU (0x00000010)        ;Bit 4
WWDG_CFR_W_5                EQU (0x00000020)        ;Bit 5
WWDG_CFR_W_6                EQU (0x00000040)        ;Bit 6

WWDG_CFR_WDGTB              EQU (0x00000180)        ;WDGTB[1:0] bits (Timer Base)
WWDG_CFR_WDGTB_0            EQU (0x00000080)        ;Bit 0
WWDG_CFR_WDGTB_1            EQU (0x00000100)        ;Bit 1

WWDG_CFR_EWI                EQU (0x00000200)        ;Early Wakeup Interrupt

; *******************  Bit definition for WWDG_SR register  *******************
WWDG_SR_EWIF                EQU (0x00000001)        ;Early Wakeup Interrupt Flag


; *****************************************************************************
;
;                                 Debug MCU
;
; *****************************************************************************
; ********************  Bit definition for DBGMCU_IDCODE register  ************
DBGMCU_IDCODE_DEV_ID        EQU (0x00000FFF)
DBGMCU_IDCODE_REV_ID        EQU (0xFFFF0000)

; ********************  Bit definition for DBGMCU_CR register  ****************
DBGMCU_CR_DBG_SLEEP         EQU (0x00000001)
DBGMCU_CR_DBG_STOP          EQU (0x00000002)
DBGMCU_CR_DBG_STANDBY       EQU (0x00000004)
DBGMCU_CR_TRACE_IOEN        EQU (0x00000020)

DBGMCU_CR_TRACE_MODE        EQU (0x000000C0)
DBGMCU_CR_TRACE_MODE_0      EQU (0x00000040);Bit 0
DBGMCU_CR_TRACE_MODE_1      EQU (0x00000080);Bit 1

; ********************  Bit definition for DBGMCU_APB1FZR1 register  **********
DBGMCU_APB1FZR1_DBG_TIM2_STOP       EQU (0x00000001)
DBGMCU_APB1FZR1_DBG_TIM3_STOP       EQU (0x00000002)
DBGMCU_APB1FZR1_DBG_TIM4_STOP       EQU (0x00000004)
DBGMCU_APB1FZR1_DBG_TIM5_STOP       EQU (0x00000008)
DBGMCU_APB1FZR1_DBG_TIM6_STOP       EQU (0x00000010)
DBGMCU_APB1FZR1_DBG_TIM7_STOP       EQU (0x00000020)
DBGMCU_APB1FZR1_DBG_RTC_STOP        EQU (0x00000400)
DBGMCU_APB1FZR1_DBG_WWDG_STOP       EQU (0x00000800)
DBGMCU_APB1FZR1_DBG_IWDG_STOP       EQU (0x00001000)
DBGMCU_APB1FZR1_DBG_I2C1_STOP       EQU (0x00200000)
DBGMCU_APB1FZR1_DBG_I2C2_STOP       EQU (0x00400000)
DBGMCU_APB1FZR1_DBG_I2C3_STOP       EQU (0x00800000)
DBGMCU_APB1FZR1_DBG_CAN_STOP        EQU (0x02000000)
DBGMCU_APB1FZR1_DBG_LPTIM1_STOP     EQU (0x80000000)

; ********************  Bit definition for DBGMCU_APB1FZR2 register  *********
DBGMCU_APB1FZR2_DBG_LPTIM2_STOP     EQU (0x00000020)

; ********************  Bit definition for DBGMCU_APB2FZ register  ***********
DBGMCU_APB2FZ_DBG_TIM1_STOP EQU (0x00000800)
DBGMCU_APB2FZ_DBG_TIM8_STOP EQU (0x00002000)
DBGMCU_APB2FZ_DBG_TIM15_STOP        EQU (0x00010000)
DBGMCU_APB2FZ_DBG_TIM16_STOP        EQU (0x00020000)
DBGMCU_APB2FZ_DBG_TIM17_STOP        EQU (0x00040000)

; *****************************************************************************
;
;                                       USB_OTG
;
; *****************************************************************************
; ********************  Bit definition for USB_OTG_GOTGCTL register  *******************
USB_OTG_GOTGCTL_SRQSCS          EQU (0x00000001)         ; Session request success
USB_OTG_GOTGCTL_SRQ             EQU (0x00000002)         ; Session request
USB_OTG_GOTGCTL_VBVALOEN        EQU (0x00000004)         ; VBUS valid override enable
USB_OTG_GOTGCTL_VBVALOVAL       EQU (0x00000008)         ; VBUS valid override value
USB_OTG_GOTGCTL_AVALOEN         EQU (0x00000010)         ; A-peripheral session valid override enable
USB_OTG_GOTGCTL_AVALOVAL        EQU (0x00000020)         ; A-peripheral session valid override value
USB_OTG_GOTGCTL_BVALOEN         EQU (0x00000040)         ; B-peripheral session valid override enable
USB_OTG_GOTGCTL_BVALOVAL        EQU (0x00000080)         ; B-peripheral session valid override value
USB_OTG_GOTGCTL_BSESVLD         EQU (0x00080000)         ;  B-session valid

; ********************  Bit definition for USB_OTG_HCFG register  *******************

USB_OTG_HCFG_FSLSPCS         EQU (0x00000003)            ; FS/LS PHY clock select
USB_OTG_HCFG_FSLSPCS_0       EQU (0x00000001)            ;Bit 0
USB_OTG_HCFG_FSLSPCS_1       EQU (0x00000002)            ;Bit 1
USB_OTG_HCFG_FSLSS           EQU (0x00000004)            ; FS- and LS-only support

; ********************  Bit definition for USB_OTG_DCFG register  *******************

USB_OTG_DCFG_DSPD            EQU (0x00000003)            ; Device speed
USB_OTG_DCFG_DSPD_0          EQU (0x00000001)            ;Bit 0
USB_OTG_DCFG_DSPD_1          EQU (0x00000002)            ;Bit 1
USB_OTG_DCFG_NZLSOHSK        EQU (0x00000004)            ; Nonzero-length status OUT handshake
USB_OTG_DCFG_DAD             EQU (0x000007F0)            ; Device address
USB_OTG_DCFG_DAD_0           EQU (0x00000010)            ;Bit 0
USB_OTG_DCFG_DAD_1           EQU (0x00000020)            ;Bit 1
USB_OTG_DCFG_DAD_2           EQU (0x00000040)            ;Bit 2
USB_OTG_DCFG_DAD_3           EQU (0x00000080)            ;Bit 3
USB_OTG_DCFG_DAD_4           EQU (0x00000100)            ;Bit 4
USB_OTG_DCFG_DAD_5           EQU (0x00000200)            ;Bit 5
USB_OTG_DCFG_DAD_6           EQU (0x00000400)            ;Bit 6
USB_OTG_DCFG_PFIVL           EQU (0x00001800)            ; Periodic (micro)frame interval
USB_OTG_DCFG_PFIVL_0         EQU (0x00000800)            ;Bit 0
USB_OTG_DCFG_PFIVL_1         EQU (0x00001000)            ;Bit 1
USB_OTG_DCFG_PERSCHIVL       EQU (0x03000000)            ; Periodic scheduling interval
USB_OTG_DCFG_PERSCHIVL_0     EQU (0x01000000)            ;Bit 0
USB_OTG_DCFG_PERSCHIVL_1     EQU (0x02000000)            ;Bit 1

; ********************  Bit definition for USB_OTG_PCGCR register  *******************
USB_OTG_PCGCR_STPPCLK         EQU (0x00000001)           ; Stop PHY clock
USB_OTG_PCGCR_GATEHCLK        EQU (0x00000002)           ; Gate HCLK
USB_OTG_PCGCR_PHYSUSP         EQU (0x00000010)           ; PHY suspended

; ********************  Bit definition for USB_OTG_GOTGINT register  *******************
USB_OTG_GOTGINT_SEDET           EQU (0x00000004)         ; Session end detected
USB_OTG_GOTGINT_SRSSCHG         EQU (0x00000100)         ; Session request success status change
USB_OTG_GOTGINT_HNSSCHG         EQU (0x00000200)         ; Host negotiation success status change
USB_OTG_GOTGINT_HNGDET          EQU (0x00020000)         ; Host negotiation detected
USB_OTG_GOTGINT_ADTOCHG         EQU (0x00040000)         ; A-device timeout change
USB_OTG_GOTGINT_DBCDNE          EQU (0x00080000)         ; Debounce done

; ********************  Bit definition for USB_OTG_DCTL register  *******************
USB_OTG_DCTL_RWUSIG          EQU (0x00000001)            ; Remote wakeup signaling
USB_OTG_DCTL_SDIS            EQU (0x00000002)            ; Soft disconnect
USB_OTG_DCTL_GINSTS          EQU (0x00000004)            ; Global IN NAK status
USB_OTG_DCTL_GONSTS          EQU (0x00000008)            ; Global OUT NAK status

USB_OTG_DCTL_TCTL            EQU (0x00000070)            ; Test control
USB_OTG_DCTL_TCTL_0          EQU (0x00000010)            ;Bit 0
USB_OTG_DCTL_TCTL_1          EQU (0x00000020)            ;Bit 1
USB_OTG_DCTL_TCTL_2          EQU (0x00000040)            ;Bit 2
USB_OTG_DCTL_SGINAK          EQU (0x00000080)            ; Set global IN NAK
USB_OTG_DCTL_CGINAK          EQU (0x00000100)            ; Clear global IN NAK
USB_OTG_DCTL_SGONAK          EQU (0x00000200)            ; Set global OUT NAK
USB_OTG_DCTL_CGONAK          EQU (0x00000400)            ; Clear global OUT NAK
USB_OTG_DCTL_POPRGDNE        EQU (0x00000800)            ; Power-on programming done

; ********************  Bit definition for USB_OTG_HFIR register  *******************
USB_OTG_HFIR_FRIVL           EQU (0x0000FFFF)            ; Frame interval

; ********************  Bit definition for USB_OTG_HFNUM register  *******************
USB_OTG_HFNUM_FRNUM           EQU (0x0000FFFF)           ; Frame number
USB_OTG_HFNUM_FTREM           EQU (0xFFFF0000)           ; Frame time remaining

; ********************  Bit definition for USB_OTG_DSTS register  *******************
USB_OTG_DSTS_SUSPSTS         EQU (0x00000001)            ; Suspend status

USB_OTG_DSTS_ENUMSPD         EQU (0x00000006)            ; Enumerated speed
USB_OTG_DSTS_ENUMSPD_0       EQU (0x00000002)            ;Bit 0
USB_OTG_DSTS_ENUMSPD_1       EQU (0x00000004)            ;Bit 1
USB_OTG_DSTS_EERR            EQU (0x00000008)            ; Erratic error
USB_OTG_DSTS_FNSOF           EQU (0x003FFF00)            ; Frame number of the received SOF

; ********************  Bit definition for USB_OTG_GAHBCFG register  *******************
USB_OTG_GAHBCFG_GINT            EQU (0x00000001)         ; Global interrupt mask
USB_OTG_GAHBCFG_HBSTLEN         EQU (0x0000001E)         ; Burst length/type
USB_OTG_GAHBCFG_HBSTLEN_0       EQU (0x00000002)         ;Bit 0
USB_OTG_GAHBCFG_HBSTLEN_1       EQU (0x00000004)         ;Bit 1
USB_OTG_GAHBCFG_HBSTLEN_2       EQU (0x00000008)         ;Bit 2
USB_OTG_GAHBCFG_HBSTLEN_3       EQU (0x00000010)         ;Bit 3
USB_OTG_GAHBCFG_DMAEN           EQU (0x00000020)         ; DMA enable
USB_OTG_GAHBCFG_TXFELVL         EQU (0x00000080)         ; TxFIFO empty level
USB_OTG_GAHBCFG_PTXFELVL        EQU (0x00000100)         ; Periodic TxFIFO empty level

; ********************  Bit definition for USB_OTG_GUSBCFG register  *******************

USB_OTG_GUSBCFG_TOCAL           EQU (0x00000007)         ; FS timeout calibration
USB_OTG_GUSBCFG_TOCAL_0         EQU (0x00000001)         ;Bit 0
USB_OTG_GUSBCFG_TOCAL_1         EQU (0x00000002)         ;Bit 1
USB_OTG_GUSBCFG_TOCAL_2         EQU (0x00000004)         ;Bit 2
USB_OTG_GUSBCFG_PHYSEL          EQU (0x00000040)         ; USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select
USB_OTG_GUSBCFG_SRPCAP          EQU (0x00000100)         ; SRP-capable
USB_OTG_GUSBCFG_HNPCAP          EQU (0x00000200)         ; HNP-capable
USB_OTG_GUSBCFG_TRDT            EQU (0x00003C00)         ; USB turnaround time
USB_OTG_GUSBCFG_TRDT_0          EQU (0x00000400)         ;Bit 0
USB_OTG_GUSBCFG_TRDT_1          EQU (0x00000800)         ;Bit 1
USB_OTG_GUSBCFG_TRDT_2          EQU (0x00001000)         ;Bit 2
USB_OTG_GUSBCFG_TRDT_3          EQU (0x00002000)         ;Bit 3
USB_OTG_GUSBCFG_PHYLPCS         EQU (0x00008000)         ; PHY Low-power clock select
USB_OTG_GUSBCFG_ULPIFSLS        EQU (0x00020000)         ; ULPI FS/LS select
USB_OTG_GUSBCFG_ULPIAR          EQU (0x00040000)         ; ULPI Auto-resume
USB_OTG_GUSBCFG_ULPICSM         EQU (0x00080000)         ; ULPI Clock SuspendM
USB_OTG_GUSBCFG_ULPIEVBUSD      EQU (0x00100000)         ; ULPI External VBUS Drive
USB_OTG_GUSBCFG_ULPIEVBUSI      EQU (0x00200000)         ; ULPI external VBUS indicator
USB_OTG_GUSBCFG_TSDPS           EQU (0x00400000)         ; TermSel DLine pulsing selection
USB_OTG_GUSBCFG_PCCI            EQU (0x00800000)         ; Indicator complement
USB_OTG_GUSBCFG_PTCI            EQU (0x01000000)         ; Indicator pass through
USB_OTG_GUSBCFG_ULPIIPD         EQU (0x02000000)         ; ULPI interface protect disable
USB_OTG_GUSBCFG_FHMOD           EQU (0x20000000)         ; Forced host mode
USB_OTG_GUSBCFG_FDMOD           EQU (0x40000000)         ; Forced peripheral mode
USB_OTG_GUSBCFG_CTXPKT          EQU (0x80000000)         ; Corrupt Tx packet

; ********************  Bit definition for USB_OTG_GRSTCTL register  *******************
USB_OTG_GRSTCTL_CSRST           EQU (0x00000001)         ; Core soft reset
USB_OTG_GRSTCTL_HSRST           EQU (0x00000002)         ; HCLK soft reset
USB_OTG_GRSTCTL_FCRST           EQU (0x00000004)         ; Host frame counter reset
USB_OTG_GRSTCTL_RXFFLSH         EQU (0x00000010)         ; RxFIFO flush
USB_OTG_GRSTCTL_TXFFLSH         EQU (0x00000020)         ; TxFIFO flush
USB_OTG_GRSTCTL_TXFNUM          EQU (0x000007C0)         ; TxFIFO number
USB_OTG_GRSTCTL_TXFNUM_0        EQU (0x00000040)         ;Bit 0
USB_OTG_GRSTCTL_TXFNUM_1        EQU (0x00000080)         ;Bit 1
USB_OTG_GRSTCTL_TXFNUM_2        EQU (0x00000100)         ;Bit 2
USB_OTG_GRSTCTL_TXFNUM_3        EQU (0x00000200)         ;Bit 3
USB_OTG_GRSTCTL_TXFNUM_4        EQU (0x00000400)         ;Bit 4
USB_OTG_GRSTCTL_DMAREQ          EQU (0x40000000)         ; DMA request signal
USB_OTG_GRSTCTL_AHBIDL          EQU (0x80000000)         ; AHB master idle

; ********************  Bit definition for USB_OTG_DIEPMSK register  *******************
USB_OTG_DIEPMSK_XFRCM           EQU (0x00000001)         ; Transfer completed interrupt mask
USB_OTG_DIEPMSK_EPDM            EQU (0x00000002)         ; Endpoint disabled interrupt mask
USB_OTG_DIEPMSK_TOM             EQU (0x00000008)         ; Timeout condition mask (nonisochronous endpoints)
USB_OTG_DIEPMSK_ITTXFEMSK       EQU (0x00000010)         ; IN token received when TxFIFO empty mask
USB_OTG_DIEPMSK_INEPNMM         EQU (0x00000020)         ; IN token received with EP mismatch mask
USB_OTG_DIEPMSK_INEPNEM         EQU (0x00000040)         ; IN endpoint NAK effective mask
USB_OTG_DIEPMSK_TXFURM          EQU (0x00000100)         ; FIFO underrun mask
USB_OTG_DIEPMSK_BIM             EQU (0x00000200)         ; BNA interrupt mask

; ********************  Bit definition for USB_OTG_HPTXSTS register  *******************
USB_OTG_HPTXSTS_PTXFSAVL        EQU (0x0000FFFF)         ; Periodic transmit data FIFO space available
USB_OTG_HPTXSTS_PTXQSAV         EQU (0x00FF0000)         ; Periodic transmit request queue space available
USB_OTG_HPTXSTS_PTXQSAV_0       EQU (0x00010000)         ;Bit 0
USB_OTG_HPTXSTS_PTXQSAV_1       EQU (0x00020000)         ;Bit 1
USB_OTG_HPTXSTS_PTXQSAV_2       EQU (0x00040000)         ;Bit 2
USB_OTG_HPTXSTS_PTXQSAV_3       EQU (0x00080000)         ;Bit 3
USB_OTG_HPTXSTS_PTXQSAV_4       EQU (0x00100000)         ;Bit 4
USB_OTG_HPTXSTS_PTXQSAV_5       EQU (0x00200000)         ;Bit 5
USB_OTG_HPTXSTS_PTXQSAV_6       EQU (0x00400000)         ;Bit 6
USB_OTG_HPTXSTS_PTXQSAV_7       EQU (0x00800000)         ;Bit 7

USB_OTG_HPTXSTS_PTXQTOP         EQU (0xFF000000)         ; Top of the periodic transmit request queue
USB_OTG_HPTXSTS_PTXQTOP_0       EQU (0x01000000)         ;Bit 0
USB_OTG_HPTXSTS_PTXQTOP_1       EQU (0x02000000)         ;Bit 1
USB_OTG_HPTXSTS_PTXQTOP_2       EQU (0x04000000)         ;Bit 2
USB_OTG_HPTXSTS_PTXQTOP_3       EQU (0x08000000)         ;Bit 3
USB_OTG_HPTXSTS_PTXQTOP_4       EQU (0x10000000)         ;Bit 4
USB_OTG_HPTXSTS_PTXQTOP_5       EQU (0x20000000)         ;Bit 5
USB_OTG_HPTXSTS_PTXQTOP_6       EQU (0x40000000)         ;Bit 6
USB_OTG_HPTXSTS_PTXQTOP_7       EQU (0x80000000)         ;Bit 7

; ********************  Bit definition for USB_OTG_HAINT register  *******************
USB_OTG_HAINT_HAINT           EQU (0x0000FFFF)           ; Channel interrupts

; ********************  Bit definition for USB_OTG_DOEPMSK register  *******************
USB_OTG_DOEPMSK_XFRCM           EQU (0x00000001)         ; Transfer completed interrupt mask
USB_OTG_DOEPMSK_EPDM            EQU (0x00000002)         ; Endpoint disabled interrupt mask
USB_OTG_DOEPMSK_STUPM           EQU (0x00000008)         ; SETUP phase done mask
USB_OTG_DOEPMSK_OTEPDM          EQU (0x00000010)         ; OUT token received when endpoint disabled mask
USB_OTG_DOEPMSK_B2BSTUP         EQU (0x00000040)         ; Back-to-back SETUP packets received mask
USB_OTG_DOEPMSK_OPEM            EQU (0x00000100)         ; OUT packet error mask
USB_OTG_DOEPMSK_BOIM            EQU (0x00000200)         ; BNA interrupt mask

; ********************  Bit definition for USB_OTG_GINTSTS register  *******************
USB_OTG_GINTSTS_CMOD            EQU (0x00000001)         ; Current mode of operation
USB_OTG_GINTSTS_MMIS            EQU (0x00000002)         ; Mode mismatch interrupt
USB_OTG_GINTSTS_OTGINT          EQU (0x00000004)         ; OTG interrupt
USB_OTG_GINTSTS_SOF             EQU (0x00000008)         ; Start of frame
USB_OTG_GINTSTS_RXFLVL          EQU (0x00000010)         ; RxFIFO nonempty
USB_OTG_GINTSTS_NPTXFE          EQU (0x00000020)         ; Nonperiodic TxFIFO empty
USB_OTG_GINTSTS_GINAKEFF        EQU (0x00000040)         ; Global IN nonperiodic NAK effective
USB_OTG_GINTSTS_BOUTNAKEFF      EQU (0x00000080)         ; Global OUT NAK effective
USB_OTG_GINTSTS_ESUSP           EQU (0x00000400)         ; Early suspend
USB_OTG_GINTSTS_USBSUSP         EQU (0x00000800)         ; USB suspend
USB_OTG_GINTSTS_USBRST          EQU (0x00001000)         ; USB reset
USB_OTG_GINTSTS_ENUMDNE         EQU (0x00002000)         ; Enumeration done
USB_OTG_GINTSTS_ISOODRP         EQU (0x00004000)         ; Isochronous OUT packet dropped interrupt
USB_OTG_GINTSTS_EOPF            EQU (0x00008000)         ; End of periodic frame interrupt
USB_OTG_GINTSTS_IEPINT          EQU (0x00040000)         ; IN endpoint interrupt
USB_OTG_GINTSTS_OEPINT          EQU (0x00080000)         ; OUT endpoint interrupt
USB_OTG_GINTSTS_IISOIXFR        EQU (0x00100000)         ; Incomplete isochronous IN transfer
USB_OTG_GINTSTS_PXFR_INCOMPISOOUT       EQU (0x00200000)         ; Incomplete periodic transfer
USB_OTG_GINTSTS_DATAFSUSP       EQU (0x00400000)         ; Data fetch suspended
USB_OTG_GINTSTS_HPRTINT         EQU (0x01000000)         ; Host port interrupt
USB_OTG_GINTSTS_HCINT           EQU (0x02000000)         ; Host channels interrupt
USB_OTG_GINTSTS_PTXFE           EQU (0x04000000)         ; Periodic TxFIFO empty
USB_OTG_GINTSTS_LPMINT           EQU (0x08000000)        ; LPM interrupt
USB_OTG_GINTSTS_CIDSCHG         EQU (0x10000000)         ; Connector ID status change
USB_OTG_GINTSTS_DISCINT         EQU (0x20000000)         ; Disconnect detected interrupt
USB_OTG_GINTSTS_SRQINT          EQU (0x40000000)         ; Session request/new session detected interrupt
USB_OTG_GINTSTS_WKUINT          EQU (0x80000000)         ; Resume/remote wakeup detected interrupt

; ********************  Bit definition for USB_OTG_GINTMSK register  *******************

USB_OTG_GINTMSK_MMISM           EQU (0x00000002)         ; Mode mismatch interrupt mask
USB_OTG_GINTMSK_OTGINT          EQU (0x00000004)         ; OTG interrupt mask
USB_OTG_GINTMSK_SOFM            EQU (0x00000008)         ; Start of frame mask
USB_OTG_GINTMSK_RXFLVLM         EQU (0x00000010)         ; Receive FIFO nonempty mask
USB_OTG_GINTMSK_NPTXFEM         EQU (0x00000020)         ; Nonperiodic TxFIFO empty mask
USB_OTG_GINTMSK_GINAKEFFM       EQU (0x00000040)         ; Global nonperiodic IN NAK effective mask
USB_OTG_GINTMSK_GONAKEFFM       EQU (0x00000080)         ; Global OUT NAK effective mask
USB_OTG_GINTMSK_ESUSPM          EQU (0x00000400)         ; Early suspend mask
USB_OTG_GINTMSK_USBSUSPM        EQU (0x00000800)         ; USB suspend mask
USB_OTG_GINTMSK_USBRST          EQU (0x00001000)         ; USB reset mask
USB_OTG_GINTMSK_ENUMDNEM        EQU (0x00002000)         ; Enumeration done mask
USB_OTG_GINTMSK_ISOODRPM        EQU (0x00004000)         ; Isochronous OUT packet dropped interrupt mask
USB_OTG_GINTMSK_EOPFM           EQU (0x00008000)         ; End of periodic frame interrupt mask
USB_OTG_GINTMSK_EPMISM          EQU (0x00020000)         ; Endpoint mismatch interrupt mask
USB_OTG_GINTMSK_IEPINT          EQU (0x00040000)         ; IN endpoints interrupt mask
USB_OTG_GINTMSK_OEPINT          EQU (0x00080000)         ; OUT endpoints interrupt mask
USB_OTG_GINTMSK_IISOIXFRM       EQU (0x00100000)         ; Incomplete isochronous IN transfer mask
USB_OTG_GINTMSK_PXFRM_IISOOXFRM EQU (0x00200000)         ; Incomplete periodic transfer mask
USB_OTG_GINTMSK_FSUSPM          EQU (0x00400000)         ; Data fetch suspended mask
USB_OTG_GINTMSK_PRTIM           EQU (0x01000000)         ; Host port interrupt mask
USB_OTG_GINTMSK_HCIM            EQU (0x02000000)         ; Host channels interrupt mask
USB_OTG_GINTMSK_PTXFEM          EQU (0x04000000)         ; Periodic TxFIFO empty mask
USB_OTG_GINTMSK_LPMINTM         EQU (0x08000000)         ; LPM interrupt Mask
USB_OTG_GINTMSK_CIDSCHGM        EQU (0x10000000)         ; Connector ID status change mask
USB_OTG_GINTMSK_DISCINT         EQU (0x20000000)         ; Disconnect detected interrupt mask
USB_OTG_GINTMSK_SRQIM           EQU (0x40000000)         ; Session request/new session detected interrupt mask
USB_OTG_GINTMSK_WUIM            EQU (0x80000000)         ; Resume/remote wakeup detected interrupt mask

; ********************  Bit definition for USB_OTG_DAINT register  *******************
USB_OTG_DAINT_IEPINT          EQU (0x0000FFFF)            ; IN endpoint interrupt bits
USB_OTG_DAINT_OEPINT          EQU (0xFFFF0000)            ; OUT endpoint interrupt bits

; ********************  Bit definition for USB_OTG_HAINTMSK register  *******************
USB_OTG_HAINTMSK_HAINTM          EQU (0x0000FFFF)        ; Channel interrupt mask

; ********************  Bit definition for USB_OTG_GRXSTSP register  *******************
USB_OTG_GRXSTSP_EPNUM            EQU (0x0000000F)        ; IN EP interrupt mask bits
USB_OTG_GRXSTSP_BCNT             EQU (0x00007FF0)        ; OUT EP interrupt mask bits
USB_OTG_GRXSTSP_DPID             EQU (0x00018000)        ; OUT EP interrupt mask bits
USB_OTG_GRXSTSP_PKTSTS           EQU (0x001E0000)        ; OUT EP interrupt mask bits

; ********************  Bit definition for USB_OTG_DAINTMSK register  *******************
USB_OTG_DAINTMSK_IEPM            EQU (0x0000FFFF)        ; IN EP interrupt mask bits
USB_OTG_DAINTMSK_OEPM            EQU (0xFFFF0000)        ; OUT EP interrupt mask bits

; ********************  Bit definition for OTG register  *******************

USB_OTG_CHNUM           EQU (0x0000000F)                 ; Channel number
USB_OTG_CHNUM_0         EQU (0x00000001)                 ;Bit 0
USB_OTG_CHNUM_1         EQU (0x00000002)                 ;Bit 1
USB_OTG_CHNUM_2         EQU (0x00000004)                 ;Bit 2
USB_OTG_CHNUM_3         EQU (0x00000008)                 ;Bit 3
USB_OTG_BCNT            EQU (0x00007FF0)                 ; Byte count
USB_OTG_DPID            EQU (0x00018000)                 ; Data PID
USB_OTG_DPID_0          EQU (0x00008000)                 ;Bit 0
USB_OTG_DPID_1          EQU (0x00010000)                 ;Bit 1
USB_OTG_PKTSTS          EQU (0x001E0000)                 ; Packet status
USB_OTG_PKTSTS_0        EQU (0x00020000)                 ;Bit 0
USB_OTG_PKTSTS_1        EQU (0x00040000)                 ;Bit 1
USB_OTG_PKTSTS_2        EQU (0x00080000)                 ;Bit 2
USB_OTG_PKTSTS_3        EQU (0x00100000)                 ;Bit 3
USB_OTG_EPNUM           EQU (0x0000000F)                 ; Endpoint number
USB_OTG_EPNUM_0         EQU (0x00000001)                 ;Bit 0
USB_OTG_EPNUM_1         EQU (0x00000002)                 ;Bit 1
USB_OTG_EPNUM_2         EQU (0x00000004)                 ;Bit 2
USB_OTG_EPNUM_3         EQU (0x00000008)                 ;Bit 3
USB_OTG_FRMNUM          EQU (0x01E00000)                 ; Frame number
USB_OTG_FRMNUM_0        EQU (0x00200000)                 ;Bit 0
USB_OTG_FRMNUM_1        EQU (0x00400000)                 ;Bit 1
USB_OTG_FRMNUM_2        EQU (0x00800000)                 ;Bit 2
USB_OTG_FRMNUM_3        EQU (0x01000000)                 ;Bit 3

; ********************  Bit definition for USB_OTG_GRXFSIZ register  *******************
USB_OTG_GRXFSIZ_RXFD    EQU (0x0000FFFF)         ; RxFIFO depth

; ********************  Bit definition for USB_OTG_DVBUSDIS register  *******************
USB_OTG_DVBUSDIS_VBUSDT EQU (0x0000FFFF)        ; Device VBUS discharge time

; ********************  Bit definition for OTG register  *******************
USB_OTG_NPTXFSA         EQU (0x0000FFFF)                 ; Nonperiodic transmit RAM start address
USB_OTG_NPTXFD          EQU (0xFFFF0000)                 ; Nonperiodic TxFIFO depth
USB_OTG_TX0FSA          EQU (0x0000FFFF)                 ; Endpoint 0 transmit RAM start address
USB_OTG_TX0FD           EQU (0xFFFF0000)                 ; Endpoint 0 TxFIFO depth

; ********************  Bit definition for USB_OTG_DVBUSPULSE register  *******************
USB_OTG_DVBUSPULSE_DVBUSP       EQU (0x00000FFF)      ; Device VBUS pulsing time

; ********************  Bit definition for USB_OTG_GNPTXSTS register  *******************
USB_OTG_GNPTXSTS_NPTXFSAV       EQU (0x0000FFFF)        ; Nonperiodic TxFIFO space available

USB_OTG_GNPTXSTS_NPTQXSAV        EQU (0x00FF0000)        ; Nonperiodic transmit request queue space available
USB_OTG_GNPTXSTS_NPTQXSAV_0      EQU (0x00010000)        ;Bit 0
USB_OTG_GNPTXSTS_NPTQXSAV_1      EQU (0x00020000)        ;Bit 1
USB_OTG_GNPTXSTS_NPTQXSAV_2      EQU (0x00040000)        ;Bit 2
USB_OTG_GNPTXSTS_NPTQXSAV_3      EQU (0x00080000)        ;Bit 3
USB_OTG_GNPTXSTS_NPTQXSAV_4      EQU (0x00100000)        ;Bit 4
USB_OTG_GNPTXSTS_NPTQXSAV_5      EQU (0x00200000)        ;Bit 5
USB_OTG_GNPTXSTS_NPTQXSAV_6      EQU (0x00400000)        ;Bit 6
USB_OTG_GNPTXSTS_NPTQXSAV_7      EQU (0x00800000)        ;Bit 7

USB_OTG_GNPTXSTS_NPTXQTOP        EQU (0x7F000000)        ; Top of the nonperiodic transmit request queue
USB_OTG_GNPTXSTS_NPTXQTOP_0      EQU (0x01000000)        ;Bit 0
USB_OTG_GNPTXSTS_NPTXQTOP_1      EQU (0x02000000)        ;Bit 1
USB_OTG_GNPTXSTS_NPTXQTOP_2      EQU (0x04000000)        ;Bit 2
USB_OTG_GNPTXSTS_NPTXQTOP_3      EQU (0x08000000)        ;Bit 3
USB_OTG_GNPTXSTS_NPTXQTOP_4      EQU (0x10000000)        ;Bit 4
USB_OTG_GNPTXSTS_NPTXQTOP_5      EQU (0x20000000)        ;Bit 5
USB_OTG_GNPTXSTS_NPTXQTOP_6      EQU (0x40000000)        ;Bit 6

; ********************  Bit definition for USB_OTG_DTHRCTL register  **************
USB_OTG_DTHRCTL_NONISOTHREN     EQU (0x00000001)         ; Nonisochronous IN endpoints threshold enable
USB_OTG_DTHRCTL_ISOTHREN        EQU (0x00000002)         ; ISO IN endpoint threshold enable

USB_OTG_DTHRCTL_TXTHRLEN        EQU (0x000007FC)         ; Transmit threshold length
USB_OTG_DTHRCTL_TXTHRLEN_0      EQU (0x00000004)         ;Bit 0
USB_OTG_DTHRCTL_TXTHRLEN_1      EQU (0x00000008)         ;Bit 1
USB_OTG_DTHRCTL_TXTHRLEN_2      EQU (0x00000010)         ;Bit 2
USB_OTG_DTHRCTL_TXTHRLEN_3      EQU (0x00000020)         ;Bit 3
USB_OTG_DTHRCTL_TXTHRLEN_4      EQU (0x00000040)         ;Bit 4
USB_OTG_DTHRCTL_TXTHRLEN_5      EQU (0x00000080)         ;Bit 5
USB_OTG_DTHRCTL_TXTHRLEN_6      EQU (0x00000100)         ;Bit 6
USB_OTG_DTHRCTL_TXTHRLEN_7      EQU (0x00000200)         ;Bit 7
USB_OTG_DTHRCTL_TXTHRLEN_8      EQU (0x00000400)         ;Bit 8
USB_OTG_DTHRCTL_RXTHREN EQU (0x00010000)         ; Receive threshold enable

USB_OTG_DTHRCTL_RXTHRLEN        EQU (0x03FE0000)         ; Receive threshold length
USB_OTG_DTHRCTL_RXTHRLEN_0      EQU (0x00020000)         ;Bit 0
USB_OTG_DTHRCTL_RXTHRLEN_1      EQU (0x00040000)         ;Bit 1
USB_OTG_DTHRCTL_RXTHRLEN_2      EQU (0x00080000)         ;Bit 2
USB_OTG_DTHRCTL_RXTHRLEN_3      EQU (0x00100000)         ;Bit 3
USB_OTG_DTHRCTL_RXTHRLEN_4      EQU (0x00200000)         ;Bit 4
USB_OTG_DTHRCTL_RXTHRLEN_5      EQU (0x00400000)         ;Bit 5
USB_OTG_DTHRCTL_RXTHRLEN_6      EQU (0x00800000)         ;Bit 6
USB_OTG_DTHRCTL_RXTHRLEN_7      EQU (0x01000000)         ;Bit 7
USB_OTG_DTHRCTL_RXTHRLEN_8      EQU (0x02000000)         ;Bit 8
USB_OTG_DTHRCTL_ARPEN   EQU (0x08000000)         ; Arbiter parking enable

; ********************  Bit definition for USB_OTG_DIEPEMPMSK register  **************
USB_OTG_DIEPEMPMSK_INEPTXFEM    EQU (0x0000FFFF)      ; IN EP Tx FIFO empty interrupt mask bits

; ********************  Bit definition for USB_OTG_DEACHINT register  *******************
USB_OTG_DEACHINT_IEP1INT        EQU (0x00000002)        ; IN endpoint 1interrupt bit
USB_OTG_DEACHINT_OEP1INT        EQU (0x00020000)        ; OUT endpoint 1 interrupt bit

; ********************  Bit definition for USB_OTG_GCCFG register  *******************
USB_OTG_GCCFG_DCDET  EQU (0x00000001)            ; Data contact detection (DCD) status
USB_OTG_GCCFG_PDET   EQU (0x00000002)            ; Primary detection (PD) status
USB_OTG_GCCFG_SDET   EQU (0x00000004)            ; Secondary detection (SD) status
USB_OTG_GCCFG_PS2DET EQU (0x00000008)            ; DM pull-up detection status
USB_OTG_GCCFG_PWRDWN EQU (0x00010000)            ; Power down
USB_OTG_GCCFG_BCDEN  EQU (0x00020000)            ; Battery charging detector (BCD) enable
USB_OTG_GCCFG_DCDEN  EQU (0x00040000)            ; Data contact detection (DCD) mode enable
USB_OTG_GCCFG_PDEN   EQU (0x00080000)            ; Primary detection (PD) mode enable
USB_OTG_GCCFG_SDEN   EQU (0x00100000)            ; Secondary detection (SD) mode enable
USB_OTG_GCCFG_VBDEN  EQU (0x00200000)            ; Secondary detection (SD) mode enable

; ********************  Bit definition for USB_OTG_GPWRDN) register  *******************
USB_OTG_GPWRDN_DISABLEVBUS   EQU (0x00000040)      ; Power down

; ********************  Bit definition for USB_OTG_DEACHINTMSK register  *******************
USB_OTG_DEACHINTMSK_IEP1INTM EQU (0x00000002)     ; IN Endpoint 1 interrupt mask bit
USB_OTG_DEACHINTMSK_OEP1INTM EQU (0x00020000)     ; OUT Endpoint 1 interrupt mask bit

; ********************  Bit definition for USB_OTG_CID register  *******************
USB_OTG_CID_PRODUCT_ID      EQU (0xFFFFFFFF)             ; Product ID field


; ********************  Bit definition for USB_OTG_GHWCFG3 register  *******************
USB_OTG_GHWCFG3_LPMMode     EQU (0x00004000)           ; LPM mode specified for Mode of Operation

; ********************  Bit definition for USB_OTG_GLPMCFG register  *******************
USB_OTG_GLPMCFG_ENBESL      EQU (0x10000000)    ; Enable best effort service latency
USB_OTG_GLPMCFG_LPMRCNTSTS  EQU (0x0E000000)    ; LPM retry count status
USB_OTG_GLPMCFG_SNDLPM      EQU (0x01000000)    ; Send LPM transaction
USB_OTG_GLPMCFG_LPMRCNT     EQU (0x00E00000)    ; LPM retry count
USB_OTG_GLPMCFG_LPMCHIDX    EQU (0x001E0000)    ; LPMCHIDX:
USB_OTG_GLPMCFG_L1ResumeOK  EQU (0x00010000)    ; Sleep State Resume OK
USB_OTG_GLPMCFG_SLPSTS      EQU (0x00008000)    ; Port sleep status
USB_OTG_GLPMCFG_LPMRSP      EQU (0x00006000)    ; LPM response
USB_OTG_GLPMCFG_L1DSEN      EQU (0x00001000)    ; L1 deep sleep enable
USB_OTG_GLPMCFG_BESLTHRS    EQU (0x00000F00)    ; BESL threshold
USB_OTG_GLPMCFG_L1SSEN      EQU (0x00000080)    ; L1 shallow sleep enable
USB_OTG_GLPMCFG_REMWAKE     EQU (0x00000040)    ; bRemoteWake value received with last ACKed LPM Token
USB_OTG_GLPMCFG_BESL        EQU (0x0000003C)    ; BESL value received with last ACKed LPM Token
USB_OTG_GLPMCFG_LPMACK      EQU (0x00000002)    ; LPM Token acknowledge enable
USB_OTG_GLPMCFG_LPMEN       EQU (0x00000001)    ; LPM support enable


; ********************  Bit definition for USB_OTG_DIEPEACHMSK1 register  *******************
USB_OTG_DIEPEACHMSK1_XFRCM   EQU (0x00000001)    ; Transfer completed interrupt mask
USB_OTG_DIEPEACHMSK1_EPDM    EQU (0x00000002)    ; Endpoint disabled interrupt mask
USB_OTG_DIEPEACHMSK1_TOM     EQU (0x00000008)    ; Timeout condition mask (nonisochronous endpoints)
USB_OTG_DIEPEACHMSK1_ITTXFEMSK       EQU (0x00000010)    ; IN token received when TxFIFO empty mask
USB_OTG_DIEPEACHMSK1_INEPNMM EQU (0x00000020)    ; IN token received with EP mismatch mask
USB_OTG_DIEPEACHMSK1_INEPNEM EQU (0x00000040)    ; IN endpoint NAK effective mask
USB_OTG_DIEPEACHMSK1_TXFURM  EQU (0x00000100)    ; FIFO underrun mask
USB_OTG_DIEPEACHMSK1_BIM     EQU (0x00000200)    ; BNA interrupt mask
USB_OTG_DIEPEACHMSK1_NAKM    EQU (0x00002000)    ; NAK interrupt mask

; ********************  Bit definition for USB_OTG_HPRT register  *******************
USB_OTG_HPRT_PCSTS   EQU (0x00000001)            ; Port connect status
USB_OTG_HPRT_PCDET   EQU (0x00000002)            ; Port connect detected
USB_OTG_HPRT_PENA    EQU (0x00000004)            ; Port enable
USB_OTG_HPRT_PENCHNG EQU (0x00000008)            ; Port enable/disable change
USB_OTG_HPRT_POCA    EQU (0x00000010)            ; Port overcurrent active
USB_OTG_HPRT_POCCHNG EQU (0x00000020)            ; Port overcurrent change
USB_OTG_HPRT_PRES    EQU (0x00000040)            ; Port resume
USB_OTG_HPRT_PSUSP   EQU (0x00000080)            ; Port suspend
USB_OTG_HPRT_PRST    EQU (0x00000100)            ; Port reset

USB_OTG_HPRT_PLSTS   EQU (0x00000C00)            ; Port line status
USB_OTG_HPRT_PLSTS_0 EQU (0x00000400)            ;Bit 0
USB_OTG_HPRT_PLSTS_1 EQU (0x00000800)            ;Bit 1
USB_OTG_HPRT_PPWR    EQU (0x00001000)            ; Port power

USB_OTG_HPRT_PTCTL   EQU (0x0001E000)            ; Port test control
USB_OTG_HPRT_PTCTL_0 EQU (0x00002000)            ;Bit 0
USB_OTG_HPRT_PTCTL_1 EQU (0x00004000)            ;Bit 1
USB_OTG_HPRT_PTCTL_2 EQU (0x00008000)            ;Bit 2
USB_OTG_HPRT_PTCTL_3 EQU (0x00010000)            ;Bit 3

USB_OTG_HPRT_PSPD    EQU (0x00060000)            ; Port speed
USB_OTG_HPRT_PSPD_0  EQU (0x00020000)            ;Bit 0
USB_OTG_HPRT_PSPD_1  EQU (0x00040000)            ;Bit 1

; ********************  Bit definition for USB_OTG_DOEPEACHMSK1 register  *******************
USB_OTG_DOEPEACHMSK1_XFRCM   EQU (0x00000001)    ; Transfer completed interrupt mask
USB_OTG_DOEPEACHMSK1_EPDM    EQU (0x00000002)    ; Endpoint disabled interrupt mask
USB_OTG_DOEPEACHMSK1_TOM     EQU (0x00000008)    ; Timeout condition mask
USB_OTG_DOEPEACHMSK1_ITTXFEMSK       EQU (0x00000010)    ; IN token received when TxFIFO empty mask
USB_OTG_DOEPEACHMSK1_INEPNMM EQU (0x00000020)    ; IN token received with EP mismatch mask
USB_OTG_DOEPEACHMSK1_INEPNEM EQU (0x00000040)    ; IN endpoint NAK effective mask
USB_OTG_DOEPEACHMSK1_TXFURM  EQU (0x00000100)    ; OUT packet error mask
USB_OTG_DOEPEACHMSK1_BIM     EQU (0x00000200)    ; BNA interrupt mask
USB_OTG_DOEPEACHMSK1_BERRM   EQU (0x00001000)    ; Bubble error interrupt mask
USB_OTG_DOEPEACHMSK1_NAKM    EQU (0x00002000)    ; NAK interrupt mask
USB_OTG_DOEPEACHMSK1_NYETM   EQU (0x00004000)    ; NYET interrupt mask

; ********************  Bit definition for USB_OTG_HPTXFSIZ register  *******************
USB_OTG_HPTXFSIZ_PTXSA       EQU (0x0000FFFF)        ; Host periodic TxFIFO start address
USB_OTG_HPTXFSIZ_PTXFD       EQU (0xFFFF0000)        ; Host periodic TxFIFO depth

; ********************  Bit definition for USB_OTG_DIEPCTL register  *******************
USB_OTG_DIEPCTL_MPSIZ        EQU (0x000007FF)         ; Maximum packet size
USB_OTG_DIEPCTL_USBAEP       EQU (0x00008000)         ; USB active endpoint
USB_OTG_DIEPCTL_EONUM_DPID   EQU (0x00010000)         ; Even/odd frame
USB_OTG_DIEPCTL_NAKSTS       EQU (0x00020000)         ; NAK status

USB_OTG_DIEPCTL_EPTYP        EQU (0x000C0000)         ; Endpoint type
USB_OTG_DIEPCTL_EPTYP_0      EQU (0x00040000)         ;Bit 0
USB_OTG_DIEPCTL_EPTYP_1      EQU (0x00080000)         ;Bit 1
USB_OTG_DIEPCTL_STALL        EQU (0x00200000)         ; STALL handshake

USB_OTG_DIEPCTL_TXFNUM       EQU (0x03C00000)         ; TxFIFO number
USB_OTG_DIEPCTL_TXFNUM_0     EQU (0x00400000)         ;Bit 0
USB_OTG_DIEPCTL_TXFNUM_1     EQU (0x00800000)         ;Bit 1
USB_OTG_DIEPCTL_TXFNUM_2     EQU (0x01000000)         ;Bit 2
USB_OTG_DIEPCTL_TXFNUM_3     EQU (0x02000000)         ;Bit 3
USB_OTG_DIEPCTL_CNAK EQU (0x04000000)         ; Clear NAK
USB_OTG_DIEPCTL_SNAK EQU (0x08000000)         ; Set NAK
USB_OTG_DIEPCTL_SD0PID_SEVNFRM       EQU (0x10000000)         ; Set DATA0 PID
USB_OTG_DIEPCTL_SODDFRM      EQU (0x20000000)         ; Set odd frame
USB_OTG_DIEPCTL_EPDIS        EQU (0x40000000)         ; Endpoint disable
USB_OTG_DIEPCTL_EPENA        EQU (0x80000000)         ; Endpoint enable

; ********************  Bit definition for USB_OTG_HCCHAR register  *******************
USB_OTG_HCCHAR_MPSIZ EQU (0x000007FF)          ; Maximum packet size

USB_OTG_HCCHAR_EPNUM EQU (0x00007800)          ; Endpoint number
USB_OTG_HCCHAR_EPNUM_0       EQU (0x00000800)          ;Bit 0
USB_OTG_HCCHAR_EPNUM_1       EQU (0x00001000)          ;Bit 1
USB_OTG_HCCHAR_EPNUM_2       EQU (0x00002000)          ;Bit 2
USB_OTG_HCCHAR_EPNUM_3       EQU (0x00004000)          ;Bit 3
USB_OTG_HCCHAR_EPDIR EQU (0x00008000)          ; Endpoint direction
USB_OTG_HCCHAR_LSDEV EQU (0x00020000)          ; Low-speed device

USB_OTG_HCCHAR_EPTYP   EQU (0x000C0000)          ; Endpoint type
USB_OTG_HCCHAR_EPTYP_0 EQU (0x00040000)          ;Bit 0
USB_OTG_HCCHAR_EPTYP_1 EQU (0x00080000)          ;Bit 1

USB_OTG_HCCHAR_MC      EQU (0x00300000)          ; Multi Count (MC) / Error Count (EC)
USB_OTG_HCCHAR_MC_0    EQU (0x00100000)          ;Bit 0
USB_OTG_HCCHAR_MC_1    EQU (0x00200000)          ;Bit 1

USB_OTG_HCCHAR_DAD     EQU (0x1FC00000)          ; Device address
USB_OTG_HCCHAR_DAD_0   EQU (0x00400000)          ;Bit 0
USB_OTG_HCCHAR_DAD_1   EQU (0x00800000)          ;Bit 1
USB_OTG_HCCHAR_DAD_2   EQU (0x01000000)          ;Bit 2
USB_OTG_HCCHAR_DAD_3   EQU (0x02000000)          ;Bit 3
USB_OTG_HCCHAR_DAD_4   EQU (0x04000000)          ;Bit 4
USB_OTG_HCCHAR_DAD_5   EQU (0x08000000)          ;Bit 5
USB_OTG_HCCHAR_DAD_6   EQU (0x10000000)          ;Bit 6
USB_OTG_HCCHAR_ODDFRM  EQU (0x20000000)          ; Odd frame
USB_OTG_HCCHAR_CHDIS   EQU (0x40000000)          ; Channel disable
USB_OTG_HCCHAR_CHENA   EQU (0x80000000)          ; Channel enable

; ********************  Bit definition for USB_OTG_HCSPLT register  *******************

USB_OTG_HCSPLT_PRTADDR EQU (0x0000007F)          ; Port address
USB_OTG_HCSPLT_PRTADDR_0       EQU (0x00000001)          ;Bit 0
USB_OTG_HCSPLT_PRTADDR_1       EQU (0x00000002)          ;Bit 1
USB_OTG_HCSPLT_PRTADDR_2       EQU (0x00000004)          ;Bit 2
USB_OTG_HCSPLT_PRTADDR_3       EQU (0x00000008)          ;Bit 3
USB_OTG_HCSPLT_PRTADDR_4       EQU (0x00000010)          ;Bit 4
USB_OTG_HCSPLT_PRTADDR_5       EQU (0x00000020)          ;Bit 5
USB_OTG_HCSPLT_PRTADDR_6       EQU (0x00000040)          ;Bit 6

USB_OTG_HCSPLT_HUBADDR EQU (0x00003F80)          ; Hub address
USB_OTG_HCSPLT_HUBADDR_0       EQU (0x00000080)          ;Bit 0
USB_OTG_HCSPLT_HUBADDR_1       EQU (0x00000100)          ;Bit 1
USB_OTG_HCSPLT_HUBADDR_2       EQU (0x00000200)          ;Bit 2
USB_OTG_HCSPLT_HUBADDR_3       EQU (0x00000400)          ;Bit 3
USB_OTG_HCSPLT_HUBADDR_4       EQU (0x00000800)          ;Bit 4
USB_OTG_HCSPLT_HUBADDR_5       EQU (0x00001000)          ;Bit 5
USB_OTG_HCSPLT_HUBADDR_6       EQU (0x00002000)          ;Bit 6

USB_OTG_HCSPLT_XACTPOS EQU (0x0000C000)          ; XACTPOS
USB_OTG_HCSPLT_XACTPOS_0       EQU (0x00004000)          ;Bit 0
USB_OTG_HCSPLT_XACTPOS_1       EQU (0x00008000)          ;Bit 1
USB_OTG_HCSPLT_COMPLSPLT       EQU (0x00010000)          ; Do complete split
USB_OTG_HCSPLT_SPLITEN         EQU (0x80000000)          ; Split enable

; ********************  Bit definition for USB_OTG_HCINT register  *******************
USB_OTG_HCINT_XFRC            EQU (0x00000001)           ; Transfer completed
USB_OTG_HCINT_CHH             EQU (0x00000002)           ; Channel halted
USB_OTG_HCINT_AHBERR          EQU (0x00000004)           ; AHB error
USB_OTG_HCINT_STALL           EQU (0x00000008)           ; STALL response received interrupt
USB_OTG_HCINT_NAK             EQU (0x00000010)           ; NAK response received interrupt
USB_OTG_HCINT_ACK             EQU (0x00000020)           ; ACK response received/transmitted interrupt
USB_OTG_HCINT_NYET            EQU (0x00000040)           ; Response received interrupt
USB_OTG_HCINT_TXERR           EQU (0x00000080)           ; Transaction error
USB_OTG_HCINT_BBERR           EQU (0x00000100)           ; Babble error
USB_OTG_HCINT_FRMOR           EQU (0x00000200)           ; Frame overrun
USB_OTG_HCINT_DTERR           EQU (0x00000400)           ; Data toggle error

; ********************  Bit definition for USB_OTG_DIEPINT register  *******************
USB_OTG_DIEPINT_XFRC            EQU (0x00000001)         ; Transfer completed interrupt
USB_OTG_DIEPINT_EPDISD          EQU (0x00000002)         ; Endpoint disabled interrupt
USB_OTG_DIEPINT_TOC             EQU (0x00000008)         ; Timeout condition
USB_OTG_DIEPINT_ITTXFE          EQU (0x00000010)         ; IN token received when TxFIFO is empty
USB_OTG_DIEPINT_INEPNE          EQU (0x00000040)         ; IN endpoint NAK effective
USB_OTG_DIEPINT_TXFE            EQU (0x00000080)         ; Transmit FIFO empty
USB_OTG_DIEPINT_TXFIFOUDRN      EQU (0x00000100)         ; Transmit Fifo Underrun
USB_OTG_DIEPINT_BNA             EQU (0x00000200)         ; Buffer not available interrupt
USB_OTG_DIEPINT_PKTDRPSTS       EQU (0x00000800)         ; Packet dropped status
USB_OTG_DIEPINT_BERR            EQU (0x00001000)         ; Babble error interrupt
USB_OTG_DIEPINT_NAK             EQU (0x00002000)         ; NAK interrupt

; ********************  Bit definition for USB_OTG_HCINTMSK register  *******************
USB_OTG_HCINTMSK_XFRCM           EQU (0x00000001)        ; Transfer completed mask
USB_OTG_HCINTMSK_CHHM            EQU (0x00000002)        ; Channel halted mask
USB_OTG_HCINTMSK_AHBERR          EQU (0x00000004)        ; AHB error
USB_OTG_HCINTMSK_STALLM          EQU (0x00000008)        ; STALL response received interrupt mask
USB_OTG_HCINTMSK_NAKM            EQU (0x00000010)        ; NAK response received interrupt mask
USB_OTG_HCINTMSK_ACKM            EQU (0x00000020)        ; ACK response received/transmitted interrupt mask
USB_OTG_HCINTMSK_NYET            EQU (0x00000040)        ; response received interrupt mask
USB_OTG_HCINTMSK_TXERRM          EQU (0x00000080)        ; Transaction error mask
USB_OTG_HCINTMSK_BBERRM          EQU (0x00000100)        ; Babble error mask
USB_OTG_HCINTMSK_FRMORM          EQU (0x00000200)        ; Frame overrun mask
USB_OTG_HCINTMSK_DTERRM          EQU (0x00000400)        ; Data toggle error mask

; ********************  Bit definition for USB_OTG_DIEPTSIZ register  *******************

USB_OTG_DIEPTSIZ_XFRSIZ          EQU (0x0007FFFF)        ; Transfer size
USB_OTG_DIEPTSIZ_PKTCNT          EQU (0x1FF80000)        ; Packet count
USB_OTG_DIEPTSIZ_MULCNT          EQU (0x60000000)        ; Packet count
; ********************  Bit definition for USB_OTG_HCTSIZ register  *******************
USB_OTG_HCTSIZ_XFRSIZ            EQU (0x0007FFFF)        ; Transfer size
USB_OTG_HCTSIZ_PKTCNT            EQU (0x1FF80000)        ; Packet count
USB_OTG_HCTSIZ_DOPING            EQU (0x80000000)        ; Do PING
USB_OTG_HCTSIZ_DPID              EQU (0x60000000)        ; Data PID
USB_OTG_HCTSIZ_DPID_0            EQU (0x20000000)        ;Bit 0
USB_OTG_HCTSIZ_DPID_1            EQU (0x40000000)        ;Bit 1

; ********************  Bit definition for USB_OTG_DIEPDMA register  *******************
USB_OTG_DIEPDMA_DMAADDR          EQU (0xFFFFFFFF)        ; DMA address

; ********************  Bit definition for USB_OTG_HCDMA register  *******************
USB_OTG_HCDMA_DMAADDR            EQU (0xFFFFFFFF)        ; DMA address

; ********************  Bit definition for USB_OTG_DTXFSTS register  *******************
USB_OTG_DTXFSTS_INEPTFSAV        EQU (0x0000FFFF)        ; IN endpoint TxFIFO space avail

; ********************  Bit definition for USB_OTG_DIEPTXF register  *******************
USB_OTG_DIEPTXF_INEPTXSA         EQU (0x0000FFFF)        ; IN endpoint FIFOx transmit RAM start address
USB_OTG_DIEPTXF_INEPTXFD         EQU (0xFFFF0000)        ; IN endpoint TxFIFO depth

; ********************  Bit definition for USB_OTG_DOEPCTL register  *******************

USB_OTG_DOEPCTL_MPSIZ          EQU (0x000007FF)       ; Maximum packet size           ;Bit 1
USB_OTG_DOEPCTL_USBAEP         EQU (0x00008000)       ; USB active endpoint
USB_OTG_DOEPCTL_NAKSTS         EQU (0x00020000)       ; NAK status
USB_OTG_DOEPCTL_SD0PID_SEVNFRM EQU (0x10000000)       ; Set DATA0 PID
USB_OTG_DOEPCTL_SODDFRM        EQU (0x20000000)       ; Set odd frame
USB_OTG_DOEPCTL_EPTYP          EQU (0x000C0000)       ; Endpoint type
USB_OTG_DOEPCTL_EPTYP_0        EQU (0x00040000)       ;Bit 0
USB_OTG_DOEPCTL_EPTYP_1        EQU (0x00080000)       ;Bit 1
USB_OTG_DOEPCTL_SNPM           EQU (0x00100000)       ; Snoop mode
USB_OTG_DOEPCTL_STALL          EQU (0x00200000)       ; STALL handshake
USB_OTG_DOEPCTL_CNAK           EQU (0x04000000)       ; Clear NAK
USB_OTG_DOEPCTL_SNAK           EQU (0x08000000)       ; Set NAK
USB_OTG_DOEPCTL_EPDIS          EQU (0x40000000)       ; Endpoint disable
USB_OTG_DOEPCTL_EPENA          EQU (0x80000000)       ; Endpoint enable

; ********************  Bit definition for USB_OTG_DOEPINT register  *******************
USB_OTG_DOEPINT_XFRC            EQU (0x00000001)         ; Transfer completed interrupt
USB_OTG_DOEPINT_EPDISD          EQU (0x00000002)         ; Endpoint disabled interrupt
USB_OTG_DOEPINT_STUP            EQU (0x00000008)         ; SETUP phase done
USB_OTG_DOEPINT_OTEPDIS         EQU (0x00000010)         ; OUT token received when endpoint disabled
USB_OTG_DOEPINT_B2BSTUP         EQU (0x00000040)         ; Back-to-back SETUP packets received
USB_OTG_DOEPINT_NYET            EQU (0x00004000)         ; NYET interrupt

; ********************  Bit definition for USB_OTG_DOEPTSIZ register  *******************

USB_OTG_DOEPTSIZ_XFRSIZ         EQU (0x0007FFFF)        ; Transfer size
USB_OTG_DOEPTSIZ_PKTCNT         EQU (0x1FF80000)        ; Packet count

USB_OTG_DOEPTSIZ_STUPCNT        EQU (0x60000000)        ; SETUP packet count
USB_OTG_DOEPTSIZ_STUPCNT_0      EQU (0x20000000)        ;Bit 0
USB_OTG_DOEPTSIZ_STUPCNT_1      EQU (0x40000000)        ;Bit 1

; ********************  Bit definition for PCGCCTL register  *******************
USB_OTG_PCGCCTL_STOPCLK         EQU (0x00000001)         ; SETUP packet count
USB_OTG_PCGCCTL_GATECLK         EQU (0x00000002)         ;Bit 0
USB_OTG_PCGCCTL_PHYSUSP         EQU (0x00000010)         ;Bit 1


    END