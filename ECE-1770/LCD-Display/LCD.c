#include "lcd.h"
#include "stm32l476xx.h"
#include <stdint.h>

/*  =========================================================================
                                 LCD MAPPING
    =========================================================================
LCD allows to display informations on six 14-segment digits and 4 bars:

  1       2       3       4       5       6
-----   -----   -----   -----   -----   -----   
|\|/| o |\|/| o |\|/| o |\|/| o |\|/|   |\|/|   BAR3
-- --   -- --   -- --   -- --   -- --   -- --   BAR2
|/|\| o |/|\| o |/|\| o |/|\| o |/|\|   |/|\|   BAR1
----- * ----- * ----- * ----- * -----   -----   BAR0

LCD segment mapping:
--------------------
  -----A-----        _ 
  |\   |   /|   COL |_|
  F H  J  K B          
  |  \ | /  |        _ 
  --G-- --M--   COL |_|
  |  / | \  |          
  E Q  P  N C          
  |/   |   \|        _ 
  -----D-----   DP  |_|

 An LCD character coding is based on the following matrix:
COM           0   1   2     3
SEG(n)      { E , D , P ,   N   }
SEG(n+1)    { M , C , COL , DP  }
SEG(23-n-1) { B , A , K ,   J   }
SEG(23-n)   { G , F , Q ,   H   }
with n positive odd number.

 The character 'A' for example is:
  -------------------------------
LSB   { 1 , 0 , 0 , 0   }
      { 1 , 1 , 0 , 0   }
      { 1 , 1 , 0 , 0   }
MSB   { 1 , 1 , 0 , 0   }
      -------------------
  'A' =  F    E   0   0 hexa

  @endverbati

*/

/* Constant table for cap characters 'A' --> 'Z' */
const uint16_t CapLetterMap[26] = {
        /* A      B      C      D      E      F      G      H      I  */
        0xFE00,0x6714,0x1d00,0x4714,0x9d00,0x9c00,0x3f00,0xfa00,0x0014,
        /* J      K      L      M      N      O      P      Q      R  */
        0x5300,0x9841,0x1900,0x5a48,0x5a09,0x5f00,0xFC00,0x5F01,0xFC01,
        /* S      T      U      V      W      X      Y      Z  */
        0xAF00,0x0414,0x5b00,0x18c0,0x5a81,0x00c9,0x0058,0x05c0
};

/* Constant table for number '0' --> '9' */
const uint16_t NumberMap[10] = {
        /* 0      1      2      3      4      5      6      7      8      9  */
        0x5F00,0x4200,0xF500,0x6700,0xEa00,0xAF00,0xBF00,0x04600,0xFF00,0xEF00
};


/* Macros used for set/reset bar LCD bar */
/* LCD BAR status: We don't write directly in LCD RAM for save the bar setting */
uint8_t t_bar[2] = {0x00,0x00};

#define BAR0_ON  t_bar[1] |= 8
#define BAR0_OFF t_bar[1] &= ~8
#define BAR1_ON  t_bar[0] |= 8
#define BAR1_OFF t_bar[0] &= ~8
#define BAR2_ON  t_bar[1] |= 2
#define BAR2_OFF t_bar[1] &= ~2
#define BAR3_ON  t_bar[0] |= 2 
#define BAR3_OFF t_bar[0] &= ~2 

#define DOT                   ((uint16_t) 0x8000 ) /* for add decimal point in string */
#define DOUBLE_DOT            ((uint16_t) 0x4000) /* for add decimal point in string */

/* code for '(' character */
#define C_OPENPARMAP          ((uint16_t) 0x0028)

/* code for ')' character */
#define C_CLOSEPARMAP         ((uint16_t) 0x0011)

/* code for 'd' character */
#define C_DMAP                ((uint16_t) 0xf300)

/* code for 'm' character */
#define C_MMAP                ((uint16_t) 0xb210)

/* code for 'n' character */
#define C_NMAP                ((uint16_t) 0x2210)

/* code for 'µ' character */
#define C_UMAP                ((uint16_t) 0x6084)

/* constant code for '*' character */
#define C_STAR                ((uint16_t) 0xA0DD)

/* constant code for '-' character */
#define C_MINUS               ((uint16_t) 0xA000)

/* constant code for '+' character */
#define C_PLUS                ((uint16_t) 0xA014)

/* constant code for '/' */
#define C_SLATCH              ((uint16_t) 0x00c0)

/* constant code for ° */
#define C_PERCENT_1           ((uint16_t) 0xec00)

/* constant code for small o */
#define C_PERCENT_2           ((uint16_t) 0xb300)

#define C_FULL                ((uint16_t) 0xffdd)


void LCD_Initialization(void){
	LCD_PIN_Init();
	LCD_Clock_Init();	
	LCD_Configure();
	LCD_Clear();
}

void LCD_PIN_Init(void){
	// LCD (24 segments, 4 commons, multiplexed 1/4 duty, 1/3 bias) on DIP28 connector
	//   VLCD = PC3
	//
	//   COM0 = PA8     COM1  = PA9      COM2  = PA10    COM3  = PB9
	//
	//   SEG0 = PA7     SEG6  = PD11     SEG12 = PB5     SEG18 = PD8
	//   SEG1 = PC5     SEG7  = PD13     SEG13 = PC8     SEG19 = PB14
	//   SEG2 = PB1     SEG8  = PD15     SEG14 = PC6     SEG20 = PB12
	//   SEG3 = PB13    SEG9  = PC7      SEG15 = PD14    SEG21 = PB0
	//   SEG4 = PB15    SEG10 = PA15     SEG16 = PD12    SEG22 = PC4
	//   SEG5 = PD9     SEG11 = PB4      SEG17 = PD10    SEG23 = PA6	
	
	/* Enable GPIOs clock */ 	
	RCC->AHB2ENR |=   RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN;
	
	// GPIO Mode: Input(00, reset), Output(01), AlterFunc(10), Analog(11, reset)
	// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
	// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
	
  /* Configure PA 6, 7, 8, 9, 10, 15 as Alternative Function 11 (0x0B)*/	
	GPIOA->MODER   &= ~(3U<<(2*6) | 3U<<(2*7) | 3U<<(2*8) | 3U<<(2*9) | 3U<<(2*10) | 3U<<(2*15));
	GPIOA->MODER   |=   2U<<(2*6) | 2U<<(2*7) | 2U<<(2*8) | 2U<<(2*9) | 2U<<(2*10) | 2U<<(2*15) ;
	GPIOA->AFR[0]  |= 0xBB000000;  
	GPIOA->AFR[1]  |= 0xB0000BBB; 
	GPIOA->OSPEEDR &= ~(3U<<(2*6) | 3U<<(2*7) | 3U<<(2*8) | 3U<<(2*9) | 3U<<(2*10) | 3U<<(2*15));
	GPIOA->OSPEEDR |=   2U<<(2*6) | 2U<<(2*7) | 2U<<(2*8) | 2U<<(2*9) | 2U<<(2*10) | 2U<<(2*15) ;
	GPIOA->PUPDR   &= ~(3U<<(2*6) | 3U<<(2*7) | 3U<<(2*8) | 3U<<(2*9) | 3U<<(2*10) | 3U<<(2*15));
  
	/* Configure PB 0, 1, 4, 5, 9, 12, 13, 14, 15 as Alternative Function 11 (0x0B)*/
	GPIOB->MODER   &= ~(3U<<(2*0) | 3U<<(2*1) | 3U<<(2*4) | 3U<<(2*5) | 3U<<(2*9) | 3U<<(2*12)| 3U<<(2*13)| 3U<<(2*14)| 3U<<(2*15));
	GPIOB->MODER   |=   2U<<(2*0) | 2U<<(2*1) | 2U<<(2*4) | 2U<<(2*5) | 2U<<(2*9) | 2U<<(2*12)| 2U<<(2*13)| 2U<<(2*14)| 2U<<(2*15);
	GPIOB->AFR[0]  |= 0x00BB00BB;  
	GPIOB->AFR[1]  |= 0xBBBB00B0; 
	GPIOB->OSPEEDR &= ~(3U<<(2*0) | 3U<<(2*1) | 3U<<(2*4) | 3U<<(2*5) | 3U<<(2*9) | 3U<<(2*12)| 3U<<(2*13)| 3U<<(2*14)| 3U<<(2*15));
	GPIOB->OSPEEDR |=   2U<<(2*0) | 2U<<(2*1) | 2U<<(2*4) | 2U<<(2*5) | 2U<<(2*9) | 2U<<(2*12)| 2U<<(2*13)| 2U<<(2*14)| 2U<<(2*15);
	GPIOB->PUPDR   &= ~(3U<<(2*0) | 3U<<(2*1) | 3U<<(2*4) | 3U<<(2*5) | 3U<<(2*9) | 3U<<(2*12)| 3U<<(2*13)| 3U<<(2*14)| 3U<<(2*15));

	
  /* Configure PC 3, 4, 5, 6, 7, 8 as Alternative Function 11 (0x0B)*/  	
	// Note: PC3 = V_LCD
	GPIOC->MODER   &= ~(3U<<(2*3) | 3U<<(2*4) | 3U<<(2*5) | 3U<<(2*6)  | 3U<<(2*7) | 3U<<(2*8));
	GPIOC->MODER   |=   2U<<(2*3) | 2U<<(2*4) | 2U<<(2*5) | 2U<<(2*6)  | 2U<<(2*7) | 2U<<(2*8);
	GPIOC->AFR[0]  |= 0xBBBBB000;   
	GPIOC->AFR[1]  |= 0x0000000B; 
	GPIOC->OSPEEDR &= ~(3U<<(2*3) | 3U<<(2*4) | 3U<<(2*5) | 3U<<(2*6)  | 3U<<(2*7) | 3U<<(2*8));
	GPIOC->OSPEEDR |=   2U<<(2*3) | 2U<<(2*4) | 2U<<(2*5) | 2U<<(2*6)  | 2U<<(2*7) | 2U<<(2*8);
	GPIOC->PUPDR   &= ~(3U<<(2*3) | 3U<<(2*4) | 3U<<(2*5) | 3U<<(2*6)  | 3U<<(2*7) | 3U<<(2*8));

	
	/* Configure PD 8, 9, 10, 11, 12, 13, 14, 15 as Alternative Function 11 (0x0B) */
	GPIOD->MODER   &= ~0xFFFF0000; // ~(3U<<(2*8) | 3U<<(2*9) | 3U<<(2*10)  | 3U<<(2*11) | 3U<<(2*12) | 3U<<(2*13)| 3U<<(2*14) | 3U<<(2*15));
	GPIOD->MODER   |= 0xAAAA0000;  // 2U<<(2*8) | 2U<<(2*9) | 2U<<(2*10)  | 2U<<(2*11) | 2U<<(2*12) | 2U<<(2*13)| 2U<<(2*14) | 2U<<(2*15);
	GPIOD->AFR[1]  |= 0xBBBBBBBB; 
	GPIOD->OSPEEDR &= 0xFFFF0000;  // ~(3U<<(2*8) | 3U<<(2*9) | 3U<<(2*10)  | 3U<<(2*11) | 3U<<(2*12) | 3U<<(2*13)| 3U<<(2*14) | 3U<<(2*15));
	GPIOD->OSPEEDR |= 0xAAAA0000;  // 2U<<(2*8) | 2U<<(2*9) | 2U<<(2*10)  | 2U<<(2*11) | 2U<<(2*12) | 2U<<(2*13)| 2U<<(2*14) | 2U<<(2*15);	
	GPIOD->PUPDR   &= 0xFFFF0000;  // ~(3U<<(2*8) | 3U<<(2*9) | 3U<<(2*10)  | 3U<<(2*11) | 3U<<(2*12) | 3U<<(2*13)| 3U<<(2*14) | 3U<<(2*15));

}

void LCD_Clock_Init(void){
	// Enable write access to Backup domain
	if ( (RCC->APB1ENR1 & RCC_APB1ENR1_PWREN) == 0)
		RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;	// Power interface clock enable
	(void) RCC->APB1ENR1;  // Delay after an RCC peripheral clock enabling
	
	// Select LSE as RTC clock soucre 
	if ( (PWR->CR1 & PWR_CR1_DBP) == 0) {
		PWR->CR1  |= PWR_CR1_DBP;				  			// Enable write access to Backup domain
		while((PWR->CR1 & PWR_CR1_DBP) == 0);  	// Wait for Backup domain Write protection disable
	}
	
	// Reset LSEON and LSEBYP bits before configuring the LSE
	RCC->BDCR &= ~(RCC_BDCR_LSEON | RCC_BDCR_LSEBYP);

	// RTC Clock selection can be changed only if the Backup Domain is reset
	RCC->BDCR |=  RCC_BDCR_BDRST;
	RCC->BDCR &= ~RCC_BDCR_BDRST;
	
	// Note from STM32L4 Reference Manual: 	
  // RTC/LCD Clock:  (1) LSE is in the Backup domain. (2) HSE and LSI are not.	
	while((RCC->BDCR & RCC_BDCR_LSERDY) == 0){  // Wait until LSE clock ready
		RCC->BDCR |= RCC_BDCR_LSEON;
	}
	
	// Select LSE as RTC clock source
	// BDCR = Backup Domain Control Register 
	RCC->BDCR	&= ~RCC_BDCR_RTCSEL;	  // RTCSEL[1:0]: 00 = No Clock, 01 = LSE, 10 = LSI, 11 = HSE
	RCC->BDCR	|= RCC_BDCR_RTCSEL_0;   // Select LSE as RTC clock	
	
	RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN;	// Power interface clock disable
	
	// Wait for the external capacitor Cext which is connected to the VLCD pin is charged (approximately 2ms for Cext=1uF) 
	
	// Enable LCD peripheral Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_LCDEN;
	
	// Enable SYSCFG 
	// RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}
	

void LCD_Configure(void){
	
	LCD->CR &= ~LCD_CR_LCDEN;	 // Disable LCD
	
	// LCD frame control register (FCR)
	// Set clock divider 
	LCD->FCR |=  0xF<<18;       // DIV[3:0] = 1111, ck_div = ck_ps/31
		
	// Set CLKPS = LCDCLK
	LCD->FCR &= ~LCD_FCR_PS;    // 0000: ck_ps = LCDCLK
	
	// Blink mode selection
  LCD->FCR &= ~LCD_FCR_BLINK; // 00: Blink disabled
															// 01: Blink enabled on SEG[0], COM[0] (1 pixel)
															// 10: Blink enabled on SEG[0], all COMs (up to 8 pixels depending on the programmed duty)
															// 11: Blink enabled on all SEGs and all COMs (all pixels)
	// Blink Frequency
	LCD->FCR &= ~LCD_FCR_BLINKF; 
	LCD->FCR |= LCD_FCR_BLINKF_1;  // 010: fLCD/32	
    
	// Set Pulse ON duration
	// Use high drive internal booster to provide larger drive current
	// Set the duration that the low-resister voltage divider is used
	LCD->FCR |= 0x7 << 4;     // PON[2:0] = 0x111
	
	// Contrast Control: specify one of the VLCD maximum voltages
	LCD->FCR &= ~LCD_FCR_CC;  // CC[2:0] bits (Contrast Control)
	LCD->FCR |= 0x5<<10; 			// 101: VLCD5, Refer to the product datasheet for the VLCDx values

	// Pulse ON duration: A short pulse will lead to lower power consumption, 
	// but displays with high internal resistance may need a longer pulse to 
	// achieve satisfactory contrast. 
	LCD->FCR &= ~LCD_FCR_PON; // PON[2:0] bits (Pulse ON Duration)
	LCD->FCR |= 0x4<<4; // 100: 4/ck_ps
    
	// Waits until the LCD FCR register is synchronized in the LCDCLK domain
	// This bit is set by hardware each time the LCD_FCR register is updated in the LCDCLK domain.
	// It is cleared by hardware when writing to the LCD_FCR register.
	while ((LCD->SR & LCD_SR_FCRSR) == 0);  // Loop until FCRSF flag is set
	
	// Select 1/4 duty
	LCD->CR &= ~(LCD_CR_DUTY);  // Clear duty
	LCD->CR |= 0x3<<2; 		      // DUTY[2:0]: 000=Static; 001=1/2; 010=1/3; 011=1/4; 100=1/8	

	// Select 1/3 bias
	LCD->CR &= ~(LCD_CR_BIAS);  // Clear bias
	LCD->CR |= 0x2<<5; 		      // BIAS[1:0]: 00=1/4; 01=1/2; 10=1/3	
	
	// MUX_SEG disabled
	// 0: SEG pin multiplexing disabled
  // 1: SEG[31:28] are multiplexed with SEG[43:40]
	LCD->CR &= ~LCD_CR_MUX_SEG; // Disable Mux Segment; 
	
	/* LCD control register */
	// VSEL: Voltage source selection
	// When the LCD is based on the internal step-up converter, the VLCD pin should be connected to a capacitor (see the product datasheet for further information).
	LCD->CR &= ~LCD_CR_VSEL;    // 0 = internal source, 1 = external source (VLCD pin)	
	
	// LCD controller enable
	// The VSEL, MUX_SEG, BIAS, DUTY and BUFEN bits are write-protected when the LCD is enabled (ENS bit in LCD_SR to 1).
	// When the LCD is disabled all COM and SEG pins are driven to VSS.
	LCD->CR |= LCD_CR_LCDEN;	 // Enable LCD
	// Loop until LCD controller is enabled
	while ((LCD->SR & LCD_SR_ENS) == 0);  // ENS is set by hardware automatically
		
	// Wait until the LCD Booster is ready
	while ((LCD->SR & LCD_SR_RDY) == 0);  // Loop until step-up converter is ready to provide the correct voltage.
	
  // Enable the display request
  LCD->SR |= LCD_SR_UDR; 	
}


void LCD_Clear(void){
  uint8_t counter = 0;

  // Wait until LCD ready */  
	while ((LCD->SR & LCD_SR_UDR) != 0); // Wait for Update Display Request Bit
  
  for (counter = 0; counter <= 15; counter++) {
    LCD->RAM[counter] = 0;
  }

  /* Update the LCD display */
	LCD->SR |= LCD_SR_UDR; 
}


void LCD_DisplayString(uint8_t* ptr){

	uint8_t position = 0;
	
	uint8_t colonOn = 0;
	uint8_t pointOn = 0;

  /* Send the string character by character on lCD */
  while ( (*ptr != 0) && (position < 6)) {
		
		if (*(ptr + 1) == ':')
				colonOn = 1;
		else
				colonOn = 0;
		
		if (* (ptr + 1) == '.')
				pointOn = 1;
		else
				pointOn = 0;
		
    LCD_WriteChar( ptr, pointOn, colonOn, position); // Display one character on LCD 
		
    ptr++;
		if (colonOn) ptr++;
		if (pointOn) ptr++;
		position++;
  }	
}



// Setting bar on LCD, writes bar value in LCD frame buffer 
void LCD_bar(void) {

	// TO wait LCD Ready *
  while ((LCD->SR & LCD_SR_UDR) != 0); // Wait for Update Display Request Bit
	// Bar 0: COM3, LCD_SEG11 -> MCU_LCD_SEG8
	// Bar 1: COM2, LCD_SEG11 -> MCU_LCD_SEG8
	// Bar 2: COM3, LCD_SEG9 -> MCU_LCD_SEG25
	// Bar 3: COM2, LCD_SEG9 -> MCU_LCD_SEG25
	
  LCD->RAM[4] &= ~(1U << 8 | 1U << 25);
  LCD->RAM[6] &= ~(1U << 8 | 1U << 25);
	
  /* bar1 bar3 */
  if (BAR0_ON)
		LCD->RAM[6] |= 1U << 8;
  
  if (BAR1_ON)
		LCD->RAM[4] |= 1U << 8;
 
	if (BAR2_ON)
		LCD->RAM[6] |= 1U << 25;
  
  if (BAR1_ON)
		LCD->RAM[4] |= 1U << 25;
	
	LCD->SR |= LCD_SR_UDR; 
}

/**
  * @brief  Converts an ascii char to the a LCD digit.
  * @param  c: a char to display.
  * @param  point: a point to add in front of char
  *         This parameter can be: POINT_OFF or POINT_ON
  * @param  colon : flag indicating if a colon has to be add in front
  *         of displayed character.
  *         This parameter can be: colon_OFF or colon_ON.
	* @param 	digit array with segment 
  * @retval None
  */
static void LCD_Conv_Char_Seg(uint8_t* c, bool point, bool colon, uint8_t* digit) {
  uint16_t ch = 0 ;
  uint8_t loop = 0, index = 0;
  
  switch (*c)
    {
    case ' ' :
      ch = 0x00;
      break;

    case '*':
      ch = C_STAR;
      break;

    case '(' :
      ch = C_OPENPARMAP;
      break;

    case ')' :
      ch = C_CLOSEPARMAP;
      break;
      
    case 'd' :
      ch = C_DMAP;
      break;
    
    case 'm' :
      ch = C_MMAP;
      break;
    
    case 'n' :
      ch = C_NMAP;
      break;

    case 'µ' :
      ch = C_UMAP;
      break;

    case '-' :
      ch = C_MINUS;
      break;

    case '+' :
      ch = C_PLUS;
      break;

    case '/' :
      ch = C_SLATCH;
      break;  
      
    case '°' :
      ch = C_PERCENT_1;
      break;  
		
    case '%' :
      ch = C_PERCENT_2; 
      break;
		
    case 255 :
      ch = C_FULL;
      break ;
    
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':			
      ch = NumberMap[*c-0x30];		
      break;
          
    default:
      /* The character c is one letter in upper case*/
      if ( (*c < 0x5b) && (*c > 0x40) )
      {
        ch = CapLetterMap[*c-'A'];
      }
      /* The character c is one letter in lower case*/
      if ( (*c <0x7b) && ( *c> 0x60) )
      {
        ch = CapLetterMap[*c-'a'];
      }
      break;
  }
       
  /* Set the digital point can be displayed if the point is on */
  if (point)
  {
    ch |= 0x0002;
  }

  /* Set the "COL" segment in the character that can be displayed if the colon is on */
  if (colon)
  {
    ch |= 0x0020;
  }		

	for (loop = 12,index=0 ;index < 4; loop -= 4,index++)
  {
    digit[index] = (ch >> loop) & 0x0f; /*To isolate the less significant digit */
  }
	
}


/**
  * @brief Write a character in the LCD frame buffer
  * @param  ch: the character to display.
  * @param  Point: a point to add in front of char
  * @param  Colon: flag indicating if a colon character has to be added in front
  *         of displayed character.         
  * @param  Position: position in the LCD of the character to write [1:6]
	*/

void LCD_WriteChar(uint8_t* ch, bool point, bool colon, uint8_t position){
  uint8_t digit[4];     /* Digit frame buffer */
   
  // Convert displayed character in segment in array digit 
  LCD_Conv_Char_Seg(ch, point, colon, digit);

  // TO wait LCD Ready *
  while ((LCD->SR & LCD_SR_UDR) != 0); // Wait for Update Display Request Bit
  
  switch (position) {
		
    /* Position 1 on LCD (digit1)*/
    case 0:
	
			LCD->RAM[0] &= ~( 1U << 4 | 1U << 23 | 1U << 22 | 1U << 3 );
      LCD->RAM[2] &= ~( 1U << 4 | 1U << 23 | 1U << 22 | 1U << 3 );
      LCD->RAM[4] &= ~( 1U << 4 | 1U << 23 | 1U << 22 | 1U << 3 );
      LCD->RAM[6] &= ~( 1U << 4 | 1U << 23 | 1U << 22 | 1U << 3 );
			/* 1G 1B 1M 1E */
      LCD->RAM[0] |= ((digit[0] & 0x1) << 4) | (((digit[0] & 0x2) >> 1) << 23) | (((digit[0] & 0x4) >> 2) << 22) | (((digit[0] & 0x8) >> 3) << 3);
      /* 1F 1A 1C 1D  */
      LCD->RAM[2] |= ((digit[1] & 0x1) << 4) | (((digit[1] & 0x2) >> 1) << 23) | (((digit[1] & 0x4) >> 2) << 22) | (((digit[1] & 0x8) >> 3) << 3);
      /* 1Q 1K 1Col 1P  */
      LCD->RAM[4] |= ((digit[2] & 0x1) << 4) | (((digit[2] & 0x2) >> 1) << 23) | (((digit[2] & 0x4) >> 2) << 22) | (((digit[2] & 0x8) >> 3) << 3);
      /* 1H 1J 1DP 1N  */
      LCD->RAM[6] |= ((digit[3] & 0x1) << 4) | (((digit[3] & 0x2) >> 1) << 23) | (((digit[3] & 0x4) >> 2) << 22) | (((digit[3] & 0x8) >> 3) << 3);

			break;

    /* Position 2 on LCD (digit2)*/
    case 1:
			
			LCD->RAM[0] &= ~( 1U << 6 | 1U << 13 | 1U << 12 | 1U << 5 );
      LCD->RAM[2] &= ~( 1U << 6 | 1U << 13 | 1U << 12 | 1U << 5 );
      LCD->RAM[4] &= ~( 1U << 6 | 1U << 13 | 1U << 12 | 1U << 5 );
      LCD->RAM[6] &= ~( 1U << 6 | 1U << 13 | 1U << 12 | 1U << 5 );
			/* 2G 2B 2M 2E */
      LCD->RAM[0] |= ((digit[0] & 0x1) << 6) | (((digit[0] & 0x2) >> 1) << 13) | (((digit[0] & 0x4) >> 2) << 12) | (((digit[0] & 0x8) >> 3) << 5);
      /* 2F 2A 2C 2D  */
      LCD->RAM[2] |= ((digit[1] & 0x1) << 6) | (((digit[1] & 0x2) >> 1) << 13) | (((digit[1] & 0x4) >> 2) << 12) | (((digit[1] & 0x8) >> 3) << 5);
      /* 2Q 2K 2Col 2P  */
      LCD->RAM[4] |= ((digit[2] & 0x1) << 6) | (((digit[2] & 0x2) >> 1) << 13) | (((digit[2] & 0x4) >> 2) << 12) | (((digit[2] & 0x8) >> 3) << 5);
      /* 2H 2J 2DP 2N  */
      LCD->RAM[6] |= ((digit[3] & 0x1) << 6) | (((digit[3] & 0x2) >> 1) << 13) | (((digit[3] & 0x4) >> 2) << 12) | (((digit[3] & 0x8) >> 3) << 5);

			break;
    
    /* Position 3 on LCD (digit3)*/
    case 2:
			
			LCD->RAM[0] &= ~( 1U << 15 | 1U << 29 | 1U << 28 | 1U << 14 );
      LCD->RAM[2] &= ~( 1U << 15 | 1U << 29 | 1U << 28 | 1U << 14 );
      LCD->RAM[4] &= ~( 1U << 15 | 1U << 29 | 1U << 28 | 1U << 14 );
      LCD->RAM[6] &= ~( 1U << 15 | 1U << 29 | 1U << 28 | 1U << 14 );
			/* 3G 3B 3M 3E */
      LCD->RAM[0] |= ((digit[0] & 0x1) << 15) | (((digit[0] & 0x2) >> 1) << 29) | (((digit[0] & 0x4) >> 2) << 28) | (((digit[0] & 0x8) >> 3) << 14);
      /* 3F 3A 3C 3D */
      LCD->RAM[2] |= ((digit[1] & 0x1) << 15) | (((digit[1] & 0x2) >> 1) << 29) | (((digit[1] & 0x4) >> 2) << 28) | (((digit[1] & 0x8) >> 3) << 14);
      /* 3Q 3K 3Col 3P  */
      LCD->RAM[4] |= ((digit[2] & 0x1) << 15) | (((digit[2] & 0x2) >> 1) << 29) | (((digit[2] & 0x4) >> 2) << 28) | (((digit[2] & 0x8) >> 3) << 14);
      /* 3H 3J 3DP  3N  */
      LCD->RAM[6] |= ((digit[3] & 0x1) << 15) | (((digit[3] & 0x2) >> 1) << 29) | (((digit[3] & 0x4) >> 2) << 28) | (((digit[3] & 0x8) >> 3) << 14);

			break;
    
    /* Position 4 on LCD (digit4)*/
    case 3:
			
			LCD->RAM[0] &= ~( 1U << 31 | 1U << 30);
			LCD->RAM[1] &= ~( 1U << 1 | 1U << 0 );
      LCD->RAM[2] &= ~( 1U << 31 | 1U << 30);
			LCD->RAM[3] &= ~( 1U << 1 | 1U << 0 );
      LCD->RAM[4] &= ~( 1U << 31 | 1U << 30);
			LCD->RAM[5] &= ~( 1U << 1 | 1U << 0 );
      LCD->RAM[6] &= ~( 1U << 31 | 1U << 30);
			LCD->RAM[7] &= ~( 1U << 1 | 1U << 0 );
			/* 4G 4B 4M 4E */
      LCD->RAM[0] |= ((digit[0] & 0x1) << 31) | (((digit[0] & 0x8) >> 3) << 30);
			LCD->RAM[1] |= (((digit[0] & 0x2) >> 1) << 1) | (((digit[0] & 0x4) >> 2) << 0);
      /* 4F 4A 4C 4D */
      LCD->RAM[2] |= ((digit[1] & 0x1) << 31) | (((digit[1] & 0x8) >> 3) << 30);
			LCD->RAM[3] |= (((digit[1] & 0x2) >> 1) << 1) | (((digit[1] & 0x4) >> 2) << 0);
      /* 4Q 4K 4Col 4P  */
      LCD->RAM[4] |= ((digit[2] & 0x1) << 31) | (((digit[2] & 0x8) >> 3) << 30);
			LCD->RAM[5] |= (((digit[2] & 0x2) >> 1) << 1) | (((digit[2] & 0x4) >> 2) << 0);
      /* 4H 4J 4DP  4N  */
      LCD->RAM[6] |= ((digit[3] & 0x1) << 31) | (((digit[3] & 0x8) >> 3) << 30);
			LCD->RAM[7] |= (((digit[3] & 0x2) >> 1) << 1) | (((digit[3] & 0x4) >> 2) << 0);

			break;
    
    /* Position 5 on LCD (digit5)*/
    case 4:
			
			LCD->RAM[0] &= ~( 1U << 25 | 1U << 24);
			LCD->RAM[1] &= ~( 1U << 3 | 1U << 2 );
      LCD->RAM[2] &= ~( 1U << 25 | 1U << 24);
			LCD->RAM[3] &= ~( 1U << 3 | 1U << 2 );
      LCD->RAM[4] &= ~( 1U << 25 | 1U << 24 );
			LCD->RAM[5] &= ~( 1U << 3 | 1U << 2 );
      LCD->RAM[6] &= ~( 1U << 25 | 1U << 24 );
			LCD->RAM[7] &= ~( 1U << 3 | 1U << 2 );
			/* 5G 5B 5M 5E */
      LCD->RAM[0] |= (((digit[0] & 0x2) >> 1) << 25) | (((digit[0] & 0x4) >> 2) << 24);
			LCD->RAM[1] |= ((digit[0] & 0x1) << 3) | (((digit[0] & 0x8) >> 3) << 2);
      /* 5F 5A 5C 5D */
      LCD->RAM[2] |= (((digit[1] & 0x2) >> 1) << 25) | (((digit[1] & 0x4) >> 2) << 24);
			LCD->RAM[3] |= ((digit[1] & 0x1) << 3) | (((digit[1] & 0x8) >> 3) << 2);
      /* 5Q 5K 5Col 5P  */
      LCD->RAM[4] |= (((digit[2] & 0x2) >> 1) << 25) | (((digit[2] & 0x4) >> 2) << 24);
			LCD->RAM[5] |= ((digit[2] & 0x1) << 3) | (((digit[2] & 0x8) >> 3) << 2);
      /* 5H 5J 5DP  5N  */
      LCD->RAM[6] |= (((digit[3] & 0x2) >> 1) << 25) | (((digit[3] & 0x4) >> 2) << 24);
			LCD->RAM[7] |= ((digit[3] & 0x1) << 3) | (((digit[3] & 0x8) >> 3) << 2);

			break;
    
    /* Position 6 on LCD (digit6)*/
    case 5:
			
			LCD->RAM[0] &= ~( 1U << 17 | 1U << 8 | 1U << 9 | 1U << 26 );
      LCD->RAM[2] &= ~( 1U << 17 | 1U << 8 | 1U << 9 | 1U << 26 );
      LCD->RAM[4] &= ~( 1U << 17 | 1U << 8 | 1U << 9 | 1U << 26 );
      LCD->RAM[6] &= ~( 1U << 17 | 1U << 8 | 1U << 9 | 1U << 26 );
			/* 6G 6B 6M 6E */
      LCD->RAM[0] |= ((digit[0] & 0x1) << 17) | (((digit[0] & 0x2) >> 1) << 8) | (((digit[0] & 0x4) >> 2) << 9) | (((digit[0] & 0x8) >> 3) << 26);
      /* 6F 6A 6C 6D */
      LCD->RAM[2] |= ((digit[1] & 0x1) << 17) | (((digit[1] & 0x2) >> 1) << 8) | (((digit[1] & 0x4) >> 2) << 9) | (((digit[1] & 0x8) >> 3) << 26);
      /* 6Q 6K 6Col 6P  */
      LCD->RAM[4] |= ((digit[2] & 0x1) << 17) | (((digit[2] & 0x2) >> 1) << 8) | (((digit[2] & 0x4) >> 2) << 9) | (((digit[2] & 0x8) >> 3) << 26);
      /* 6H 6J 6DP  6N  */
      LCD->RAM[6] |= ((digit[3] & 0x1) << 17) | (((digit[3] & 0x2) >> 1) << 8) | (((digit[3] & 0x4) >> 2) << 9) | (((digit[3] & 0x8) >> 3) << 26);

			break;
    
     default:
      break;
  }

  /* Refresh LCD  bar */
  //LCD_bar();

  // Update the LCD display 
	// Set the Update Display Request.
	//
	// Each time software modifies the LCD_RAM, it must set the UDR bit to transfer the updated
	// data to the second level buffer. The UDR bit stays set until the end of the update and during
	// this time the LCD_RAM is write protected.
	//
	// When the display is enabled, the update is performed only for locations for which
	// commons are active (depending on DUTY). For example if DUTY = 1/2, only the
	// LCD_DISPLAY of COM0 and COM1 will be updated.
	LCD->SR |= LCD_SR_UDR; 								// Update display request. Cleared by hardware
	//while ((LCD->SR & LCD_SR_UDD) == 0);	// Wait Until the LCD display is done
	//LCD->CLR &= ~LCD_CLR_UDDC;            // Clear UDD flag
  
}