#include "RTC.h"

// Use the 32.768 kHz low-speed external clock as RTC clock source

// Helper macro to convert a value from 2 digit decimal format to BCD format
// __VALUE__ Byte to be converted
#define __RTC_CONVERT_BIN2BCD(__VALUE__) (uint8_t)((((__VALUE__) / 10) << 4) | ((__VALUE__) % 10))
#define __RTC_CONVERT_BCD2BIN(__VALUE__) (uint8_t)(((uint8_t)((__VALUE__) & (uint8_t)0xF0) >> (uint8_t)0x4) * 10 + ((__VALUE__) & (uint8_t)0x0F))

#define RTC_WEEKDAY_MONDAY              ((uint32_t)0x01) /*!< Monday    */
#define RTC_WEEKDAY_TUESDAY             ((uint32_t)0x02) /*!< Tuesday   */
#define RTC_WEEKDAY_WEDNESDAY           ((uint32_t)0x03) /*!< Wednesday */
#define RTC_WEEKDAY_THURSDAY            ((uint32_t)0x04) /*!< Thursday  */
#define RTC_WEEKDAY_FRIDAY              ((uint32_t)0x05) /*!< Friday    */
#define RTC_WEEKDAY_SATURDAY            ((uint32_t)0x06) /*!< Saturday  */
#define RTC_WEEKDAY_SUNDAY              ((uint32_t)0x07) /*!< Sunday    */

#define RTC_MONTH_JANUARY               ((uint8_t)0x01)  /*!< January   */
#define RTC_MONTH_FEBRUARY              ((uint8_t)0x02)  /*!< February  */
#define RTC_MONTH_MARCH                 ((uint8_t)0x03)  /*!< March     */
#define RTC_MONTH_APRIL                 ((uint8_t)0x04)  /*!< April     */
#define RTC_MONTH_MAY                   ((uint8_t)0x05)  /*!< May       */
#define RTC_MONTH_JUNE                  ((uint8_t)0x06)  /*!< June      */
#define RTC_MONTH_JULY                  ((uint8_t)0x07)  /*!< July      */
#define RTC_MONTH_AUGUST                ((uint8_t)0x08)  /*!< August    */
#define RTC_MONTH_SEPTEMBER             ((uint8_t)0x09)  /*!< September */
#define RTC_MONTH_OCTOBER               ((uint8_t)0x10)  /*!< October   */
#define RTC_MONTH_NOVEMBER              ((uint8_t)0x11)  /*!< November  */
#define RTC_MONTH_DECEMBER              ((uint8_t)0x12)  /*!< December  */

uint32_t BIN2BCD(uint32_t bin)
{
  uint32_t ret_val = __RTC_CONVERT_BIN2BCD(bin);
  return ret_val;
}


uint32_t BCD2BIN(uint32_t bcd)
{
  uint32_t ret_val = __RTC_CONVERT_BCD2BIN(bcd);
  return ret_val;
}

//Assignment1:
void RTC_Set_Time(uint32_t Hour, uint32_t Minute, uint32_t Second){
  //RTC->TR = xxxx
}

//Assignment2:
void RTC_Read_Time(uint32_t *Hour, uint32_t *Minute, uint32_t *Second){
  //xxxx = RTC->TR
}

void RTC_BAK_SetRegister(uint32_t BackupRegister, uint32_t Data)
{
  register uint32_t tmp = 0;

  tmp = (uint32_t)(&(RTC->BKP0R));
  tmp += (BackupRegister * 4);

  /* Write the specified register */
  *(__IO uint32_t *)tmp = (uint32_t)Data;
}

void RTC_Init(void){

  /* Enables the PWR Clock and Enables access to the backup domain #######*/
  /* To change the source clock of the RTC feature (LSE, LSI), you have to:
     - Enable the power clock
     - Enable write access to configure the RTC clock source (to be done once after reset).
     - Reset the Back up Domain
     - Configure the needed RTC clock source */
  RTC_Clock_Init();

  // Disable RTC registers write protection
  RTC_Disable_Write_Protection();

  // Enter in initialization mode
  RTC->ISR |= 0xFFFFFFFF;

  while((RTC->ISR & (1 << 6)) == 0) {}  //According AN4759, we need to wait until INITF is set after entering initialization mode

  // Configure the Time
  RTC_Set_Time(0x11, 0x59, 0x55); // // Set Time: 11:59:55 AM

  // Exit of initialization mode
  RTC->ISR &= ~RTC_ISR_INIT;
  while((RTC->ISR & RTC_ISR_RSF) == 0);   /* Wait for synchro */
  /* Note: Needed only if Shadow registers is enabled */
  /* LL_RTC_IsShadowRegBypassEnabled function can be used */

  // Enable RTC registers write protection
  RTC_Enable_Write_Protection();

  // Writes a data in a RTC Backup data Register1
  // to indicate date/time updated
  RTC_BAK_SetRegister(1, 0x32F2);

}

void RTC_Clock_Init(void){

  // Enable write access to Backup domain
  if ( (RCC->APB1ENR1 & RCC_APB1ENR1_PWREN) == 0 ) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;  // Power interface clock enable
    (void) RCC->APB1ENR1;  // Delay after an RCC peripheral clock enabling
  }

  // Select LSE as RTC clock source
  if ( (PWR->CR1 & PWR_CR1_DBP) == 0) {
    PWR->CR1  |= PWR_CR1_DBP;               // Enable write access to Backup domain
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
  RCC->BDCR	&= ~RCC_BDCR_RTCSEL;    // RTCSEL[1:0]: 00 = No Clock, 01 = LSE, 10 = LSI, 11 = HSE
  RCC->BDCR	|= RCC_BDCR_RTCSEL_0;   // Select LSE as RTC clock

  RCC->BDCR |= RCC_BDCR_RTCEN;      // Enable RTC

  RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN;  // Power interface clock disable
}

void RTC_Disable_Write_Protection(void){
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
}

void RTC_Enable_Write_Protection(void){
  RTC->WPR = 0xFF;
}
