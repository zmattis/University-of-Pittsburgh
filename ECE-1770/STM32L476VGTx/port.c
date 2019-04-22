#include "stm32l476xx.h"

void config_NVIC_in_C()
{
  NVIC_SetPriority(SysTick_IRQn, 1);    // Set Priority to 1
  NVIC_EnableIRQ(SysTick_IRQn);         // Enable EXTI0_1 interrupt in NVIC

  NVIC_SetPriority(RTC_Alarm_IRQn, 0);  // Set Priority to 2
  NVIC_EnableIRQ(RTC_Alarm_IRQn);       // Enable RTC Alarm (A and B) interrupt in NVIC
}

void System_Clock_Init(void){

  RCC->CR |= RCC_CR_MSION;

  // Select MSI as the clock source of System Clock
  RCC->CFGR &= ~RCC_CFGR_SW;


  // Wait until MSI is ready
  while ((RCC->CR & RCC_CR_MSIRDY) == 0);

  // MSIRANGE can be modified when MSI is OFF (MSION=0) or when MSI is ready (MSIRDY=1).
  RCC->CR &= ~RCC_CR_MSIRANGE;
  RCC->CR |= RCC_CR_MSIRANGE_7;  // Select MSI 8 MHz

  // The MSIRGSEL bit in RCC-CR select which MSIRANGE is used.
  // If MSIRGSEL is 0, the MSIRANGE in RCC_CSR is used to select the MSI clock range.  (This is the default)
  // If MSIRGSEL is 1, the MSIRANGE in RCC_CR is used.
  RCC->CR |= RCC_CR_MSIRGSEL;

  // Enable MSI and wait until it's ready
  while ((RCC->CR & RCC_CR_MSIRDY) == 0);
}

/*
void BIN2BCD(uint8_t bin)
{
  //?return and the input type??
  uint8_t bcd = (bin / 10) << 4;
  bcd |= (bin % 10);
}

void BCD2BIN(uint8_t bcd)
{
  uint8_t bin = ((bcd & 0xF0) >> 4);
  bin += (bcd & 0x0F);
}
*/
