
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_pwr.h"              // Keil::Device:StdPeriph Drivers:PWR
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC

#include "stm32f10x_rtc.h"              // Keil::Device:StdPeriph Drivers:RTC
#include "RTC.h"
#include "BACKUP.h"

#define rtc_access_code             0x9999
uint32_t msTicks;
void delay_ms ( int ms ) {
	uint32_t currTicks = msTicks ;
	
	while ( ( msTicks - currTicks) < ms ) {
		
	}
}
uint32_t ttimee;

void setupRTC(void){
	RCC->APB1ENR |= (RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN); //Enable the power and backup interface clocks by setting the PWREN and BKPEN bitsin the RCC_APB1ENR register
	PWR->CR |= PWR_CR_DBP;                     //Enable access to the backup registers and the RTC.
	RCC->BDCR |= RCC_BDCR_LSEON;               //External Low Speed oscillator enable 
	while((RCC->BDCR & RCC_BDCR_LSERDY) == 0); //Wait until external oscillisator is stabilised
	RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;
	RCC->BDCR &= ~((1 << 8) | (1 << 9));       //Unusable clock source, here to prevent warnings, turn off clock sources to RTC.

	RCC->BDCR |= RCC_BDCR_RTCEN;                //RTC clock enable
	RTC->CRL &= ~RTC_CRL_RSF;                  //Clear registers Synchronized Flag
	while(!(RTC->CRL & RTC_CRL_RSF));          //Wait for the RSF bit in RTC_CRL to be set by hardware

	while((RTC->CRL & RTC_CRL_RTOFF) == 0);    //Wait for RTOFF It is not possible to write to the RTC_CR register while the peripheral is completing a previous write operation
	RTC->CRL |= RTC_CRL_CNF;                   //Set the CNF bit to enter configuration mode
	/* Set RTC COUNTER MSB word */
	RTC->CNTH = (1453);  //Random time
	/* Set RTC COUNTER LSB word */
	RTC->CNTL = ((1923)& 0x0000FFFF);
	RTC->CRH |= RTC_CRH_SECIE;                 //Second Interrupt Enable
	RTC->CRL &= ~RTC_CRL_CNF;                  //Exit configuration mode
	while((RTC->CRL & RTC_CRL_RTOFF) == 0);    //Wait for RTOFF 
}
u32 rtc_set_counter_val(u32 ts)
{
	while((RTC->CRL & RTC_CRL_RTOFF) == 0);    //Wait for RTOFF It is not possible to write to the RTC_CR register while the peripheral is completing a previous write operation
	RTC->CRL |= RTC_CRL_CNF;                   //Set the CNF bit to enter configuration mode

	RTC->CNTH = ts>>16;
	RTC->CNTL = ts & 0x0000FFFF;
	RTC->CRL &= ~RTC_CRL_CNF;                  //Exit configuration mode
	while((RTC->CRL & RTC_CRL_RTOFF) == 0);    //Wait for RTOFF 
}
u32 rtc_get_counter_val(void)
{
    return (RTC->CNTH << 16) | RTC->CNTL;
}
uint32_t xx;
int main()
{
	unsigned char i = 0;

	 setupRTC();
	u32 tse =0xDEADBEEF;
	rtc_set_counter_val( tse );

	while(1)
	{
		xx = rtc_get_counter_val();   
	};
}

void SysTick_Handler(){
	msTicks++;
}
