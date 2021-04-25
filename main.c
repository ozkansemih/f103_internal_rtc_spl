
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
unsigned char RTC_init()
{
//    unsigned char timeout = 0;
//    RCC_APB1PeriphClockCmd( RCC_APB1Periph_BKP ,ENABLE );
//    RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR ,ENABLE );
//	
//	RTC->CRL &= 0x1F;
////    if(BKP_DR1 != rtc_access_code)
//    {
//RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR ,1 ); //  enable_power_control_module(true);
//RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR ,1 ); //       enable_backup_module(true);
//disable_backup_domain_write_protection(true);
//BDCR_BDRST_BB = (uint32_t) 1; //RCC_BackupResetCmd(ENABLE);
//BDCR_BDRST_BB = (uint32_t) 0; //RCC_BackupResetCmd(DISABLE);
////        set_backup_domain_software_reset(true);
////        set_backup_domain_software_reset(false);
//BDCR_ADDRESS = RCC_LSE_OFF; //       RCC_LSEConfig( RCC_LSE_OFF );
////        bypass_LSE_with_external_clock(false);
//RCC_LSEConfig(RCC_LSE_ON);
////        enable_LSE(true);
//        while((LSE_ready() == false) && (timeout < 250))        {            timeout++;            delay_ms(10);        };    
//RCC_RTCCLKConfig ( LSE_clock );
//RCC->BDCR &= (~(0x3 << 8)); RCC->BDCR |= (1 << 8); //select_RTC_clock_source(LSE_clock);
//BDCR_RTCEN_BB = (uint32_t)1; //RCC_RTCCLKCmd ( ENABLE );
//RTC_WaitForLastTask();
////        while(get_RTC_operation_state() == false);
//RTC_WaitForSynchro();
////        while(get_RTC_register_sync_state() == false);
////		enable_RTC_second_interrupt(true);
////        while(get_RTC_operation_state() == false);
////        RTC_EnterConfigMode();
////        set_RTC_configuration_flag(true);
//RTC_SetPrescaler( 32767 );
////        set_RTC_prescalar(32767);
////        set_RTC_configuration_flag(false);
//RTC_WaitForLastTask();
////        while(get_RTC_operation_state() == false);
////        BKP_DR1 = rtc_access_code;
//SET_BIT(RTC->CRL, RTC_CRL_CNF);
//disable_backup_domain_write_protection(false);
// RTC_SetCounter( 0x0053);
////        set_RTC(cal_year, cal_month, cal_date, cal_hour, cal_minute, cal_second);
//    }

//    else
//    {
//        while(get_RTC_register_sync_state() == false);
//        enable_RTC_second_interrupt(true);
//        while(get_RTC_operation_state() == false);
//    }

//    NVIC_IntEnable(IVT_INT_RTC);
   
    return 0;
}
uint32_t ttimee;

void setupRTC(void){
    RCC->APB1ENR |= (RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN); //Enable the power and backup interface clocks by setting the PWREN and BKPEN bitsin the RCC_APB1ENR register
    PWR->CR |= PWR_CR_DBP;                     //Enable access to the backup registers and the RTC.
  RCC->BDCR |= RCC_BDCR_LSEON;               //External Low Speed oscillator enable 
    while((RCC->BDCR & RCC_BDCR_LSERDY) == 0); //Wait until external oscillisator is stabilised
    RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;
  RCC->BDCR &= ~((1 << 8) | (1 << 9));       //Unusable clock source, here to prevent warnings, turn off clock sources to RTC.
//    RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;          //Select Source (LSE oscillator clock used as RTC clock )
    
	
	RCC->BDCR |= RCC_BDCR_RTCEN;                //RTC clock enable
    RTC->CRL &= ~RTC_CRL_RSF;                  //Clear registers Synchronized Flag
    while(!(RTC->CRL & RTC_CRL_RSF));          //Wait for the RSF bit in RTC_CRL to be set by hardware

    while((RTC->CRL & RTC_CRL_RTOFF) == 0);    //Wait for RTOFF It is not possible to write to the RTC_CR register while the peripheral is completing a previous write operation
    RTC->CRL |= RTC_CRL_CNF;                   //Set the CNF bit to enter configuration mode
    /* Set RTC COUNTER MSB word */
  RTC->CNTH = (12*3600 + 40*60 + 00);  //Random time
  /* Set RTC COUNTER LSB word */
  RTC->CNTL = ((12*3600 + 40*60 + 00)& 0x0000FFFF);
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
             
             if(1)
             {
			 xx = rtc_get_counter_val();

             }
     };
}























//#include "stm32f10x.h"                  // Device header
//#include "misc.h"                       // Keil::Device:StdPeriph Drivers:Framework
//#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
//#include "stm32f10x_pwr.h"              // Keil::Device:StdPeriph Drivers:PWR
//#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
//#include "stm32f10x_rtc.h"              // Keil::Device:StdPeriph Drivers:RTC
// void delay_ms( int ms ) {
// 
// while ( --ms*10){
// 
// }
// 
// 
// }
//unsigned char RTC_Init(void)
//{
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
//	                           uint32_t       tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);\

//	PWR_BackupAccessCmd(ENABLE);
//	
//	if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN)
//	{
//		RCC_BackupResetCmd(ENABLE);
//		RCC_BackupResetCmd(DISABLE);

//		RCC_LSEConfig(RCC_LSE_ON);
////		while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY) {}
//		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

//		RTC_SetPrescaler(0x7FFF); // §£§ã§ä§Ñ§ß§à§Ó§Ú§ä§Ú §á§à§Õ?§Ý§ð§Ó§Ñ§é, §ë§à§Ò §Ô§à§Õ§Ú§ß§ß§Ú§Ü §â§Ñ§ç§å§Ó§Ñ§Ó §ã§Ö§Ü§å§ß§Õ§Ú

//		RCC_RTCCLKCmd(ENABLE);

//		RTC_WaitForSynchro();
//		return 1;
//	}
//	return 0;
//}

//void SetSysClockToHSE(void)
//{
//	ErrorStatus HSEStartUpStatus;

//	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
//    /* RCC system reset(for debug purpose) */
//    RCC_DeInit();

//    /* Enable HSE */
//    RCC_HSEConfig( RCC_HSE_ON);

//    /* Wait till HSE is ready */
//    HSEStartUpStatus = RCC_WaitForHSEStartUp();

//    if (HSEStartUpStatus == SUCCESS)
//    {
//        /* Enable Prefetch Buffer */
//        //FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);

//        /* Flash 0 wait state */
//        //FLASH_SetLatency( FLASH_Latency_0);

//        /* HCLK = SYSCLK */
//        RCC_HCLKConfig( RCC_SYSCLK_Div1);

//        /* PCLK2 = HCLK */
//        RCC_PCLK2Config( RCC_HCLK_Div1);

//        /* PCLK1 = HCLK */
//        RCC_PCLK1Config(RCC_HCLK_Div1);

//        /* Select HSE as system clock source */
//        RCC_SYSCLKConfig( RCC_SYSCLKSource_HSE);
//  

//        /* Wait till PLL is used as system clock source */
//        while (RCC_GetSYSCLKSource() != 0x04)
//        {
//        }
//    }
//    else
//    { /* If HSE fails to start-up, the application will have wrong clock configuration.
//     User can add here some code to deal with this error */

//        /* Go to infinite loop */
//        while (1)
//        {
//        }
//    }
//}


//#define JULIAN_DATE_BASE	2440588

//typedef struct
//{
//	uint8_t RTC_Hours;
//	uint8_t RTC_Minutes;
//	uint8_t RTC_Seconds;
//	uint8_t RTC_Date;
//	uint8_t RTC_Wday;
//	uint8_t RTC_Month;
//	uint16_t RTC_Year;
//} RTC_DateTimeTypeDef;

//// Get current date
//void RTC_GetDateTime(uint32_t RTC_Counter, RTC_DateTimeTypeDef* RTC_DateTimeStruct) {
//	unsigned long time;
//	unsigned long t1, a, b, c, d, e, m;
//	int year = 0;
//	int mon = 0;
//	int wday = 0;
//	int mday = 0;
//	int hour = 0;
//	int min = 0;
//	int sec = 0;
//	uint64_t jd = 0;;
//	uint64_t jdn = 0;

//	jd = ((RTC_Counter+43200)/(86400>>1)) + (2440587<<1) + 1;
//	jdn = jd>>1;

//	time = RTC_Counter;
//	t1 = time/60;
//	sec = time - t1*60;

//	time = t1;
//	t1 = time/60;
//	min = time - t1*60;

//	time = t1;
//	t1 = time/24;
//	hour = time - t1*24;

//	wday = jdn%7;

//	a = jdn + 32044;
//	b = (4*a+3)/146097;
//	c = a - (146097*b)/4;
//	d = (4*c+3)/1461;
//	e = c - (1461*d)/4;
//	m = (5*e+2)/153;
//	mday = e - (153*m+2)/5 + 1;
//	mon = m + 3 - 12*(m/10);
//	year = 100*b + d - 4800 + (m/10);

//	RTC_DateTimeStruct->RTC_Year = year;
//	RTC_DateTimeStruct->RTC_Month = mon;
//	RTC_DateTimeStruct->RTC_Date = mday;
//	RTC_DateTimeStruct->RTC_Hours = hour;
//	RTC_DateTimeStruct->RTC_Minutes = min;
//	RTC_DateTimeStruct->RTC_Seconds = sec;
//	RTC_DateTimeStruct->RTC_Wday = wday;
//}

//// Convert Date to Counter
//uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef* RTC_DateTimeStruct) {
//	uint8_t a;
//	uint16_t y;
//	uint8_t m;
//	uint32_t JDN;

//	a=(14-RTC_DateTimeStruct->RTC_Month)/12;
//	y=RTC_DateTimeStruct->RTC_Year+4800-a;
//	m=RTC_DateTimeStruct->RTC_Month+(12*a)-3;

//	JDN=RTC_DateTimeStruct->RTC_Date;
//	JDN+=(153*m+2)/5;
//	JDN+=365*y;
//	JDN+=y/4;
//	JDN+=-y/100;
//	JDN+=y/400;
//	JDN = JDN -32045;
//	JDN = JDN - JULIAN_DATE_BASE;
//	JDN*=86400;
//	JDN+=(RTC_DateTimeStruct->RTC_Hours*3600);
//	JDN+=(RTC_DateTimeStruct->RTC_Minutes*60);
//	JDN+=(RTC_DateTimeStruct->RTC_Seconds);

//	return JDN;
//}

//void RTC_GetMyFormat(RTC_DateTimeTypeDef* RTC_DateTimeStruct, char * buffer) {
//	const char WDAY0[] = "Monday";
//	const char WDAY1[] = "Tuesday";
//	const char WDAY2[] = "Wednesday";
//	const char WDAY3[] = "Thursday";
//	const char WDAY4[] = "Friday";
//	const char WDAY5[] = "Saturday";
//	const char WDAY6[] = "Sunday";
//	const char * WDAY[7]={WDAY0, WDAY1, WDAY2, WDAY3, WDAY4, WDAY5, WDAY6};

//	const char MONTH1[] = "January";
//	const char MONTH2[] = "February";
//	const char MONTH3[] = "March";
//	const char MONTH4[] = "April";
//	const char MONTH5[] = "May";
//	const char MONTH6[] = "June";
//	const char MONTH7[] = "July";
//	const char MONTH8[] = "August";
//	const char MONTH9[] = "September";
//	const char MONTH10[] = "October";
//	const char MONTH11[] = "November";
//	const char MONTH12[] = "December";
//	const char * MONTH[12]={MONTH1, MONTH2, MONTH3, MONTH4, MONTH5, MONTH6, MONTH7, MONTH8, MONTH9, MONTH10, MONTH11, MONTH12};

//}
////========================================================================================

//int main(void)
//{
//	char buffer[80] = {0 };
//	uint32_t RTC_Counter = 0;
//	RTC_DateTimeTypeDef	RTC_DateTime;
//	SystemCoreClockUpdate ();

//SysTick_Config( SystemCoreClock / 1000);
//	SetSysClockToHSE();


//	if (RTC_Init() == 1) {
//		// §Á§Ü§ë§à §á§Ö§â§ê§Ñ ?§ß?§è?§Ñ§Ý?§Ù§Ñ§è?§ñ RTC §£§ã§ä§Ñ§ß§à§Ó§Ý§ð?§Þ§à §á§à§é§Ñ§ä§Ü§à§Ó§å §Õ§Ñ§ä§å, §ß§Ñ§á§â§Ú§Ü§Ý§Ñ§Õ 22.09.2016 14:30:00
//		RTC_DateTime.RTC_Date = 22;
//		RTC_DateTime.RTC_Month = 9;
//		RTC_DateTime.RTC_Year = 2016;

//		RTC_DateTime.RTC_Hours = 14;
//		RTC_DateTime.RTC_Minutes = 30;
//		RTC_DateTime.RTC_Seconds = 00;

//		// §±?§ã§Ý§ñ ?§ß?§è?§Ñ§Ý?§Ù§Ñ§è?? §á§à§ä§â?§Ò§ß§Ñ §Ù§Ñ§ä§â§Ú§Þ§Ü§Ñ. §¢§Ö§Ù §ß§Ö? §é§Ñ§ã §ß§Ö §Ó§ã§ä§Ñ§ß§à§Ó§Ý§ð?§ä§î§ã§ñ.
//		delay_ms(500);
//		RTC_SetCounter(RTC_GetRTC_Counter(&RTC_DateTime));
//	}

//	while(1)
//	{
//		RTC_Counter = RTC_GetCounter();
// 
//		RTC_GetDateTime(RTC_Counter, &RTC_DateTime);


//		// §¶§å§ß§Ü§è?§ñ §Ô§Ö§ß§Ö§â§å? §å §Ò§å§æ§Ö§â? §Õ§Ñ§ä§å §Ó§Ý§Ñ§ã§ß§à§Ô§à §æ§à§â§Þ§Ñ§ä§å
//		RTC_GetMyFormat(&RTC_DateTime, buffer);

//		/* delay */
//		while (RTC_Counter == RTC_GetCounter()) {

//		}

//    }
//}

void SysTick_Handler(){
msTicks++;
}
