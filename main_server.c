/*Base register adddress header file*/
#include "stm32l1xx.h"
/*Library related header files*/
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_usart.h"

#include <string.h>
#include "ESP8266_lowlevel_conf.h"
#include "sonic.h"

void SystemClock_Config(void);
uint8_t ESP8266_SendCmd(uint8_t*);
void ESP8266_RespBufferReset(void);
void WIFI(void);
void SonicSensor(void);
void WIFI_wait(void);
void LED_GPIO_Conf(void);
void Time_base_conf(void);
void State_Def(void);

#define MAX_RESP_BUFFER_SIZE			200

uint8_t resp[MAX_RESP_BUFFER_SIZE] = {0};
uint8_t idx;
uint8_t state = 0;

uint16_t uwIC1 = 0;
uint16_t uwIC2 = 0;
uint16_t uwDiff = 0;
uint16_t uhICIndex = 0;


float period = 0;
float distance = 0;

uint32_t TIM2CLK;
uint32_t PSC;
uint32_t IC1PSC;

int main()
{
	
		SystemClock_Config();
		GPIO_Config();
	LED_GPIO_Conf();
		WIFI();
		
//		LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_7);
//		LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_5);
		
		TIMx_IC_Config();
		while(1)
		{
			WIFI_wait();
			SonicSensor();
			if(state==1){
				if(distance/100<=5)
					state=2;
			}
			else{
				if(distance/100<=5)
					state=2;
				else
					state=0;
			}
			State_Def();
		}
		
}
void WIFI_wait(void)
{
	(ESP_USART_LOWLEVEL_Recv(resp, idx) != 1)?(idx = (idx + 1) % MAX_RESP_BUFFER_SIZE):(idx);		
		if(strstr((const char*)resp, "Book"))
		{
			ESP8266_SendCmd((uint8_t*)"AT+CIPSEND=0,4\r\n") ;
			ESP8266_RespBufferReset() ;
			ESP8266_SendCmd((uint8_t*)"Booking") ;
			ESP8266_RespBufferReset() ;
			state=1;
		}
}
void State_Def(void)
{
	switch(state){
		case 0:
		{LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_1|LL_GPIO_PIN_2);
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_0);
		break;}
		case 1:
		{LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_0|LL_GPIO_PIN_2);
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_1);
			LL_TIM_EnableCounter(TIM4);
		break;}
		case 2:
		{LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_1|LL_GPIO_PIN_0);
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_2);
		break;}
	}
}
uint8_t ESP8266_SendCmd(uint8_t* cmd)
{
	ESP_USART_LOWLEVEL_Transmit(cmd);
	while(1)
	{
		
		(ESP_USART_LOWLEVEL_Recv(resp, idx) != 1)?(idx = (idx + 1) % MAX_RESP_BUFFER_SIZE):(idx);		
		if(strstr((const char*)resp, "OK"))
		{
			return 0;
		}
	}
}

void ESP_ServerStart()
{
	while(1)
	{
		
		(ESP_USART_LOWLEVEL_Recv(resp, idx) != 1)?(idx = (idx + 1) % MAX_RESP_BUFFER_SIZE):(idx);		
		if(strstr((const char*)resp, "CONNECT"))
		{
			return;
		}
	}
}

void ESP8266_RespBufferReset(void)
{
	memset(resp, NULL, MAX_RESP_BUFFER_SIZE);
	idx = 0;
}

void WIFI(void)
{
	ESP_USART_LOWLEVEL_Conf();
	ESP_USART_Start();
	
	ESP8266_SendCmd((uint8_t*)"AT+RST\r\n");
	ESP8266_RespBufferReset();	
	ESP8266_SendCmd((uint8_t*)"AT+RESTORE\r\n");
	ESP8266_RespBufferReset();
	LL_mDelay(1000); //Prevent ESP8266 flooding message
	ESP8266_SendCmd((uint8_t*)"AT+CWMODE=1\r\n");
	ESP8266_RespBufferReset();
	ESP8266_SendCmd((uint8_t*)"AT+RST\r\n");
	ESP8266_RespBufferReset();	
	LL_mDelay(1000);
	ESP8266_SendCmd((uint8_t*)"AT+CWJAP=\"MosQuito\",\"mos40542\"\r\n");
	ESP8266_RespBufferReset();	
	ESP8266_SendCmd((uint8_t*)"AT+CWJAP?\r\n");
	ESP8266_RespBufferReset();	
	LL_mDelay(1000);
	//Further execute ESP8266 AT command accrodingly to role
/*----------------SERVER------------------------------------------------------*/
	ESP8266_SendCmd((uint8_t*)"AT+CIPMUX=1\r\n");
	ESP8266_RespBufferReset();
	ESP8266_SendCmd((uint8_t*)"AT+CIPSERVER=1,2759\r\n");
	ESP8266_RespBufferReset();
	
//	ESP8266_SendCmd((uint8_t*)"AT+CIFSR\r\n");
//	ESP8266_RespBufferReset();
	
	ESP_ServerStart(); //Server wait for client to connect then process to next state
	ESP8266_RespBufferReset();


}

void TIM2_IRQHandler(void)
{
		if(uhICIndex ==0 ){
			if(LL_TIM_IsActiveFlag_CC3(TIM2) == SET)
			{
				LL_TIM_ClearFlag_CC3(TIM2);
					//Detect 1st rising edge
					uwIC1 = LL_TIM_IC_GetCaptureCH3(TIM2);
					uhICIndex = 1;
			}
		}
			
		else if(uhICIndex == 1)	
		{
			if(LL_TIM_IsActiveFlag_CC4(TIM2) == SET)
			{
				LL_TIM_ClearFlag_CC4(TIM2);
				//Detect 2nd falling edge
				uwIC2 = LL_TIM_IC_GetCaptureCH4(TIM2);
				if(uwIC2 > uwIC1)
						uwDiff = uwIC2 - uwIC1;
					else if(uwIC2 < uwIC1)
						uwDiff = ((LL_TIM_GetAutoReload(TIM2) - uwIC1) + uwIC2) + 1;
					uhICIndex = 2;
			}
		}
		
}

void LED_GPIO_Conf(void){
	LL_GPIO_InitTypeDef gpio;
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
		//GPIO_Config
		gpio.Mode = LL_GPIO_MODE_OUTPUT;
		gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		gpio.Pull = LL_GPIO_PULL_NO;
		gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		gpio.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 ;
		LL_GPIO_Init(GPIOC,&gpio);
}

void SonicSensor(void){
		
			LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_9);
			LL_mDelay(1);
			LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_9);
			if(uhICIndex == 2)
			{
					//Period calculation
					PSC = LL_TIM_GetPrescaler(TIM2) + 1;
					TIM2CLK = SystemCoreClock / PSC;
					IC1PSC = __LL_TIM_GET_ICPSC_RATIO(LL_TIM_IC_GetPrescaler(TIM2, LL_TIM_CHANNEL_CH3));
					
					period = (uwDiff*(PSC) * 1.0) / (TIM2CLK *IC1PSC * 1.0); //calculate uptime period
					distance = (period*340)/2.0;
					uhICIndex = 0;
			} 
	
}
void Time_base_conf(void)
{
	LL_TIM_InitTypeDef timbase;
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	
	timbase.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase.CounterMode = LL_TIM_COUNTERDIRECTION_UP;
	timbase.Autoreload = 10000 - 1;
	timbase.Prescaler = 16000 - 1;
	
	LL_TIM_Init(TIM4,&timbase);
	
	LL_TIM_EnableIT_UPDATE(TIM4);
	
	NVIC_SetPriority(TIM4_IRQn, 0);
	NVIC_EnableIRQ(TIM4_IRQn);
	
	//LL_TIM_EnableCounter(TIM4);
}	
void TIM4_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM4) == SET)
	{
		LL_TIM_ClearFlag_UPDATE(TIM4);
		state=0;
		LL_TIM_DisableCounter(TIM4);
	}
}
void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}
