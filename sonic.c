#include "sonic.h"

void GPIO_Config(void)
{
		LL_GPIO_InitTypeDef timic_gpio;
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
		//GPIO_Config
		timic_gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	  timic_gpio.Alternate = LL_GPIO_AF_1;
		timic_gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		timic_gpio.Pull = LL_GPIO_PULL_NO;
		timic_gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		timic_gpio.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11 ;
		LL_GPIO_Init(GPIOB,&timic_gpio);

}
	
void TIMx_IC_Config(void)
{
		LL_TIM_IC_InitTypeDef timic;
	
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
		
		//TIM_IC Configure CH1
		timic.ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
		timic.ICFilter = LL_TIM_IC_FILTER_FDIV1_N2;
		timic.ICPolarity = LL_TIM_IC_POLARITY_RISING;
		timic.ICPrescaler = LL_TIM_ICPSC_DIV1;
//		LL_TIM_IC_Init(TIM2, LL_TIM_CHANNEL_CH1, &timic);
//		LL_TIM_EnableIT_CC1(TIM2);
//		LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
	
		LL_TIM_IC_Init(TIM2, LL_TIM_CHANNEL_CH3, &timic);
	
		timic.ICPolarity = LL_TIM_IC_POLARITY_FALLING;
		LL_TIM_IC_Init(TIM2, LL_TIM_CHANNEL_CH4, &timic);
	
		NVIC_SetPriority(TIM2_IRQn, 0);
		
		NVIC_EnableIRQ(TIM2_IRQn);
//		LL_TIM_EnableIT_CC1(TIM2);
		LL_TIM_EnableIT_CC3(TIM2);
		LL_TIM_EnableIT_CC4(TIM2);
//		LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
		LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
		LL_TIM_EnableCounter(TIM2);
}
