#include "stm32F10x_tim.h"


void tim6_init(uint16_t period, uint16_t prescaler)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;    
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	// void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	

	TIM_TimeBaseInitStructure.TIM_Period            = period;
	TIM_TimeBaseInitStructure.TIM_Prescaler         = prescaler;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);
	
	// ?? TIM6 ? NVIC
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ARRPreloadConfig(TIM6, ENABLE);
	TIM_ITConfig(TIM6, TIM_IT_Update, DISABLE);   // ??????,???????
	TIM_Cmd(TIM6, DISABLE);  // ?????
}

// ???????
void tim6_start(uint16_t arr)
{
	TIM6->ARR = arr;   // ??????
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM6, ENABLE);
}
