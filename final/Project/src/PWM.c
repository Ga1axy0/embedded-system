/***********************************************************************
文件名称：LED.C
功    能：led  IO初始化
编写时间：2013.4.25
编 写 人：
注    意：
***********************************************************************/
#include "stm32f10x.h"

void PWM_Configuration(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE|RCC_APB2Periph_TIM1|RCC_APB2Periph_AFIO, ENABLE);	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE); //Timer1?????  ?????????? tim1??????? ???tim1
	
// 	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_8 | GPIO_Pin_9  ;  		//
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
// 	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_0 | GPIO_Pin_1  ;  		//
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);	
// 	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_8 | GPIO_Pin_13  ;  		//
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_Init(GPIOE, &GPIO_InitStructure);		
	
		GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13  ;  		//
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOE, &GPIO_InitStructure);	
	
		//TIM_DeInit(TIM1); //
	
    //配置时间基，按此配置，TIM2频率为72M/TIM_Period
		TIM_TimeBaseStructure.TIM_Period = arr;   						//周期       
		TIM_TimeBaseStructure.TIM_Prescaler = psc;                        //不分频
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;                    //时钟分割为0  
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //向上计数模式    
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

    //配置输出模式
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;               //脉冲宽度调制模式1 
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //输出状态使能  
		//互补输出使能
   // TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; 
		
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;        //输出极性低
    //TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		//TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
		//TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
		//TIM_OCInitStructure.TIM_Pulse = 0;  							//占空比
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);						//T3-CH1
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);		
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);						//T3-CH2
		TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);		
		TIM_OC3Init(TIM1, &TIM_OCInitStructure);						//T3-CH3
		TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);	
		
		//TIM_ARRPreloadConfig(TIM1, ENABLE);
		TIM_Cmd(TIM1, ENABLE) ;

		//TIM1->RCR = 0;                   
		//TIM_CtrlPWMOutputs(TIM1,ENABLE);        //TIM1定时器
   		
}

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)
void PWM_test()
{
		u16 pwmval=0;
		u8 dir =1;
		SCB->AIRCR = AIRCR_VECTKEY_MASK | NVIC_PriorityGroup_2;     //设置NVIC中断分组2：2位抢占优先级，2位响应优先级
	  LED_Configuration();
//		GPIOE->BSRR = GPIO_Pin_8;
		GPIOE->BSRR = GPIO_Pin_9;
//		GPIOE->BSRR = GPIO_Pin_10;
		GPIOE->BSRR = GPIO_Pin_11;
	while(1)
	{
		delay_ms(200);
		if(dir) pwmval++;
		else pwmval--;
		if(pwmval>=900)dir=0;
		if(pwmval==0) dir=1;
		TIM1->CCR1 = pwmval; //时钟计数值最大值5000，与电压最大值5000范围相同，所以直接赋值
		//TIM1->CCR2 = 0 ; //时钟计数值最大值5000，与电压最大值5000范围相同，所以直接赋值
//		TIM1->CCR3 = 3000 ; //时钟计数值最大值5000，与电压最大值5000范围相同，所以直接赋值
	}
}