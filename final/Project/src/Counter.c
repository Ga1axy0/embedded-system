/***********************************************************************
文件名称：LED.C
功    能：led  IO初始化
编写时间：2013.4.25
编 写 人：
注    意：
***********************************************************************/
#include "stm32f10x.h"



void Counter_Configuration()
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_ClocksTypeDef RCC_ClockFreq;
//	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;	
//	TIM_ICInitTypeDef TIM_ICInitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	
//	
//	/*PB6-> ?????? */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M????
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//		
//	
//	  TIM_TimeBaseStructure.TIM_Period = 0x1000;   //ARR,     
//	  TIM_TimeBaseStructure.TIM_Prescaler = 0;       
//	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
//	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
//	  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
//	
//	  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
//	  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//	  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//	  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//	  TIM_ICInitStructure.TIM_ICFilter = 0x0;
//	  
//	  TIM_ICInit(TIM4, &TIM_ICInitStructure);
//	  TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);
//	
//	  /* Select the slave Mode: Reset Mode */
//	  TIM4->SMCR |= 0x07 ;  //外部触发模式
//	  /* Enable the Master/Slave Mode */

//	TIM_SetCounter(TIM4,0);
//	TIM_Cmd(TIM4, ENABLE);
	
		/*****************寄存器版**********************/
		RCC->APB2ENR|=1<<3;    //使能PORTB时钟	
		GPIOB->CRL|=0x04000000;  //PB.6浮空输入  
	  
		RCC->APB1ENR|=1<<2;    //使能TIM4时钟	
		TIM4->ARR=0x1000;  	//设定计数器自动重装值
		TIM4->PSC=0;  	    //不预分频
		
		TIM4->CCMR1|=1<<0;		//CC1S=01 	选择输入端 IC1映射到TI1上
		TIM4->CCMR1|=0<<4; 		//IC1F=0000 配置输入滤波器 不滤波
		TIM4->CCMR1|=0<<10; 	//IC2PS=00 	配置输入分频,不分频 

		TIM4->CCER|=0<<1; 		//CC1P=0	上升沿捕获
		TIM4->CCER|=1<<0; 		//CC1E=1 	允许捕获计数器的值到捕获寄存器中
		
		TIM4->SMCR = 0x50 ;   //选择同步计数器的触发模式为定时器输入1
	  TIM4->SMCR |= 0x07;  //外部触发模式
		TIM4->CNT=0;         //初始化计数器
		TIM4->CR1|=0x01;    //使能定时器4*/
		
}


int TimeCnt=0;
void Counter_test()
{
	  LED_Configuration();
		Counter_Configuration();
		GPIOE->BSRR = GPIO_Pin_8;
		GPIOE->BSRR = GPIO_Pin_9;
		GPIOE->BSRR = GPIO_Pin_10;
		GPIOE->BSRR = GPIO_Pin_11;
		while(1)
		{
			TimeCnt = TIM4->CNT;
			if(TimeCnt%2==0)
				GPIOE->BSRR = GPIO_Pin_8;
			else GPIOE->BRR = GPIO_Pin_8;
		}
}