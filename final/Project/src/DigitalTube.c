#include "DigitalTube.h"
#include "stm32f10x.h"


unsigned char smgduan[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,
												 0x80,0x90,0x88,0x83,0xc6,0xa1,0x86,0x8e};

											
void DigitalTube_Configuration(void)
{
	/*******************库函数版************************/
//	GPIO_InitTypeDef GPIO_InitStructure;
//	/* Enable the GPIO_LED Clock */
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOC , ENABLE); 						 
//			
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | \
//					GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(GPIOD, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	/*****************寄存器版**********************/
	RCC->APB2ENR|=3<<4;    //使能PORTC和PORTD时钟	 
	GPIOC->CRH&=0xFF0000FF; 
	GPIOC->CRH|=0x00333300;//PC.10 11 12 13 推挽输出   	
//	GPIOC->ODR|=15<<10;      //PC.10 11 12 13 输出高

	GPIOD->CRL&=0X00000000; 
	GPIOD->CRL|=0X33333333;//PD.0 1 2 3 4 5 6 7 推挽输出   
//  GPIOD->ODR |= 0xFF;      //PD.0 1 2 3 4 5 6 7 输出高
	
	GPIOD->BSRR = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;      //输出高电平
	GPIOD->BSRR = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIOC->BSRR = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
}

void DigitalTube_test(void)
{
		DigitalTube_Configuration();
		//SEG_Configuration();
		while(1)
		{
				GPIOC->BRR = GPIO_Pin_10;
				GPIOC->BSRR = GPIO_Pin_11;
				GPIOC->BSRR = GPIO_Pin_12;
				GPIOC->BSRR = GPIO_Pin_13;
				GPIOD->ODR &= 0xff00;
				GPIOD->ODR |= 0xc0;
				delay_ms(5);
			
				GPIOC->BRR = GPIO_Pin_11;
				GPIOC->BSRR = GPIO_Pin_10;
				GPIOC->BSRR = GPIO_Pin_12;
				GPIOC->BSRR = GPIO_Pin_13;
				GPIOD->ODR &= 0xff00;
				GPIOD->ODR |= 0xf9;
				delay_ms(5);
			
				GPIOC->BRR = GPIO_Pin_12;
				GPIOC->BSRR = GPIO_Pin_11;
				GPIOC->BSRR = GPIO_Pin_10;
				GPIOC->BSRR = GPIO_Pin_13;
				GPIOD->ODR &= 0xff00;
				GPIOD->ODR |= 0xa4;
				delay_ms(5);
				
				GPIOC->BRR = GPIO_Pin_13;
				GPIOC->BSRR = GPIO_Pin_11;
				GPIOC->BSRR = GPIO_Pin_12;
				GPIOC->BSRR = GPIO_Pin_10;
				GPIOD->ODR &= 0xff00;
				GPIOD->ODR |= 0xb0;
				delay_ms(5);
				
		}
}
