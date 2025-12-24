/***********************************************************************
文件名称：LED.C
功    能：led  IO初始化
编写时间：2013.4.25
编 写 人：
注    意：
***********************************************************************/
#include "stm32f10x.h"
#include "Register_config.h"

#define LED1_ON 		GPIO_ResetBits(GPIOE , GPIO_Pin_8)
#define LED2_ON 		GPIO_ResetBits(GPIOE , GPIO_Pin_9)
#define LED3_ON 		GPIO_ResetBits(GPIOE , GPIO_Pin_10)
#define LED4_ON 		GPIO_ResetBits(GPIOE , GPIO_Pin_11)


#define LED1_OFF 		GPIO_SetBits(GPIOE , GPIO_Pin_8)
#define LED2_OFF 		GPIO_SetBits(GPIOE , GPIO_Pin_9)
#define LED3_OFF 		GPIO_SetBits(GPIOE , GPIO_Pin_10)
#define LED4_OFF 		GPIO_SetBits(GPIOE , GPIO_Pin_11)

void LED_Configuration(void);
void LED_Blink(void);
void One_LED_ON(unsigned char led_num);

int Delay(uint32_t nCount);
/***********************************************************************
函数名称：LED_Configuration(void)
功    能：完成LED的配置
输入参数：
输出参数：
编写时间：2013.4.25
编 写 人：
注    意：LED1->PE2,
		  LED2->PE3,
		  LED3->PE4,
		  LED4->PE5,
***********************************************************************/

	



void LED_TURN(void)
{
	LED_Configuration();
	while(1)
	{
		GPIOE_BSRR &= ~(1<<8);      //清除第8bit
		GPIOE_BSRR |= 1<<24;         //清除对应的ODR8位为0，即PE8引脚输出低电平
	
		Delay(0xfffff);
		Delay(0xfffff);
	
	  GPIOE_BSRR |= 1<<8; 
		
		Delay(0xfffff);
		Delay(0xfffff);
	}

}
/***********************************************************************
函数名称：LED_Blink(void)
功    能：完成LED闪烁
输入参数：
输出参数：
编写时间：2013.4.25
编 写 人：
注    意：
***********************************************************************/
void LED_Blink(void)
{
	LED_Configuration();	//LED配置 
	while(1)
	{
		GPIO_SetBits(GPIOE , GPIO_Pin_8);
		GPIO_SetBits(GPIOE , GPIO_Pin_9);
		GPIO_SetBits(GPIOE , GPIO_Pin_10);
		GPIO_SetBits(GPIOE , GPIO_Pin_11);
		Delay(0xfffff);
		Delay(0xfffff);
		/*====LED-OFF=======*/ 
		GPIO_ResetBits(GPIOE , GPIO_Pin_8);
		GPIO_ResetBits(GPIOE , GPIO_Pin_9);
		GPIO_ResetBits(GPIOE , GPIO_Pin_10);
		GPIO_ResetBits(GPIOE , GPIO_Pin_11);
		Delay(0xfffff);
		Delay(0xfffff); 
	}
}
/***********************************************************************
函数名称：One_LED_ON(unsigned char led_num)
功    能：实现点亮一个LED灯
输入参数：
输出参数：
编写时间：2013.4.25
编 写 人：
注    意：
***********************************************************************/
void One_LED_ON(unsigned char led_num)
{	
	/*****熄灭四个led灯******/
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4_OFF;
	switch(led_num)
	{
		case 1:
		{
			LED1_ON;
			break;
		}
		case 2:
		{
			LED2_ON;
			break;		
		}
		case 3:
		{
			LED3_ON;
			break;		
		}
		case 4:
		{
			LED4_ON;
			break;		
		}
		default:
		{
			break;	
		}
	}		
}

void One_LED_OFF(unsigned char led_num)
{	
	switch(led_num)
	{
		case 1:
		{
			LED1_OFF;
			break;
		}
		case 2:
		{
			LED1_OFF;
			break;		
		}
		case 3:
		{
			LED1_OFF;
			break;		
		}
		case 4:
		{
			LED1_OFF;
			break;		
		}
		default:
		{
			break;	
		}
	}		
}

int Delay(uint32_t nCount)
{ 
  while(nCount > 0)
  { 
  	  nCount --;   
  }
}