/***********************************************************************
文件名称：LED.C
功    能：led  IO初始化
编写时间：2013.4.25
编 写 人：
注    意：以下代码仅供参考
***********************************************************************/
#include "stm32f10x.h"

#define RCC_APB2ENR         (*((volatile unsigned int*)0x40021018))   // APB2 外设时钟使能寄存器
#define GPIOE_CRH           (*((volatile unsigned int*)0x40011804))   // 端口配置高寄存器
#define GPIOE_BSRR          (*((volatile unsigned int*)0x40011810))   // 端口位设置/复位寄存器
#define GPIOE_IDR           (*((volatile unsigned int*)0x40011808))   // 端口输入数据寄存器	
#define GPIOE_ODR           (*((volatile unsigned int*)0x4001180C))   // 端口输出数据寄存器
	
#define GPIOB_CRL           (*((volatile unsigned int*)0x40010C00))   // 端口配置低寄存器
#define GPIOB_IDR           (*((volatile unsigned int*)0x40010C08))   // 端口输入数据寄存器	
#define GPIOB_BSRR          (*((volatile unsigned int*)0x40010C10))   // 端口位设置/复位寄存器	

#define PE8	 8
#define PE9	 9
#define PE10 10
#define PE11 11

void LED_Init(void)
{	
	//PE8
	RCC_APB2ENR |= 1<<6;          //使能PORTE时钟	
	GPIOE_CRH &=0XFFFFFFF0;       //清除PE8引脚原来设置  
	GPIOE_CRH |=0x3;			        //设置CNF8[1:0]为0x00：通用推挽输出模式，MODE8[1:0]为0x11：输出模式
	GPIOE_BSRR = 1<<8;           //第8bit置1，则设置对应的ODR8位为1，即PE8引脚输出高电平
	
	//PE9
	GPIOE_CRH &=0XFFFFFF0F;
	GPIOE_CRH |=0x30;			       
	GPIOE_BSRR = 1<<9;         
	
	//PE10 
	GPIOE_CRH &=0XFFFFF0FF;
	GPIOE_CRH |=0x300;	
	GPIOE_BSRR = 1<<10;

	//PE11
	GPIOE_CRH &=0XFFFF0FFF;
	GPIOE_CRH |=0x3000;	
	GPIOE_BSRR = 1<<11;
}



void Delay(unsigned int nCount)
{ 
  while(nCount > 0)
  { 
  	  nCount --;   
  }
}
int abc;

void Togglepin(int pin)
{
	GPIOE_BSRR = 1<<(pin +16);        
	Delay(0xfffff);

	GPIOE_BSRR = 1<<pin; 
		
}
void LED_Turn(void)
{
	int  bcd;
	abc = 1;
	while(1)
	{
		abc ++ ;
		bcd ++ ;
		GPIOE_BSRR = 1<<24;        	//清除对应的ODR8位为0，即PE8引脚输出低电平.BSRR高16位清0，低16位置位 
//		GPIOE_BRR = 1<<8; 				//等效语句
//		GPIOE_ODR &= ~(1 << 8); 	//等效语句
		Delay(0xfffff);
		Delay(0xfffff);
	
	  GPIOE_BSRR = 1<<8; 
		
		Delay(0xfffff);
		Delay(0xfffff);
	}
}

void led_stream(void)
{
	int i;
	for(i=8;i<12;i++)
	{
		Togglepin(i);
	}
}



void KEY_Init(void)
{
		RCC_APB2ENR |=1<<3;    //使能PORTB时钟	
	  GPIOB_CRL &=~(0xf<<24);// PB.6上拉输入	
		GPIOB_CRL |=(0x08<<24);// PB.6上拉输入	
	  //GPIOB_ODR |= (1<<6);
		GPIOB_BSRR = (1<< 6);


}


u8 KEY_Scan(void)
{	 

	u8 key_1= 0;
	if(GPIOB_IDR & 0x40)
			key_1=0;
	else 
  		key_1=1;
	if(key_1==1)
	{
		delay_ms(100);//去抖动 
		if(GPIOB_IDR & 0x40)
			key_1=0;

			
		if(key_1==1)
			return 1;
	} 
 	return 0;// 无按键按下
}
