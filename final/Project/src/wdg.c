#include "wdg.h"
#include "main.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板V3
//看门狗 驱动代码		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/10
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	  
  
//初始化独立看门狗
//prer:分频数:0~7(只有低3位有效!)
//分频因子=4*2^prer.但最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/40 (ms).
void IWDG_Init(u8 prer,u16 rlr) 
{
	IWDG->KR=0X5555;//使能对IWDG->PR和IWDG->RLR的写		 										  
  	IWDG->PR=prer;  //设置分频系数   
  	IWDG->RLR=rlr;  //从加载寄存器 IWDG->RLR  
	IWDG->KR=0XAAAA;//reload											   
  	IWDG->KR=0XCCCC;//使能看门狗	
}
//喂独立看门狗
void IWDG_Feed(void)
{
	IWDG->KR=0XAAAA;//reload											   
}

void IWDG_test()
{
	LED_Configuration();		  	//初始化与LED连接的硬件接口  
	Count_KEY_Init();         	//初始化与按键连接的硬件接口
 	delay_ms(1000);			        //延时500ms,让人可以看到DS0灭的状态
 	IWDG_Init(4,625);    	//与分频数为64,重载值为625,溢出时间为1s	   
	One_LED_ON(1);				 	//点亮灯
	while(1)
	{
		if(KEY_Scan()==1)    //如果计数按键按下,则喂狗
		{
			IWDG_Feed();
		}
		delay_ms(10);
	}		 
}



















