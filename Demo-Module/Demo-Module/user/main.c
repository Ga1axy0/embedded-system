/**
  ******************************************************************************
  * @file InputCaptureMode/main.c 
  * @author   MCD Application Team
  * @version  V3.0.0
  * @date     04/27/2009
  * @brief    Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/** @addtogroup InputCaptureMode
  * @{
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
extern void NEG_init(void);
extern void Keyborad_init(void); // 修正函数名: 与 NEG.c 中保持一致
extern void Keyborad_with_Display(int *a); // 修正拼写: Keyborad
void BoardInit()
{
  /* System Clocks Configuration */
	SystemInit();
}

int main(void)
{
	u8 t=0;
	// 在这里写初始化代码，例如初始化按键连接的引脚为输入，led连接的引脚为输出
	//可以是函数，也可以直接写代码。建议使用函数调用方式，提高程序主题的可读性。
  NEG_init();
  Keyborad_init();

  int a[4]={1,2,3,4};

	while(1)
	{
  Keyborad_with_Display(a); // 中断驱动更新 + 定时器自动扫描显示
	}
	return 1;
}



/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
