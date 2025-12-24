/***********************************************************************
文件名称：LED.C
功    能：led  IO初始化
编写时间：2013.4.25
编 写 人：
注    意：
***********************************************************************/
#include "stm32f10x.h"
#include "stdio.h"
#include "Register_config.h"

#define _STD_LIB
//#define _STD_REG_MODE
//#define _SELF_REG_MODE

#define ROW_PORT GPIOE
#define COL_PORT GPIOE

#define ROW_1 GPIO_Pin_4
#define ROW_2 GPIO_Pin_5
#define ROW_3 GPIO_Pin_6
#define ROW_4 GPIO_Pin_7

#define COL_1 GPIO_Pin_0
#define COL_2 GPIO_Pin_1
#define COL_3 GPIO_Pin_2
#define COL_4 GPIO_Pin_3

void delay_ms(int n)
{
	int i ;
	for (i = 0 ; i < n ;i ++);
}
	

void KEY_Configuration(void)
{
	#ifdef _STD_LIB
		GPIO_InitTypeDef GPIO_InitStructure;  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 |GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; //行
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(ROW_PORT,&GPIO_InitStructure); 
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; //列
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(GPIOE,&GPIO_InitStructure); 
	#endif 
	
	#ifdef  _STD_REG_MODE 
	/*****************寄存器版**********************/
		RCC->APB2ENR|=1<<6;    //使能PORTE时钟	
		GPIOE->CRL=0X33338888;//PE.0 1 2 3  上拉输入  4 5 6 7 推挽输出  
	#endif 
	#ifdef _SELF_REG_MODE
	  RCC_APB2ENR |= 1<<6;      //使能PORTE时钟	
		GPIOE_CRL =0X33338888;    //PE.0 1 2 3  上拉输入  4 5 6 7 推挽输出  
	#endif
}



int key_value = 0 ;
int key_col = 0,key_row=0 ;
int key_code = 0 ;


int  KEY_read(void)
{


	key_code = 0 ;
	GPIO_ResetBits(ROW_PORT,ROW_1);
	GPIO_SetBits(ROW_PORT,ROW_2 |ROW_3|ROW_4);
	key_row = ROW_2+ROW_3+ROW_4;	
	key_col=GPIO_ReadInputData(COL_PORT) & 0xf ;
	if ( key_col != 0xf )
		key_code =  key_row+key_col ;

	
	GPIO_ResetBits(ROW_PORT,ROW_2);
	GPIO_SetBits(ROW_PORT,ROW_1 |ROW_3|ROW_4);
	key_row = ROW_1+ROW_3+ROW_4;	
	key_col=GPIO_ReadInputData(COL_PORT) & 0xf ;	
	if ( key_col != 0xf )
		key_code =  key_row+key_col ;
	
	
	GPIO_ResetBits(ROW_PORT,ROW_3);
	GPIO_SetBits(ROW_PORT,ROW_1 |ROW_2|ROW_4);
	key_row = ROW_1+ROW_2+ROW_4;	
	key_col=GPIO_ReadInputData(COL_PORT) & 0xf ;		
	if ( key_col != 0xf )
		key_code =  key_row+key_col ;

	
	GPIO_ResetBits(ROW_PORT,ROW_4);
	GPIO_SetBits(ROW_PORT,ROW_1 |ROW_2|ROW_3);
	key_row = ROW_1+ROW_2+ROW_3;	
	key_col=GPIO_ReadInputData(COL_PORT) & 0xf ;		
	if ( key_col != 0xf )
		key_code =  key_row+key_col ;

	return key_code;
}


u8 KEY_Scan()
{	 
	u8 key_1 = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
	if(key_1==0)
	{
		delay_ms(100);//去抖动 
		if(key_1==0)return 1;
	} 
 	return 0;// 无按键按下
}

int keydown=0;
void key_test()
{
	KEY_Configuration();
	while(1)
	{
		keydown = KEY_read();
		if(keydown)
		{

			keydown=0;
			delay_ms(2000);
		}
	}
}