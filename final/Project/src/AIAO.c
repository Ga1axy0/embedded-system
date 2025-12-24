/***********************************************************************
文件名称：LED.C
功    能：led  IO初始化
编写时间：2013.4.25
编 写 人：
注    意：
***********************************************************************/
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dac.h"
void AI_Configuration()
{      
////ADC手动转换,在定时器里启动转换
//		ADC_InitTypeDef ADC_InitStructure;

//	  GPIO_InitTypeDef GPIO_InitStructure;
//  /* ADC1 configuration ------------------------------------------------------*/
//		RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1| RCC_APB2Periph_GPIOC, ENABLE); 		
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);	
//	
//  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
////	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
//  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
//  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
//  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//  ADC_InitStructure.ADC_NbrOfChannel = 1;
//  ADC_Init(ADC1, &ADC_InitStructure);

//// Enable ADC1 
//  ADC_Cmd(ADC1, ENABLE);

//  /* Enable ADC1 reset calibaration register */   
//  ADC_ResetCalibration(ADC1);
//  /* Check the end of ADC1 reset calibration register */
//  while(ADC_GetResetCalibrationStatus(ADC1));

//  /* Start ADC1 calibaration */
//  ADC_StartCalibration(ADC1);
//  /* Check the end of ADC1 calibration */
//  while(ADC_GetCalibrationStatus(ADC1));
//	ADC_SoftwareStartConvCmd(ADC1,ENABLE);	
	

	
	/*****************寄存器版**********************/
	//先初始化IO口
 	RCC->APB2ENR|=1<<4;    //使能PORTC口时钟 
	GPIOC->CRL&=0XFFFFFF0F;//PC1 anolog输入 
	RCC->APB2ENR|=1<<9;    //ADC1时钟使能	  
	RCC->APB2RSTR|=1<<9;   //ADC1复位
	RCC->APB2RSTR&=~(1<<9);//复位结束	    
	RCC->CFGR&=~(3<<14);   //分频因子清零	
	//SYSCLK/DIV2=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!
	//否则将导致ADC准确度下降! 
	RCC->CFGR|=2<<14;      	 
	ADC1->CR1&=0XF0FFFF;   //工作模式清零
	ADC1->CR1|=0<<16;      //独立工作模式  
	ADC1->CR1&=~(1<<8);    //非扫描模式	  
	ADC1->CR2&=~(1<<1);    //单次转换模式
	ADC1->CR2&=~(7<<17);	   
	ADC1->CR2|=7<<17;	   //软件控制转换  
	ADC1->CR2|=1<<20;      //使用用外部触发(SWSTART)!!!	必须使用一个事件来触发
	ADC1->CR2&=~(1<<11);   //右对齐	 
	
	ADC1->CR2|=1<<23;      //使能温度传感器
	
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1|=0<<20;     //1个转换在规则序列中 也就是只转换规则序列1 			
	
	//设置通道11的采样时间
	ADC1->SMPR1&=~(3*1);   //通道11采样时间清空	  
 	ADC1->SMPR1|=7<<(3*1); //通道11  239.5周期,提高采样时间可以提高精确度	 
	
	ADC1->SMPR1&=~(7<<3*6);//清除通道16原来的设置	 
	ADC1->SMPR1|=7<<(3*6); //通道16  239.5周期,提高采样时间可以提高精确度	 
	
	ADC1->CR2|=1<<0;	   //开启AD转换器	 
	ADC1->CR2|=1<<3;       //使能复位校准  
	while(ADC1->CR2&1<<3); //等待校准结束 			 
    //该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。 		 
	ADC1->CR2|=1<<2;        //开启AD校准	   
	while(ADC1->CR2&1<<2);  //等待校准结束
	//该位由软件设置以开始校准，并在校准结束时由硬件清除  
	
}


void DAC1_Configuration(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    DAC_InitTypeDef DAC_InitType;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );      //??PORTA????
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;                 // ????
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;          //?`???
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
     
	
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );      //??DAC????



    DAC_InitType.DAC_Trigger=DAC_Trigger_None;    //??????? TEN1=0
    DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//???????
    DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//???????
    DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;    //DAC1?????? BOFF1=1
    DAC_Init(DAC_Channel_1,&DAC_InitType);     //???DAC??1

    DAC_Cmd(DAC_Channel_1, ENABLE);  //??DAC1

    DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12??????????DAC?



}


//获得ADC1某个通道的值
//ch:通道值 0~16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	//设置转换序列	  		 
	ADC1->SQR3&=0XFFFFFFE0;//规则序列1 通道ch
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<22;       //启动规则转换通道 
	while(!(ADC1->SR&1<<1));//等待转换结束	 	   
	return ADC1->DR;		//返回adc值	
}
//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 
//得到温度值
//返回值:温度值(扩大了100倍,单位:℃.)
short Get_Temprate(void)
{
	u32 adcx;
	short result;
 	double temperate;
	adcx=Get_Adc_Average(16,20);	//读取通道16,20次取平均
	temperate=(float)adcx*(3.3/4096);		//电压值 
	temperate=(1.43-temperate)/0.0043+25;	//转换为温度值 	 
	result=temperate*100;					//扩大100倍.
	return result;
}

//vol:0~3300,??0~3.3V
void AO_test(u16 dac)
{
    DAC_SetChannel1Data(DAC_Align_12b_R,dac);//12??????????DAC?
		//DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
}


u16 AItmp,Temprate;
int flag=20;
void AIAO_test()
{

	while(1)
	{
		AItmp=Get_Adc(11);
		Temprate=Get_Temprate();
		if(flag-->0)
		{
			printf("%d\t%.2f℃\n",AItmp,(float)Temprate/100);
		}
		//ADC_RegularChannelConfig(ADC1, ADC_Channel_11 , 1, ADC_SampleTime_28Cycles5 );		  			    
		//ADC_SoftwareStartConvCmd(ADC1, ENABLE);   // 转换一次
		
		AO_test(AItmp);
		delay_ms(1000);	
	}
}
