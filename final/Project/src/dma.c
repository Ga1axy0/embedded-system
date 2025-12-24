//DMA.c
#include "dma.h"
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_dma.h"


void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;				//????????
	SysTick->VAL = 0x00;					  //???????
	SysTick->CTRL = 0x00000005;			//??????HCLK,?????
	while(!(SysTick->CTRL & 0x00010000));	//?????0
	SysTick->CTRL = 0x00000004;			//?????
}


void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 

void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 



uint16_t MyDMA_Size;                    // 传输计数器

void MyDMA_Init(uint32_t *AddrA, uint32_t *AddrB, uint16_t Size)
{
	MyDMA_Size = Size;                    
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);      // 开启DMA1总线时钟
	
	DMA_InitTypeDef DMA_InitStructure;                      // 初始化结构体
																													
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)AddrA;       // 外设站点基地址
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;  // 每次转运的数据宽度，字节传输
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;          // 转运完成一次之后地址自增
	
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AddrB;           // 目的站点基地址，后面两个参数原理与前面一样
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;      // 外设站点作为源数据地址 传输方向为 A -- B
	DMA_InitStructure.DMA_BufferSize = Size;                // 传输单元的个数，即传送次数
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;           // 传输模式，由于是传送数组，选择非循环模式，只传输一轮，不自动重装。
	DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;             // 操作的是两个内存中的数组，选择存储器到存储器模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;   // 指定通道的优先级，一个通道随便选
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);            // DMA1，通道1
	
	DMA_Cmd(DMA1_Channel1, DISABLE);                        // 初始化函数先失能DMA
}

void MyDMA_Transfer(uint32_t size)
{
	DMA_Cmd(DMA1_Channel1, DISABLE);                        // 转运之前先关闭DMA
	DMA1_Channel1->CNDTR = size; //注意根据手册描述，这个是一个递减计数器，指示剩余要传输的数据个数
//	DMA_SetCurrDataCounter(DMA1_Channel1, MyDMA_Size);      // 设置传输计数器大小，V3.5以上的版本带这个函数，3.0不带
	DMA_Cmd(DMA1_Channel1, ENABLE);                         // 开启DMA，开始转运 
	
	while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);      // 等待转运完成 DMA1_FLAG_TC1 DMA1，通道1转运完成标志位
	DMA_ClearFlag(DMA1_FLAG_TC1);                           // 清除标志位
}



