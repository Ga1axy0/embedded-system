#ifndef __EXTI_H
#define __EXTI_H
#include "stm32f10x.h"



void MY_NVIC_PriorityGroupConfig(void);
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);
void EXTIX_Init(void);
void EXTI9_5_IRQHandler(void);
void EXTI_test(void);


#endif

/******END OF FILE****/