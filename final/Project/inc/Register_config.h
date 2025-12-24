#ifndef __Register_Config_H
#define __Register_Config_H

#define RCC_AHBENR          (*((volatile unsigned int*)0x40021014))   // AHB外设时钟使能寄存器
#define RCC_APB2ENR         (*((volatile unsigned int*)0x40021018))   // APB2 外设时钟使能寄存器
#define RCC_APB1ENR         (*((volatile unsigned int*)0x4002101C))   // APB1 外设时钟使能寄存器


#define GPIOE_BASE_ADDR          (0x40011800)
#define GPIOE_CRL           (*((volatile unsigned int*)(GPIOE_BASE+0x00)))   // 端口配置低寄存器
#define GPIOE_CRH           (*((volatile unsigned int*)(GPIOE_BASE+0x04)))   // 端口配置高寄存器
#define GPIOE_IDR           (*((volatile unsigned int*)(GPIOE_BASE+0x08)))   // 端口输入数据寄存器
#define GPIOE_ODR           (*((volatile unsigned int*)(GPIOE_BASE+0x0C)))   // 端口输出数据寄存器
#define GPIOE_BSRR          (*((volatile unsigned int*)(GPIOE_BASE+0x10)))   // 端口位设置/复位寄存器
#define GPIOE_BRR           (*((volatile unsigned int*)(GPIOE_BASE+0x14)))   // 端口位复位寄存器

#define GPIOB_BASE_ADDR          (0x40010C00)
#define GPIOB_CRL           (*((volatile unsigned int*)0x40010C00))   // 端口配置低寄存器
#define GPIOB_BSRR           (*((volatile unsigned int*)0x40010C10))   // 端口配置低寄存器


#define DMA_BASE_ADDR            (0x40020000)
#define DMA_ISR             (*((volatile unsigned int*)0x40020000))   // DMA中断状态寄存器
#define DMA_IFCR            (*((volatile unsigned int*)0x40020004))   // DMA中断标志清除寄存器
#define DMA_CCR4	  				(*((volatile unsigned int*)0x40020044))   // DMA通道4配置寄存器地址
#define DMA_CCR5	 				  (*((volatile unsigned int*)0x40020058))   // DMA通道5配置寄存器地址
#define DMA_CNDTR4	 			  (*((volatile unsigned int*)0x40020048))   // DMA通道4传输数量寄存器地址
#define DMA_CNDTR5	  		 	(*((volatile unsigned int*)0x4002005C))   // DMA通道5传输数量寄存器地址
#define DMA_CPAR4	 				  (*((volatile unsigned int*)0x4002004C))   // DMA通道4外设地址寄存器地址
#define DMA_CPAR5	  				(*((volatile unsigned int*)0x40020060))   // DMA通道5外设地址寄存器地址
#define DMA_CMAR4	  				(*((volatile unsigned int*)0x40020050))   // DMA通道4存储器地址寄存器地址
#define DMA_CMAR5	  				(*((volatile unsigned int*)0x40020064))   // DMA通道5存储器地址寄存器地址
	
#define USART1_BASE_ADDR          (0x40013800) 
#define USART1_DR	  				(*((volatile unsigned int*)0x40013804))   // 串口1数据寄存器地址
#define USART1_CR3	        (*((volatile unsigned int*)0x40013814))   //控制寄存器 3
	

#define SCB_AIRCR           (*((volatile unsigned int*)0xE000ED0C))   //应用中断/复位控制寄存器
#define NVIC_ISER           ((volatile unsigned int*)0xE000E100)      //Irq使能设置寄存器
#define NVIC_IP             ((volatile unsigned char*)0xE000E400)     //Irq优先级寄存器(在 STM32 中只使用 4 位，高 4 位有效)
#define EXTI_IMR            (*((volatile  unsigned int*)0x40010400))  //外部中断屏蔽寄存器
#define EXTI_RTSR           (*((volatile  unsigned int*)0x40010408))	//上升沿触发选择寄存器
#define EXTI_FTSR           (*((volatile  unsigned int*)0x4001040c))  //下降沿触发选择寄存器
#define EXTI_PR             (*((volatile  unsigned int*)0x40010414))  //挂起寄存器
#define AFIO_EXTICR2        (*((volatile  unsigned int*)0x4001000C))  //外部中断配置寄存器2
#endif
/******END OF FILE****/
