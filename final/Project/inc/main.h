#ifndef __STM32_MAIN_H
#define __STM32_MAIN_H

#include "stm32f10x.h"
#include "sys.h"
#include "DigitalTube.h"
#include "wdg.h"
#include "dma.h"		
#include "lcd.h"		
#include "EXTI.h"
#include "Register_config.h"

void LED_Configuration(void);
void LED_Blink(void);
void One_LED_ON(unsigned char led_num);
void One_LED_OFF(unsigned char led_num);
void LED_TURN(void);
	
void BEEP_Configuration(void);
void Beep(void);

void KEY_Configuration(void);
void Count_KEY_Init(void);
int  KEY_read(void);
void key_test(void);
u8 KEY_Scan(void);

void uart_test(void);
void uart1_init(void);

void IWDG_test(void);

void AI_Configuration(void);
void DAC1_Configuration(void);
void AIAO_test(void);

void LCD_Configuration(void);
void LCD_test(void);
void Touch_test(void);		

void Counter_test(void);

void PWM_Configuration(u16 arr,u16 psc);
void PWM_test(void);

void DMA_test(void);
void DMA_test1(void);

void ENC25Q80_SPI2_Init(void);
u32 SPI_FLASH_ReadDeviceID(void);

void IIC_Configuration(void);
void IIC_test(void);

void delay_init(u8 SYSCLK);
void delay_s(u16 ns);
void delay_ms(u16 nms);
void delay_us(u32 nus);
#endif
/******END OF FILE****/
