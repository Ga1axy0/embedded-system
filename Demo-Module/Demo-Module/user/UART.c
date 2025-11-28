/***********************************************************************
文件名称：LED.C
功    能：led  IO初始化
编写时间：2013.4.25
编 写 人：
注    意：
***********************************************************************/
#include "stm32f10x.h"
#include <stm32f10x_usart.h>
#include "stdio.h"
#include "stdint.h"

extern const uint32_t SystemFrequency; // 声明外部变量


void RS232_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 	USART_InitTypeDef USART_InitStructure; 
	//引脚时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//串口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


  /*
  *  USART1_TX -> PA9 , USART1_RX ->	PA10
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		//串口1初始化
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);//发送完成中断
	//USART_ClearITPendingBit(USART1, USART_IT_TXE);//清除中断TXE位
	USART_Cmd(USART1, ENABLE);
	
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
  
  /* Enable the Ethernet global Interrupt */
  
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//如果还有其他中断，按照下面类似的增加即可
//	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	  
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure); 
}
/***********************************************************************
函数名称：void USART1_IRQHandler(void) 
功    能：完成SCI的数据的接收，并做标识
输入参数：
输出参数：
编写时间：2012.11.22
编 写 人：
注    意  RS485用的是USART3.
***********************************************************************/


u8 RS232InData;
#define USART_BUF_LEN  			200  	//定义最大接收字节数 200
u8 USART_Rxbuf[USART_BUF_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8 USART_Txbuf[USART_BUF_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 RXPos=0;
u16 FrameFlag = 0;
u16 RecvTimeOver=0;
u16 SendPos,SendBufLen;

void USART1_IRQHandler(void)  
{
		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(
		{
			if(RXPos<=USART_BUF_LEN)//缓冲区未满
			{
				USART_Rxbuf[RXPos]=USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据 ;
				RXPos++;
			}
			RecvTimeOver = 10 ;  //每次接收到数据，超时检测时间10ms
    } 	

		if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) 
		{
				//USART_ClearITPendingBit(USART1, USART_IT_TXE);           /* Clear the USART transmit interrupt       */
			  if ( SendPos < SendBufLen )
				{
					USART_SendData(USART1,USART_Txbuf[SendPos]);
					SendPos ++ ;
				}
				else
    {
        USART_ITConfig(USART1, USART_IT_TXE, DISABLE); // <— 发送完禁用TXE中断
    }

		}	
}


void RS232_test()
{
    // 等待上一次发送完成 (检查 TXE 中断是否已禁用)
    while ((USART1->CR1 & USART_CR1_TXEIE) == USART_CR1_TXEIE);

    char msg[] = "Hello\r\n";
    SendBufLen = 7;
    memcpy(USART_Txbuf, msg, SendBufLen);

    SendPos = 0;
    //USART_SendData(USART1, USART_Txbuf[SendPos++]); // 先发一个字节
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);   // 再开TXE中断
}



void uart1_init()	
{
	RS232_Configuration();		
	NVIC_Configuration();
	// 配置 SysTick 为 1ms 中断，用于 Modbus 断帧检测
	SysTick_Config(SystemFrequency / 1000);
}

void SysTick_Handler(void)
{
	if(RecvTimeOver > 0) {
		RecvTimeOver--;
	if(RecvTimeOver == 0) {
	// reception idle for required timeout -> frame complete
		FrameFlag = 1;
	}
	}
}

#define MODBUS_SLAVE_ID    0x01
#define HOLD_REG_NUM       6200

u16 HoldReg[HOLD_REG_NUM];

u16 Modbus_CRC16(const u8 *buf, u16 len)
{
    u16 crc = 0xFFFF;
    for(u16 i = 0; i < len; i++) {
        crc ^= buf[i];
        for(u8 j = 0; j < 8; j++) {
            if(crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}



void Modbus_SendBuffer(u8 *buf, u16 len)
{
    if(len == 0 || len > USART_BUF_LEN) return;

#ifdef USE_RS485
    // set DE high -> transmit
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
    // small delay to allow driver to enable (optional)
    for(volatile int i=0;i<100;i++);
#endif

    memcpy((void*)USART_Txbuf, buf, len);
    SendBufLen = len;
    SendPos = 0;

    // Put first byte into DR to start TX. This will make TXE=1 when DR empty after it's moved to shift register.
    USART_SendData(USART1, USART_Txbuf[SendPos++]);

    // Enable TXE interrupt for following bytes
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

// -----------------------------------------------------------------------------
// Modbus parser / handler for 0x03 and 0x10
// - Call when FrameFlag == 1 (frame timeout)
// -----------------------------------------------------------------------------
void Modbus_Analyze(void)
{
    if(FrameFlag == 0) return;
    FrameFlag = 0;

    u16 len = RXPos;
    if(len < 4) { RXPos = 0; return; } // too short

    // check CRC
    u16 crc_recv = (u16)USART_Rxbuf[len-2] | ((u16)USART_Rxbuf[len-1] << 8);
    u16 crc_calc = Modbus_CRC16(USART_Rxbuf, len-2);
    if(crc_recv != crc_calc) { RXPos = 0; return; }

    u8 slave = USART_Rxbuf[0];
    if(slave != MODBUS_SLAVE_ID) { RXPos = 0; return; }

    u8 func = USART_Rxbuf[1];

    // 0x03 Read Holding Registers
    if(func == 0x03) {
        if(len < 8) { RXPos = 0; return; }
        u16 start = ((u16)USART_Rxbuf[2] << 8) | USART_Rxbuf[3];
        u16 num   = ((u16)USART_Rxbuf[4] << 8) | USART_Rxbuf[5];
        if(num < 1 || num > 125) { RXPos = 0; return; }
        if((start + num) > HOLD_REG_NUM) { RXPos = 0; return; }

        u16 tx_idx = 0;
        u8 resp[USART_BUF_LEN];
        resp[tx_idx++] = MODBUS_SLAVE_ID;
        resp[tx_idx++] = 0x03;
        resp[tx_idx++] = (u8)(num * 2);

        for(u16 i=0;i<num;i++) {
            u16 v = HoldReg[start + i];
            resp[tx_idx++] = (u8)(v >> 8);
            resp[tx_idx++] = (u8)(v & 0xFF);
        }

        u16 crc = Modbus_CRC16(resp, tx_idx);
        resp[tx_idx++] = (u8)(crc & 0xFF);
        resp[tx_idx++] = (u8)(crc >> 8);

        Modbus_SendBuffer(resp, tx_idx);
        RXPos = 0;
        return;
    }

    // 0x10 Write Multiple Registers
    if(func == 0x10) {
        if(len < 9) { RXPos = 0; return; }
        u16 start = ((u16)USART_Rxbuf[2] << 8) | USART_Rxbuf[3];
        u16 num   = ((u16)USART_Rxbuf[4] << 8) | USART_Rxbuf[5];
        u8 bytecnt = USART_Rxbuf[6];
        if(num < 1 || num > 123) { RXPos = 0; return; }
        if(bytecnt != (u8)(num * 2)) { RXPos = 0; return; }
        if((start + num) > HOLD_REG_NUM) { RXPos = 0; return; }
        if(len < (7 + bytecnt + 2)) { RXPos = 0; return; }

        for(u16 i=0;i<num;i++) {
            u16 v = ((u16)USART_Rxbuf[7 + 2*i] << 8) | USART_Rxbuf[7 + 2*i + 1];
            HoldReg[start + i] = v;
        }

        // Reply with slave id, func, start hi, start lo, qty hi, qty lo, CRC
        u8 resp[8];
        resp[0] = MODBUS_SLAVE_ID;
        resp[1] = 0x10;
        resp[2] = (u8)(start >> 8);
        resp[3] = (u8)(start & 0xFF);
        resp[4] = (u8)(num >> 8);
        resp[5] = (u8)(num & 0xFF);
        u16 crc = Modbus_CRC16(resp, 6);
        resp[6] = (u8)(crc & 0xFF);
        resp[7] = (u8)(crc >> 8);

        Modbus_SendBuffer(resp, 8);
        RXPos = 0;
        return;
    }

    // other function codes ignored for now
    RXPos = 0;
}

void Modbus_InitRegs(void)
{
for(u16 i=0;i<HOLD_REG_NUM;i++) HoldReg[i] = i;
}