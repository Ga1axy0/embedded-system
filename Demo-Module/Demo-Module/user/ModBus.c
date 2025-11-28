/*
 * File: modbus_rtu_stm32f103_mod.c
 * Purpose: Modbus RTU slave implementing function codes 0x03 (Read Holding Registers)
 *          and 0x10 (Write Multiple Registers) on STM32F1 (StdPeriph Lib).
 * MCU: STM32F103 (adjust clocks/pins as needed)
 * USART: USART1 (PA9 TX, PA10 RX)
 * Optional RS485 DE control on PA8 (define USE_RS485 to enable)
 * Author: Generated for user
 * Date: 2025-11-26
 */

#include "stm32f10x.h"
#include <string.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// Type shortcuts (to match user's original style)
// -----------------------------------------------------------------------------
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
#define MODBUS_SLAVE_ID    0x01
#define HOLD_REG_NUM       128
#define USART_BUF_LEN      256
//#define USE_RS485         // Uncomment to enable RS485 DE control on PA8

// -----------------------------------------------------------------------------
// Buffers & state
// -----------------------------------------------------------------------------
u8 USART_Rxbuf[USART_BUF_LEN];
u8 USART_Txbuf[USART_BUF_LEN];
volatile u16 RXPos = 0;           // number of received bytes in current frame
volatile u16 FrameFlag = 0;       // set when a full Modbus frame is received
volatile u16 RecvTimeOver = 0;    // reception timeout counter (ms), 3.5 char ~ depends on baud

volatile u16 SendPos = 0;
volatile u16 SendBufLen = 0;

u16 HoldReg[HOLD_REG_NUM];

// -----------------------------------------------------------------------------
// CRC16 (Modbus) - low byte first
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// UART / NVIC init
// -----------------------------------------------------------------------------
void RS232_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // Enable clocks: GPIOA and USART1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

#ifdef USE_RS485
    // PA8 as DE (output push-pull)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8); // default receive
#endif

    // PA9 TX (AF push-pull)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA10 RX (floating input)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    // Enable RXNE interrupt. TXE interrupt will be enabled when we start sending.
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// -----------------------------------------------------------------------------
// SysTick for RecvTimeOver (1ms tick)
// Call this in init: SysTick_Config(SystemCoreClock/1000);
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// USART1 IRQ: handle RXNE and TXE
// - RXNE: store byte and restart RecvTimeOver
// - TXE : send next byte; when finished disable TXE
// -----------------------------------------------------------------------------
void USART1_IRQHandler(void)
{
    // RX not empty
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        u8 d = (u8)USART_ReceiveData(USART1);
        if(RXPos < USART_BUF_LEN) {
            USART_Rxbuf[RXPos++] = d;
        }
        // set timeout to 4 char-times (~ depends on baud). For 115200, 3.5 char ~ 0.3ms, but to be safe use >=3ms
        RecvTimeOver = 5; // 5 ms (tune if necessary for lower baudrates)
    }

    // TX register empty -> send next byte
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
        if(SendPos < SendBufLen) {
            USART_SendData(USART1, USART_Txbuf[SendPos++]);
        } else {
            // All bytes placed into DR. Disable TXE and optionally enable TC if you want to wait for actual completion
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

#ifdef USE_RS485
            // For RS485, wait TC to ensure last byte has physically left the shift register before switching to receive
            // Enable TC interrupt and handle DE reset in TC handler (not implemented here). Simple approach: small delay then clear DE
            // Alternatively enable TC interrupt and in TC handler clear DE and disable TC.
            // For simplicity: enable TC and handle below
            USART_ITConfig(USART1, USART_IT_TC, ENABLE);
#endif
        }
    }

    // Transmission complete - used here for RS485 direction control
    if(USART_GetITStatus(USART1, USART_IT_TC) != RESET) {
#ifdef USE_RS485
        // clear pending bit and switch DE low (receive)
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        GPIO_ResetBits(GPIOA, GPIO_Pin_8);
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
#endif
    }
}

// -----------------------------------------------------------------------------
// Helper: start sending buffer via TXE-driven interrupt
// This function will drive DE high (if RS485) and start by sending first byte
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Simple test/demo: populate holding registers with sample data
// -----------------------------------------------------------------------------
void Modbus_InitRegs(void)
{
    for(u16 i=0;i<HOLD_REG_NUM;i++) HoldReg[i] = i;
}

// -----------------------------------------------------------------------------
// Initialization entry
// -----------------------------------------------------------------------------
void uart1_init(void)
{
    RS232_Configuration();
    NVIC_Configuration();
    // SysTick 1ms
    SysTick_Config(SystemCoreClock / 1000);
}

// -----------------------------------------------------------------------------
// main
// -----------------------------------------------------------------------------
int main(void)
{
    SystemInit();
    uart1_init();
    Modbus_InitRegs();

    while(1) {
        if(FrameFlag) {
            Modbus_Analyze();
        }
        // application tasks ...
    }
}

/*
Notes:
- Tweak RecvTimeOver (in USART IRQ we set to 5 ms) according to your baudrate.
  Modbus RTU requires 3.5 char times of silence to indicate frame end; at 115200 baud
  this is very small (~0.3ms). For robustness we use a few ms.
- If you need precise Modbus timing at very high baud with strict RTU spec, consider
  using a hardware timer to measure 3.5 char times precisely.
- For RS485 DE handling we enabled TC interrupt to clear DE after transfer. If you
  use RS485, ensure USE_RS485 is defined and you route DE to PA8 (or change pin).
- This file uses StdPeriph drivers (stm32f10x_xxx). Include correct library in your
  Keil/STM32CubeIDE project and adjust SystemInit/SystemCoreClock as needed.
*/
