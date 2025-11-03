 
/***********************************************************************
文件名称：LED.C
功    能：led  IO初始化
编写时间：2013.4.25
编 写 人：
注    意：以下代码仅供参考
***********************************************************************/
#include "stm32f10x.h"
#include "inc/KEY.h"
#include "sys.h"

// 自动连发参数(基于调用频率的计数, 可按实际主循环速度调整)
#define KBD_DEBOUNCE_COUNT  2    // 去抖: 需要连续一致的采样次数
#define KBD_REPEAT_START  50   // 长按后开始连发前的等待次数
#define KBD_REPEAT_NEXT   10   // 连发周期(每隔多少次调用返回一次)


#define RCC_APB2ENR         (*((volatile unsigned int*)0x40021018))   // APB2 外设时钟使能寄存器
#define GPIOE_CRH           (*((volatile unsigned int*)0x40011804))   // 端口配置高寄存器
#define GPIOE_CRL           (*((volatile unsigned int*)0x40011800))   // 端口配置低寄存器
#define GPIOE_BSRR          (*((volatile unsigned int*)0x40011810))   // 端口位设置/复位寄存器
#define GPIOE_IDR           (*((volatile unsigned int*)0x40011808))   // 端口输入数据寄存器	
#define GPIOE_ODR           (*((volatile unsigned int*)0x4001180C))   // 端口输出数据寄存器
	
#define GPIOB_CRL           (*((volatile unsigned int*)0x40010C00))   // 端口配置低寄存器
#define GPIOB_IDR           (*((volatile unsigned int*)0x40010C08))   // 端口输入数据寄存器	
#define GPIOB_BSRR          (*((volatile unsigned int*)0x40010C10))   // 端口位设置/复位寄存器

#define GPIOC_CRL           (*((volatile unsigned int*)0x40011000))   // 端口配置低寄存器
#define GPIOC_CRH           (*((volatile unsigned int*)0x40011004))   // 端口配置高寄存器
#define GPIOC_BSRR          (*((volatile unsigned int*)0x40011010))   // 端口位设置/复位寄存器
#define GPIOC_ODR           (*((volatile unsigned int*)0x4001100C))   // 端口输出数据寄存器

#define GPIOD_CRL		   (*((volatile unsigned int*)0x40011400))   // 端口配置低寄存器
#define GPIOD_CRH		   (*((volatile unsigned int*)0x40011404))   // 端口配置高寄存器
#define GPIOD_BSRR         (*((volatile unsigned int*)0x40011410))   // 端口位设置/复位寄存器
#define GPIOD_ODR          (*((volatile unsigned int*)0x4001140C))   // 端口输出数据寄存器

#define GPIO_PIN_0 			   ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1 			   ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2 			   ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3 			   ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4 			   ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5 			   ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6 			   ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7 			   ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8 			   ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9 			   ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10 		   ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11 		   ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12 		   ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13 		   ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14 		   ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15 		   ((uint16_t)0x8000)  /* Pin 15 selected   */

// 扫描当前矩阵键盘状态: 有键返回编码0~15, 无键返回-1
static int keypad_scan_now(void)
{
	const unsigned int ROW_PINS = (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	const unsigned int COL_PINS = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

	int row = -1, col = -1;
	unsigned int col_state, col_active;

	// 快速存在性检测: 全部行拉低 -> 只要有列低则有键
	GPIOE_BSRR = (ROW_PINS << 16);
	delay_ms(1);
	col_state  = GPIOE_IDR & COL_PINS;        // 空闲=高
	col_active = (~col_state) & COL_PINS;     // 有低电平的列
	if (col_active == 0) {
		GPIOE_BSRR = ROW_PINS; // 恢复行高
		return -1;
	}

	// 逐行定位
	GPIOE_BSRR = ROW_PINS;
	for (int r = 0; r < 4; r++) {
		GPIOE_BSRR = ROW_PINS;                          // 全部高
		GPIOE_BSRR = (1u << ((4 + r) + 16));            // 当前行拉低
		delay_ms(1);
		col_state  = GPIOE_IDR & COL_PINS;
		col_active = (~col_state) & COL_PINS;
		if (col_active) {
			row = r;
			for (int c = 0; c < 4; c++) {
				if (col_active & (1u << c)) { col = c; break; }
			}
			break;
		}
	}
	GPIOE_BSRR = ROW_PINS; // 恢复行高

	if (row >= 0 && col >= 0) {
		return (row * 4 + col);
	}
	return -1;
}

#define nixie_a GPIO_PIN_0
#define nixie_b GPIO_PIN_1
#define nixie_c GPIO_PIN_2
#define nixie_d GPIO_PIN_3
#define nixie_e GPIO_PIN_4
#define nixie_f GPIO_PIN_5
#define nixie_g GPIO_PIN_6
#define nixie_h GPIO_PIN_7

void Delay(unsigned int nCount)
{ 
  while(nCount > 0)
  { 
  	  nCount --;   
  }
}

void LED_Init(void)
{	
	//PE8
	RCC_APB2ENR |= 1<<6;          //使能PORTE时钟	
	GPIOE_CRH &=0XFFFFFFF0;       //清除PE8引脚原来设置  
	GPIOE_CRH |=0x3;			        //设置CNF8[1:0]为0x00：通用推挽输出模式，MODE8[1:0]为0x11：输出模式
	GPIOE_BSRR = 1<<8;           //第8bit置1，则设置对应的ODR8位为1，即PE8引脚输出高电平
	
	//PE9
	GPIOE_CRH &=0XFFFFFF0F;
	GPIOE_CRH |=0x30;			       
	GPIOE_BSRR = 1<<9;         
	
	//PE10 
	GPIOE_CRH &=0XFFFFF0FF;
	GPIOE_CRH |=0x300;	
	GPIOE_BSRR = 1<<10;

	//PE11
	GPIOE_CRH &=0XFFFF0FFF;
	GPIOE_CRH |=0x3000;	
	GPIOE_BSRR = 1<<11;
}


void Nixie_init(void){
	RCC_APB2ENR |= 3<<4;
	GPIOC_CRH &=0XFF0000FF;
	GPIOC_CRH |=0X00333300;
	GPIOC_ODR |= 15 << 10;

	GPIOD_CRL &=0X00000000;
	GPIOD_CRL |=0X33333333;
	GPIOD_CRH |= 0XFF;

	GPIOD_BSRR = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIOD_BSRR = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIOC_BSRR = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;															
}

void KeyBoard_init(void){
	RCC_APB2ENR |= (1 << 6);
	GPIOE_CRL = 0x33338888;
	GPIOE_BSRR = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOE_BSRR = (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
}

int readKey(void){
	// 组合行为: 短按=松开时上报一次; 长按=等待一段时间后开始持续上报
	static int key_down = 0;       // 是否处于按下状态
	static int last_code = -1;     // 当前按下的键编码
	static int repeating = 0;      // 是否已经进入连发阶段
	static int repeat_counter = 0; // 连发计数器(到0触发一次)

	// 读取当前按键状态(带简单去抖)
	int cur = keypad_scan_now();
	if (cur >= 0) {
		Delay(1000);
		int cur2 = keypad_scan_now();
		if (cur2 >= 0) cur = cur2; else cur = -1;
	}

	if (!key_down) {
		// 之前未按 -> 检测到按下则进入按下状态，但先不输出(短按时在松开再上报)
		if (cur >= 0) {
			key_down = 1;
			last_code = cur;
			repeating = 0;
			repeat_counter = KBD_REPEAT_START; // 启动连发等待计数
		}
		return -1;
	} else {
		// 已在按下保持
		if (cur < 0) {
			// 检测到松开 -> 再次确认释放去抖
			Delay(1000);
			if (keypad_scan_now() < 0) {
				int ret = -1;
				if (!repeating) {
					// 短按: 从未进入连发, 则在松开时返回一次
					ret = last_code;
				}
				// 复位状态
				key_down = 0;
				last_code = -1;
				repeating = 0;
				repeat_counter = 0;
				return ret;
			}
			return -1;
		}

		// 仍然按着同一键
		if (!repeating) {
			if (repeat_counter > 0) {
				repeat_counter--;
				return -1; // 还未进入连发
			} else {
				// 进入连发并输出一次
				repeating = 1;
				repeat_counter = KBD_REPEAT_NEXT;
				return last_code;
			}
		} else {
			// 连发周期中
			if (repeat_counter > 0) {
				repeat_counter--;
				return -1;
			} else {
				repeat_counter = KBD_REPEAT_NEXT;
				return last_code;
			}
		}
	}
}

void Nixie_test(){
	GPIOD_BSRR = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIOD_BSRR = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIOC_BSRR = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;															
	
	int i=0;

	GPIOD_BSRR = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7) << 16;
	for(i=0;i<4;i++){
		switch (i)
		{
		case 0:
			GPIOC_BSRR = GPIO_PIN_10 << 16;
			break;
		case 1:
			GPIOC_BSRR = GPIO_PIN_11 << 16;
			break;
		case 2:
			GPIOC_BSRR = GPIO_PIN_12 << 16;
			break;
		case 3:
			GPIOC_BSRR = GPIO_PIN_13<< 16;
			break;
		default:
			break;
		}
		
	}
}



void Nixie_Display(int *num){
	int i=0;
	GPIOD_BSRR = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIOD_BSRR = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIOC_BSRR = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;	
	for(i=0;i<4;i++){
		
		switch(num[i]){
			case 0:
				GPIOD_BSRR = (nixie_a | nixie_b | nixie_c | nixie_d | nixie_e | nixie_f) << 16;
				break;
			case 1:
				GPIOD_BSRR = (nixie_b | nixie_c) << 16;
				break;
			case 2:
				GPIOD_BSRR = (nixie_a | nixie_b | nixie_g | nixie_e | nixie_d) << 16;
				break;
			case 3:
				GPIOD_BSRR = (nixie_a | nixie_b | nixie_c | nixie_d | nixie_g) << 16;
				break;
			case 4:
				GPIOD_BSRR = (nixie_b | nixie_g | nixie_f | nixie_c) << 16;
				break;
			case 5:
				GPIOD_BSRR = (nixie_a | nixie_f | nixie_g | nixie_c | nixie_d) << 16;
				break;
			case 6:
				GPIOD_BSRR = (nixie_a | nixie_f | nixie_g | nixie_c | nixie_d | nixie_e) << 16;
				break;
			case 7:
				GPIOD_BSRR = (nixie_a | nixie_b | nixie_c) << 16;
				break;
			case 8:
				GPIOD_BSRR = (nixie_a | nixie_b | nixie_c | nixie_d | nixie_e | nixie_f | nixie_g ) << 16;
				break;
			case 9:
				GPIOD_BSRR = (nixie_a | nixie_b | nixie_c | nixie_d | nixie_f | nixie_g ) << 16;
				break;
			case 10:
				GPIOD_BSRR = (nixie_a | nixie_b | nixie_c  | nixie_e | nixie_f | nixie_g ) << 16;
				break;
			case 11:
				GPIOD_BSRR = (nixie_c | nixie_d | nixie_e | nixie_f | nixie_g ) << 16;
				break;	
			case 12:
				GPIOD_BSRR = (nixie_g | nixie_e |nixie_d) << 16;
				break;
			case 13:
				GPIOD_BSRR = (nixie_b | nixie_c | nixie_d | nixie_e | nixie_g ) << 16;
				break;
			case 14:
				GPIOD_BSRR = (nixie_a | nixie_d | nixie_e | nixie_f | nixie_g ) << 16;
				break;
			case 15:
				GPIOD_BSRR = (nixie_a  | nixie_e | nixie_f | nixie_g ) << 16;
				break;
			default:
				//do nothing
				break;
		}

		switch (i)
		{
		case 0:
			GPIOC_BSRR = GPIO_PIN_10 << 16;
			break;
		case 1:
			GPIOC_BSRR = GPIO_PIN_11 << 16;
			break;
		case 2:
			GPIOC_BSRR = GPIO_PIN_12 << 16;
			break;
		case 3:
			GPIOC_BSRR = GPIO_PIN_13 << 16;
			break;
		default:
			break;
		}
		
		Delay(5000);

		GPIOD_BSRR = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
		GPIOD_BSRR = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
		GPIOC_BSRR = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;	
	}
}



int abc;

void Togglepin(int pin)
{
	GPIOE_BSRR = 1<<(pin +16);        
	Delay(0xfffff);

	GPIOE_BSRR = 1<<pin; 
		
}
void LED_Turn(void)
{
	int  bcd;
	abc = 1;
	while(1)
	{
		abc ++ ;
		bcd ++ ;
		GPIOE_BSRR = 1<<24;        	//清除对应的ODR8位为0，即PE8引脚输出低电平.BSRR高16位清0，低16位置位 
//		GPIOE_BRR = 1<<8; 				//等效语句
//		GPIOE_ODR &= ~(1 << 8); 	//等效语句
		Delay(0xfffff);
		Delay(0xfffff);
	
	  GPIOE_BSRR = 1<<8; 
		
		Delay(0xfffff);
		Delay(0xfffff);
	}
}

void led_stream(void)
{
	int i;
	for(i=8;i<12;i++)
	{
		Togglepin(i);
	}
}



void KEY_Init(void)
{
		RCC_APB2ENR |=1<<3;    //使能PORTB时钟	
	  GPIOB_CRL &=~(0xf<<24);// PB.6上拉输入	
		GPIOB_CRL |=(0x08<<24);// PB.6上拉输入	
	  //GPIOB_ODR |= (1<<6);
		GPIOB_BSRR = (1<< 6);


}


u8 KEY_Scan(void)
{	 

	u8 key_1= 0;
	if(GPIOB_IDR & 0x40)
			key_1=0;
	else 
  		key_1=1;
	if(key_1==1)
	{
		delay_ms(100);//去抖动 
		if(GPIOB_IDR & 0x40)
			key_1=0;

			
		if(key_1==1)
			return 1;
	} 
 	return 0;// 无按键按下
}
