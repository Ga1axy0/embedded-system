#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
#include "misc.h"

#define KBD_DEBOUNCE_COUNT  2    // 去抖: 需要连续一致的采样次数
#define KBD_REPEAT_START  200   // 长按后开始连发前的等待次数
#define KBD_REPEAT_NEXT   50   // 连发周期(每隔多少次调用返回一次)

void Delay(unsigned int nCount)
{ 
  while(nCount > 0)
  { 
  	  nCount --;   
  }
}

// ---------------- 数码管刷新 ----------------
// 采用共阴极(段、位均低有效)的既有逻辑: 先全部置高, 再对当前位需要的段拉低, 最后拉低该位选通信号。
// 使用 TIM4 周期性中断实现逐位扫描, 不再阻塞 Delay。

static volatile int g_digits[4] = {0,0,0,0};    // 当前显示的 4 个数码管内容(0~15 对应 SEG_MASK)
static volatile uint8_t g_cur_digit = 0;        // 当前正在刷新的位索引 0~3

static void NEG_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    // 位选: PC10~PC13
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // 段选: PD0~PD7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                                  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

static void Nixie_Timer_Init(uint16_t refresh_hz_per_digit)
{
    // 目标: 每“位”的刷新频率(Hz)。由于一次中断仅刷新一位, 中断频率 = 每位目标频率 × 4。
    // 例: 400Hz/位 => 中断 1600Hz，整帧(四位全扫一遍) 400Hz。
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseInitTypeDef tb;
    // 72MHz 主频 -> 预分频 7200 得到 10kHz 计数频率
    // 计数周期 = 10000 / (refresh_hz_per_digit*4)
    uint32_t isr_hz = (uint32_t)refresh_hz_per_digit * 4u;
    if (isr_hz < 1) isr_hz = 1;
    uint32_t arr = 10000u / isr_hz; if (arr == 0) arr = 1; // 最小 1
    tb.TIM_Period = (uint16_t)(arr - 1); // 自动重装值
    tb.TIM_Prescaler = 7200 - 1;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &tb);

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM4, ENABLE);
}

// 设定显示缓冲 (非阻塞)
void Neg_display(int *num)
{
    for (int i = 0; i < 4; i++) {
        int v = num[i];
        if (v < 0) v = 0; if (v > 15) v = 15;
        g_digits[i] = v;
    }
}

void NEG_init(void)
{
    NEG_GPIO_Init();
    Nixie_Timer_Init(500); // 每位 500Hz 刷新(整帧 250Hz, ISR≈1kHz)。需要更快可改参数。
}


static volatile uint8_t g_need_scan = 0;          // 是否需要进行一次扫描(任一列触发)
static volatile uint8_t g_pending_cols_mask = 0;  // 哪些列产生了下降沿(位0..3 对应 PE0..PE3) 按位记录，可同时按多键

void Keyborad_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

    // 列: PE0~PE3 输入上拉 (EXTI)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // 行: PE4~PE7 推挽输出 -> 默认拉低便于列检测到低触发
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);

    // EXTI 映射与配置 (下降沿)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource2);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource3);

    EXTI_InitTypeDef exti;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;
    exti.EXTI_LineCmd = ENABLE;

    exti.EXTI_Line = EXTI_Line0; EXTI_Init(&exti);
    exti.EXTI_Line = EXTI_Line1; EXTI_Init(&exti);
    exti.EXTI_Line = EXTI_Line2; EXTI_Init(&exti);
    exti.EXTI_Line = EXTI_Line3; EXTI_Init(&exti);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannel = EXTI0_IRQn; NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel = EXTI1_IRQn; NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel = EXTI2_IRQn; NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel = EXTI3_IRQn; NVIC_Init(&nvic);
}

#define nixie_a GPIO_Pin_0
#define nixie_b GPIO_Pin_1
#define nixie_c GPIO_Pin_2
#define nixie_d GPIO_Pin_3
#define nixie_e GPIO_Pin_4
#define nixie_f GPIO_Pin_5
#define nixie_g GPIO_Pin_6
#define nixie_h GPIO_Pin_7

// 注意: 使用标准库 GPIO_ResetBits/SetBits 时, 这里应当使用“引脚位掩码本身”,
// 不需要像直接写 BSRR 高16位那样再 <<16。之前 <<16 会导致传入的掩码无效, 造成全灭。
static const uint16_t SEG_MASK[16] = {
    (nixie_a | nixie_b | nixie_c | nixie_d | nixie_e | nixie_f),       // 0
    (nixie_b | nixie_c),                                               // 1
    (nixie_a | nixie_b | nixie_g | nixie_e | nixie_d),                 // 2
    (nixie_a | nixie_b | nixie_c | nixie_d | nixie_g),                 // 3
    (nixie_b | nixie_g | nixie_f | nixie_c),                           // 4
    (nixie_a | nixie_f | nixie_g | nixie_c | nixie_d),                           // 5
    (nixie_a | nixie_f | nixie_g | nixie_c | nixie_d | nixie_e),       // 6
    (nixie_a | nixie_b | nixie_c),                                               // 7
    (nixie_a | nixie_b | nixie_c | nixie_d | nixie_e | nixie_f | nixie_g),       // 8
    (nixie_a | nixie_b | nixie_c | nixie_d | nixie_f | nixie_g),                 // 9
    (nixie_a | nixie_b | nixie_c  | nixie_e | nixie_f | nixie_g),                 // A 10
    (nixie_c | nixie_d | nixie_e | nixie_f | nixie_g),                           // b 11
    (nixie_g | nixie_e |nixie_d),                           // C 12
    (nixie_b | nixie_c | nixie_d | nixie_e | nixie_g ),                           // d 13
    (nixie_a | nixie_d | nixie_e | nixie_f | nixie_g ),                 // E 14
    (nixie_a  | nixie_e | nixie_f | nixie_g)                  // F 15
};



static int keyborad_scan_column(uint8_t col_index)
{
    if (col_index > 3) return -1;
    const uint16_t ROW_PINS = (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
    const uint16_t COL_PIN  = (uint16_t)(1u << col_index); 

    GPIO_SetBits(GPIOE, ROW_PINS);
    for (uint8_t r = 0; r < 4; r++) {
        GPIO_SetBits(GPIOE, ROW_PINS);              
        GPIO_ResetBits(GPIOE, (GPIO_Pin_4 << r));  
        for (volatile int d = 0; d < 300; ++d) { __NOP(); }
        uint16_t col_state = GPIO_ReadInputData(GPIOE) & COL_PIN;
        if (col_state == 0) { 
            GPIO_ResetBits(GPIOE, ROW_PINS); 
            return r * 4 + col_index;
        }
    }
    GPIO_ResetBits(GPIOE, ROW_PINS); 
    return -1;
}

int keyborad_sran(void)
{
    // 遍历所有列做一次列触发式扫描
    for (uint8_t c = 0; c < 4; ++c) {
        int code = keyborad_scan_column(c);
        if (code >= 0) return code;
    }
    return -1;
}

int read_Key(void){
	// 组合行为: 短按=松开时上报一次; 长按=等待一段时间后开始持续上报
	static int key_down = 0;       // 是否处于按下状态
	static int last_code = -1;     // 当前按下的键编码
	static int repeating = 0;      // 是否已经进入连发阶段
	static int repeat_counter = 0; // 连发计数器(到0触发一次)

    // 读取当前按键状态(带简单去抖) —— 优先使用中断记录的列集合做定向扫描
    int cur = -1;
    if (g_need_scan) {
        // 扫描所有触发的列(支持同时按多键，只取第一个找到的键)
        for (uint8_t c = 0; c < 4; ++c) {
            if (g_pending_cols_mask & (1u << c)) {
                int code = keyborad_scan_column(c);
                if (code >= 0) { cur = code; break; }
            }
        }
        g_need_scan = 0; // 标志复位
        g_pending_cols_mask = 0; // 清空列集合
    }
	if (cur >= 0) {
        // 简易去抖: 再次确认仍然按下(无需长延时阻塞显示)
        for (volatile int d=0; d<500; ++d) { __NOP(); }
        int recheck = keyborad_scan_column(cur % 4);
        if (recheck < 0) cur = -1; // 去抖失败
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
			if (g_need_scan == 0) {
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

void Keyborad_with_Display(int *a)
{
    // 始终刷新缓冲(允许外部直接修改 a[] 后显示立即更新)
    Neg_display(a);

    int tmp = read_Key(); // 调用包含去抖、连发逻辑的函数
    if (tmp != -1) {
        a[3] = a[2];
        a[2] = a[1];
        a[1] = a[0];
        a[0] = tmp;
        Neg_display(a); // 更新显示
    }
}

// ---------------- 中断服务函数原型(放此便于编译器看到引用) ----------------
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line0);
        g_pending_cols_mask |= (1u << 0);
        g_need_scan = 1;
    }
}
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line1);
        g_pending_cols_mask |= (1u << 1);
        g_need_scan = 1;
    }
}
void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line2);
        g_pending_cols_mask |= (1u << 2);
        g_need_scan = 1;
    }
}
void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line3);
        g_pending_cols_mask |= (1u << 3);
        g_need_scan = 1;
    }
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

        // 熄灭当前位与段（防止段残留导致下一位显示同样数字）
        switch (g_cur_digit) {
            case 0: GPIO_SetBits(GPIOC, GPIO_Pin_10); break;
            case 1: GPIO_SetBits(GPIOC, GPIO_Pin_11); break;
            case 2: GPIO_SetBits(GPIOC, GPIO_Pin_12); break;
            case 3: GPIO_SetBits(GPIOC, GPIO_Pin_13); break;
            default: break;
        }
        GPIO_SetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                              GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);

        // 切换到下一个位
        g_cur_digit = (g_cur_digit + 1) & 0x03;

        // 输出该位的段数据
        int val = g_digits[g_cur_digit];
        GPIO_ResetBits(GPIOD, SEG_MASK[val]);
        // 选通位（低有效）
        switch (g_cur_digit) {
            case 0: GPIO_ResetBits(GPIOC, GPIO_Pin_10); break;
            case 1: GPIO_ResetBits(GPIOC, GPIO_Pin_11); break;
            case 2: GPIO_ResetBits(GPIOC, GPIO_Pin_12); break;
            case 3: GPIO_ResetBits(GPIOC, GPIO_Pin_13); break;
            default: break;
        }
    }
}
