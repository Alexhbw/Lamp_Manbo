#include "bsp_buzzer.h"
#include "main.h"
#include "tim.h"
#include "stm32f4xx_hal_tim.h"
#include "cmsis_os.h"
#define BUZZER_TIM      htim4
#define BUZZER_CH       TIM_CHANNEL_3

// 统一固定一个 ARR，靠 PSC 来调频（移植简单）
// 1000-1：计数 1000 个 tick 一周期
#define BUZZER_ARR      (999U)

void buzzer_init(){
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
// 获取 TIM4 的实际时钟（以 APB1 为例；若你 TIM4 在 APB2，改成 GetPCLK2Freq）
static uint32_t buzzer_tim_clk_hz(void)
{
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();

    // 若 APB1 分频不是 1，很多 STM32 系列定时器时钟 = PCLK*2
    // 这里用寄存器判断，兼容性较好
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        return pclk * 2U;

    return pclk;
}

//void buzzer_on(uint16_t psc, uint16_t pwm)
//{
//    __HAL_TIM_PRESCALER(&htim4, psc);
//    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

//}

// 你原接口：psc 控频，pwm 是 CCR（占空比计数值）
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_DISABLE(&BUZZER_TIM);

    __HAL_TIM_SET_PRESCALER(&BUZZER_TIM, psc);
    // 如果你还需要改频率，建议同时设置 ARR（否则只改 psc 不好控频）
    // __HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, arr);

    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_CH, pwm);

    __HAL_TIM_SET_COUNTER(&BUZZER_TIM, 0);

    // 强制产生一次更新事件：立即加载 PSC/ARR
    BUZZER_TIM.Instance->EGR = TIM_EGR_UG;

    __HAL_TIM_ENABLE(&BUZZER_TIM);
}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

// freq_hz = 0 表示静音
void buzzer_set_freq(uint32_t freq_hz)
{
    if (freq_hz == 0U)
    {
        buzzer_off();
        return;
    }

    uint32_t timclk = buzzer_tim_clk_hz();
    uint32_t arrp1  = (BUZZER_ARR + 1U);

    // 计算 PSC（做边界保护）
    uint32_t psc_calc = timclk / (freq_hz * arrp1);
    if (psc_calc == 0U) psc_calc = 1U;
    psc_calc -= 1U;
    if (psc_calc > 0xFFFFU) psc_calc = 0xFFFFU;

    uint16_t psc = (uint16_t)psc_calc;
    uint16_t ccr = (uint16_t)(arrp1 / 2U); // 50% duty

    buzzer_on(psc, ccr);
}

static void buzzer_play_tone(uint16_t freq, uint16_t ms)
{
    buzzer_set_freq(freq);
		osDelay(ms);
    buzzer_off();
}


void buzzer_play_song(int song_list)
{
	if(song_list == 1){
		for (uint32_t i = 0; i < sizeof(song_windowsxp_start)/sizeof(song_windowsxp_start[0]); i++)
    {
        buzzer_play_tone(song_windowsxp_start[i].freq, song_windowsxp_start[i].ms);
    }
		
	}
	
	if(song_list == 2){
		for (uint32_t i = 0; i < sizeof(song_happy)/sizeof(song_happy[0]); i++)
    {
        buzzer_play_tone(song_happy[i].freq, song_happy[i].ms);
    }
		
	}
	
	if(song_list == 3){
		for (uint32_t i = 0; i < sizeof(song_sad)/sizeof(song_sad[0]); i++)
    {
        buzzer_play_tone(song_sad[i].freq, song_sad[i].ms);
    }
		
	}
	
	if(song_list == 4){
		for (uint32_t i = 0; i < sizeof(song_nat)/sizeof(song_nat[0]); i++)
    {
        buzzer_play_tone(song_nat[i].freq, song_nat[i].ms);
    }
		
	}
	
	if(song_list == 5){
		for (uint32_t i = 0; i < sizeof(song_angery)/sizeof(song_angery[0]); i++)
    {
        buzzer_play_tone(song_angery[i].freq, song_angery[i].ms);
    }
		
	}
    
}

