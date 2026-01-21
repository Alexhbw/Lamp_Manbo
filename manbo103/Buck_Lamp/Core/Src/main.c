/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "OLED.h"
#include "Face_Config.h"
#include "OLED_Data.h"
#include <math.h>
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define R1          50000.0f    // 分压电阻R1（50kΩ）
#define R2          10000.0f    // 分压电阻R2（10kΩ）
#define ADC_REF_VOL 3.3f        // ADC参考电压（3.3V）
#define ADC_RES     4096.0f     // 12位ADC分辨率（2^12）
#define PWM_ARR     1439.0f     // 定时器ARR值（PWM周期）

#define AN1_ON HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
#define AN1_OFF HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);

#define AN2_ON HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
#define AN2_OFF HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);

#define PWM_ON HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
#define PWM_OFF HAL_GPIO_WritePin          (GPIOC,GPIO_PIN_1,0);

/* 语音模块接收相关 (USART1) */
uint8_t g_voice_buffer[11];     // 语音模块发送的是 FF XX XX AF，4字节
uint32_t voice_lock_tick = 0;   // 语音控制锁定计时器
#define VOICE_LOCK_TIME 5000    // 语音指令执行后，锁定5秒不被视觉干扰

/* 记录当前控制源 */
uint8_t control_source = 0;     // 0: 视觉, 1: 语音

extern uint16_t Face_Mode; 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ------------------- PI参数配置 -------------------
typedef struct {
    float kp;         // 比例系数
    float ki;         // 积分系数
	float retarget;   // 预目标值（补偿VDDA偏差）
    float target;     // 目标值（输出电压）
    float feedback;   // 反馈值（采用反馈电压）
    float error;      // 误差（target - feedback）
    float integral;   // 积分项
    float output;     // PI输出（PWM占空比对应的CCR值）
    float out_min;    // 输出最小值（避免占空比为0）
    float out_max;    // 输出最大值（避免占空比过大）
} PI_HandleTypeDef;

PI_HandleTypeDef pi_buck;  // BUCK的PI控制器
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// ------------------- ADC采样变量 -------------------
uint16_t adc_buf[10];      // DMA采样缓冲区（10个数据，用于滑动平均滤波）
float v_out;               // BUCK实际输出电压（换算后）

// ------------------- 串口重定向（printf用） -------------------
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 串口接收相关变量 */
/* DMA 接收相关 */
uint8_t g_rx_buffer[8];  // 接收 3 字节数据包
uint8_t g_tt_code = 0xFF; // 全局存储当前的 tt 指令码

/* 模式目标电压定义 */
#define VOLT_OFF     0.0f
#define VOLT_WEAK    8.0f   // 弱光电压，根据实际亮度调整
#define VOLT_STRONG  15.0f  // 强光电压
/* 视觉模式锁定相关 */
uint8_t active_visual_tt = 0xFF; // 当前正在执行的视觉模式
uint32_t vision_mode_tick = 0;   // 视觉模式开始执行的时间戳
#define MIN_VISION_TIME 2000     // 视觉模式最小执行时间：2秒
/* 投票过滤器相关 */
#define FILTER_SIZE 7
uint8_t tt_history[FILTER_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 存储最近7次收到的 tt 码
uint8_t history_idx = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void PI_Init(PI_HandleTypeDef *pi);
float ADC_Filter(uint16_t *buf, uint8_t len);
void ADC_Convert_Vol(void);
void PI_Calculate(PI_HandleTypeDef *pi);
void Process_Light_Logic(void);
void Process_Voice_Logic(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	PI_Init(&pi_buck);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_buf,10);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	OLED_Init();
	OLED_ShowImage(0,0,128,64,Face_sleep);
	OLED_Update();
	// 开启 USART1 的 DMA 接收（语音模块）
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_voice_buffer, 12);

	// 开启 USART3 的 DMA 接收（视觉模块）
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_rx_buffer, 9);
	HAL_Delay(1000);	//先睡一秒中再醒来
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// （1）模式解析
        
        // （2）更新PI反馈值，执行PI计算
			
        
        // （4）OLED表情刷新
		//  Face_Config();
		//  OLED_Update();
	  
	    // （5）刷新
		 // HAL_Delay(10);	//（每10ms一次，平衡实时性与CPU占用）
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void light_task(){
	ADC_Convert_Vol();
	pi_buck.feedback = v_out;
	PI_Calculate(&pi_buck);
	
	// （3）更新PWM占空比（CCR1值）
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)pi_buck.output);
	
}

void oled_task(){
	 Face_Config();
	 OLED_Update();
	
}

//--------------------------------------------------
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(huart->Instance == USART1)
    {
		Process_Voice_Logic();	
      
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_voice_buffer, 12);
		
    }
	if(huart->Instance == USART3)
    {
        Process_Light_Logic();
      
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_rx_buffer, 9);
    }
}


void PI_Init(PI_HandleTypeDef *pi) {
    pi->kp = 40.0f;        // 比例系数（初始值，需调试）
    pi->ki = 20.0f;        // 积分系数（初始值，需调试）
	pi->retarget = 12.0f;   // 目标输出电压
    pi->target = pi->retarget * 1.00;     // 真实输出电压
    pi->integral = 0.0f;   // 积分项清零
    pi->out_min = 0.0f;   // 最小CCR值（避免占空比过小，无法启动）
    pi->out_max = 1300.0f; // 最大CCR值（避免占空比过大，MOS管过热）
}

// 滑动平均滤波（减少ADC采样噪声）
float ADC_Filter(uint16_t *buf, uint8_t len) {
    uint32_t sum = 0;
    for(uint8_t i=0; i<len; i++) {
        sum += buf[i];
    }
    return (float)sum / len;
}

//  ADC采样值换算为实际输出电压
void ADC_Convert_Vol(void) {
    float adc_avg = ADC_Filter(adc_buf, 10);  // 10点滑动平均
    float v_adc = adc_avg * ADC_REF_VOL / ADC_RES;  // ADC输入电压
    v_out = v_adc * (R1 + R2) / R2;  // 换算为BUCK实际输出电压
}

// PI计算（位置式PI，适合电压闭环）
void PI_Calculate(PI_HandleTypeDef *pi) {
    // 1. 计算误差
    pi->error = pi->retarget - pi->feedback;
    
    // 2. 积分项（防积分饱和）
    pi->integral += pi->ki * pi->error;
    if(pi->integral > pi->out_max) pi->integral = pi->out_max;
    if(pi->integral < pi->out_min) pi->integral = pi->out_min;
    
    // 3. PI输出计算
    pi->output = pi->kp * pi->error + pi->integral;
    
    // 4. 输出限幅（确保在PWM占空比范围内）
    if(pi->output > pi->out_max) pi->output = pi->out_max;
    if(pi->output < pi->out_min) pi->output = pi->out_min;
}

#include <math.h> // 必须包含数学库用于呼吸灯 sin 函数

//// 视觉逻辑判断
void Process_Light_Logic(void) {
    uint32_t now = HAL_GetTick();
	// --- 核心改动：检查语音锁定 ---
    // 如果当前时间距离上次语音指令不到 5 秒，则不执行视觉逻辑
   // if (voice_lock_tick != 0 && (now - voice_lock_tick < VOICE_LOCK_TIME)) return; 
		for(int i =0;i<7;i++){
			if (g_rx_buffer[i] == 0x98 && g_rx_buffer[i+2] == 0x34) {
        uint8_t raw_tt = g_rx_buffer[i+1];
		
				//将收到的tt存入缓冲区
				tt_history[history_idx] = raw_tt;
				history_idx = (history_idx + 1) % FILTER_SIZE;
				 active_visual_tt = raw_tt;
			}
		
		}

    
	
//	// 2. 投票过滤算法
//	uint8_t counts[256] = {0};	//记录次数
//	uint8_t winner_tt = 0xFF;
//	uint8_t max_count = 0;
//	
//	for(int i =0; i < FILTER_SIZE; i++){
//		if(tt_history[i] != 0xFF){
//			counts[tt_history[i]]++;
//			if(counts[tt_history[i]] > max_count){
//				max_count = counts[tt_history[i]];
//				winner_tt = tt_history[i];
//			}
//		}
//	}
//	
//	// 3. 只有当最高票数 >= 4 时，才认为识别结果可信
//    uint8_t confirmed_tt = active_visual_tt; // 默认维持当前状态
//    if (max_count >= 4) {
//        confirmed_tt = winner_tt;
//    }
	
//	// 4. 结合之前的“2秒最小执行时间”逻辑
//    if (confirmed_tt != active_visual_tt) {
//        if (now - vision_mode_tick >= MIN_VISION_TIME) {
//            active_visual_tt = confirmed_tt;
//            vision_mode_tick = now;
//        }
//    }
//	
	// 5. 执行对应的动作（最少2s）
    switch (active_visual_tt) {
        case 0x00: case 0xFF:// 台灯无光
						Face_Mode= 1;	//萌萌表情
            break;

        case 0x01: case 0x22: case 0x33: case 0x44: //跟随模式
            pi_buck.retarget = 8.0f; // 弱光
						Face_Mode= 5;	//大眼睛
            break;
		
		case 0x11:	//识别并停止
			pi_buck.retarget = 15.0f; // 强光 
			Face_Mode= 4;	//非常快乐
			break;

        case 0x55: // 生气
            pi_buck.retarget = 15.0f; // 强光 
			Face_Mode= 3;	//生气
            break;

        case 0x77: //高兴	// 周期 1s 闪烁
            if ((now / 500) % 2 == 0) pi_buck.retarget = 12.0f;
            else pi_buck.retarget = 0.0f;
			Face_Mode= 2;	//高兴
            break;
		
		case 0x88:	//悲伤
			pi_buck.retarget = 8.0f; // 弱光
			Face_Mode= 6;	//内向
			break;

        case 0x66: //自然状态	// 呼吸灯
            {
                // 使用 sin 函数产生 0.0 到 1.0 之间的变化
                float breathing = (sinf(now * 0.002f) + 1.0f) / 2.0f; 
                pi_buck.retarget = breathing * 6.0f + 6.0f; // 在 6-12V 之间呼吸
						Face_Mode= 1;	//萌萌表情
            }
            break;

        default:
            // 未识别到有效指令时，可保持当前状态或关灯
						//pi_buck.retarget = 0.0f ;
							Face_Mode= 1;	//萌萌表情
            break;
    }

    // 更新 PI 控制器的目标值
    pi_buck.target = pi_buck.retarget;
		memset(&g_rx_buffer,0,sizeof(g_rx_buffer));
}

//语音控制逻辑
void Process_Voice_Logic(void) {
    // 检查帧头 0xFF 和 帧尾 0xAF
	for(int i=0;i<7;i++){
		if (g_voice_buffer[i] == 0xFF && g_voice_buffer[i+3] == 0xAF) {
        
        // 识别到有效语音指令
        uint8_t cmd = g_voice_buffer[i+1];
		
		// 语音控制一旦触发，立即重置视觉状态，防止冲突
        active_visual_tt = 0xFE; // 设置一个无关码，打破视觉锁定
       // voice_lock_tick = HAL_GetTick();

        switch (cmd) {
            case 0x11: // 一档亮度
                pi_buck.retarget = 8.0f;
                Face_Mode = 2;
						AN1_ON;
						AN2_OFF;
						PWM_ON;
               // voice_lock_tick = HAL_GetTick(); // 触发锁定
                break;
            case 0x22: //  关闭灯光
                pi_buck.retarget = 0.0f;
                Face_Mode = 4;
						AN1_OFF;
						AN2_OFF;
						PWM_OFF;
              //  voice_lock_tick = HAL_GetTick();
                break;
            case 0x33: // 二档亮度 
                pi_buck.retarget = 12.0f;
                Face_Mode = 3;
             //   voice_lock_tick = HAL_GetTick();
                break;
            case 0x44: // 三档亮度
                pi_buck.retarget = 15.0f;
                Face_Mode = 1;
            //    voice_lock_tick = HAL_GetTick();
                break;
						case 0x77: // 一档亮度

						AN1_ON;
						AN2_OFF;
						PWM_ON;
               // voice_lock_tick = HAL_GetTick(); // 触发锁定
                break;
            case 0x88: //  关闭灯光
						AN1_OFF;
						AN2_OFF;
						PWM_OFF;
						default:break;
        }
        
        // 清空缓冲区关键字节，防止重复触发
//        g_voice_buffer[0] = 0x00; 
		
    }
		memset(&g_voice_buffer,0,sizeof(g_voice_buffer));
	}
    
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
