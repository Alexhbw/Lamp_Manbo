/**
 * @file miniPC_process.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   用于处理miniPC的数据，包括解析和发送
 * @version 0.1
 * @date 2024-01-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef MINIPC_PROCESS_H
#define MINIPC_PROCESS_H

#include "stdint.h"
#include "bsp_usart.h"

#define NAC_RECV_HEADER1 0xAA // 视觉接收数据帧头

#define NAC_RECV_TAIL    0x55 // 视觉发送数据帧尾

#define NAC_SEND_HEADER 0x98 // 视觉发送数据帧头
#define NAC_SEND_TAIL   0x34 // 视觉发送数据帧尾

#define VISION_RECV_SIZE   7 // 当前为固定值,25字节
#define VISION_SEND_SIZE   12

// #pragma pack(1) // 1字节对齐

/* 视觉通信初始化接收结构体 */
typedef struct
{
    uint8_t header1;          // 帧头 0xAA

	  uint8_t tail;            // 帧尾 0x55
} Nac_Recv_Init_Config_s;

/* 视觉通信初始化发送结构体 */
typedef struct
{
    uint8_t header;        // 头帧校验位
    uint8_t tail;          // 尾帧校验位
} Nac_Send_Init_Config_s;


/* 视觉实例初始化配置结构体 */
typedef struct
{
    Nac_Recv_Init_Config_s recv_config; // 接收数据结构体
    Nac_Send_Init_Config_s send_config; // 发送数据结构体
    USART_Init_Config_s usart_config;      // 串口实例结构体
} Nac_Init_Config_s;


typedef struct {
    uint8_t header1;          // 帧头 0xAA

    int16_t x;    // 1字节
    int16_t y; // 1字节
	
		int get;
	
    uint8_t tail;            // 帧尾 0x55
} Nac_Recv_s;

typedef struct {
    uint8_t header;          // 帧头 0xAA
		int8_t level;
    uint8_t tail;            // 帧尾 0x55
} Nac_Send_s;

/* 视觉通信模块实例 */
typedef struct
{
    Nac_Recv_s *recv_data; // 接收数据结构体指针
    Nac_Send_s *send_data; // 发送数据结构体指针
    USART_Instance *usart;    // 串口实例指针
} Nac_Instance;




/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Nac_Recv_s *NacRecvRegister(Nac_Recv_Init_Config_s *recv_config);


/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Nac_Send_s *NacSendRegister(Nac_Send_Init_Config_s *send_config);


/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Nac_Recv_s *NacInit(UART_HandleTypeDef *Nac_usart_handle);


/**
 * @brief 发送函数
 *
 *
 */
void NacSend(uint8_t set);


/**
 * @brief 设置发送给视觉的数据
 */
void NacSetAltitude(int16_t motor_lf_rpm,int16_t motor_rf_rpm,int16_t motor_lb_rpm,int16_t motor_rb_rpm,float chassis_yaw);
#endif // MINIPC_PROCESS_H