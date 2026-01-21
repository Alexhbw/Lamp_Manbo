/**
 * @file robot_def.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   机器人定义,包含机器人的各种参数
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __ROBOT_DEF_H__
#define __ROBOT_DEF_H__

#include "stdint.h"
#include "ins_task.h"

#define RAD_2_DEGREE        57.2957795f    // 180/pi

#define STEERING_CHASSIS_ALIGN_ECD_LF   948 // 舵电机 A 4编码器值，若有机械改动需要修改
#define STEERING_CHASSIS_ALIGN_ECD_LB   2250 // 舵电机 B 2编码器值，若有机械改动需要修改
#define STEERING_CHASSIS_ALIGN_ECD_RF   3820// 舵电机 C 1编码器值，若有机械改动需要修改
#define STEERING_CHASSIS_ALIGN_ECD_RB   6252 // 舵电机 D 3编码器值，若有机械改动需要修改

#define STEERING_CHASSIS_ALIGN_ANGLE_LF STEERING_CHASSIS_ALIGN_ECD_LF / 8192.f * 360.f // 舵轮 A 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_LB STEERING_CHASSIS_ALIGN_ECD_LB / 8192.f * 360.f // 舵轮 B 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_RF STEERING_CHASSIS_ALIGN_ECD_RF / 8192.f * 360.f // 舵轮 C 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_RB STEERING_CHASSIS_ALIGN_ECD_RB / 8192.f * 360.f // 舵轮 D 对齐角度

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输


/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */



typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float vw;           // 旋转速度
    float last_yaw;     // 纯前进时需要保持直线的角度
		float offset_w;     //因为直线偏转，pid产生的回正力
	
	  float real_vx;
    float real_vy;
    float real_wz;

	//	attitude_t *Chassis_IMU_data;
} Chassis_Ctrl_Cmd_s;

#pragma pack() // 取消压缩
#endif

