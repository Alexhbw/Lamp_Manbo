#include "chassis.h"
#include "shoot.h"
#include "arm.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "ins_task.h"
#include "Test.h"





//-----------------------------------------总体的初始化函数----------------------------
void all_init_Task(){

	ChassisInit();
	//ArmInit();
	//ShootInit();	
}

//-----------------------------------------总体的执行函数-------------------------------
void all_cmd_Task(){

//	if((uint8_t)rc_cmd->rc.switch_left == (uint8_t)1)
//	{
//		ChassisTask();
//		ShootTask();		
//	}
//		else if((uint8_t)rc_cmd->rc.switch_left == (uint8_t)3)
//	{
//		ChassisTask();
//		//ArmTask();
//	}
//	else 	if((uint8_t)rc_cmd->rc.switch_left == (uint8_t)2)
//	{
//		ChassisTask();	
//	}
//	ChassisTask();
	ChassisTask();
}
