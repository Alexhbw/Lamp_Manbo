#ifndef __CHASSIS_H_
#define __CHASSIS_H_
#include "robot_cmd.h"
#include "robot_def.h"
void ChassisInit(void);
void ChassisTask(void);
void Gimbal_Control();
extern  float V;
extern  RC_ctrl_t *rc_cmd;
extern Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
#endif 
