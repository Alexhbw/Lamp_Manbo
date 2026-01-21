#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "ins_task.h"
#include "user_lib.h"
#include "robot_def.h"
#include "shoot.h"
#include "miniPC_process.h"

static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb;                                     // left right forward back
static DJIMotor_Instance *motor_steering_lf, *motor_steering_rf, *motor_steering_lb, *motor_steering_rb; // 6020电机 

static DJIMotor_Instance *motor_yaw, *motor_pitch;  

static PID_Instance yaw_follow_pid,pitch_follow_pid;  // 底盘跟随PID

static Nac_Recv_s *nac_ctrl; // 视觉控制信息

static float yaw_angle,pitch_angle = 0;
//static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
//static float at_lf, at_rf, at_lb, at_rb; // 底盘的角度解算后的临时输出,待进行限幅
//Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
//float V;
// RC_ctrl_t *rc_cmd;
void MotorInit()
{
    //rc_cmd = RemoteControlInit(&huart3);
	
	// 四个轮子的参数一样,改tx_id和反转标志位即可
//    Motor_Init_Config_s chassis_motor_config = {
//        .can_init_config.can_handle   = &hcan1,
//        .controller_param_init_config = {
//            .speed_PID = {
//                .Kp            = 3, // 4.5
//                .Ki            = 0.5, // 0
//                .Kd            = 0,   // 0
//                .IntegralLimit = 5000,
//                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                .MaxOut        = 16000,
//            },
//            .current_PID = {
//                .Kp            = 0.4, // 0.4
//                .Ki            = 0.2,   // 0
//                .Kd            = 0,
//                .IntegralLimit = 3000,
//                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                .MaxOut        = 16000,
//            },
//        },
//        .controller_setting_init_config = {
//            .angle_feedback_source = MOTOR_FEED,
//            .speed_feedback_source = MOTOR_FEED,
//            .outer_loop_type       = SPEED_LOOP,
//            .close_loop_type       = SPEED_LOOP | CURRENT_LOOP,
//        },
//        .motor_type = M3508,
//    };
//    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
//    chassis_motor_config.can_init_config.tx_id                             = 4;
//    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
//    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

//    chassis_motor_config.can_init_config.tx_id                             = 1;
//    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
//    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

//    chassis_motor_config.can_init_config.tx_id                             = 3;
//    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
//    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

//    chassis_motor_config.can_init_config.tx_id                             = 2;
//    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
//    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

    // 6020电机初始化
		
		nac_ctrl     = NacInit(&huart1); // 初始化视觉控制
		
    Motor_Init_Config_s chassis_motor_steering_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp                = 35,
                .Ki                = 0.2,
                .Kd                = 0.11,
                .CoefA             = 3,
                .CoefB             = 0.1,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .IntegralLimit     = 100,
                .MaxOut            = 10000,
                .Derivative_LPF_RC = 0.001,
                .DeadBand          = 0.5,
            },
            .current_PID = {
                .Kp            = 2,
                .Ki            = 0.01,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
                .IntegralLimit = 1000,
                .MaxOut        = 3000,
                .Output_LPF_RC = 0.03,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
		chassis_motor_steering_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis_motor_steering_config.can_init_config.tx_id = 1;
    motor_pitch                                 = DJIMotorInit(&chassis_motor_steering_config);
		
		chassis_motor_steering_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_steering_config.can_init_config.tx_id = 3;
    motor_yaw                                   = DJIMotorInit(&chassis_motor_steering_config);
//    chassis_motor_steering_config.can_init_config.tx_id = 2;
//    motor_steering_lb                                   = DJIMotorInit(&chassis_motor_steering_config);
//    chassis_motor_steering_config.can_init_config.tx_id = 3;
//    motor_steering_rb                                   = DJIMotorInit(&chassis_motor_steering_config);


		PID_Init_Config_s chassis_follow_pid_conf = {
        .Kp                = 6, // 6
        .Ki                = 0.0f,
        .Kd                = 0.0, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit     = 50, // 200
        .MaxOut            = 10000,
        .Derivative_LPF_RC = 0.01, // 0.01
    };
    PIDInit(&yaw_follow_pid, &chassis_follow_pid_conf);
		
		PID_Init_Config_s chassis_follow_pid_conf2 = {
        .Kp                = 6, // 6
        .Ki                = 0.0f,
        .Kd                = 0.0, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit     = 50, // 200
        .MaxOut            = 10000,
        .Derivative_LPF_RC = 0.01, // 0.01
    };
    PIDInit(&pitch_follow_pid, &chassis_follow_pid_conf);
		
}

/**
 * @brief 使舵电机角度最小旋转，取优弧，防止电机旋转不必要的行程
 *          例如：上次角度为0，目标角度为135度，
 *          电机会选择逆时针旋转至-45度，而不是顺时针旋转至135度，
 *          两个角度都会让轮电机处于同一平行线上
 *
 * @param angle 目标角度
 * @param last_angle 上次角度
 *
 */

//static void MinmizeRotation(float *angle, const float *last_angle, float *speed)
//{
//    float rotation = *angle - *last_angle;
//    ANGLE_LIMIT_360_TO_180_ABS(rotation);
//    if (rotation > 90) {
//        *angle -= 180;
//        *speed = -(*speed);
//    } else if (rotation < -90) {
//        *angle += 180;
//        *speed = -(*speed);
//    }
//    ANGLE_LIMIT_360_TO_180_ABS(*angle);
//}

static void MinmizeRotation(float *angle, const float *last_angle)
{
    float rotation = *angle - *last_angle;
   // ANGLE_LIMIT_360_TO_180_ABS(rotation);
    if (rotation > 180) {
        *angle -= 360;
       
    } else if (rotation < -180) {
        *angle += 360;
        
    }
   // ANGLE_LIMIT_360_TO_180_ABS(*angle);
}

void motorbsp()
{
	
		
		DJIMotorSetRef(motor_pitch, 100);
		
	
	
	DJIMotorControl();
}


