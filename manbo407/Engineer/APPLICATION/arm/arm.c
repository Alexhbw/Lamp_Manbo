#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "ins_task.h"
#include "user_lib.h"
#include "robot_def.h"
#include "arm.h"
#include "miniPC_process.h"
#include "gpio.h"
#include <math.h>
#include <stdint.h>
#include "bsp_buzzer.h"
#define STEP_COUNT 200     // 圆上的插值点数量
#define RADIUS 0.06f        // 圆的半径（单位：米）
#define DISTANCE 0.5f      // 云台到圆心的距离（单位：米）

static float vy_cmd_last = 0.0f, vp_cmd_last = 0.0f; // 上次输出(用于斜率限制)
static int state_isinit = 1;
static int isinit = 1;
static float v_yaw,v_pitch;
static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb;                                     // left right forward back
static DJIMotor_Instance *motor_steering_lf, *motor_steering_rf, *motor_steering_lb, *motor_steering_rb; // 6020电机 

static DJIMotor_Instance *motor_yaw, *motor_pitch;  

static PID_Instance yaw_follow_pid,pitch_follow_pid;  // 底盘跟随PID

static Nac_Recv_s *nac_ctrl; // 视觉控制信息

static int state = 0;

static float center_angle_yaw,center_angle_picth,c_yaw,c_picth = 0;

static float yaw_angle,pitch_angle = 0;
//static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
//static float at_lf, at_rf, at_lb, at_rb; // 底盘的角度解算后的临时输出,待进行限幅
//Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
//float V;
// RC_ctrl_t *rc_cmd;
void ArmInit()
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
		
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
		Motor_Init_Config_s chassis_motor_steering_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp                = 20.55,
                .Ki                = 0.1,
                .Kd                = 0.11,
                .CoefA             = 3,
                .CoefB             = 0.1,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .IntegralLimit     = 100,
                .MaxOut            = 4000,
                .Derivative_LPF_RC = 0.001,
                .DeadBand          = 0.5,
            },
            .current_PID = {
                .Kp            = 2.1,
                .Ki            = 0.05,
                .Kd            = 0.02,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
                .IntegralLimit = 1000,
                .MaxOut        = 4000,
                .Output_LPF_RC = 0.03,
            },
					
//							.angle_PID = {
//                .Kp                = 0.7,
//                .Ki                = 0.01,//0.05
//                .Kd                = 0,
//                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
//                .IntegralLimit     = 30,
//                .CoefB             = 0.1,
//                .CoefA             = 5,
//                .MaxOut            = 12,
//                .Derivative_LPF_RC = 0.005,
//                .Output_LPF_RC     = 0.02,
//           
//						},
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = CURRENT_LOOP|SPEED_LOOP ,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
		chassis_motor_steering_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_steering_config.can_init_config.tx_id = 1;
    motor_pitch                                 = DJIMotorInit(&chassis_motor_steering_config);
		
		chassis_motor_steering_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis_motor_steering_config.can_init_config.tx_id = 3;
    motor_yaw                                   = DJIMotorInit(&chassis_motor_steering_config);
		DJIMotorReset(&motor_yaw);
//		Motor_Init_Config_s chassis_motor_steering_config = {
//        .can_init_config.can_handle   = &hcan1,
//        .controller_param_init_config = {
//            .angle_PID = {
//                .Kp                = 40,
//                .Ki                = 6,
//                .Kd                = 0.5,
//                .CoefA             = 3,
//                .CoefB             = 0.1,
//                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
//                .IntegralLimit     = 30,
//                .MaxOut            = 20000,
//                .Derivative_LPF_RC = 0.001,
//                .DeadBand          = 0.5,
//            },
//            .speed_PID = {
//                .Kp            = 10,
//                .Ki            = 0.5,
//                .Kd            = 0.001,
//                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
//                .IntegralLimit = 4000,
//                .MaxOut        = 16000,
//                .Output_LPF_RC = 0.03,
//            },
//        },
//        .controller_setting_init_config = {
//            .angle_feedback_source = MOTOR_FEED,
//            .speed_feedback_source = MOTOR_FEED,
//            .outer_loop_type       = ANGLE_LOOP,
//            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
//            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
//        },
//        .motor_type = GM6020,
//    };
//		chassis_motor_steering_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
//    chassis_motor_steering_config.can_init_config.tx_id = 3;
//    motor_yaw                                   = DJIMotorInit(&chassis_motor_steering_config);
//		chassis_motor_steering_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
//    chassis_motor_steering_config.can_init_config.tx_id = 1;
//    motor_pitch                                   = DJIMotorInit(&chassis_motor_steering_config);
		

		PID_Init_Config_s chassis_follow_pid_conf = {
        .Kp                = 3,// 6
        .Ki                = 0.1,
        .Kd                = 0.5, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit     = 10, // 200
        .MaxOut            = 300,
        .Derivative_LPF_RC = 0.01, // 0.01
    };
    PIDInit(&yaw_follow_pid, &chassis_follow_pid_conf);
		
//		PID_Init_Config_s chassis_follow_pid_conf2 = {
//        .Kp                = 70, // 6
//        .Ki                = 0.6,
//        .Kd                = 0.07, // 0.5
//        .DeadBand          = 0.5,
//        .CoefA             = 0.2,
//        .CoefB             = 0.3,
//        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
//        .IntegralLimit     = 30, // 200
//        .MaxOut            = 500,
//        .Derivative_LPF_RC = 0.01, // 0.01
//    };
//    PIDInit(&pitch_follow_pid, &chassis_follow_pid_conf2);
		PID_Init_Config_s chassis_follow_pid_conf2 = {
//        .Kp                = 1.5, // 6
//        .Ki                = 0.65f,
//        .Kd                = 0.1, // 0.5
//        .DeadBand          = 0.5,
//        .CoefA             = 0.2,
//        .CoefB             = 0.3,
//        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
//        .IntegralLimit     = 100, // 200
//        .MaxOut            = 300,
//        .Derivative_LPF_RC = 0.01, // 0.01
		  	.Kp                = 2,// 6
        .Ki                = 0.1,
        .Kd                = 0.5, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit     = 10, // 200
        .MaxOut            = 300,
        .Derivative_LPF_RC = 0.01, // 0.01
    };
    PIDInit(&pitch_follow_pid, &chassis_follow_pid_conf2);
		buzzer_init();
		
}


void key_state(){
	DJIMotorSetRef(motor_yaw,c_yaw );
  DJIMotorSetRef(motor_pitch,c_picth);
	
	DJIMotorControl();
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET){
		osDelay(50);
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET){
			state++;
			state_isinit = 1;
			if(state>3)state = 0;
		}
	}
}



static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float absf(float v) { return v >= 0 ? v : -v; }



/* dt: 控制周期(秒) */
void GimbalVisionFollow(float dt)
{
    float x = nac_ctrl->x;
    float y = nac_ctrl->y;

    const float target_x = 276;
    const float target_y = 184;

    /* ==== 你只需要调这几个参数 ==== */
    const float DEAD_X = 2.0f;       // 像素/单位死区
    const float DEAD_Y = 2.0f;

    const float TAU_ERR = 0.03f;     // 误差低通时间常数(秒) 0.02~0.06 常用
    const float V_MAX_YAW = 250.0f; // 输出最大速度指令(按你的DJI接口单位)
    const float V_MAX_PIT = 200.0f;

    const float A_MAX_YAW = 650.0f; // 最大加速度(单位/秒)  越小越“柔”
    const float A_MAX_PIT = 500.0f;
    /* ============================ */

    static float ex_f = 0.0f, ey_f = 0.0f;         // 滤波后的误差
    
    static uint8_t inited = 0;

    uint8_t has_target = !(nac_ctrl->get == 0XFF||nac_ctrl->get==0x00);

    /* pitch 限位：保留你的保护逻辑 */
    float pit_ang = motor_pitch->measure.angle_single_round;
		uint8_t pit_ok = (pit_ang >145 && pit_ang < 320.0f);
    if (!pit_ok) {
				vp_cmd_last = 0.0f;
        DJIMotorSetRef(motor_pitch, 0);
        DJIMotorControl();
        return;
    }
		
		float yaw_ang = motor_pitch->measure.total_angle;
		uint8_t yaw_ok = (yaw_ang >-120 && yaw_ang < 120.0f);
    if (!yaw_ok) {
				vy_cmd_last = 0.0f;
        DJIMotorSetRef(motor_yaw, 0);
        DJIMotorControl();
        return;
    }

    /* 无目标：平滑回零（不要一下子置0，避免“顿挫”） */
    if (!has_target) {
        float step_y = A_MAX_YAW * dt;
        float step_p = A_MAX_PIT * dt;

        vy_cmd_last += clampf(0.0f - vy_cmd_last, -step_y, step_y);
        vp_cmd_last += clampf(0.0f - vp_cmd_last, -step_p, step_p);

        DJIMotorSetRef(motor_yaw,   vy_cmd_last);
        DJIMotorSetRef(motor_pitch, vp_cmd_last);
        DJIMotorControl();

        inited = 0; // 让下次重新初始化滤波，避免跳变
        return;
    }
		static float wg =0;
    /* 计算原始误差 */
		if(nac_ctrl->get == 0x99){
			wg = 0.5;
		}
		else{
			wg = 0.3;
		}
    float ex = -wg*(x - target_x);
    float ey = -wg*(y - target_y);

    /* 死区：中心附近不动，解决抖动 */
    if (absf(ex) < DEAD_X) ex = 0.0f;
    if (absf(ey) < DEAD_Y) ey = 0.0f;

    /* 一阶低通：抑制视觉噪声 */
    if (!inited) { ex_f = ex; ey_f = ey; inited = 1; }
    float alpha = dt / (TAU_ERR + dt);
    ex_f += alpha * (ex - ex_f);
    ey_f += alpha * (ey - ey_f);

    /* PID 外环：误差 -> 速度指令 */
    v_yaw   = PIDCalculate(&yaw_follow_pid,   ex_f, 0.0f);
    v_pitch = PIDCalculate(&pitch_follow_pid, ey_f, 0.0f);

    /* 输出限幅：防止给太大速度 */
    v_yaw   = clampf(v_yaw,   -V_MAX_YAW, V_MAX_YAW);
    v_pitch = clampf(v_pitch, -V_MAX_PIT, V_MAX_PIT);

    /* 斜率限制：限制加速度，让输出更“圆” */
    float step_y = A_MAX_YAW * dt;
    float step_p = A_MAX_PIT * dt;

    vy_cmd_last += clampf(v_yaw   - vy_cmd_last, -step_y, step_y);
    vp_cmd_last += clampf(v_pitch - vp_cmd_last, -step_p, step_p);

    DJIMotorSetRef(motor_yaw,   vy_cmd_last);
    DJIMotorSetRef(motor_pitch, vp_cmd_last);

    DJIMotorControl();
}

void refresh(){
if(nac_ctrl->get == 0X55||nac_ctrl->get == 0X66||nac_ctrl->get == 0X77||nac_ctrl->get == 0X88||nac_ctrl->get == 0X11){
		state = nac_ctrl->get; 
	}
	else{
		state = 0;
	}
	
}

void happy_action(){
			vy_cmd_last = 80;//顺时针
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			
			vy_cmd_last = -60;//顺时针
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
				vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			
			
			vy_cmd_last = 80;//顺时针
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			
			vy_cmd_last = 80;//顺时针
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			
			vy_cmd_last = -60;//顺时针
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
				vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			
			
			vy_cmd_last = 80;//顺时针
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);
			vp_cmd_last =60;//下
			osDelay(300);
			vp_cmd_last =-60;//下
			osDelay(300);

			vy_cmd_last = 0;
			vp_cmd_last =0;
			
			osDelay(2000);
			
}

void sad_action(){
	float pit_ang = motor_pitch->measure.angle_single_round;
	uint8_t pit_ok = (pit_ang >145 && pit_ang < 270.0f);
  vp_cmd_last =80;
	if(!pit_ok){
		vp_cmd_last = 0;
		vy_cmd_last = 50;//顺时针
		osDelay(1000);
		vy_cmd_last = -50;//顺时针
		osDelay(1000);
		vy_cmd_last = 50;//顺时针
		osDelay(1000);
		vy_cmd_last = -50;//顺时针
		osDelay(1000);
		vy_cmd_last = 50;//顺时针
		osDelay(1000);
		vy_cmd_last = -50;//顺时针
		osDelay(1000);
		vy_cmd_last = 50;//顺时针
		osDelay(1000);
		vy_cmd_last = -50;//顺时针
		osDelay(1000);
		vy_cmd_last = 0;
		vp_cmd_last =-80;
		osDelay(900);
		vp_cmd_last =0;
	
		osDelay(2000);
	}
	
	
}

void angry_action(){
	
	
		vp_cmd_last = 0;
		vy_cmd_last = 300;//顺时针
		osDelay(300);
		vy_cmd_last = -300;//顺时针
		osDelay(500);
		vy_cmd_last = 300;//顺时针
		osDelay(400);
		vy_cmd_last = -300;//顺时针
		osDelay(400);
		vy_cmd_last = 300;//顺时针
		osDelay(400);
		vy_cmd_last = -300;//顺时针
		osDelay(400);
		vy_cmd_last = 300;//顺时针
		osDelay(400);
		vy_cmd_last = -300;//顺时针
		osDelay(400);
	
	
		vy_cmd_last =  0;
		vp_cmd_last =0;

		osDelay(2000);
	
	
	
}
void Arm_bsp(){
	const float dt=0.001;
	
	
	switch(state){
		case 0:
			refresh();
			GimbalVisionFollow(dt);
			break;
		case 0x11:{
			vy_cmd_last = 0;
			vp_cmd_last =0;
			osDelay(10000);
			state = 0;
			break;
		}
		case 0x55:{
			angry_action();
			break;
		}
		case 0x66:{
			break;
		}
		case 0x77:{
			happy_action();
			break;
		}
		case 0x88:{
			sad_action();
			break;
		}
		default:break;
	
	}

		
	
	
}
void song_task(){
	if(isinit){
		buzzer_play_song(1);
		isinit = 0;
	}
	switch(state){
		case 0x11:{
			vy_cmd_last = 0;
			vp_cmd_last =0;
			buzzer_off();
			osDelay(10000);
			state = 0;
			break;
		}
		case 0x55:{
			buzzer_play_song(5);
			state = 0;
			break;
		}
		case 0x66:{
			buzzer_play_song(4);
			state = 0;
			refresh();
			break;
		}
		case 0x77:{
			buzzer_play_song(2);
			state = 0;
			break;
		}
		case 0x88:{
			buzzer_play_song(3);
			state = 0;
			break;
		}
		default:
			buzzer_off();
			state = 0;
			break;
	
	}
}

void armmotorbsp(){
	DJIMotorSetRef(motor_yaw, vy_cmd_last);
	DJIMotorSetRef(motor_pitch, vp_cmd_last);
	DJIMotorControl();
	NacSend(state);
}



void Emotion_task(){
//	showBlink();
}