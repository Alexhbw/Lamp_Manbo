#include "chassis.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "ins_task.h"
#include "user_lib.h"
#include "robot_def.h"
#include "miniPC_process.h"

static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb;                                     // left right forward back
static DJIMotor_Instance *motor_steering_lf, *motor_steering_rf, *motor_steering_lb, *motor_steering_rb; // 6020电机 

static DJIMotor_Instance *motor_yaw, *motor_pitch;  

static PID_Instance chassis_follow_pid;  // 底盘跟随PID

static Nac_Recv_s *nac_ctrl; // 视觉控制信息

static float yaw_angle,pitch_angle = 0;
//static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
//static float at_lf, at_rf, at_lb, at_rb; // 底盘的角度解算后的临时输出,待进行限幅
//Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
//float V;
// RC_ctrl_t *rc_cmd;
void ChassisInit()
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
            .angle_PID = {
                .Kp                = 20,
                .Ki                = 0,
                .Kd                = 0.5,
                .CoefA             = 3,
                .CoefB             = 0.1,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .IntegralLimit     = 1000,
                .MaxOut            = 20000,
                .Derivative_LPF_RC = 0.001,
                .DeadBand          = 0.5,
            },
            .speed_PID = {
                .Kp            = 10,
                .Ki            = 0.5,
                .Kd            = 0.001,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
                .IntegralLimit = 4000,
                .MaxOut        = 16000,
                .Output_LPF_RC = 0.03,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    chassis_motor_steering_config.can_init_config.tx_id = 3;
    motor_yaw                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 1;
    motor_pitch                                   = DJIMotorInit(&chassis_motor_steering_config);
//    chassis_motor_steering_config.can_init_config.tx_id = 2;
//    motor_steering_lb                                   = DJIMotorInit(&chassis_motor_steering_config);
//    chassis_motor_steering_config.can_init_config.tx_id = 3;
//    motor_steering_rb                                   = DJIMotorInit(&chassis_motor_steering_config);


		PID_Init_Config_s chassis_follow_pid_conf = {
        .Kp                = 520, // 6
        .Ki                = 0.1f,
        .Kd                = 17, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit     = 500, // 200
        .MaxOut            = 10000,
        .Derivative_LPF_RC = 0.01, // 0.01
    };
    PIDInit(&chassis_follow_pid, &chassis_follow_pid_conf);
		
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

void Gimbal_Control()
{
	float yaw_last, pitch_last; // 上次的角度
	yaw_last   = motor_yaw->   measure.total_angle;
	pitch_last = motor_pitch-> measure.total_angle;
	
//	ANGLE_LIMIT_360_TO_180_ABS(yaw_angle);
//	ANGLE_LIMIT_360_TO_180_ABS(pitch_angle);

	MinmizeRotation(&yaw_angle, &yaw_last);
	MinmizeRotation(&pitch_angle, &pitch_last);
      
	DJIMotorSetRef(motor_yaw, yaw_angle);
  DJIMotorSetRef(motor_pitch, pitch_angle);
	
	DJIMotorControl();
}



void ChassisTask(){
	yaw_angle = pitch_angle = 0;
	osDelay(5000);
	yaw_angle = pitch_angle = 90;
	osDelay(1000);
	yaw_angle = pitch_angle = 0;
	osDelay(5000);
	yaw_angle = pitch_angle = 290;
	osDelay(1000);
}
//void GetCmd(){
//	 
//	//解算似乎有点问题，左走变直线，直线变左走，因此这直接改两个轴
//	chassis_ctrl_cmd.vy = ((float)rc_cmd->rc.rocker_l_/660)*10000;
//	chassis_ctrl_cmd.vx= ((float)rc_cmd->rc.rocker_l1/660)*10000;
//	chassis_ctrl_cmd.vw = ((float)rc_cmd->rc.dial/660)*10000;

//}


//void SteeringWheelCalculate_Speed(){
//	float theta = atan(1.0/1.0); //45度角的弧度制表达
//	float chassis_vx = chassis_ctrl_cmd.vx;
//	float chassis_vy = chassis_ctrl_cmd.vy;
//	float chassis_vw = chassis_ctrl_cmd.vw;
//	
//		vt_lf
//		= sqrt(	pow(chassis_vx - chassis_vw*sin(theta),2)
//					+	pow(chassis_vy - chassis_vw*cos(theta),2)) ;    
//																																																																															
//		vt_rf
//		= -sqrt(	pow(chassis_vx - chassis_vw*sin(theta),2)
//					+	pow(chassis_vy + chassis_vw*cos(theta),2));
//					
//		vt_lb
//		= -sqrt(	pow(chassis_vx + chassis_vw*sin(theta),2)
//					+	pow(chassis_vy + chassis_vw*cos(theta),2)) ;    
//																																																																															
//		vt_rb
//		= sqrt(	pow(chassis_vx + chassis_vw*sin(theta),2)
//					+	pow(chassis_vy - chassis_vw*cos(theta),2));
//		DJIMotorSetRef(motor_lf, vt_lf );
//    DJIMotorSetRef(motor_rf, -vt_rf );
//    DJIMotorSetRef(motor_lb, -vt_lb );
//    DJIMotorSetRef(motor_rb, vt_rb );
//		DJIMotorControl();
//}

//void SteeringWheelCalculate_Angle(){
//	float theta = atan(1.0/1.0); //45度角的弧度制表达
//	float chassis_vx = chassis_ctrl_cmd.vx;
//	float chassis_vy = chassis_ctrl_cmd.vy;
//	float chassis_vw = chassis_ctrl_cmd.vw;
// 
// 

//	if((chassis_vx==0)&&(chassis_vy==0)&&(chassis_vw==0))              
//	{
//		at_lf=(atan2((chassis_vy-chassis_vw*sin(theta)),
//							(chassis_vx-chassis_vw*cos(theta)))*180/3.14)+STEERING_CHASSIS_ALIGN_ANGLE_LF;       
//		at_rb=(atan2((chassis_vy+chassis_vw*sin(theta)),
//							(chassis_vx-chassis_vw*cos(theta)))*180/3.14)+STEERING_CHASSIS_ALIGN_ANGLE_RB;       
//		at_rf=(atan2((chassis_vy-chassis_vw*sin(theta)),
//							(chassis_vx+chassis_vw*cos(theta)))*180/3.14)/+STEERING_CHASSIS_ALIGN_ANGLE_RF;      
//		at_lb=(atan2((chassis_vy+chassis_vw*sin(theta)),
//							(chassis_vx+chassis_vw*cos(theta)))*180/3.14)/+STEERING_CHASSIS_ALIGN_ANGLE_LB;      
//	}
//	else           
//	{
//		at_lf=(atan2((chassis_vy-chassis_vw*sin(theta)),
//							(chassis_vx-chassis_vw*cos(theta)))*180/3.14)+STEERING_CHASSIS_ALIGN_ANGLE_LF;       
//		at_rb=(atan2((chassis_vy+chassis_vw*sin(theta)),
//							(chassis_vx-chassis_vw*cos(theta)))*180/3.14)+STEERING_CHASSIS_ALIGN_ANGLE_RB;       
//		at_rf=(atan2((chassis_vy-chassis_vw*sin(theta)),
//							(chassis_vx+chassis_vw*cos(theta)))*180/3.14)+STEERING_CHASSIS_ALIGN_ANGLE_RF;      
//		at_lb=(atan2((chassis_vy+chassis_vw*sin(theta)),
//							(chassis_vx+chassis_vw*cos(theta)))*180/3.14)+STEERING_CHASSIS_ALIGN_ANGLE_LB;       
//	}

//	
//	float at_lf_last, at_rf_last, at_lb_last, at_rb_last; // 上次的角度
//    at_lb_last = motor_steering_lb->measure.last_ecd;
//    at_lf_last = motor_steering_lf->measure.last_ecd;
//    at_rf_last = motor_steering_rf->measure.last_ecd;
//    at_rb_last = motor_steering_rb->measure.last_ecd;	

//	 
//	DJIMotorSetRef(motor_steering_lf, at_lf);
//  DJIMotorSetRef(motor_steering_rf, at_rf);
//  DJIMotorSetRef(motor_steering_lb, at_lb);
//  DJIMotorSetRef(motor_steering_rb, at_rb);
//	DJIMotorControl();
////	memcpy(chassis_handle->lastSteeringAngletarget, wheel_angle, 4 * sizeof(fp32));

//}

//void jiesuan(){
//	SteeringWheelCalculate_Speed();
//	SteeringWheelCalculate_Angle();
//		
//}	
///**
// * @brief 舵轮电机角度解算
// *
// */
//static void SteeringWheelCalculate()
//{
//    float offset_lf, offset_rf, offset_lb, offset_rb;     // 用于计算舵轮的角度
//    float at_lf_last, at_rf_last, at_lb_last, at_rb_last; // 上次的角度
//    at_lb_last = motor_steering_lb->measure.last_ecd;
//    at_lf_last = motor_steering_lf->measure.last_ecd;
//    at_rf_last = motor_steering_rf->measure.last_ecd;
//    at_rb_last = motor_steering_rb->measure.last_ecd;
//	
//		float chassis_vx = chassis_ctrl_cmd.vx;
//		float chassis_vy = chassis_ctrl_cmd.vy;
//		float chassis_vw = chassis_ctrl_cmd.vw;
//    if (0) {
//        // at_lf = at_lf_last;
//        // at_rf = at_rf_last;
//        // at_lb = at_lb_last;
//        // at_rb = at_rb_last;
//        vt_lb = 0, vt_lf = 0, vt_rf = 0, vt_rb = 0;
//    } else {
//        // 生成预计算变量，减少计算量，空间换时间
//        // chassis_vx = chassis_vx * 1.5;chassis_vy = chassis_vy * 1.5;
////        float w      = chassis_cmd_recv.wz * CHASSIS_WHEEL_OFFSET * SQRT2;
//				float w = chassis_vw;
//			
//        float temp_x = chassis_vx - w, temp_y = chassis_vy - w;
//        arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_lf); // lf：y- , x-
//        temp_y = chassis_vy + w;                                 // 重复利用变量,temp_x = chassis_vy - w;与上次相同因此注释
//        arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_lb); // lb: y+ , x-
//        temp_x = chassis_vx + w;                                 // temp_y = chassis_vx + w;与上次相同因此注释
//        arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_rb); // rb: y+ , x+
//        temp_y = chassis_vy - w;                                 // temp_x = chassis_vy + w;与上次相同因此注释
//        arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_rf); // rf: y- , x+

//        // 计算角度偏移
//        offset_lf = atan2f(chassis_vy - w, chassis_vx - w) * RAD_2_DEGREE; // lf:  y- , x-
//        offset_rf = atan2f(chassis_vy - w, chassis_vx + w) * RAD_2_DEGREE; // rf:  y- , x+
//        offset_lb = atan2f(chassis_vy + w, chassis_vx - w) * RAD_2_DEGREE; // lb:  y+ , x-
//        offset_rb = atan2f(chassis_vy + w, chassis_vx + w) * RAD_2_DEGREE; // rb:  y+ , x+

//        at_lf = STEERING_CHASSIS_ALIGN_ANGLE_LF + offset_lf; 
//        at_rf = STEERING_CHASSIS_ALIGN_ANGLE_RF + offset_rf;
//        at_lb = STEERING_CHASSIS_ALIGN_ANGLE_LB + offset_lb;
//        at_rb = STEERING_CHASSIS_ALIGN_ANGLE_RB + offset_rb;
//								
//				ANGLE_LIMIT_360_TO_180_ABS(at_lf);
//        ANGLE_LIMIT_360_TO_180_ABS(at_rf);
//        ANGLE_LIMIT_360_TO_180_ABS(at_lb);
//        ANGLE_LIMIT_360_TO_180_ABS(at_rb);

//        MinmizeRotation(&at_lf, &at_lf_last, &vt_lf);
//        MinmizeRotation(&at_rf, &at_rf_last, &vt_rf);
//        MinmizeRotation(&at_lb, &at_lb_last, &vt_lb);
//        MinmizeRotation(&at_rb, &at_rb_last, &vt_rb);
//   if(w==0){ 

//    DJIMotorSetRef(motor_steering_lf, at_lf+10);
//    DJIMotorSetRef(motor_steering_rf, at_rf);
//    DJIMotorSetRef(motor_steering_lb, at_lb);
//    DJIMotorSetRef(motor_steering_rb, at_rb+10);
//		
//		DJIMotorSetRef(motor_lf, vt_lf );
//    DJIMotorSetRef(motor_rf, -vt_rf );
//    DJIMotorSetRef(motor_lb, -vt_lb );
//    DJIMotorSetRef(motor_rb, vt_rb );
//	}
//			else{
//		motor_steering_lf->motor_controller.angle_PID.Kp=7;
//		motor_steering_lf->motor_controller.speed_PID.Kp=5;
//		motor_steering_lf->motor_controller.angle_PID.Ki=0.05;
//		motor_steering_rf->motor_controller.angle_PID.Kp=5;
//		motor_steering_rf->motor_controller.speed_PID.Kp=5;
//		motor_steering_rf->motor_controller.angle_PID.Ki=0.05;
//		DJIMotorSetRef(motor_steering_lf, -at_lf-10);
//    DJIMotorSetRef(motor_steering_rf, -at_rf);
//    DJIMotorSetRef(motor_steering_lb, at_lb);
//    DJIMotorSetRef(motor_steering_rb, at_rb+10);
//		
//		DJIMotorSetRef(motor_lf, vt_lf );
//    DJIMotorSetRef(motor_rf, -vt_rf );
//    DJIMotorSetRef(motor_lb, vt_lb );
//    DJIMotorSetRef(motor_rb, vt_rb );
//			
//			
//			
//			
//			}
//}
//}


//void ChassisTask()
//{
//	
//	GetCmd();
////	chassis_ctrl_cmd.vy = 0;
////	chassis_ctrl_cmd.vx = 0;
////	chassis_ctrl_cmd.vw = 0;
//	SteeringWheelCalculate();

//}
