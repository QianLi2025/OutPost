#include "main.h"
#include "can.h"
#include "bsp_can.h"
#include "gpio.h"
#include "pid.h"

extern moto_info_t motor_yaw_info_1;

#define TARGET_SPEED 1500  // 目标转速，以RPM为单位  
#define MOTOR_FORWARD 1     // 正转  
#define MOTOR_REVERSE -1    // 反转  
#define MOTOR_STOP 0        // 停止  

int motor_direction = MOTOR_FORWARD; // 初始设置为正转

PID_Increment_Struct PID_Speed = {10.0f, 0.2f, 0.1f};

#define MOTOR_CAN_ID 0x200     
#define CAN_Handle hcan1       
#define MAX_OUTPUT 16384       



float GetMotorSpeed()
{
    int16_t speed = 0;
	speed=motor_yaw_info_1.rotor_speed;
    return speed;
}




float PID_Increment(PID_Increment_Struct *PID, float Current, float Target)
{
    float err, out, proportion, differential;
    err = Target - Current;
    proportion = err - PID->Error_Last1;

    
    differential = err - 2 * PID->Error_Last1 + PID->Error_Last2;
    out = PID->Out_Last + PID->Kp * proportion + PID->Ki * err + PID->Kd * differential;
    PID->Error_Last2 = PID->Error_Last1;
    PID->Error_Last1 = err;
    PID->Out_Last = out;
    return out;
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{ 
//	float target_speed = 19 * 24 * motor_direction;
//	float current_speed = GetMotorSpeed();
//	float control_output;
//	control_output = PID_Increment(&PID_Speed, current_speed, target_speed);
//	if (control_output > MAX_OUTPUT)
//		control_output = MAX_OUTPUT;
//	else if (control_output < -MAX_OUTPUT)
//		control_output = -MAX_OUTPUT;
//	CAN_cmd_chassis(control_output, control_output, control_output, control_output);
//}


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
//{  
//    if (GPIO_Pin == GPIO_PIN_0)
//	{  
//        // 切换电机方向  
//        if (motor_direction == MOTOR_FORWARD) 
//		{  
//            motor_direction = MOTOR_REVERSE; // 改为反转  
//        }
//		else 
//		{  
//            motor_direction = MOTOR_FORWARD;  // 改为正转  
//        }  
//    }  
//} 
