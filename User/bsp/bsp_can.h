#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "M3508motors.h"
#include "pid.h"

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
extern CAN_HandleTypeDef hcan1;


void pid_all_init(void);
typedef struct
{
	int16_t rotor_angle;
	int16_t rotor_speed;
	int16_t torque_current;
	int16_t temp;
}moto_info_t;

void Can_filter_init(void);
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_chassis1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//float a1 ;
//float a2 ;
//float a3 ;  
//float a4 ; 

//uint8_t motor_data[16];

#endif
