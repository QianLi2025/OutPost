#ifndef PID_H
#define PID_H

#include "main.h"
#include "can.h"
#include "bsp_can.h"
#include "gpio.h"



extern moto_info_t motor_yaw_info;


typedef struct {
    float Kp, Ki, Kd;          
    float Error_Last1;         
    float Error_Last2;         
    float Out_Last;           
} PID_Increment_Struct;

float PID_Increment(PID_Increment_Struct *PID, float Current, float Target);

#endif
