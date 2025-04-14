#ifndef PID_H
#define PID_H

#include "main.h"
#include "can.h"
#include "bsp_can.h"
#include "gpio.h"



extern moto_info_t motor_yaw_info;

typedef struct
{
       float kp, ki, kd; //三个系数
    float error, lastError; //误差、上次误差
    float integral, maxIntegral; //积分、积分限幅
    float output, maxOutput; //输出、输出限幅
}PID;
//串级PID的结构体，包含两个单级PID
typedef struct
{
    PID inner; //内环
    PID outer; //外环
    float output; //串级输出，等于inner.output
}CascadePID;
 void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb);
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);
// 整车移动量转换为单轮速度  x:前+后-  y:左+右-  z:逆+顺-
void Move_Transfrom(float *TargetA,float *TargetB,float *TargetC,float *TargetD,float Vx,float Vy,float Vz,float Car_H,float Car_W);
//typedef struct {
//    float Kp, Ki, Kd;          
//    float Error_Last1;         
//    float Error_Last2;         
//    float Out_Last;           
//} PID_Increment_Struct;

//float PID_Increment(PID_Increment_Struct *PID, float Current, float Target);
#endif
