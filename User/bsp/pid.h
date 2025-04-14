#ifndef PID_H
#define PID_H

#include "main.h"
#include "can.h"
#include "bsp_can.h"
#include "gpio.h"



extern moto_info_t motor_yaw_info;

typedef struct
{
       float kp, ki, kd; //����ϵ��
    float error, lastError; //���ϴ����
    float integral, maxIntegral; //���֡������޷�
    float output, maxOutput; //���������޷�
}PID;
//����PID�Ľṹ�壬������������PID
typedef struct
{
    PID inner; //�ڻ�
    PID outer; //�⻷
    float output; //�������������inner.output
}CascadePID;
 void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb);
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);
// �����ƶ���ת��Ϊ�����ٶ�  x:ǰ+��-  y:��+��-  z:��+˳-
void Move_Transfrom(float *TargetA,float *TargetB,float *TargetC,float *TargetD,float Vx,float Vy,float Vz,float Car_H,float Car_W);
//typedef struct {
//    float Kp, Ki, Kd;          
//    float Error_Last1;         
//    float Error_Last2;         
//    float Out_Last;           
//} PID_Increment_Struct;

//float PID_Increment(PID_Increment_Struct *PID, float Current, float Target);
#endif
