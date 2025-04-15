#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "senddatatask.h"
#include "string.h"
#include "bsp_can.h"
#include "M3508motors.h"

/***********oˉ??¨??******************/

extern moto_info_t motor_yaw_info_1;
extern moto_info_t motor_yaw_info_2;
extern moto_info_t motor_yaw_info_3;
extern moto_info_t motor_yaw_info_4;
extern moto_info_t motor_yaw_info_5;

extern float a1 ;
extern float a2 ;
extern float a3 ;  
extern float a4 ; 
extern float a5 ; 

uint8_t motor_data[22];

int8_t roll_direction;
int8_t state=0;
int8_t z_state=0;

static void Data_Concatenation(uint8_t *data, uint16_t data_lenth);

/*************?????*****************/

Controller_t tx_data; // ??¨????÷·￠?μ?y??


void StartSendDataTask(void const *argument)
{
    // uint8_t index = 0;
    uint32_t wait_time = xTaskGetTickCount();
    for (;;)
    {
			if(roll_motor.total_angle>0.35)
			{roll_direction=1;}
			else if(roll_motor.total_angle<-0.35)
			{roll_direction=2;}
			else
			{roll_direction=0;}
			
			if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_RESET){
				state = 1;
			}
			else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_SET){
				state =0;
			}
      if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_RESET)&&(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == GPIO_PIN_SET))
      {
        z_state=1;
      }
      else if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_SET)&&(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == GPIO_PIN_RESET))
      {
        z_state=2;
      }
      else if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_SET)&&(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == GPIO_PIN_SET))
      {
        z_state=0;
      }
		// 使用 memcpy 将每个浮点数的字节复制到 int8_t 数组中  
		memcpy(motor_data, &a1, sizeof(a1)); // 将 a1 的字节复制到 motor_data  
		memcpy(motor_data + 4, &a2, sizeof(a2)); // 将 a2 的字节复制到 motor_data[4]  
		memcpy(motor_data + 8, &a3, sizeof(a3)); // 将 a3 的字节复制到 motor_data[8]  
		memcpy(motor_data + 12, &a4, sizeof(a4)); // 将 a4 的字节复制到 motor_data[12]
			
	  memcpy(motor_data + 16, &roll_direction, sizeof(roll_direction));
    memcpy(motor_data + 17, &state, sizeof(state));
    memcpy(motor_data + 18, &z_state, sizeof(z_state));
			
        uint8_t data[DATA_LENGTH] = {0};//,motor_yaw_info_3.rotor_angle,motor_yaw_info_1.rotor_angle,motor_yaw_info_4.rotor_angle,motor_yaw_info_5.rotor_angle};
				memcpy(data, motor_data, sizeof(motor_data));
				Data_Concatenation(data, DATA_LENGTH);
        HAL_UART_Transmit(&huart1, (uint8_t *)(&tx_data), sizeof(tx_data), 50);
        osDelayUntil(&wait_time, 50);
    }
}

/**
 * @brief ???′??ˉ?￡??????￠üá?￠???Ρ￠?β??????????
 * @param data ???ε?y???
 * @param data_lenth ???γ¤??
 */
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    /// ?????
    tx_data.frame_header.sof = 0xA5;                              // ??????????1?¨?? 0xA5
    tx_data.frame_header.data_length = data_lenth;                // ????????ε?¤??
    tx_data.frame_header.seq = seq++;                             // °üвoō
    append_CRC8_check_sum((uint8_t *)(&tx_data.frame_header), 5); // ????? CRC8 У?λ
    /// üáID
    tx_data.cmd_id = CONTROLLER_CMD_ID;
    /// ????
    memcpy(tx_data.data, data, data_lenth);
    /// ?βCRC16￡??°üУ?
    append_CRC16_check_sum((uint8_t *)(&tx_data), DATA_FRAME_LENGTH);
}
