#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "senddatatask.h"
#include "string.h"
#include "bsp_can.h"

/***********����������******************/

extern moto_info_t motor_yaw_info_1;
extern moto_info_t motor_yaw_info_2;
extern moto_info_t motor_yaw_info_3;
extern moto_info_t motor_yaw_info_4;
extern moto_info_t motor_yaw_info_5;

extern float a1 ;
extern float a2 ;
extern float a3 ;  
extern float a4 ; 

uint8_t motor_data[16];

static void Data_Concatenation(uint8_t *data, uint16_t data_lenth);

/*************ȫ�ֱ�����*****************/

Controller_t tx_data; // �Զ�����������͵�����


void StartSendDataTask(void const *argument)
{
    // uint8_t index = 0;
    uint32_t wait_time = xTaskGetTickCount();
    for (;;)
    {
		// 使用 memcpy 将每个浮点数的字节复制到 int8_t 数组中  
		memcpy(motor_data, &a1, sizeof(a1)); // 将 a1 的字节复制到 motor_data  
		memcpy(motor_data + 4, &a2, sizeof(a2)); // 将 a2 的字节复制到 motor_data[4]  
		memcpy(motor_data + 8, &a3, sizeof(a3)); // 将 a3 的字节复制到 motor_data[8]  
		memcpy(motor_data + 12, &a4, sizeof(a4)); // 将 a4 的字节复制到 motor_data[12]
        uint8_t data[DATA_LENGTH] = {0};//,motor_yaw_info_3.rotor_angle,motor_yaw_info_1.rotor_angle,motor_yaw_info_4.rotor_angle,motor_yaw_info_5.rotor_angle};
        memcpy(data, motor_data, 4 * sizeof(float));
		Data_Concatenation(data, DATA_LENGTH);
        HAL_UART_Transmit(&huart1, (uint8_t *)(&tx_data), sizeof(tx_data), 50);
        osDelayUntil(&wait_time, 50);
    }
}

/**
 * @brief ����ƴ�Ӻ�������֡ͷ�������롢���ݶΡ�֡βͷƴ�ӳ�һ������
 * @param data ���ݶε�����ָ��
 * @param data_lenth ���ݶγ���
 */
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    /// ֡ͷ����
    tx_data.frame_header.sof = 0xA5;                              // ����֡��ʼ�ֽڣ��̶�ֵΪ 0xA5
    tx_data.frame_header.data_length = data_lenth;                // ����֡�����ݶεĳ���
    tx_data.frame_header.seq = seq++;                             // �����
    append_CRC8_check_sum((uint8_t *)(&tx_data.frame_header), 5); // ���֡ͷ CRC8 У��λ
    /// ������ID
    tx_data.cmd_id = CONTROLLER_CMD_ID;
    /// ���ݶ�
    memcpy(tx_data.data, data, data_lenth);
    /// ֡βCRC16������У��
    append_CRC16_check_sum((uint8_t *)(&tx_data), DATA_FRAME_LENGTH);
}
