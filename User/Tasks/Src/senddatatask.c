#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "senddatatask.h"
#include "string.h"
#include "bsp_can.h"

/***********º¯Êı¶¨ÒåÇø******************/

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

/*************È«¾Ö±äÁ¿Çø*****************/

Controller_t tx_data; // ×Ô¶¨Òå¿ØÖÆÆ÷·¢ËÍµÄÊı¾İ


void StartSendDataTask(void const *argument)
{
    // uint8_t index = 0;
    uint32_t wait_time = xTaskGetTickCount();
    for (;;)
    {
		// ä½¿ç”¨ memcpy å°†æ¯ä¸ªæµ®ç‚¹æ•°çš„å­—èŠ‚å¤åˆ¶åˆ° int8_t æ•°ç»„ä¸­  
		memcpy(motor_data, &a1, sizeof(a1)); // å°† a1 çš„å­—èŠ‚å¤åˆ¶åˆ° motor_data  
		memcpy(motor_data + 4, &a2, sizeof(a2)); // å°† a2 çš„å­—èŠ‚å¤åˆ¶åˆ° motor_data[4]  
		memcpy(motor_data + 8, &a3, sizeof(a3)); // å°† a3 çš„å­—èŠ‚å¤åˆ¶åˆ° motor_data[8]  
		memcpy(motor_data + 12, &a4, sizeof(a4)); // å°† a4 çš„å­—èŠ‚å¤åˆ¶åˆ° motor_data[12]
        uint8_t data[DATA_LENGTH] = {0};//,motor_yaw_info_3.rotor_angle,motor_yaw_info_1.rotor_angle,motor_yaw_info_4.rotor_angle,motor_yaw_info_5.rotor_angle};
        memcpy(data, motor_data, 4 * sizeof(float));
		Data_Concatenation(data, DATA_LENGTH);
        HAL_UART_Transmit(&huart1, (uint8_t *)(&tx_data), sizeof(tx_data), 50);
        osDelayUntil(&wait_time, 50);
    }
}

/**
 * @brief Êı¾İÆ´½Óº¯Êı£¬½«Ö¡Í·¡¢ÃüÁîÂë¡¢Êı¾İ¶Î¡¢Ö¡Î²Í·Æ´½Ó³ÉÒ»¸öÊı×é
 * @param data Êı¾İ¶ÎµÄÊı×éÖ¸Õë
 * @param data_lenth Êı¾İ¶Î³¤¶È
 */
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    /// Ö¡Í·Êı¾İ
    tx_data.frame_header.sof = 0xA5;                              // Êı¾İÖ¡ÆğÊ¼×Ö½Ú£¬¹Ì¶¨ÖµÎª 0xA5
    tx_data.frame_header.data_length = data_lenth;                // Êı¾İÖ¡ÖĞÊı¾İ¶ÎµÄ³¤¶È
    tx_data.frame_header.seq = seq++;                             // °üĞòºÅ
    append_CRC8_check_sum((uint8_t *)(&tx_data.frame_header), 5); // Ìí¼ÓÖ¡Í· CRC8 Ğ£ÑéÎ»
    /// ÃüÁîÂëID
    tx_data.cmd_id = CONTROLLER_CMD_ID;
    /// Êı¾İ¶Î
    memcpy(tx_data.data, data, data_lenth);
    /// Ö¡Î²CRC16£¬Õû°üĞ£Ñé
    append_CRC16_check_sum((uint8_t *)(&tx_data), DATA_FRAME_LENGTH);
}
