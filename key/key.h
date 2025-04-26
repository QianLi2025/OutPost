#ifndef __KEY_H
#define __KEY_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef enum {
    KEY_STATE_RELEASE = 0,    // 按键未按下
    KEY_STATE_PRESS,          // 按键按下（瞬时）
    KEY_STATE_LONG_PRESS      // 长按状态
} Key_State;

typedef struct {
    GPIO_TypeDef* GPIOx;      // GPIO端口
    uint16_t GPIO_Pin;        // GPIO引脚
    Key_State state;          // 当前状态
    uint32_t press_duration;  // 按下持续时间（ms）
} Key_HandleTypeDef;

void Key_Init(Key_HandleTypeDef* hkey, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Key_Update(Key_HandleTypeDef* hkey);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H */
