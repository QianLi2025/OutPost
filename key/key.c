#include "key.h"

void Key_Init(Key_HandleTypeDef* hkey, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    if(hkey == NULL) return;

    hkey->GPIOx = GPIOx;
    hkey->GPIO_Pin = GPIO_Pin;
    hkey->state = KEY_STATE_RELEASE;
    hkey->press_duration = 0;
}

void Key_Update(Key_HandleTypeDef* hkey)
{
    static uint8_t last_state = GPIO_PIN_SET;

		uint8_t current_state = HAL_GPIO_ReadPin(hkey->GPIOx, hkey->GPIO_Pin);

    if(current_state == GPIO_PIN_RESET)
		{
        if(last_state == GPIO_PIN_SET)
				{
            hkey->state = KEY_STATE_PRESS;
            hkey->press_duration = 0;
        }
				else {
            hkey->press_duration += 10;     
            if(hkey->press_duration >= 2000)
						{
                hkey->state = KEY_STATE_LONG_PRESS;
            }
        }
    }
		else
		{
        hkey->state = KEY_STATE_RELEASE;
        hkey->press_duration = 0;
    }
    
    last_state = current_state;
}
