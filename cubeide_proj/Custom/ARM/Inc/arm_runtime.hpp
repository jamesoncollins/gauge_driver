#ifndef INC_ARM_RUNTIME_HPP_
#define INC_ARM_RUNTIME_HPP_

#include <cstdint>

#include "platform_api.h"

int get_x12_ticks_speed(float speed);
int get_x12_ticks_rpm(float rpm);
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA_to_SPI(TIM_HandleTypeDef *htim, const uint32_t *pData, uint16_t Length);

#endif /* INC_ARM_RUNTIME_HPP_ */
