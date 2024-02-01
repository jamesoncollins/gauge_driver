/*
 * common.h
 *
 *  Created on: Jan 28, 2024
 *      Author: user
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include "stm32wbxx_hal.h"

#define SYS_TICKS_PER_US (uint32_t) ((float) SystemCoreClock * (float) 0.000001)
#define US_PER_SYS_TICK (float) (1.0f / (float) SystemCoreClock * (float) 1e6)

/*
 * globals to be defined elsewhere
 */
//extern SPI_HandleTypeDef hspi1;
//extern SPI_HandleTypeDef hspi2;
//extern DMA_HandleTypeDef hdma_spi2_tx;
//extern volatile unsigned spi1_lock, spi2_lock;

#endif /* INC_COMMON_H_ */
