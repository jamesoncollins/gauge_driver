#ifndef INC_HOST_HAL_H_
#define INC_HOST_HAL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Minimal STM-HAL compatibility surface for host simulator builds.
 * Add to this as compile/link errors identify missing APIs.
 */
typedef enum
{
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct { int _unused; } I2C_HandleTypeDef;
typedef struct { int _unused; } SPI_HandleTypeDef;
typedef struct { int _unused; } LPTIM_HandleTypeDef;
typedef struct { int _unused; } TIM_HandleTypeDef;
typedef struct { int _unused; } UART_HandleTypeDef;
typedef struct { int _unused; } RTC_HandleTypeDef;
typedef struct { int _unused; } DMA_HandleTypeDef;
typedef struct { int _unused; } GPIO_TypeDef;

typedef int IRQn_Type;

typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
} GPIO_PinState;

#ifndef HAL_MAX_DELAY
#define HAL_MAX_DELAY (0xFFFFFFFFU)
#endif

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* INC_HOST_HAL_H_ */
