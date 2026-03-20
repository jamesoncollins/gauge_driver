#ifndef INC_X86_HAL_H_
#define INC_X86_HAL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Minimal STM-HAL compatibility surface for x86 builds.
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

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_X86_HAL_H_ */
