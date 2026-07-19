#include "host_hal.h"
#include <chrono>
#include <thread>

static const auto kTickStart = std::chrono::steady_clock::now();

extern "C" uint32_t HAL_GetTick(void)
{
  const auto now = std::chrono::steady_clock::now();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - kTickStart).count();
  return static_cast<uint32_t>(ms);
}

extern "C" void HAL_Delay(uint32_t ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

extern "C" void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  (void)GPIOx;
  (void)GPIO_Pin;
  (void)PinState;
}

extern "C" GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  (void)GPIOx;
  (void)GPIO_Pin;
  return GPIO_PIN_RESET;
}
