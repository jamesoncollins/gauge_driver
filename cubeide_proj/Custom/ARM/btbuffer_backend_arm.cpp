#ifndef __arm__
#error "THIS CODE IS FOR ARM"
#endif

#include "btbuffer_backend_arm.hpp"

extern "C" {
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
}

ArmBTBufferBackend::ArmBTBufferBackend(IRQn_Type *irqs, int num_irqs)
{
  if (num_irqs < 0)
    num_irqs = 0;
  if (num_irqs > maxIRQs)
    num_irqs = maxIRQs;
  num_irq_ = num_irqs;
  for (int i = 0; i < num_irq_; ++i)
    irq_list_[i] = irqs[i];
}

void ArmBTBufferBackend::lock()
{
  for (int i = 0; i < num_irq_; ++i)
    HAL_NVIC_DisableIRQ(irq_list_[i]);
}

void ArmBTBufferBackend::unlock()
{
  for (int i = 0; i < num_irq_; ++i)
    HAL_NVIC_EnableIRQ(irq_list_[i]);
}

void ArmBTBufferBackend::emit_readnext(const BTBufferData &item)
{
  Custom_STM_App_Update_Char(CUSTOM_STM_READNEXT, (uint8_t *)&item);
}
