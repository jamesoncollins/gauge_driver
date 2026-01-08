
#include <stdlib.h>
#include <string.h>
#include "stm32wbxx_hal.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "BTBuffer.hpp"


BTBuffer *BTBuffer::BTBuffer_ = nullptr;

void BTBuffer::disableIRQs()
{
  for(int i=0; i<numIRQ; i++)
  {
    HAL_NVIC_DisableIRQ(irqList[i]);
  }
}

void BTBuffer::enableIRQs()
{
  for(int i=0; i<numIRQ; i++)
  {
    HAL_NVIC_EnableIRQ(irqList[i]);
  }
}

BTBuffer::BTBuffer( IRQn_Type *irqs, int num_irqs )
{
  for(int i=0; i<num_irqs; i++)
  {
    irqList[i] = irqs[i];
  }
  numIRQ = num_irqs;
}

BTBuffer* BTBuffer::GetInstance ()
{
  if (BTBuffer_ == nullptr)
    exit(-1);
  return BTBuffer_;

}

void BTBuffer::CreateInstance( IRQn_Type *irqs, int num_irqs )
{
  if (BTBuffer_ == nullptr)
  {
    BTBuffer_ = new BTBuffer (irqs, num_irqs);
  }
  else
  {
    exit(-1);
  }
}


bool BTBuffer::isEmpty()
{
  return (head == tail);
}

bool BTBuffer::isFull()
{
  return ((head + 1) % numBuffers) == tail;
}

bool BTBuffer::popBuffer()
{
  BTBuffer *BTBuffer = BTBuffer::GetInstance();
  if(BTBuffer == nullptr)
    return false;
  if(BTBuffer->isEmpty())
    return false;
  Custom_STM_App_Update_Char(
      CUSTOM_STM_READNEXT,
      (uint8_t*)&BTBuffer->buffer[BTBuffer->head]
      );
  BTBuffer->head = (BTBuffer->head + 1) % BTBuffer->numBuffers;
  return true;
}

bool BTBuffer::pushBuffer( uint16_t id1, uint16_t id2, uint32_t timestamp, const uint8_t *data, int datalen )
{
  BTBuffer *BTBuffer = BTBuffer::GetInstance();
  if(BTBuffer == nullptr)
    return false;
  if(BTBuffer->isFull())
    return false;
  if(datalen>BTBuffer::dataLen)
    return false;
  BTBuffer->disableIRQs();
  BTBuffer->buffer[BTBuffer->tail].id1 = id1;
  BTBuffer->buffer[BTBuffer->tail].id2 = id2;
  BTBuffer->buffer[BTBuffer->tail].timestamp = timestamp;
  memcpy(
      BTBuffer->buffer[BTBuffer->tail].data,
      data,
      dataLen);
  BTBuffer->tail = (BTBuffer->tail + 1) % BTBuffer->numBuffers;
  BTBuffer->enableIRQs();
  return true;
}
