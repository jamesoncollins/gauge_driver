
#include <stdlib.h>
#include <string.h>
#include "BTBuffer.hpp"
#include "BTBufferBackend.hpp"


BTBuffer *BTBuffer::BTBuffer_ = nullptr;

BTBuffer::BTBuffer(BTBufferBackend *backend) : backend_(backend) {}

BTBuffer* BTBuffer::GetInstance ()
{
  if (BTBuffer_ == nullptr)
    exit(-1);
  return BTBuffer_;

}

void BTBuffer::CreateInstance(BTBufferBackend *backend)
{
  if (BTBuffer_ == nullptr)
  {
    BTBuffer_ = new BTBuffer(backend);
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
  if(BTBuffer->backend_ != nullptr)
    BTBuffer->backend_->emit_readnext(BTBuffer->buffer[BTBuffer->head]);
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
  if(datalen < 0)
    return false;
  if(datalen>BTBuffer::dataLen)
    return false;
  if(BTBuffer->backend_ != nullptr)
    BTBuffer->backend_->lock();
  BTBuffer->buffer[BTBuffer->tail].id1 = id1;
  BTBuffer->buffer[BTBuffer->tail].id2 = id2;
  BTBuffer->buffer[BTBuffer->tail].timestamp = timestamp;
  memcpy(
      BTBuffer->buffer[BTBuffer->tail].data,
      data,
      (size_t)datalen);
  if(datalen < BTBuffer::dataLen)
  {
    memset(
        &BTBuffer->buffer[BTBuffer->tail].data[datalen],
        0,
        (size_t)(BTBuffer::dataLen - datalen));
  }
  BTBuffer->tail = (BTBuffer->tail + 1) % BTBuffer->numBuffers;
  if(BTBuffer->backend_ != nullptr)
    BTBuffer->backend_->unlock();
  return true;
}
