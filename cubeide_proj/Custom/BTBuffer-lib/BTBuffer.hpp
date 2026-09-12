
#pragma once

#include <stdint.h>
#include "platform_api.h"

static constexpr uint8_t kBTBufferRecordLen = 64;
static constexpr uint8_t kBTBufferMetadataLen = sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint32_t);
static constexpr uint8_t kBTBufferPayloadLen = kBTBufferRecordLen - kBTBufferMetadataLen;

typedef struct
{
  uint16_t id1, id2;
  uint32_t timestamp;
  uint8_t data[kBTBufferPayloadLen];
}
BTBufferData;

class BTBufferBackend;

class BTBuffer
{

protected:
  BTBuffer(BTBufferBackend *backend);
  static BTBuffer *BTBuffer_;

public:
  static constexpr int dataLen = sizeof(((BTBufferData *)0)->data);
  static const int numBuffers = 32;

  BTBuffer (BTBuffer &other) = delete;
  void operator= (const BTBuffer&) = delete;
  static bool IsCreated();
  static BTBuffer* GetInstance ();
  static void CreateInstance(BTBufferBackend *backend);
  static bool pushBuffer( uint16_t id1, uint16_t id2, uint32_t timestamp, const uint8_t *data, int datalen );
  static bool popBuffer();

private:



  /*
   * we're going to use a waste slot in order to avoid needing
   * to lock access to the full flag.
   * https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/
   */
  BTBufferData buffer[numBuffers];
  int head = 0; // read from
  int tail = 0; // write to
  bool isEmpty(), isFull();

  BTBufferBackend *backend_ = nullptr;

};




