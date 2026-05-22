#pragma once

#include "BTBuffer.hpp"

class BTBufferBackend
{
public:
  virtual ~BTBufferBackend() = default;
  virtual void lock() = 0;
  virtual void unlock() = 0;
  virtual void emit_readnext(const BTBufferData &item) = 0;
};

