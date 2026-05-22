#ifndef CUSTOM_ARM_BTBUFFER_BACKEND_ARM_HPP_
#define CUSTOM_ARM_BTBUFFER_BACKEND_ARM_HPP_

#ifndef __arm__
#error "THIS HEADER IS FOR ARM"
#endif

#include "platform_api.h"
#include "../BTbuffer-lib/BTBufferBackend.hpp"

class ArmBTBufferBackend : public BTBufferBackend
{
public:
  ArmBTBufferBackend(IRQn_Type *irqs, int num_irqs);
  void lock() override;
  void unlock() override;
  void emit_readnext(const BTBufferData &item) override;

private:
  static const int maxIRQs = 16;
  IRQn_Type irq_list_[maxIRQs] = {};
  int num_irq_ = 0;
};

#endif /* CUSTOM_ARM_BTBUFFER_BACKEND_ARM_HPP_ */

