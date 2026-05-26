#ifndef INC_PLATFORM_SERVICES_HPP_
#define INC_PLATFORM_SERVICES_HPP_

#include "cpp_main.h"

class PlatformServices
{
public:
  virtual ~PlatformServices() = default;

  virtual void init(SharedRenderCtx &ctx, RuntimeState &state, int &draw_step, uint32_t &timer_draw_ms) = 0;
  virtual void service_background() = 0;
  virtual void poll_inputs() = 0;
  virtual void sample_state(RuntimeState &state) = 0;
  virtual void update_actuators(RuntimeState &state) = 0;
  virtual bool should_render(uint32_t timer_draw_ms) const = 0;
  virtual bool should_exit() const = 0;
  virtual void shutdown() = 0;
};

PlatformServices *create_platform_services();

#endif /* INC_PLATFORM_SERVICES_HPP_ */
