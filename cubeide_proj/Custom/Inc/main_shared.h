#ifndef INC_MAIN_SHARED_H_
#define INC_MAIN_SHARED_H_

#include "cpp_main.h"

template <typename StepFn>
inline void run_shared_main_loop(bool &exit_flag, StepFn &&step_fn)
{
  while (!exit_flag)
  {
    step_fn();
  }
}

void render_step_shared(const RuntimeState &state, SharedRenderCtx &ctx, int &draw_step, uint32_t &timer_draw_ms);
void render_ctx_init_shared(SharedRenderCtx &ctx);
RuntimeState runtime_state_from_sample(const PlatformSample &sample);
int compute_rpm_mode_shared(float rpm, int prev_mode);

void platform_main_init(SharedRenderCtx &ctx, RuntimeState &state, int &draw_step, uint32_t &timer_draw_ms);
void platform_main_step(RuntimeState &state, bool &exit_requested, bool &render_requested, uint32_t timer_draw_ms);
void platform_main_shutdown();

#endif /* INC_MAIN_SHARED_H_ */
