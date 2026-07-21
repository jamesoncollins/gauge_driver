#ifndef INC_MAIN_SHARED_H_
#define INC_MAIN_SHARED_H_

#include "board_model.hpp"

template <typename StepFn>
inline void run_shared_main_loop(bool &exit_flag, StepFn &&step_fn)
{
  while (!exit_flag)
  {
    step_fn();
  }
}

void render_step_shared(const RuntimeState &state, SharedRenderCtx &ctx, int &draw_step, uint32_t &timer_draw_ms);
void render_step_shared(const RuntimeState &state, const BoardSharedData &data, SharedRenderCtx &ctx, int &draw_step, uint32_t &timer_draw_ms);
void render_ctx_init_shared(SharedRenderCtx &ctx);
RuntimeState runtime_state_from_board_data(const BoardSharedData &data);
int compute_rpm_mode_shared(float rpm, int prev_mode);
bool runtime_diag_square_enabled();
uint32_t runtime_diag_fps();
void runtime_diag_set_square_enabled(bool enabled);
void runtime_diag_reset_shared();

#endif /* INC_MAIN_SHARED_H_ */
