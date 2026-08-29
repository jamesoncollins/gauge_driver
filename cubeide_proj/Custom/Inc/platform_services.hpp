#ifndef INC_PLATFORM_SERVICES_HPP_
#define INC_PLATFORM_SERVICES_HPP_

#include "board_model.hpp"

/*
 * Platform lifecycle and data flow.
 * Shared code owns the main loop, display rendering, and shared UI/widget
 * state. Platforms provide hardware/display resources and publish data.
 */
void board_init(BoardSharedData &data, SharedRenderCtx &ctx);
void board_update();
bool board_check_exit();
bool board_display_ready();
void board_render_before(const RuntimeState &state, const BoardSharedData &data, SharedRenderCtx &ctx);
void board_render_after(const RuntimeState &state, const BoardSharedData &data, SharedRenderCtx &ctx);
void board_shutdown();
void board_reset_loop_diag();

#endif /* INC_PLATFORM_SERVICES_HPP_ */
