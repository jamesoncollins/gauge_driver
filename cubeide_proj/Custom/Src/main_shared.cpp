#include <cmath>
#include <cstdio>

#include "main_shared.h"
#include "build_config.hpp"
#include "platform_services.hpp"
#include "telemetry.hpp"
#include "gui_layout.hpp"
#include "gauge_layouts.hpp"
#include "gfx.h"
#include "ugfx_widgets.h"
#include "../ECUK-lib/ECUK.hpp"
#include "../Quaternion/Quaternion.hpp"
namespace
{
constexpr float kGimbalScale = 5.0f;
bool g_diag_square_enabled = false;
uint32_t g_diag_fps = 0;
uint32_t g_diag_fps_frames = 0;
uint32_t g_diag_fps_last_ms = 0;
bool g_shift_alarm_fast_path_active = false;

void render_diag_overlay(const RuntimeState &state, const SharedRenderCtx &ctx, color_t color)
{
  if (!runtime_diag_square_enabled() || !platform_state_has(state.data_mask, PLATFORM_DATA_TIMING_DIAG))
    return;

  static const int diag_size = 86;
  const int xdiag = ((int)ctx.screen_width - diag_size) / 2;
  const int ydiag = ((int)ctx.screen_height - diag_size) / 2;
  char logBuf[32];
  gdispFillArea(xdiag - 1, ydiag - 1, diag_size, diag_size, GFX_BLACK);
  gdispDrawBox(xdiag - 1, ydiag - 1, diag_size, diag_size, color);
  (void)std::snprintf(logBuf, sizeof(logBuf), "fps %lu", (unsigned long)runtime_diag_fps());
  gdispFillString(xdiag + 4, ydiag + 6, logBuf, ctx.font10, color, GFX_BLACK);
  (void)std::snprintf(logBuf, sizeof(logBuf), "loop %lu", (unsigned long)state.loop_period_ms);
  gdispFillString(xdiag + 4, ydiag + 20, logBuf, ctx.font10, color, GFX_BLACK);
  (void)std::snprintf(logBuf, sizeof(logBuf), "worst %lu", (unsigned long)state.worst_loop_period_ms);
  gdispFillString(xdiag + 4, ydiag + 34, logBuf, ctx.font10, color, GFX_BLACK);
  (void)std::snprintf(logBuf, sizeof(logBuf), "cnt %lu", (unsigned long)state.loop_count);
  gdispFillString(xdiag + 4, ydiag + 48, logBuf, ctx.font10, color, GFX_BLACK);
}
void record_completed_frame(uint32_t now)
{
  if (g_diag_fps_last_ms == 0U)
    g_diag_fps_last_ms = now;
  ++g_diag_fps_frames;
  const uint32_t fps_elapsed_ms = now - g_diag_fps_last_ms;
  if (fps_elapsed_ms >= 1000U)
  {
    g_diag_fps = (g_diag_fps_frames * 1000U) / fps_elapsed_ms;
    g_diag_fps_frames = 0;
    g_diag_fps_last_ms = now;
  }
}

void compute_gimbal_from_board_acceleration(const BoardAccelerationVector &accel, int &gimbal_x, int &gimbal_y)
{
  const float pitch_rad = get_build_config().vehicle.board_mount_pitch_deg * 3.14159265358979323846f / 180.0f;
  const float cos_pitch = std::cos(pitch_rad);
  const float sin_pitch = std::sin(pitch_rad);
  float rotated_accel[3] = {accel.x_mps2, accel.y_mps2, accel.z_mps2};

  rotateVectorKnownPitch(rotated_accel, cos_pitch, sin_pitch);
  gimbal_x = (int)(-rotated_accel[1] * kGimbalScale);
  gimbal_y = (int)(-rotated_accel[0] * kGimbalScale);
}
}

void render_ctx_init_shared(SharedRenderCtx &ctx)
{
  if (ctx.line_plot_tps != nullptr && ctx.line_plot_tps->isInit == false)
    linePlotInit(ctx.line_plot_tps, ctx.line_plot_tps_data, 20, 210, 78, 100, 0);

  if (ctx.line_plot_knock != nullptr && ctx.line_plot_knock->isInit == false)
    linePlotInit(ctx.line_plot_knock, ctx.line_plot_knock_data, 20, 210, 78, 15, GFX_RED);
}

#if defined(__GNUC__)
#define GAUGE_WEAK __attribute__((weak))
#else
#define GAUGE_WEAK
#endif

bool runtime_diag_square_enabled()
{
  return g_diag_square_enabled;
}

uint32_t runtime_diag_fps()
{
  return g_diag_fps;
}

void runtime_diag_set_square_enabled(bool enabled)
{
  g_diag_square_enabled = enabled;
}

void runtime_diag_reset_shared()
{
  g_diag_fps = 0;
  g_diag_fps_frames = 0;
  g_diag_fps_last_ms = HAL_GetTick();
  board_reset_loop_diag();
}

GAUGE_WEAK void board_reset_loop_diag()
{
}

GAUGE_WEAK void board_render_before(const RuntimeState &state, const BoardSharedData &data, SharedRenderCtx &ctx)
{
  (void)state;
  (void)data;
  (void)ctx;
}

GAUGE_WEAK void board_render_after(const RuntimeState &state, const BoardSharedData &data, SharedRenderCtx &ctx)
{
  (void)state;
  (void)data;
  (void)ctx;
}

RuntimeState runtime_state_from_board_data(const BoardSharedData &data)
{
  RuntimeState state = {};
  if (data.rpm.supported)
  {
    state.data_mask |= PLATFORM_DATA_RPM;
    if (data.rpm.good)
      state.rpm = data.rpm.value;
  }
  if (data.speed_mph.supported)
  {
    state.data_mask |= PLATFORM_DATA_SPEED_MPH;
    if (data.speed_mph.good)
      state.speed_mph = data.speed_mph.value;
  }
  if (data.loop_count.supported || data.loop_period_ms.supported || data.worst_loop_period_ms.supported)
  {
    state.data_mask |= PLATFORM_DATA_TIMING_DIAG;
    state.loop_count = data.loop_count.value;
    state.loop_period_ms = data.loop_period_ms.value;
    state.worst_loop_period_ms = data.worst_loop_period_ms.value;
  }
  if (data.acceleration_mps2.supported)
  {
    state.data_mask |= PLATFORM_DATA_GIMBAL;
    if (data.acceleration_mps2.good)
    {
      compute_gimbal_from_board_acceleration(data.acceleration_mps2.value, state.gimbal_x, state.gimbal_y);
    }
  }
  if (data.startup_init_error.supported)
  {
    state.data_mask |= PLATFORM_DATA_STARTUP_ERROR;
    state.startup_init_error = data.startup_init_error.good && data.startup_init_error.value;
  }
  if (data.startup_init_error_code.supported && data.startup_init_error_code.good)
  {
    state.startup_init_error_code = data.startup_init_error_code.value;
  }
  if (data.lamp_on.supported)
  {
    state.data_mask |= PLATFORM_DATA_WARN_LAMP;
    state.warn_lamp_on = data.lamp_on.good && data.lamp_on.value;
  }
  if (data.high_beam.supported)
  {
    state.data_mask |= PLATFORM_DATA_WARN_HIGH_BEAM;
    state.warn_high_beam = data.high_beam.good && data.high_beam.value;
  }
  if (data.ecu_supported)
  {
    state.data_mask |= PLATFORM_DATA_ECU;
    state.ecu = data.ecu;
    state.ecu_param_tps_index = data.ecu_param_tps_index;
    state.ecu_param_wb_index = data.ecu_param_wb_index;
    state.ecu_param_map_index = data.ecu_param_map_index;
    state.ecu_param_knock_index = data.ecu_param_knock_index;
    state.ecu_param_timing_index = data.ecu_param_timing_index;
    state.ecu_param_afr_target_index = data.ecu_param_afr_target_index;
    state.ecu_param_fuel_trim_front_low_index = data.ecu_param_fuel_trim_front_low_index;
    state.ecu_param_fuel_trim_front_med_index = data.ecu_param_fuel_trim_front_med_index;
    state.ecu_param_fuel_trim_front_high_index = data.ecu_param_fuel_trim_front_high_index;
    state.ecu_param_fuel_trim_rear_low_index = data.ecu_param_fuel_trim_rear_low_index;
    state.ecu_param_fuel_trim_rear_med_index = data.ecu_param_fuel_trim_rear_med_index;
    state.ecu_param_fuel_trim_rear_high_index = data.ecu_param_fuel_trim_rear_high_index;
    state.ecu_flasher = data.ecu_flasher;
  }
  if (data.btn.supported)
  {
    state.data_mask |= PLATFORM_DATA_BTN;
    state.btn = data.btn.value;
  }
  return state;
}

int compute_rpm_mode_shared(float rpm, int prev_mode)
{
  const VehicleConfig &vehicle = get_build_config().vehicle;
  if (rpm >= vehicle.rpm_alert_final)
    return 2;

  if (prev_mode > 0)
  {
    if (rpm < vehicle.rpm_alert_reset)
      return 0;
    return 1;
  }

  if (rpm >= vehicle.rpm_alert_init)
    return 1;

  return 0;
}

void render_step_shared(const RuntimeState &state, SharedRenderCtx &ctx, int &draw_step, uint32_t &timer_draw_ms)
{
  static const BoardSharedData empty_data = {};
  render_step_shared(state, empty_data, ctx, draw_step, timer_draw_ms);
}

void render_step_shared(const RuntimeState &state, const BoardSharedData &data, SharedRenderCtx &ctx, int &draw_step, uint32_t &timer_draw_ms)
{
  color_t amber = (ctx.amber_ptr != nullptr) ? *ctx.amber_ptr : GFX_AMBER_YEL;
  bool flush_after_hooks = false;

  if (gui_layout_consume_transition())
  {
    draw_step = 0;
    gdispClear(GFX_BLACK);
  }

  board_render_before(state, data, ctx);

  if (state.rpm_mode >= 2)
  {
    const int SHIFT_SIZE = 100;
    gdispClear(GFX_BLACK);
    gdispFillCircle((ctx.screen_width >> 1), (ctx.screen_height >> 1), SHIFT_SIZE, GFX_RED);
    const uint32_t now = HAL_GetTick();
    record_completed_frame(now);
    draw_step = 0;
    timer_draw_ms = now;
    g_shift_alarm_fast_path_active = true;
    ctx.render_cycle_complete = true;
    board_render_after(state, data, ctx);
    ctx.render_cycle_complete = false;
    gdispFlush();
    return;
  }

  if (g_shift_alarm_fast_path_active)
  {
    draw_step = 0;
    g_shift_alarm_fast_path_active = false;
  }

  const int current_step = draw_step++;
  if (current_step == 0)
  {
    gdispClear(GFX_BLACK);
  }
  else
  {
    gui_layout_render_step(state, data, ctx, current_step);
  }

  if (current_step >= (int)gui_layout_render_step_count() - 1)
  {
    const uint32_t now = HAL_GetTick();
    record_completed_frame(now);
    flush_after_hooks = true;
    draw_step = 0;
    timer_draw_ms = now;
  }

  ctx.render_cycle_complete = flush_after_hooks;
  board_render_after(state, data, ctx);
  ctx.render_cycle_complete = false;

  if (flush_after_hooks)
  {
    render_diag_overlay(state, ctx, amber);
    gdispFlush();
  }
}

namespace
{
struct SharedMainLoopState
{
  BoardSharedData board_data = {};
  SharedRenderCtx render_ctx = {};
  RuntimeState state = {};
  BoardSharedData render_board_data = {};
  RuntimeState render_state = {};
  int draw_step = 0;
  uint32_t timer_draw_ms = 0;
  uint32_t timer_telemetry_ms = 0;
  bool initialized = false;
  bool exit_requested = false;
};

SharedMainLoopState g_main_loop;

void main_loop_init(SharedMainLoopState &loop)
{
  if (loop.initialized)
    return;

  loop.timer_draw_ms = HAL_GetTick();
  loop.timer_telemetry_ms = loop.timer_draw_ms;
  board_init(loop.board_data, loop.render_ctx, loop.draw_step, loop.timer_draw_ms);
  gauge_layouts_register();
  loop.state = runtime_state_from_board_data(loop.board_data);
  render_ctx_init_shared(loop.render_ctx);
  loop.initialized = true;
}

void main_loop_step(SharedMainLoopState &loop)
{
  if (!loop.initialized || loop.exit_requested)
    return;

  board_update();

  const int prev_rpm_mode = loop.state.rpm_mode;
  loop.state = runtime_state_from_board_data(loop.board_data);
  loop.state.rpm_mode = compute_rpm_mode_shared(loop.state.rpm, prev_rpm_mode);
  gauge_layouts_update(loop.state);

  const uint32_t now_ms = HAL_GetTick();
  if ((now_ms - loop.timer_telemetry_ms) >= get_print_interval_ms())
  {
    loop.timer_telemetry_ms = now_ms;
    (void)telemetry_publish_board_data(loop.board_data, now_ms);
  }

  if (board_check_exit())
  {
    loop.exit_requested = true;
    return;
  }

  if (((HAL_GetTick() - loop.timer_draw_ms) >= get_draw_interval_ms()) && board_display_ready())
  {
    if (loop.draw_step == 0 || loop.state.rpm_mode >= 2)
    {
      loop.render_board_data = loop.board_data;
      loop.render_state = loop.state;
    }
    render_step_shared(loop.render_state, loop.render_board_data, loop.render_ctx, loop.draw_step, loop.timer_draw_ms);
    loop.board_data.mark_all_read();
  }
}

void main_loop_shutdown(SharedMainLoopState &loop)
{
  if (!loop.initialized)
    return;

  board_shutdown();
  loop.initialized = false;
}
}

extern "C" void main_cpp()
{
  main_loop_init(g_main_loop);

#if !defined(GAUGE_HOST_BACKEND_EMSCRIPTEN)
  while (!g_main_loop.exit_requested)
  {
    main_loop_step(g_main_loop);
  }

  main_loop_shutdown(g_main_loop);
#endif
}

extern "C" void main_cpp_step()
{
  main_loop_step(g_main_loop);
}

extern "C" void main_cpp_shutdown()
{
  main_loop_shutdown(g_main_loop);
}
