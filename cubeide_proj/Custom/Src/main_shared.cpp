#include <cmath>
#include <cstdio>

#include "main_shared.h"
#include "build_config.hpp"
#include "platform_services.hpp"
#include "telemetry.hpp"
#include "gfx.h"
#include "ugfx_widgets.h"
#include "../ECUK-lib/ECUK.hpp"
#if defined(GAUGE_HOST_BACKEND_EMSCRIPTEN)
extern "C" void sdl_driver_poll(void);
#endif

void render_ctx_init_shared(SharedRenderCtx &ctx)
{
  if (ctx.line_plot_tps != nullptr && ctx.line_plot_tps->isInit == false)
    linePlotInit(ctx.line_plot_tps, ctx.line_plot_tps_data, 20, 200, 50, 100, 0);

  if (ctx.line_plot_knock != nullptr && ctx.line_plot_knock->isInit == false)
    linePlotInit(ctx.line_plot_knock, ctx.line_plot_knock_data, 20, 200, 50, 15, GFX_RED);
}

RuntimeState runtime_state_from_sample(const PlatformSample &sample)
{
  RuntimeState state = {};
  state.data_mask = sample.data_mask;
  state.rpm = sample.rpm;
  state.speed_mph = sample.speed_mph;
  state.elapsed_ms = sample.elapsed_ms;
  state.loop_count = sample.loop_count;
  state.loop_period_ms = sample.loop_period_ms;
  state.worst_loop_period_ms = sample.worst_loop_period_ms;
  state.gimbal_x = sample.gimbal_x;
  state.gimbal_y = sample.gimbal_y;
  state.startup_init_error = sample.startup_init_error;
  state.warn_batt = sample.warn_batt;
  state.warn_brake = sample.warn_brake;
  state.warn_4ws = sample.warn_4ws;
  state.warn_lamp_on = sample.warn_lamp_on;
  state.warn_high_beam = sample.warn_high_beam;
  state.ecu = sample.ecu;
  state.ecu_param_tps_index = sample.ecu_param_tps_index;
  state.ecu_param_wb_index = sample.ecu_param_wb_index;
  state.ecu_param_map_index = sample.ecu_param_map_index;
  state.ecu_param_knock_index = sample.ecu_param_knock_index;
  state.ecu_flasher = sample.ecu_flasher;
  state.btn = sample.btn;
  return state;
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
      constexpr float pitch_rad = 75.0f * 3.14159265358979323846f / 180.0f;
      constexpr float gimbal_scale = 5.0f;
      const float cos_pitch = std::cos(pitch_rad);
      const float sin_pitch = std::sin(pitch_rad);
      const BoardAccelerationVector &accel = data.acceleration_mps2.value;
      const float pitch_corrected_y = (accel.y_mps2 * cos_pitch) - (accel.z_mps2 * sin_pitch);
      state.gimbal_x = (int)(accel.x_mps2 * gimbal_scale);
      state.gimbal_y = (int)(pitch_corrected_y * gimbal_scale);
    }
  }
  if (data.startup_init_error.supported)
  {
    state.data_mask |= PLATFORM_DATA_STARTUP_ERROR;
    state.startup_init_error = data.startup_init_error.good && data.startup_init_error.value;
  }
  if (data.lamp_on.supported)
  {
    state.data_mask |= PLATFORM_DATA_WARN_LAMP;
    state.warn_lamp_on = data.lamp_on.good && data.lamp_on.value;
  }
  if (data.ecu_supported)
  {
    state.data_mask |= PLATFORM_DATA_ECU;
    state.ecu = data.ecu;
    state.ecu_param_tps_index = data.ecu_param_tps_index;
    state.ecu_param_wb_index = data.ecu_param_wb_index;
    state.ecu_param_map_index = data.ecu_param_map_index;
    state.ecu_param_knock_index = data.ecu_param_knock_index;
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

static void render_ecu_section(const RuntimeState &state, font_t fontValue, font_t font20, color_t amber)
{
  if (!platform_state_has(state.data_mask, PLATFORM_DATA_ECU) || state.ecu == nullptr)
    return;

  ECUK::ecuParam_t *map_p = state.ecu->getParam(state.ecu_param_map_index);
  ECUK::ecuParam_t *wb_p = state.ecu->getParam(state.ecu_param_wb_index);
  if (map_p == nullptr || wb_p == nullptr)
    return;

  char map_text[16];
  char wb_text[16];
  (void)std::snprintf(map_text, sizeof(map_text), "%2.1f", map_p->val);
  (void)std::snprintf(wb_text, sizeof(wb_text), "%2.1f", wb_p->val);
  gdispFillString(20, 20, "WB", font20, amber, GFX_BLACK);
  gdispFillString(74, 7, wb_text, fontValue, amber, GFX_BLACK);
  gdispFillString(20, 65, "MAP", font20, amber, GFX_BLACK);
  gdispFillString(74, 52, map_text, fontValue, amber, GFX_BLACK);

  bool show_error = !state.ecu->isConnected();
  if (state.ecu_flasher != nullptr)
    show_error = flasher_fun(state.ecu_flasher);
  if (!state.ecu->isConnected() && show_error)
    gdispFillString(20, 80, "ECU ERR      ", font20, GFX_RED, GFX_BLACK);
}

void render_step_shared(const RuntimeState &state, SharedRenderCtx &ctx, int &draw_step, uint32_t &timer_draw_ms)
{
  static const BoardSharedData empty_data = {};
  render_step_shared(state, empty_data, ctx, draw_step, timer_draw_ms);
}

void render_step_shared(const RuntimeState &state, const BoardSharedData &data, SharedRenderCtx &ctx, int &draw_step, uint32_t &timer_draw_ms)
{
  color_t amber = (ctx.amber_ptr != nullptr) ? *ctx.amber_ptr : GFX_AMBER_YEL;

  switch (draw_step++)
  {
    case 0:
      gdispClear(GFX_BLACK);
      break;

    case 1:
      if (ctx.gimball != nullptr && platform_state_has(state.data_mask, PLATFORM_DATA_GIMBAL))
        drawGimball(ctx.gimball, 78, 205, 45, state.gimbal_x, state.gimbal_y);
      break;

    case 2:
    {
      render_ecu_section(state, ctx.fontValue, ctx.font20, amber);
      break;
    }

    case 3:
    {
      static char tmpString[4] = {'N', 0, 0, 0};
      switch (state.btn)
      {
        case BTN_OK: tmpString[0] = 'O'; break;
        case BTN_U:  tmpString[0] = 'U'; break;
        case BTN_D:  tmpString[0] = 'D'; break;
        case BTN_L:  tmpString[0] = 'L'; break;
        case BTN_R:  tmpString[0] = 'R'; break;
        default: break;
      }
      gdispFillString(210, 74, tmpString, ctx.font20, amber, GFX_BLACK);
      break;
    }

    case 4:
    {
      ECUK::ecuParam_t *tps_p = nullptr;
      ECUK::ecuParam_t *knock_p = nullptr;
      if (state.ecu != nullptr)
      {
        tps_p = state.ecu->getParam(state.ecu_param_tps_index);
        knock_p = state.ecu->getParam(state.ecu_param_knock_index);
      }

      if (ctx.line_plot_tps != nullptr && tps_p != nullptr && tps_p->isNew)
      {
        linePlotPush(ctx.line_plot_tps, (int)tps_p->val);
        tps_p->isNew = false;
      }
      if (ctx.line_plot_tps != nullptr)
        linePlot(10, 149, ctx.line_plot_tps);

      if (ctx.line_plot_knock != nullptr && knock_p != nullptr && knock_p->isNew)
      {
        linePlotPush(ctx.line_plot_knock, (int)knock_p->val);
        knock_p->isNew = false;
      }
      if (ctx.line_plot_knock != nullptr)
        linePlot(10, 149, ctx.line_plot_knock);
      break;
    }

    case 5:
    {
      if (platform_state_has(state.data_mask, PLATFORM_DATA_STARTUP_ERROR) && state.startup_init_error)
        gdispFillString((ctx.screen_width >> 1) - 50, (ctx.screen_height >> 1), "ERR", ctx.fontLCD, GFX_RED, GFX_BLACK);

#ifdef DIAG_SQUARE
      {
        char logBuf[32];
        static const int xdiag = 30, ydiag = 192;
        gdispFillArea(xdiag - 1, ydiag - 1, 70, 70, GFX_BLACK);
        gdispDrawBox(xdiag - 1, ydiag - 1, 70, 70, GFX_AMBER);
        (void)std::snprintf(logBuf, sizeof(logBuf), "%lu", (unsigned long)state.loop_period_ms);
        gdispFillString(xdiag, ydiag + 0, logBuf, ctx.font10, GFX_AMBER, GFX_BLACK);
        (void)std::snprintf(logBuf, sizeof(logBuf), "%lu", (unsigned long)state.worst_loop_period_ms);
        gdispFillString(xdiag, ydiag + 10, logBuf, ctx.font10, GFX_AMBER, GFX_BLACK);
        (void)std::snprintf(logBuf, sizeof(logBuf), "%lu", (unsigned long)state.loop_count);
        gdispFillString(xdiag, ydiag + 20, logBuf, ctx.font10, GFX_AMBER, GFX_BLACK);
      }
#endif

      for (std::size_t i = 0; i < data.warning_light_count; ++i)
      {
        const BoardWarningLight &warning = data.warning_lights[i];
        if (!warning.supported || !warning.good || !warning.active)
          continue;

        if (warning.style == BOARD_WARNING_STYLE_IMAGE && warning.image != nullptr)
          gdispImageDraw(warning.image, warning.x, warning.y, warning.image->width, warning.image->height, 0, 0);
        else if (warning.label != nullptr)
          gdispFillString(warning.x, warning.y, warning.label, ctx.font20, warning.color, GFX_BLACK);
      }

      if (platform_state_has(state.data_mask, PLATFORM_DATA_WARN_LAMP) && state.warn_lamp_on)
      {
        if (ctx.amber_ptr != nullptr)
          *ctx.amber_ptr = GFX_AMBER_SAE;
        setColors(GFX_AMBER_SAE, GFX_RED, GFX_BLACK);
      }
      else
      {
        if (ctx.amber_ptr != nullptr)
          *ctx.amber_ptr = GFX_AMBER_YEL;
        setColors(GFX_AMBER_YEL, GFX_RED, GFX_BLACK);
      }
      break;
    }

    case 6:
    {
      const int WARN_SIZE = 20;
      const int WARN_FINAL_SIZE = 70;
      const int SHIFT_SIZE = 100;
      const VehicleConfig &vehicle = get_build_config().vehicle;
      const int range = vehicle.rpm_alert_final - vehicle.rpm_alert_init;
      int over = (int)state.rpm - vehicle.rpm_alert_init;
      int percent = (64 * over) / (range > 0 ? range : 1);
      int current_warn_size = WARN_SIZE + (((WARN_FINAL_SIZE - WARN_SIZE) * percent) >> 6);

      if (state.rpm_mode >= 2)
      {
        gdispFillCircle((ctx.screen_width >> 1), (ctx.screen_height >> 1), SHIFT_SIZE, GFX_RED);
      }
      else if (state.rpm_mode >= 1 && current_warn_size > 0)
      {
        gdispFillDualCircle((ctx.screen_width >> 1), (ctx.screen_height >> 1), WARN_FINAL_SIZE, GFX_BLACK, WARN_FINAL_SIZE, GFX_GREEN);
        gdispFillCircle((ctx.screen_width >> 1), (ctx.screen_height >> 1), current_warn_size, GFX_YELLOW);
      }
      break;
    }

    default:
      gdispFlush();
      draw_step = 0;
      timer_draw_ms = HAL_GetTick();
      break;
  }
}

namespace
{
struct SharedMainLoopState
{
  BoardSharedData board_data = {};
  SharedRenderCtx render_ctx = {};
  RuntimeState state = {};
  int draw_step = 0;
  uint32_t timer_draw_ms = 0;
  bool initialized = false;
  bool exit_requested = false;
};

SharedMainLoopState g_main_loop;

void main_loop_init(SharedMainLoopState &loop)
{
  if (loop.initialized)
    return;

  loop.timer_draw_ms = HAL_GetTick();
  board_init(loop.board_data, loop.render_ctx, loop.draw_step, loop.timer_draw_ms);
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
  (void)telemetry_publish_board_data(loop.board_data, HAL_GetTick());

  if (board_check_exit())
  {
    loop.exit_requested = true;
    return;
  }

  if (((HAL_GetTick() - loop.timer_draw_ms) >= get_draw_interval_ms()) && board_display_ready())
  {
    render_step_shared(loop.state, loop.board_data, loop.render_ctx, loop.draw_step, loop.timer_draw_ms);
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
#if defined(GAUGE_HOST_BACKEND_EMSCRIPTEN)
  sdl_driver_poll();
#endif
}

extern "C" void main_cpp_shutdown()
{
  main_loop_shutdown(g_main_loop);
}
