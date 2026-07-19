#include <cmath>
#include <cstdio>

#include "main_shared.h"
#include "build_config.hpp"
#include "platform_services.hpp"
#include "telemetry.hpp"
#include "gfx.h"
#include "ugfx_widgets.h"
#include "../ECUK-lib/ECUK.hpp"
#include "../Quaternion/Quaternion.hpp"
namespace
{
constexpr float kGimbalScale = 5.0f;

void compute_gimbal_from_board_acceleration(const BoardAccelerationVector &accel, int &gimbal_x, int &gimbal_y)
{
  const float pitch_rad = get_build_config().vehicle.board_mount_pitch_deg * 3.14159265358979323846f / 180.0f;
  const float cos_pitch = std::cos(pitch_rad);
  const float sin_pitch = std::sin(pitch_rad);
  float rotated_accel[3] = {accel.x_mps2, accel.y_mps2, accel.z_mps2};

  // Restore the original gimbal path: rotate the mounted board frame by the
  // known pitch, then map rotated Y/X into widget X/Y.
  rotateVectorKnownPitch(rotated_accel, cos_pitch, sin_pitch);
  gimbal_x = (int)(-rotated_accel[1] * kGimbalScale);
  gimbal_y = (int)(-rotated_accel[0] * kGimbalScale);
}
}

void render_ctx_init_shared(SharedRenderCtx &ctx)
{
  if (ctx.line_plot_tps != nullptr && ctx.line_plot_tps->isInit == false)
    linePlotInit(ctx.line_plot_tps, ctx.line_plot_tps_data, 20, 130, 42, 100, 0);

  if (ctx.line_plot_knock != nullptr && ctx.line_plot_knock->isInit == false)
    linePlotInit(ctx.line_plot_knock, ctx.line_plot_knock_data, 20, 130, 42, 15, GFX_RED);
}

#if defined(__GNUC__)
#define GAUGE_WEAK __attribute__((weak))
#else
#define GAUGE_WEAK
#endif

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

static bool ecu_param_is_fresh(const ECUK::ecuParam_t *param, uint32_t now_ms)
{
  return param != nullptr && (now_ms - param->lastTime_ms) <= 1000U;
}

static void render_ecu_section(const RuntimeState &state, font_t fontValue, font_t font20, color_t amber)
{
  if (!platform_state_has(state.data_mask, PLATFORM_DATA_ECU) || state.ecu == nullptr)
    return;

  ECUK::ecuParam_t *map_p = state.ecu->getParam(state.ecu_param_map_index);
  ECUK::ecuParam_t *wb_p = state.ecu->getParam(state.ecu_param_wb_index);
  if (map_p == nullptr || wb_p == nullptr)
    return;

  static const UgfxMeterBand wb_bands[] = {
      {10.0f, 12.0f, GFX_GREEN},
      {12.0f, 15.0f, GFX_AMBER_YEL},
      {15.0f, 20.0f, GFX_RED},
  };
  static const UgfxMeterBand map_bands[] = {
      {-20.0f, 0.0f, GFX_AMBER_YEL},
      {0.0f, 15.0f, GFX_GREEN},
      {15.0f, 20.0f, GFX_RED},
  };
  static UgfxTextBarMeter wb_meter;
  static UgfxTextBarMeter map_meter;

  wb_meter.setBounds(24, 85, 192, 62);
  wb_meter.setColors(amber, GFX_RED, GFX_BLACK);
  wb_meter.configure("O2", "AFR", 10.0f, 16.0f, 1, font20, fontValue);
  wb_meter.setBands(wb_bands, sizeof(wb_bands) / sizeof(wb_bands[0]));
  wb_meter.setMode(UGFX_TEXT_BAR_METER_SEGMENT);
  wb_meter.setBarHeight(18);
  wb_meter.setSegmentSize(16);

  map_meter.setBounds(24, 8, 192, 62);
  map_meter.setColors(amber, GFX_RED, GFX_BLACK);
  map_meter.configure("MAP", "PSI", -15.0f, 20.0f, 1, font20, fontValue);
  map_meter.setBands(map_bands, sizeof(map_bands) / sizeof(map_bands[0]));
  map_meter.setMode(UGFX_TEXT_BAR_METER_BIPOLAR);
  map_meter.setReferenceValue(0.0f);
  map_meter.setBarHeight(16);

  const uint32_t now_ms = HAL_GetTick();
  const bool ecu_connected = state.ecu->isConnected();
  wb_meter.setValue(wb_p->val, ecu_connected && ecu_param_is_fresh(wb_p, now_ms));
  map_meter.setValue(map_p->val, ecu_connected && ecu_param_is_fresh(map_p, now_ms));
  wb_meter.draw();
  map_meter.draw();

  bool show_error = !ecu_connected;
  if (state.ecu_flasher != nullptr)
    show_error = flasher_fun(state.ecu_flasher);
  if (!ecu_connected && show_error)
    gdispFillString(20, 144, "ECU ERR      ", font20, GFX_RED, GFX_BLACK);
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

  board_render_before(state, data, ctx);

  switch (draw_step++)
  {
    case 0:
      gdispClear(GFX_BLACK);
      break;

    case 1:
      if (ctx.gimball != nullptr && platform_state_has(state.data_mask, PLATFORM_DATA_GIMBAL))
        drawGimball(ctx.gimball, 56, 203, 34, state.gimbal_x, state.gimbal_y);
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
      gdispFillString(205, 120, tmpString, ctx.font20, amber, GFX_BLACK);
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
        linePlot(80, 236, ctx.line_plot_tps);

      if (ctx.line_plot_knock != nullptr && knock_p != nullptr && knock_p->isNew)
      {
        linePlotPush(ctx.line_plot_knock, (int)knock_p->val);
        knock_p->isNew = false;
      }
      if (ctx.line_plot_knock != nullptr)
        linePlot(80, 236, ctx.line_plot_knock);
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

      data.warning_panel.render(ctx);
      break;
    }

    default:
      flush_after_hooks = true;
      draw_step = 0;
      timer_draw_ms = HAL_GetTick();
      break;
  }

  ctx.render_cycle_complete = flush_after_hooks;
  board_render_after(state, data, ctx);
  ctx.render_cycle_complete = false;

  if (flush_after_hooks)
    gdispFlush();
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
}

extern "C" void main_cpp_shutdown()
{
  main_loop_shutdown(g_main_loop);
}
