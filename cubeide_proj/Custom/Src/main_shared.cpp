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
    linePlotInit(ctx.line_plot_tps, ctx.line_plot_tps_data, 20, 135, 42, 100, 0);

  if (ctx.line_plot_knock != nullptr && ctx.line_plot_knock->isInit == false)
    linePlotInit(ctx.line_plot_knock, ctx.line_plot_knock_data, 20, 135, 42, 15, GFX_RED);
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

static void render_high_beam_telltale(const RuntimeState &state, SharedRenderCtx &ctx)
{
  if (!platform_state_has(state.data_mask, PLATFORM_DATA_WARN_HIGH_BEAM) || !state.warn_high_beam || ctx.beam_img == nullptr)
    return;

  const coord_t image_x = (coord_t)(190);
  const coord_t image_y = (coord_t)(170);
  gdispImageDraw(ctx.beam_img, image_x, image_y, ctx.beam_img->width, ctx.beam_img->height, 0, 0);
}

static bool ecu_param_is_fresh(const ECUK::ecuParam_t *param, uint32_t now_ms)
{
  return param != nullptr && (now_ms - param->lastTime_ms) <= 1000U;
}

static void render_ecu_section(const RuntimeState &state, font_t fontValue, font_t font20, color_t amber)
{
  if (!platform_state_has(state.data_mask, PLATFORM_DATA_ECU) || state.ecu == nullptr)
    return;

  ECUK::ecuParam_t *wb_p = state.ecu->getParam(state.ecu_param_wb_index);
  if (wb_p == nullptr)
    return;

  static const UgfxMeterBand wb_bands[] = {
      {10.0f, 12.0f, GFX_GREEN},
      {12.0f, 15.0f, GFX_AMBER_YEL},
      {15.0f, 20.0f, GFX_RED},
  };
  static UgfxTextBarMeter wb_meter;

  wb_meter.setBounds(24, 8, 192, 62);
  wb_meter.setColors(amber, GFX_RED, GFX_BLACK);
  wb_meter.configure("O2", "AFR", 10.0f, 16.0f, 1, font20, fontValue);
  wb_meter.setBands(wb_bands, sizeof(wb_bands) / sizeof(wb_bands[0]));
  wb_meter.setMode(UGFX_TEXT_BAR_METER_SEGMENT);
  wb_meter.setBarHeight(18);
  wb_meter.setSegmentSize(16);

#if 1
  // MAP/PSI is intentionally left out of the LCD UI for FPS; the car has a physical PSI gauge.
  static const UgfxMeterBand map_bands[] = {
      {-20.0f, 0.0f, GFX_AMBER_YEL},
      {0.0f, 15.0f, GFX_GREEN},
      {15.0f, 20.0f, GFX_RED},
  };
  static UgfxTextBarMeter map_meter;
  ECUK::ecuParam_t *map_p = state.ecu->getParam(state.ecu_param_map_index);
  map_meter.setBounds(24, 93, 192, 62);
  map_meter.setColors(amber, GFX_RED, GFX_BLACK);
  map_meter.configure("MAP", "PSI", -15.0f, 20.0f, 1, font20, fontValue);
  map_meter.setBands(map_bands, sizeof(map_bands) / sizeof(map_bands[0]));
  map_meter.setMode(UGFX_TEXT_BAR_METER_BIPOLAR);
  map_meter.setReferenceValue(0.0f);
  map_meter.setBarHeight(16);
#endif

  const uint32_t now_ms = HAL_GetTick();
  const bool ecu_connected = state.ecu->isConnected();
  wb_meter.setValue(wb_p->val, ecu_connected && ecu_param_is_fresh(wb_p, now_ms));
  wb_meter.draw();
#if 1
  if (map_p != nullptr)
  {
    map_meter.setValue(map_p->val, ecu_connected && ecu_param_is_fresh(map_p, now_ms));
    map_meter.draw();
  }
#endif

  bool show_error = !ecu_connected;
  if (state.ecu_flasher != nullptr)
    show_error = flasher_fun(state.ecu_flasher);
  if (!ecu_connected && show_error)
  {
    const coord_t error_x = 44;
    const coord_t error_y = 52;
    const coord_t error_w = 152;
    const coord_t error_h = 62;
    gdispFillArea(error_x, error_y, error_w, error_h, GFX_BLACK);
    gdispDrawBox(error_x, error_y, error_w, error_h, GFX_AMBER_YEL);
    gdispFillStringBox(error_x + 4,
                       error_y + 4,
                       error_w - 8,
                       error_h - 8,
                       "ECU ERR",
                       font20,
                       GFX_RED,
                       GFX_BLACK,
                       (gJustify)(gJustifyCenter | gJustifyNoWordWrap));
  }
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

  switch (draw_step++)
  {
    case 0:
      gdispClear(GFX_BLACK);
      break;

    case 1:
    {
      render_ecu_section(state, ctx.fontValue, ctx.font20, amber);
      break;
    }

    case 2:
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
        linePlot(77, 240, ctx.line_plot_tps);

      if (ctx.line_plot_knock != nullptr && knock_p != nullptr && knock_p->isNew)
      {
        linePlotPush(ctx.line_plot_knock, (int)knock_p->val);
        knock_p->isNew = false;
      }
      if (ctx.line_plot_knock != nullptr)
        linePlot(77, 240, ctx.line_plot_knock);
      break;
    }

    case 3:
    {
      if (platform_state_has(state.data_mask, PLATFORM_DATA_STARTUP_ERROR) && state.startup_init_error)
      {
        char err_string[16];
        (void)std::snprintf(err_string, sizeof(err_string), "ERR %02lX", (unsigned long)(state.startup_init_error_code & 0xFFU));
        gdispFillString((ctx.screen_width >> 1) - 72, (ctx.screen_height >> 1), err_string, ctx.fontLCD, GFX_RED, GFX_BLACK);
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

    case 4:
    {
      if (ctx.gimball != nullptr && platform_state_has(state.data_mask, PLATFORM_DATA_GIMBAL))
        drawGimball(ctx.gimball, 50, 210, 34, state.gimbal_x, state.gimbal_y);
      break;
    }

    case 5:
    {
      render_high_beam_telltale(state, ctx);

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

      const uint32_t now = HAL_GetTick();
      record_completed_frame(now);
      flush_after_hooks = true;
      draw_step = 0;
      timer_draw_ms = now;
      break;
    }

    default:
      draw_step = 0;
      break;
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
