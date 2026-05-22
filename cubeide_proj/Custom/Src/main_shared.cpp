#include <cstdio>

#include "main_shared.h"
#include "gfx.h"
#include "ugfx_widgets.h"
#include "../ECUK-lib/ECUK.hpp"

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

int compute_rpm_mode_shared(float rpm, int prev_mode)
{
  if (rpm < RPM_ALERT_INIT)
    return 0;
  if (prev_mode == 0 && rpm >= RPM_ALERT_INIT && rpm < RPM_ALERT_FINAL)
    return 1;
  if (rpm >= RPM_ALERT_FINAL)
    return 2;
  return prev_mode;
}

static void render_ecu_section(const RuntimeState &state, font_t fontLCD, font_t font20, color_t amber)
{
  if (state.ecu == nullptr)
    return;

  ECUK::ecuParam_t *map_p = state.ecu->getParam(state.ecu_param_map_index);
  ECUK::ecuParam_t *wb_p = state.ecu->getParam(state.ecu_param_wb_index);
  if (map_p == nullptr || wb_p == nullptr)
    return;

  char map_text[16];
  char wb_text[16];
  (void)std::snprintf(map_text, sizeof(map_text), "MAP %2.1f", map_p->val);
  (void)std::snprintf(wb_text, sizeof(wb_text), "WB %2.1f", wb_p->val);
  gdispFillString(20, 52, map_text, fontLCD, amber, GFX_BLACK);
  gdispFillString(20, 7, wb_text, fontLCD, amber, GFX_BLACK);

  bool show_error = !state.ecu->isConnected();
  if (state.ecu_flasher != nullptr)
    show_error = flasher_fun(state.ecu_flasher);
  if (!state.ecu->isConnected() && show_error)
    gdispFillString(20, 80, "ECU ERR      ", font20, GFX_RED, GFX_BLACK);
}

static void render_speed_rpm_section(const RuntimeState &state, font_t fontLCD, color_t amber)
{
  char logBuf[32];
  (void)std::snprintf(logBuf, sizeof(logBuf), "%d", (int)state.speed_mph);
  gdispFillString(15, 110 + 42, logBuf, fontLCD, amber, GFX_BLACK);

  (void)std::snprintf(logBuf, sizeof(logBuf), "%d", (int)state.rpm);
  gdispFillString(15, 155 + 42, logBuf, fontLCD, amber, GFX_BLACK);
}

void render_step_shared(const RuntimeState &state, SharedRenderCtx &ctx, int &draw_step, uint32_t &timer_draw_ms)
{
  color_t amber = (ctx.amber_ptr != nullptr) ? *ctx.amber_ptr : GFX_AMBER_YEL;

  switch (draw_step++)
  {
    case 0:
      gdispClear(GFX_BLACK);
      break;

    case 1:
      if (ctx.gimball != nullptr)
        drawGimball(ctx.gimball, 168 + 10, 48, 45, state.gimbal_x, state.gimbal_y);
      break;

    case 2:
    {
      render_ecu_section(state, ctx.fontLCD, ctx.font20, amber);
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
      render_speed_rpm_section(state, ctx.fontLCD, amber);
      break;

    case 5:
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

    case 6:
    {
      if (state.startup_init_error)
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

      if (state.warn_batt && ctx.batt_img != nullptr)
        gdispImageDraw(ctx.batt_img, 140, 200, ctx.batt_img->width, ctx.batt_img->height, 0, 0);
      if (state.warn_brake)
        gdispFillString(120, 233, "BRAKE", ctx.font20, GFX_RED, GFX_BLACK);
      if (state.warn_4ws)
        gdispFillString(175, 205, "4WS", ctx.font20, GFX_YELLOW, GFX_BLACK);

      if (state.warn_lamp_on)
      {
        if (ctx.amber_ptr != nullptr)
          *ctx.amber_ptr = GFX_AMBER_SAE;
        setColors(GFX_AMBER_SAE, GFX_RED, GFX_BLACK);
        if (state.warn_high_beam && ctx.beam_img != nullptr)
          gdispImageDraw(ctx.beam_img, 190, 223, ctx.beam_img->width, ctx.beam_img->height, 0, 0);
      }
      else
      {
        if (ctx.amber_ptr != nullptr)
          *ctx.amber_ptr = GFX_AMBER_YEL;
        setColors(GFX_AMBER_YEL, GFX_RED, GFX_BLACK);
      }
      break;
    }

    case 7:
    {
      const int WARN_SIZE = 20;
      const int WARN_FINAL_SIZE = 70;
      const int SHIFT_SIZE = 100;
      const int range = RPM_ALERT_FINAL - RPM_ALERT_INIT;
      int over = (int)state.rpm - RPM_ALERT_INIT;
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

extern "C" void main_cpp()
{
  SharedRenderCtx render_ctx = {};
  RuntimeState state = {};
  int draw_step = 0;
  uint32_t timer_draw_ms = HAL_GetTick();
  bool loop_exit = false;

  platform_main_init(render_ctx, state, draw_step, timer_draw_ms);
  render_ctx_init_shared(render_ctx);
  run_shared_main_loop(loop_exit, [&]() {
    bool render_requested = false;
    platform_main_step(state, loop_exit, render_requested, timer_draw_ms);
    if (!loop_exit && render_requested)
      render_step_shared(state, render_ctx, draw_step, timer_draw_ms);
  });
  platform_main_shutdown();
}
