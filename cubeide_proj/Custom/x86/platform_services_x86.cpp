#ifndef __x86_64__
#error "THIS PLATFORM IS ONLY FOR X86"
#endif

#include <cstdint>
#include <cstdio>
#include <cmath>
#include "../../res/batt.c"
#include "../../res/beam.c"

#include "main_shared.h"
#include "build_config.hpp"
#include "platform_services.hpp"
#include "gfx.h"
#include "ugfx_widgets.h"
#include "app_model.hpp"
#include "sim_ecuk.hpp"

struct HostPlatformCtx
{
  color_t amber;
  font_t font20;
  font_t font10;
  font_t fontLCD;
  coord_t cx;
  coord_t cy;
  uint32_t t0_ms;
  uint32_t timer_draw_ms;
  gImage batt_img;
  gImage beam_img;
  Gimball_t gimball;
  LinePlot_t line_plot_tps;
  LinePlot_t line_plot_knock;
  int tps_plot_data[20];
  int knock_plot_data[20];
};

static HostPlatformCtx g_host_ctx;
static SharedRenderCtx g_host_render_ctx;
static uint32_t g_loop_count = 0;
static uint32_t g_loop_period_ms = 0;
static uint32_t g_worst_loop_period_ms = 0;
static uint32_t g_loop_last_tick_ms = 0;
static CoreSignals g_signals;
static SimECUK g_sim_ecu;
static ECUK *g_ecu = &g_sim_ecu;
static BoardSharedData *g_board_data = nullptr;
static BoardWarningLight *g_warn_batt = nullptr;
static BoardWarningLight *g_warn_brake = nullptr;
static BoardWarningLight *g_warn_4ws = nullptr;
static BoardWarningLight *g_warn_high_beam = nullptr;

static void x86_bringup_hardware()
{
  gfxInit();
  gdispClear(GFX_BLACK);

  g_host_ctx.amber = HTML2COLOR(0xFFB000);
  g_host_ctx.font20 = gdispOpenFont("DejaVuSans20");
  g_host_ctx.font10 = gdispOpenFont("DejaVuSans10");
  g_host_ctx.fontLCD = gdispOpenFont("lcddot_tr80");
  g_host_ctx.cx = gdispGetWidth() / 2;
  g_host_ctx.cy = gdispGetHeight() / 2;
  g_host_ctx.t0_ms = HAL_GetTick();
  g_host_ctx.timer_draw_ms = g_host_ctx.t0_ms;
  g_loop_last_tick_ms = g_host_ctx.t0_ms;

  gdispImageOpenMemory(&g_host_ctx.batt_img, batt);
  gdispImageOpenMemory(&g_host_ctx.beam_img, beam);
  g_host_render_ctx = {
    .amber_ptr = &g_host_ctx.amber,
    .font10 = g_host_ctx.font10,
    .font20 = g_host_ctx.font20,
    .fontLCD = g_host_ctx.fontLCD,
    .screen_width = (coord_t)gdispGetWidth(),
    .screen_height = (coord_t)gdispGetHeight(),
    .batt_img = &g_host_ctx.batt_img,
    .beam_img = &g_host_ctx.beam_img,
    .gimball = &g_host_ctx.gimball,
    .line_plot_tps = &g_host_ctx.line_plot_tps,
    .line_plot_tps_data = g_host_ctx.tps_plot_data,
    .line_plot_knock = &g_host_ctx.line_plot_knock,
    .line_plot_knock_data = g_host_ctx.knock_plot_data,
  };
}

static PlatformSample x86_collect_platform_sample()
{
  PlatformSample sample = {};
  sample.data_mask =
      PLATFORM_DATA_RPM |
      PLATFORM_DATA_SPEED_MPH |
      PLATFORM_DATA_TIMING_DIAG |
      PLATFORM_DATA_GIMBAL |
      PLATFORM_DATA_STARTUP_ERROR |
      PLATFORM_DATA_WARN_BATT |
      PLATFORM_DATA_WARN_BRAKE |
      PLATFORM_DATA_WARN_4WS |
      PLATFORM_DATA_WARN_LAMP |
      PLATFORM_DATA_WARN_HIGH_BEAM |
      PLATFORM_DATA_ECU |
      PLATFORM_DATA_BTN;
  const uint32_t now = HAL_GetTick();
  sample.elapsed_ms = now - g_host_ctx.t0_ms;
  g_loop_period_ms = now - g_loop_last_tick_ms;
  g_loop_last_tick_ms = now;
  if (g_loop_period_ms > g_worst_loop_period_ms)
    g_worst_loop_period_ms = g_loop_period_ms;
  sample.loop_count = g_loop_count++;
  sample.loop_period_ms = g_loop_period_ms;
  sample.worst_loop_period_ms = g_worst_loop_period_ms;
  const float t = sample.elapsed_ms / 1000.0f;

  g_signals.rpm.value = 900.0f + 3000.0f * (0.5f + 0.5f * std::sinf(t * 1.2f));
  g_signals.rpm.fresh = true;
  g_signals.rpm.valid = true;
  g_signals.rpm.timestamp_ms = now;

  g_signals.speed_mph.value = g_signals.rpm.value / (9000.0f / 180.0f);
  g_signals.speed_mph.fresh = true;
  g_signals.speed_mph.valid = true;
  g_signals.speed_mph.timestamp_ms = now;

  g_signals.board_batt_v.value = 13.5f + 0.2f * std::sinf(t * 0.2f);
  g_signals.board_batt_v.fresh = true;
  g_signals.board_batt_v.valid = true;
  g_signals.board_batt_v.timestamp_ms = now;

  g_sim_ecu.simulate(now);

  sample.rpm = g_signals.rpm.value;
  sample.speed_mph = g_signals.speed_mph.value;
  sample.ecu = g_ecu;
  sample.ecu_param_tps_index = SimECUK::PARAM_TPS;
  sample.ecu_param_wb_index = SimECUK::PARAM_WB;
  sample.ecu_param_map_index = SimECUK::PARAM_MAP;
  sample.ecu_param_knock_index = SimECUK::PARAM_KNOCK;
  sample.ecu_flasher = nullptr;
  sample.startup_init_error = false;

  sample.btn = BTN_INV;
  if (((int)t % 9) == 1) sample.btn = BTN_U;
  if (((int)t % 9) == 3) sample.btn = BTN_D;
  if (((int)t % 9) == 5) sample.btn = BTN_L;
  if (((int)t % 9) == 7) sample.btn = BTN_R;

  sample.warn_4ws = true;
  sample.warn_lamp_on = (((int)t % 8) < 4);
  sample.warn_high_beam = (((int)t % 6) >= 3);
  sample.warn_batt = (((int)t % 10) >= 2);
  sample.warn_brake = (((int)t % 11) >= 3);
  return sample;
}

static void x86_publish_current_data()
{
  if (g_board_data == nullptr)
    return;

  PlatformSample sample = x86_collect_platform_sample();
  const uint32_t now = HAL_GetTick();

  g_board_data->rpm.publish(sample.rpm, now);
  g_board_data->speed_mph.publish(sample.speed_mph, now);
  g_board_data->elapsed_ms.publish(sample.elapsed_ms, now);
  g_board_data->loop_count.publish(sample.loop_count, now);
  g_board_data->loop_period_ms.publish(sample.loop_period_ms, now);
  g_board_data->worst_loop_period_ms.publish(sample.worst_loop_period_ms, now);
  BoardAccelerationVector accel = {};
  accel.x_mps2 = 6.3f * std::sinf((sample.elapsed_ms / 1000.0f) * 0.95f);
  accel.y_mps2 = 24.3f * std::cosf((sample.elapsed_ms / 1000.0f) * 1.15f);
  accel.z_mps2 = 0.0f;
  g_board_data->acceleration_mps2.publish(accel, now);
  g_board_data->startup_init_error.publish(sample.startup_init_error, now);
  g_board_data->lamp_on.publish(sample.warn_lamp_on, now);
  g_board_data->btn.publish(sample.btn, now);

  g_board_data->ecu_supported = true;
  g_board_data->ecu = sample.ecu;
  g_board_data->ecu_param_tps_index = sample.ecu_param_tps_index;
  g_board_data->ecu_param_wb_index = sample.ecu_param_wb_index;
  g_board_data->ecu_param_map_index = sample.ecu_param_map_index;
  g_board_data->ecu_param_knock_index = sample.ecu_param_knock_index;
  g_board_data->ecu_flasher = sample.ecu_flasher;

  if (g_warn_batt != nullptr)
    g_warn_batt->publish(sample.warn_batt);
  if (g_warn_brake != nullptr)
    g_warn_brake->publish(sample.warn_brake);
  if (g_warn_4ws != nullptr)
    g_warn_4ws->publish(sample.warn_4ws);
  if (g_warn_high_beam != nullptr)
    g_warn_high_beam->publish(sample.warn_high_beam);
}

void board_init(BoardSharedData &data, SharedRenderCtx &ctx, int &draw_step, uint32_t &timer_draw_ms)
{
  g_board_data = &data;
  x86_bringup_hardware();
  g_sim_ecu.connect();
  g_warn_batt = data.add_warning_image("batt", 140, 200, &g_host_ctx.batt_img);
  g_warn_brake = data.add_warning_light("brake", "BRAKE", 120, 233, GFX_RED);
  g_warn_4ws = data.add_warning_light("4ws", "4WS", 175, 205, GFX_YELLOW);
  g_warn_high_beam = data.add_warning_image("high_beam", 190, 223, &g_host_ctx.beam_img);
  ctx = g_host_render_ctx;
  x86_publish_current_data();
  draw_step = 0;
  timer_draw_ms = g_host_ctx.timer_draw_ms;
}

void board_update()
{
  x86_publish_current_data();
  HAL_Delay(16);
}

bool board_check_exit()
{
  return false;
}

bool board_display_ready()
{
  return true;
}

void board_shutdown()
{
}

HAL_StatusTypeDef HAL_TIM_Base_Start_DMA_to_SPI(TIM_HandleTypeDef *htim, const uint32_t *pData, uint16_t Length)
{
  (void)htim;
  (void)pData;
  (void)Length;
  return HAL_OK;
}
