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
static int g_rpm_mode = 0;
static uint32_t g_loop_count = 0;
static uint32_t g_loop_period_ms = 0;
static uint32_t g_worst_loop_period_ms = 0;
static uint32_t g_loop_last_tick_ms = 0;
static CoreSignals g_signals;
static SimECUK g_sim_ecu;
static ECUK *g_ecu = &g_sim_ecu;

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
  const int gimbal_radius = 45;
  sample.gimbal_x = (int)(std::sinf(t * 0.95f) * gimbal_radius * 0.7f);
  sample.gimbal_y = (int)(std::cosf(t * 1.15f) * gimbal_radius * 0.7f);
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

class X86PlatformServices final : public PlatformServices
{
public:
  void init(SharedRenderCtx &ctx, RuntimeState &state, int &draw_step, uint32_t &timer_draw_ms) override
  {
    x86_bringup_hardware();
    g_sim_ecu.connect();
    ctx = g_host_render_ctx;
    state = runtime_state_from_sample(x86_collect_platform_sample());
    draw_step = 0;
    timer_draw_ms = g_host_ctx.timer_draw_ms;
  }

  void service_background() override
  {
  }

  void poll_inputs() override
  {
  }

  void sample_state(RuntimeState &state) override
  {
    state = runtime_state_from_sample(x86_collect_platform_sample());
  }

  void update_actuators(RuntimeState &state) override
  {
    g_rpm_mode = compute_rpm_mode_shared(state.rpm, g_rpm_mode);
    state.rpm_mode = g_rpm_mode;
    HAL_Delay(16);
  }

  bool should_render(uint32_t timer_draw_ms) const override
  {
    return ((HAL_GetTick() - timer_draw_ms) >= get_draw_interval_ms());
  }

  bool should_exit() const override
  {
    return false;
  }

  void shutdown() override
  {
  }
};

PlatformServices *create_platform_services()
{
  static X86PlatformServices services;
  return &services;
}

HAL_StatusTypeDef HAL_TIM_Base_Start_DMA_to_SPI(TIM_HandleTypeDef *htim, const uint32_t *pData, uint16_t Length)
{
  (void)htim;
  (void)pData;
  (void)Length;
  return HAL_OK;
}
