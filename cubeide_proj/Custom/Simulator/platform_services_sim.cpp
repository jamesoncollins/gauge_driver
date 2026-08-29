#include <cstdint>
#include <cstdio>
#include <cmath>
#include "../../res/batt.c"
#include "../../res/beam.c"
#include "../../res/mask_mono.c"

#include "main_shared.h"
#include "build_config.hpp"
#include "platform_services.hpp"
#include "gfx.h"
#include "ugfx_widgets.h"
#include "app_model.hpp"
#include "sim_ecuk.hpp"

struct PlatformSample
{
  uint64_t data_mask;
  float rpm;
  float speed_mph;
  uint32_t elapsed_ms;
  uint32_t loop_count;
  uint32_t loop_period_ms;
  uint32_t worst_loop_period_ms;
  int gimbal_x;
  int gimbal_y;
  bool startup_init_error;
  bool warn_batt;
  bool warn_brake;
  bool warn_4ws;
  bool warn_lamp_on;
  bool warn_high_beam;
  ECUK *ecu;
  int ecu_param_tps_index;
  int ecu_param_wb_index;
  int ecu_param_map_index;
  int ecu_param_knock_index;
  int ecu_param_timing_index = 0;
  int ecu_param_afr_target_index = 0;
  int ecu_param_fuel_trim_front_low_index;
  int ecu_param_fuel_trim_front_med_index;
  int ecu_param_fuel_trim_front_high_index;
  int ecu_param_fuel_trim_rear_low_index;
  int ecu_param_fuel_trim_rear_med_index;
  int ecu_param_fuel_trim_rear_high_index;
  flasher_t *ecu_flasher;
  button_e btn;
};

struct HostPlatformCtx
{
  color_t amber;
  font_t font20;
  font_t font10;
  font_t fontLCD;
  font_t fontValue;
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


static float sim_clampf(float value, float low, float high)
{
  if (value < low)
    return low;
  if (value > high)
    return high;
  return value;
}

static float sim_lerpf(float from, float to, float amount)
{
  return from + (to - from) * sim_clampf(amount, 0.0f, 1.0f);
}

static float sim_smoothstep(float edge0, float edge1, float value)
{
  const float x = sim_clampf((value - edge0) / (edge1 - edge0), 0.0f, 1.0f);
  return x * x * (3.0f - 2.0f * x);
}

static bool sim_mask_visible_at(int x, int y, int screen_w, int screen_h)
{
  if (screen_w <= 0 || screen_h <= 0)
    return false;

  int mask_x = (x * (int)mask_mono_width) / screen_w;
  int mask_y = (y * (int)mask_mono_height) / screen_h;
  if (mask_x < 0)
    mask_x = 0;
  if (mask_y < 0)
    mask_y = 0;
  if (mask_x >= (int)mask_mono_width)
    mask_x = (int)mask_mono_width - 1;
  if (mask_y >= (int)mask_mono_height)
    mask_y = (int)mask_mono_height - 1;

  const uint8_t row_byte = mask_mono_bits[mask_y * mask_mono_stride + mask_x / 8];
  return (row_byte & (uint8_t)(1u << (7 - (mask_x % 8)))) != 0;
}

static void sim_draw_plastic_overlay(const SharedRenderCtx &ctx)
{
  const int screen_w = ctx.screen_width > 0 ? ctx.screen_width : (int)gdispGetWidth();
  const int screen_h = ctx.screen_height > 0 ? ctx.screen_height : (int)gdispGetHeight();
  const color_t plastic_color = HTML2COLOR(0x3F4442);
  const color_t rim_highlight = HTML2COLOR(0x68716E);
  const color_t rim_shadow = HTML2COLOR(0x111514);

  for (int y = 0; y < screen_h; ++y)
  {
    int plastic_run_start = -1;
    int first_visible = -1;
    int last_visible = -1;

    for (int x = 0; x < screen_w; ++x)
    {
      const bool visible = sim_mask_visible_at(x, y, screen_w, screen_h);
      if (visible)
      {
        if (first_visible < 0)
          first_visible = x;
        last_visible = x;

        if (plastic_run_start >= 0)
        {
          gdispFillArea(plastic_run_start, y, x - plastic_run_start, 1, plastic_color);
          plastic_run_start = -1;
        }
      }
      else if (plastic_run_start < 0)
      {
        plastic_run_start = x;
      }
    }

    if (plastic_run_start >= 0)
      gdispFillArea(plastic_run_start, y, screen_w - plastic_run_start, 1, plastic_color);

    if (first_visible >= 0)
    {
      gdispFillArea(first_visible, y, 1, 1, rim_highlight);
      gdispFillArea(last_visible, y, 1, 1, rim_shadow);
    }
  }
}
static SimVehicleSnapshot sim_make_wot_pull(uint32_t elapsed_ms)
{
  const float t = elapsed_ms / 1000.0f;
  const float cycle_s = 28.0f;
  float cycle_t = std::fmod(t, cycle_s);
  if (cycle_t < 0.0f)
    cycle_t += cycle_s;

  SimVehicleSnapshot out = {};
  out.rpm = 900.0f;
  out.speed_mph = 0.0f;
  out.throttle_pct = 2.0f;
  out.wideband_afr = 14.7f;
  out.map_psi = -8.5f;
  out.knock_count = 0.0f;
  out.timing_deg = 14.0f;
  out.afr_target = 14.7f;
  out.battery_v = 13.8f + 0.15f * std::sinf(t * 0.37f);
  out.fuel_trim_front_low_pct = 100.0f + 3.0f * std::sinf(t * 0.31f) - 0.8f;
  out.fuel_trim_front_med_pct = 100.0f + 3.5f * std::sinf(t * 0.23f + 1.2f) + 0.8f;
  out.fuel_trim_front_high_pct = 100.0f + 4.0f * std::sinf(t * 0.19f + 2.4f);
  out.fuel_trim_rear_low_pct = 100.0f + 3.0f * std::sinf(t * 0.29f + 2.1f) + 0.5f;
  out.fuel_trim_rear_med_pct = 100.0f + 3.5f * std::sinf(t * 0.21f + 0.4f) - 0.8f;
  out.fuel_trim_rear_high_pct = 100.0f + 4.0f * std::sinf(t * 0.17f + 1.6f) + 0.6f;
  out.acceleration_mps2 = 0.0f;
  out.gear = 0;

  if (cycle_t < 1.2f)
  {
    out.rpm = 900.0f + 80.0f * std::sinf(cycle_t * 5.0f);
    return out;
  }

  cycle_t -= 1.2f;

  static const float gear_time_s[] = {1.85f, 2.10f, 2.65f, 3.35f, 4.45f, 5.65f};
  static const float gear_start_rpm[] = {3200.0f, 4050.0f, 4250.0f, 4450.0f, 4650.0f, 4850.0f};
  static const float mph_per_1000_rpm[] = {5.55f, 8.55f, 12.15f, 16.65f, 21.55f, 27.25f};
  static const float boost_target_psi[] = {11.0f, 14.0f, 15.5f, 16.2f, 16.5f, 16.5f};
  const float shift_s = 0.28f;
  static const float shift_rpm_drop[] = {4050.0f, 4250.0f, 4450.0f, 4650.0f, 4850.0f};

  for (int gear = 0; gear < 6; ++gear)
  {
    const float pull_s = gear_time_s[gear];
    if (cycle_t <= pull_s)
    {
      const float pull = sim_clampf(cycle_t / pull_s, 0.0f, 1.0f);
      const float rpm_curve = sim_smoothstep(0.0f, 1.0f, pull);
      const float shift_rpm = 6520.0f;
      out.gear = gear + 1;
      out.rpm = sim_lerpf(gear_start_rpm[gear], shift_rpm, rpm_curve);
      out.speed_mph = out.rpm * mph_per_1000_rpm[gear] / 1000.0f;
      out.throttle_pct = 100.0f;

      const float spool = sim_smoothstep(3300.0f, 5200.0f, out.rpm) * sim_smoothstep(0.05f, 0.45f, pull);
      out.map_psi = sim_lerpf(-1.0f, boost_target_psi[gear], spool);
      out.wideband_afr = sim_lerpf(12.6f, 11.2f, spool);
      out.afr_target = 11.2f;
      out.timing_deg = sim_lerpf(18.0f, 8.0f, spool);
      out.knock_count = (out.rpm > 5850.0f) ? (0.8f + 1.5f * std::pow((out.rpm - 5850.0f) / 700.0f, 2.0f)) : 0.0f;
      out.knock_count += 0.35f * (0.5f + 0.5f * std::sinf(t * 18.0f + (float)gear));
      out.acceleration_mps2 = sim_lerpf(8.8f - (float)gear * 0.95f, 4.4f - (float)gear * 0.35f, pull);
      return out;
    }

    cycle_t -= pull_s;
    if (gear < 5 && cycle_t <= shift_s)
    {
      const float shift = sim_clampf(cycle_t / shift_s, 0.0f, 1.0f);
      out.gear = gear + 1;
      out.rpm = sim_lerpf(6520.0f, shift_rpm_drop[gear], shift);
      out.speed_mph = out.rpm * mph_per_1000_rpm[gear] / 1000.0f;
      out.throttle_pct = sim_lerpf(100.0f, 22.0f, sim_smoothstep(0.0f, 0.45f, shift));
      out.map_psi = sim_lerpf(boost_target_psi[gear], -2.5f, sim_smoothstep(0.0f, 0.7f, shift));
      out.wideband_afr = sim_lerpf(11.4f, 13.3f, shift);
      out.afr_target = sim_lerpf(11.2f, 14.7f, shift);
      out.timing_deg = sim_lerpf(8.0f, 16.0f, shift);
      out.knock_count = 1.0f + 0.8f * (0.5f + 0.5f * std::sinf(t * 28.0f));
      out.acceleration_mps2 = -2.0f;
      return out;
    }
    cycle_t -= shift_s;
  }

  const float coast = sim_clampf(cycle_t / 3.0f, 0.0f, 1.0f);
  out.gear = 6;
  out.rpm = sim_lerpf(4850.0f, 2500.0f, coast);
  out.speed_mph = sim_lerpf(132.0f, 70.0f, coast);
  out.throttle_pct = sim_lerpf(18.0f, 4.0f, coast);
  out.map_psi = sim_lerpf(-2.0f, -9.0f, coast);
  out.wideband_afr = sim_lerpf(13.5f, 15.2f, coast);
  out.afr_target = 14.7f;
  out.timing_deg = sim_lerpf(16.0f, 28.0f, coast);
  out.knock_count = 0.0f;
  out.acceleration_mps2 = -3.0f;
  return out;
}
static void sim_bringup_hardware()
{
  gfxInit();
  gdispClear(GFX_BLACK);

  g_host_ctx.amber = HTML2COLOR(0xFFB000);
  g_host_ctx.font20 = gdispOpenFont("DejaVuSans20");
  g_host_ctx.font10 = gdispOpenFont("DejaVuSans10");
  g_host_ctx.fontLCD = gdispOpenFont("lcddot_tr80");
  g_host_ctx.fontValue = gdispOpenFont("BITSUMIS84_Numbers");
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
    .fontValue = g_host_ctx.fontValue,
    .screen_width = (coord_t)gdispGetWidth(),
    .screen_height = (coord_t)gdispGetHeight(),
    .batt_img = &g_host_ctx.batt_img,
    .beam_img = &g_host_ctx.beam_img,
    .gimball = &g_host_ctx.gimball,
    .line_plot_tps = &g_host_ctx.line_plot_tps,
    .line_plot_tps_data = g_host_ctx.tps_plot_data,
    .line_plot_knock = &g_host_ctx.line_plot_knock,
    .line_plot_knock_data = g_host_ctx.knock_plot_data,
    .render_cycle_complete = false,
  };
}

static PlatformSample sim_collect_platform_sample()
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
  if (sample.elapsed_ms >= 5000U && !g_sim_ecu.isConnected())
    g_sim_ecu.connect();
  const SimVehicleSnapshot vehicle = sim_make_wot_pull(sample.elapsed_ms);

  g_signals.rpm.value = vehicle.rpm;
  g_signals.rpm.fresh = true;
  g_signals.rpm.valid = true;
  g_signals.rpm.timestamp_ms = now;

  g_signals.speed_mph.value = vehicle.speed_mph;
  g_signals.speed_mph.fresh = true;
  g_signals.speed_mph.valid = true;
  g_signals.speed_mph.timestamp_ms = now;

  g_signals.board_batt_v.value = vehicle.battery_v;
  g_signals.board_batt_v.fresh = true;
  g_signals.board_batt_v.valid = true;
  g_signals.board_batt_v.timestamp_ms = now;

  g_sim_ecu.simulate(now, vehicle);

  sample.rpm = g_signals.rpm.value;
  sample.speed_mph = g_signals.speed_mph.value;
  sample.ecu = g_ecu;
  sample.ecu_param_tps_index = SimECUK::PARAM_TPS;
  sample.ecu_param_wb_index = SimECUK::PARAM_WB;
  sample.ecu_param_map_index = SimECUK::PARAM_MAP;
  sample.ecu_param_knock_index = SimECUK::PARAM_KNOCK;
  sample.ecu_param_timing_index = SimECUK::PARAM_TIMING;
  sample.ecu_param_afr_target_index = SimECUK::PARAM_AFR_TARGET;
  sample.ecu_param_fuel_trim_front_low_index = SimECUK::PARAM_FFTL;
  sample.ecu_param_fuel_trim_front_med_index = SimECUK::PARAM_FFTM;
  sample.ecu_param_fuel_trim_front_high_index = SimECUK::PARAM_FFTH;
  sample.ecu_param_fuel_trim_rear_low_index = SimECUK::PARAM_RFTL;
  sample.ecu_param_fuel_trim_rear_med_index = SimECUK::PARAM_RFTM;
  sample.ecu_param_fuel_trim_rear_high_index = SimECUK::PARAM_RFTH;
  sample.ecu_flasher = nullptr;
  sample.startup_init_error = false;

  sample.btn = BTN_INV;
  if (((int)t % 9) == 1) sample.btn = BTN_U;
  if (((int)t % 9) == 3) sample.btn = BTN_D;
  if (((int)t % 9) == 5) sample.btn = BTN_L;
  if (((int)t % 9) == 7) sample.btn = BTN_R;

  const int warning_phase = (int)t % 16;
  const bool warnings_clear = warning_phase < 4;
  sample.warn_4ws = !warnings_clear && warning_phase >= 8;
  sample.warn_lamp_on = (((int)t % 8) < 4);
  sample.warn_high_beam = (((int)t % 6) >= 3);
  sample.warn_batt = !warnings_clear && (((int)t % 10) >= 2);
  sample.warn_brake = !warnings_clear && (((int)t % 11) >= 3);
  return sample;
}

static void sim_publish_current_data()
{
  if (g_board_data == nullptr)
    return;

  PlatformSample sample = sim_collect_platform_sample();
  const uint32_t now = HAL_GetTick();

  g_board_data->rpm.publish(sample.rpm, now);
  g_board_data->speed_mph.publish(sample.speed_mph, now);
  g_board_data->elapsed_ms.publish(sample.elapsed_ms, now);
  g_board_data->loop_count.publish(sample.loop_count, now);
  g_board_data->loop_period_ms.publish(sample.loop_period_ms, now);
  g_board_data->worst_loop_period_ms.publish(sample.worst_loop_period_ms, now);
  g_board_data->startup_init_error_code.publish(sample.startup_init_error ? 1U : 0U, now);
  const SimVehicleSnapshot vehicle = sim_make_wot_pull(sample.elapsed_ms);
  BoardAccelerationVector accel = {};
  const float pitch_rad = get_build_config().vehicle.board_mount_pitch_deg * 3.14159265358979323846f / 180.0f;
  float pitch_cos = std::cos(pitch_rad);
  if (std::fabs(pitch_cos) < 0.001f)
    pitch_cos = 1.0f;
  accel.x_mps2 = -vehicle.acceleration_mps2 / pitch_cos;
  accel.y_mps2 = 0.0f;
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
  g_board_data->ecu_param_timing_index = sample.ecu_param_timing_index;
  g_board_data->ecu_param_afr_target_index = sample.ecu_param_afr_target_index;
  g_board_data->ecu_param_fuel_trim_front_low_index = sample.ecu_param_fuel_trim_front_low_index;
  g_board_data->ecu_param_fuel_trim_front_med_index = sample.ecu_param_fuel_trim_front_med_index;
  g_board_data->ecu_param_fuel_trim_front_high_index = sample.ecu_param_fuel_trim_front_high_index;
  g_board_data->ecu_param_fuel_trim_rear_low_index = sample.ecu_param_fuel_trim_rear_low_index;
  g_board_data->ecu_param_fuel_trim_rear_med_index = sample.ecu_param_fuel_trim_rear_med_index;
  g_board_data->ecu_param_fuel_trim_rear_high_index = sample.ecu_param_fuel_trim_rear_high_index;
  g_board_data->ecu_flasher = sample.ecu_flasher;

  if (g_warn_batt != nullptr)
    g_warn_batt->publish(sample.warn_batt);
  if (g_warn_brake != nullptr)
    g_warn_brake->publish(sample.warn_brake);
  if (g_warn_4ws != nullptr)
    g_warn_4ws->publish(sample.warn_4ws);
  g_board_data->high_beam.publish(sample.warn_high_beam, now);
}

void board_init(BoardSharedData &data, SharedRenderCtx &ctx, int &draw_step, uint32_t &timer_draw_ms)
{
  g_board_data = &data;
  sim_bringup_hardware();
  g_warn_batt = data.add_warning_image("batt", 140, 38, &g_host_ctx.batt_img);
  g_warn_brake = data.add_warning_light("brake", "BRAKE", 110, 70, GFX_RED);
  g_warn_4ws = data.add_warning_light("4ws", "4WS", 175, 45, GFX_YELLOW);
  ctx = g_host_render_ctx;
  sim_publish_current_data();
  draw_step = 0;
  timer_draw_ms = g_host_ctx.timer_draw_ms;
}

void board_update()
{
  sim_publish_current_data();
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

void board_render_after(const RuntimeState &state, const BoardSharedData &data, SharedRenderCtx &ctx)
{
  (void)state;
  (void)data;
#if defined(GAUGE_HOST_BACKEND_WIN32)
  if (!ctx.render_cycle_complete)
    return;
#endif
  sim_draw_plastic_overlay(ctx);
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
