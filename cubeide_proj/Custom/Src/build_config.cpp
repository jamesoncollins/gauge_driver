#include "build_config.hpp"

namespace
{
constexpr float kMphPerHz = (0.8425872f * 1.015625f);
constexpr float kRpmPerHz = 20.0f;
constexpr float kBoardMountPitchDeg = 75.0f;
constexpr uint8_t kSpeedTicksPerOdoTick = 3U;
constexpr uint8_t kOdoStepsPerTick = 6U;

// These are instrument output ceilings, independent of engine warning RPMs.
constexpr GaugeChannelConfig make_speed_gauge_config()
{
  GaugeChannelConfig gauge{};
  gauge.max_units = 180.0f;
  gauge.max_input_hz = 500.0f; // 2 ms minimum edge spacing; about 428 MPH.
  gauge.median_periods = true;
  gauge.q_jerk = 5.0f;
  gauge.meas_var = 1.0f;
  gauge.gate_sigma = 5.0f;
  gauge.zero_speed_thresh_units = 5.0f;
  gauge.zero_periods_without_tick = 3.0f;
  gauge.max_accel_units_per_s = 60.0f;
  gauge.needle_min_units = 10.0f;
  gauge.needle_zero_angle_deg = 3.0f;
  gauge.needle_degrees_per_unit = 1.35f;
  gauge.needle_microsteps_per_degree = 12.0f;
  return gauge;
}

constexpr GaugeChannelConfig make_tach_gauge_config()
{
  GaugeChannelConfig gauge{};
  gauge.max_units = 10000.0f;
  gauge.max_input_hz = 1000.0f; // 1 ms minimum edge spacing; 20,000 RPM.
  gauge.median_periods = true;
  gauge.q_jerk = 10000.0f; // Separate process-noise tuning, not an RPM limit.
  gauge.meas_var = 9.0f;
  gauge.gate_sigma = 5.0f;
  gauge.zero_speed_thresh_units = 400.0f;
  gauge.zero_periods_without_tick = 3.0f;
  gauge.max_accel_units_per_s = 4000.0f;
  gauge.needle_min_units = 500.0f;
  gauge.needle_zero_angle_deg = 5.0f;
  gauge.needle_degrees_per_unit = 0.0221f;
  gauge.needle_microsteps_per_degree = 12.0f;
  return gauge;
}

PlatformConfig make_platform_config()
{
#if defined(GAUGE_PLATFORM_SIMULATOR)
  return {PlatformKind::host_simulator, "host_simulator"};
#elif defined(GAUGE_PLATFORM_HARDWARE)
  return {PlatformKind::stm32wb55, "stm32wb55"};
#else
#error "No GAUGE_PLATFORM_* macro defined"
#endif
}

BoardConfig make_board_config()
{
#if defined(GAUGE_PLATFORM_SIMULATOR)
  return {BoardKind::sim_host, "sim_host"};
#elif defined(GAUGE_PLATFORM_HARDWARE)
  return {BoardKind::board_3000gt_rev_a, "board_3000gt_rev_a"};
#else
#error "No GAUGE_PLATFORM_* macro defined"
#endif
}

DisplayConfig make_display_config()
{
#if defined(GAUGE_DISPLAY_WIN32)
  return {DisplayKind::software, "win32", 800, 600, 20, 50, 100};
#elif defined(GAUGE_DISPLAY_SDL)
  return {DisplayKind::software, "sdl", 800, 600, 20, 50, 100};
#elif defined(GAUGE_DISPLAY_ST7789VI)
  return {DisplayKind::st7789vi_lcd, "st7789vi_lcd", 240, 280, 20, 50, 100};
#elif defined(GAUGE_DISPLAY_S6E63D6)
  return {DisplayKind::s6e63d6_oled, "s6e63d6_oled", 240, 320, 20, 50, 100};
#else
#error "No GAUGE_DISPLAY_* macro defined"
#endif
}

VehicleConfig make_vehicle_config()
{
#if defined(GAUGE_SIM_PROFILE_3000GT_SOFT) || defined(GAUGE_SIM_PROFILE_NONE)
  return {VehicleKind::vehicle_3000gt, "3000gt", 5500, 5700, 6500, kMphPerHz, kRpmPerHz, kBoardMountPitchDeg, kSpeedTicksPerOdoTick, kOdoStepsPerTick, make_speed_gauge_config(), make_tach_gauge_config()};
#else
#error "No GAUGE_SIM_PROFILE_* macro defined"
#endif
}
}

const BuildConfig &get_build_config()
{
  static const BuildConfig config = {
      make_platform_config(),
      make_board_config(),
      make_display_config(),
      make_vehicle_config()};
  return config;
}

uint32_t get_draw_interval_ms()
{
  const uint16_t fps = get_build_config().display.target_fps;
  if (fps == 0U)
    return 50U;
  return 1000U / (uint32_t)fps;
}

uint32_t get_led_interval_ms()
{
  return (uint32_t)get_build_config().display.led_period_ms;
}

uint32_t get_print_interval_ms()
{
  return (uint32_t)get_build_config().display.print_period_ms;
}
