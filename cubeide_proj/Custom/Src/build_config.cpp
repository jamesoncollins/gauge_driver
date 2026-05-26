#include "build_config.hpp"

namespace
{
constexpr float kMphPerHz = (0.8425872f * 1.015625f);
constexpr float kRpmPerHz = 20.0f;
constexpr uint8_t kSpeedTicksPerOdoTick = 3U;
constexpr uint8_t kOdoStepsPerTick = 6U;
}

const BuildConfig &get_build_config()
{
#if defined(TARGET_SIM_3000GT_SOFT)
  static const BuildConfig config = {
      {PlatformKind::x86_sim, "x86_sim"},
      {BoardKind::sim_host, "sim_host"},
      {DisplayKind::software, "software", 800, 600, 20, 50, 100},
      {VehicleKind::vehicle_3000gt, "3000gt", 5500, 5700, 6500, kMphPerHz, kRpmPerHz, kSpeedTicksPerOdoTick, kOdoStepsPerTick}};
#elif defined(TARGET_3000GT_LCD)
  static const BuildConfig config = {
      {PlatformKind::stm32wb55, "stm32wb55"},
      {BoardKind::board_3000gt_rev_a, "board_3000gt_rev_a"},
      {DisplayKind::st7789vi_lcd, "st7789vi_lcd", 240, 280, 20, 50, 100},
      {VehicleKind::vehicle_3000gt, "3000gt", 5500, 5700, 6500, kMphPerHz, kRpmPerHz, kSpeedTicksPerOdoTick, kOdoStepsPerTick}};
#else
  static const BuildConfig config = {
      {PlatformKind::stm32wb55, "stm32wb55"},
      {BoardKind::board_3000gt_rev_a, "board_3000gt_rev_a"},
      {DisplayKind::s6e63d6_oled, "s6e63d6_oled", 240, 320, 20, 50, 100},
      {VehicleKind::vehicle_3000gt, "3000gt", 5500, 5700, 6500, kMphPerHz, kRpmPerHz, kSpeedTicksPerOdoTick, kOdoStepsPerTick}};
#endif
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
