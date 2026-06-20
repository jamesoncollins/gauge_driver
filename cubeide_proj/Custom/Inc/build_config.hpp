#ifndef INC_BUILD_CONFIG_HPP_
#define INC_BUILD_CONFIG_HPP_

#include <cstdint>

enum class PlatformKind
{
  host_simulator,
  stm32wb55
};

enum class BoardKind
{
  sim_host,
  board_3000gt_rev_a
};

enum class DisplayKind
{
  software,
  s6e63d6_oled,
  st7789vi_lcd
};

enum class VehicleKind
{
  vehicle_3000gt
};

struct PlatformConfig
{
  PlatformKind kind;
  const char *name;
};

struct BoardConfig
{
  BoardKind kind;
  const char *name;
};

struct DisplayConfig
{
  DisplayKind kind;
  const char *name;
  uint16_t width;
  uint16_t height;
  uint16_t target_fps;
  uint16_t print_period_ms;
  uint16_t led_period_ms;
};

struct VehicleConfig
{
  VehicleKind kind;
  const char *name;
  int rpm_alert_reset;
  int rpm_alert_init;
  int rpm_alert_final;
  float mph_per_hz;
  float rpm_per_hz;
  uint8_t speed_ticks_per_odo_tick;
  uint8_t odo_steps_per_tick;
};

struct BuildConfig
{
  PlatformConfig platform;
  BoardConfig board;
  DisplayConfig display;
  VehicleConfig vehicle;
};

const BuildConfig &get_build_config();
uint32_t get_draw_interval_ms();
uint32_t get_led_interval_ms();
uint32_t get_print_interval_ms();

#endif /* INC_BUILD_CONFIG_HPP_ */
