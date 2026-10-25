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

// Vehicle/instrument-specific parameters. Units are MPH for speed and RPM for
// tach. max_units is the output ceiling, independent of engine warning RPMs.
struct GaugeChannelConfig
{
  float max_units;
  float max_input_hz; // Capture plausibility ceiling, separate from output limit.
  bool median_periods; // Median of three qualified periods before Kalman correction.
  float q_jerk;
  float meas_var;
  float gate_sigma;
  float zero_speed_thresh_units;
  float zero_periods_without_tick;
  float max_accel_units_per_s;
  float needle_min_units;
  float needle_zero_angle_deg;
  float needle_degrees_per_unit;
  float needle_microsteps_per_degree;
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
  float board_mount_pitch_deg;
  uint8_t speed_ticks_per_odo_tick;
  uint8_t odo_steps_per_tick;
  GaugeChannelConfig speed_gauge;
  GaugeChannelConfig tach_gauge;
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
