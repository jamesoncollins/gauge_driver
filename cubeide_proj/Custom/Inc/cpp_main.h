
#ifndef INC_MAIN_CPP_H_
#define INC_MAIN_CPP_H_

#include "platform_api.h"
#ifdef __cplusplus
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include "ugfx_widgets.h"
#endif

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C"
{
void main_cpp();
}
#else
void main_cpp(void);
#endif

/*
 * button type for bluetooth UI
 */
typedef enum
{
  BTN_OK = 0,
  BTN_U, BTN_D, BTN_L, BTN_R,
  BTN_INV = 255
}
button_e;

#ifdef __cplusplus
class ECUK;

enum PlatformDataKey : uint64_t
{
  PLATFORM_DATA_NONE = 0ULL,
  PLATFORM_DATA_RPM = 1ULL << 0,
  PLATFORM_DATA_SPEED_MPH = 1ULL << 1,
  PLATFORM_DATA_TIMING_DIAG = 1ULL << 2,
  PLATFORM_DATA_GIMBAL = 1ULL << 3,
  PLATFORM_DATA_STARTUP_ERROR = 1ULL << 4,
  PLATFORM_DATA_WARN_BATT = 1ULL << 5,
  PLATFORM_DATA_WARN_BRAKE = 1ULL << 6,
  PLATFORM_DATA_WARN_4WS = 1ULL << 7,
  PLATFORM_DATA_WARN_LAMP = 1ULL << 8,
  PLATFORM_DATA_WARN_HIGH_BEAM = 1ULL << 9,
  PLATFORM_DATA_ECU = 1ULL << 10,
  PLATFORM_DATA_BTN = 1ULL << 11
};

static inline bool platform_state_has(uint64_t data_mask, PlatformDataKey key)
{
  return (data_mask & (uint64_t)key) != 0ULL;
}
#endif

#ifdef __cplusplus
template <typename T>
struct BoardValue
{
  T value = {};
  bool supported = false;
  bool good = false;
  bool fresh = false;
  bool ack = true;
  uint32_t timestamp_ms = 0;

  void publish(const T &new_value, uint32_t now_ms, bool value_good = true)
  {
    value = new_value;
    timestamp_ms = now_ms;
    supported = true;
    good = value_good;
    fresh = true;
    ack = false;
  }

  void mark_read()
  {
    fresh = false;
    ack = true;
  }
};

typedef enum
{
  BOARD_WARNING_STYLE_TEXT = 0,
  BOARD_WARNING_STYLE_IMAGE = 1
} BoardWarningStyle;

struct BoardWarningLight
{
  const char *key = nullptr;
  const char *label = nullptr;
  BoardWarningStyle style = BOARD_WARNING_STYLE_TEXT;
  bool supported = false;
  bool active = false;
  bool good = false;
  bool fresh = false;
  bool ack = true;
  color_t color = GFX_RED;
  int x = 0;
  int y = 0;
  gImage *image = nullptr;

  void configure(const char *warning_key, const char *warning_label, int draw_x, int draw_y, color_t draw_color)
  {
    key = warning_key;
    label = warning_label;
    style = BOARD_WARNING_STYLE_TEXT;
    x = draw_x;
    y = draw_y;
    color = draw_color;
    supported = true;
    good = true;
  }

  void configure_image(const char *warning_key, int draw_x, int draw_y, gImage *draw_image)
  {
    key = warning_key;
    label = nullptr;
    style = BOARD_WARNING_STYLE_IMAGE;
    x = draw_x;
    y = draw_y;
    image = draw_image;
    supported = true;
    good = true;
  }

  void publish(bool is_active)
  {
    active = is_active;
    fresh = true;
    ack = false;
  }

  void mark_read()
  {
    fresh = false;
    ack = true;
  }
};

struct BoardStringValue
{
  const char *key = nullptr;
  char value[32] = {};
  bool supported = false;
  bool good = false;
  bool fresh = false;
  bool ack = true;
  uint32_t timestamp_ms = 0;

  void configure(const char *value_key)
  {
    key = value_key;
    supported = true;
  }

  void publish(const char *new_value, uint32_t now_ms, bool value_good = true)
  {
    std::snprintf(value, sizeof(value), "%s", new_value != nullptr ? new_value : "");
    timestamp_ms = now_ms;
    supported = true;
    good = value_good;
    fresh = true;
    ack = false;
  }

  void mark_read()
  {
    fresh = false;
    ack = true;
  }
};

struct BoardAccelerationVector
{
  float x_mps2 = 0.0f;
  float y_mps2 = 0.0f;
  float z_mps2 = 0.0f;
};

struct BoardSharedData
{
  static constexpr std::size_t MAX_WARNING_LIGHTS = 12;
  static constexpr std::size_t MAX_STRING_VALUES = 8;

  BoardValue<float> rpm;
  BoardValue<float> speed_mph;
  BoardValue<uint32_t> elapsed_ms;
  BoardValue<uint32_t> loop_count;
  BoardValue<uint32_t> loop_period_ms;
  BoardValue<uint32_t> worst_loop_period_ms;
  BoardValue<BoardAccelerationVector> acceleration_mps2;
  BoardValue<bool> startup_init_error;
  BoardValue<bool> lamp_on;
  BoardValue<button_e> btn;

  bool ecu_supported = false;
  ECUK *ecu = nullptr;
  int ecu_param_tps_index = 0;
  int ecu_param_wb_index = 0;
  int ecu_param_map_index = 0;
  int ecu_param_knock_index = 0;
  flasher_t *ecu_flasher = nullptr;

  BoardWarningLight warning_lights[MAX_WARNING_LIGHTS];
  std::size_t warning_light_count = 0;
  BoardStringValue string_values[MAX_STRING_VALUES];
  std::size_t string_value_count = 0;

  BoardWarningLight *add_warning_light(const char *key, const char *label, int x, int y, color_t color)
  {
    if (warning_light_count >= MAX_WARNING_LIGHTS)
      return nullptr;
    BoardWarningLight &warning = warning_lights[warning_light_count++];
    warning.configure(key, label, x, y, color);
    return &warning;
  }

  BoardWarningLight *add_warning_image(const char *key, int x, int y, gImage *image)
  {
    if (warning_light_count >= MAX_WARNING_LIGHTS)
      return nullptr;
    BoardWarningLight &warning = warning_lights[warning_light_count++];
    warning.configure_image(key, x, y, image);
    return &warning;
  }

  BoardStringValue *add_string_value(const char *key)
  {
    if (string_value_count >= MAX_STRING_VALUES)
      return nullptr;
    BoardStringValue &string_value = string_values[string_value_count++];
    string_value.configure(key);
    return &string_value;
  }

  void mark_all_read()
  {
    rpm.mark_read();
    speed_mph.mark_read();
    elapsed_ms.mark_read();
    loop_count.mark_read();
    loop_period_ms.mark_read();
    worst_loop_period_ms.mark_read();
    acceleration_mps2.mark_read();
    startup_init_error.mark_read();
    lamp_on.mark_read();
    btn.mark_read();
    for (std::size_t i = 0; i < warning_light_count; ++i)
      warning_lights[i].mark_read();
    for (std::size_t i = 0; i < string_value_count; ++i)
      string_values[i].mark_read();
  }
};

typedef struct
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
  flasher_t *ecu_flasher;
  button_e btn;
} PlatformSample;

typedef struct
{
  uint64_t data_mask;
  float rpm;
  float speed_mph;
  uint32_t elapsed_ms;
  uint32_t loop_count;
  uint32_t loop_period_ms;
  uint32_t worst_loop_period_ms;
  int rpm_mode;
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
  flasher_t *ecu_flasher;
  button_e btn;
} RuntimeState;

typedef struct
{
  color_t *amber_ptr;
  font_t font10;
  font_t font20;
  font_t fontLCD;
  coord_t screen_width;
  coord_t screen_height;
  gImage *batt_img;
  gImage *beam_img;
  Gimball_t *gimball;
  LinePlot_t *line_plot_tps;
  int *line_plot_tps_data;
  LinePlot_t *line_plot_knock;
  int *line_plot_knock_data;
} SharedRenderCtx;
#endif

/*
 * function declarations
 */
int get_x12_ticks_speed( float  );
int get_x12_ticks_rpm( float  );
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA_to_SPI(TIM_HandleTypeDef *htim, const uint32_t *pData, uint16_t Length);

#if defined(CUSTOM_PLATFORM_ARM) && defined(__cplusplus)
class PI4IOE5V6416;
class SwitecX12;
void platform_poll_bulb_inputs(PI4IOE5V6416 &ioexp_screen, uint16_t &bulbVals);
void platform_process_ble_and_lowrate(uint32_t &timerLED, uint32_t &loopCnt, uint32_t loopPeriod, uint32_t worstLoopPeriod);
void platform_drain_bt_budget();
void platform_update_inertial();
void platform_maybe_usb_print(uint32_t &timerPrint, int &logBufInd, char *logBuf, int bufLen);
bool platform_should_exit(uint32_t &timerIGN);
void platform_update_loop_diag(uint32_t &loopCnt, uint32_t &loopPeriod, uint32_t &worstLoopPeriod, uint32_t &timerLoop);
#endif


/*
 * Optional build flags used for diagnostics/test behaviors.
 * Runtime tunables are configured in build_config.
 */
//#define SWEEP_GAUGES  // sweep needles forever
//#define SIM_GAUGES       // generate simulated rpm and mph

// WARNING
// this feature shares a pin with teh PCD backlight.  i dont have any code yet to deconflict this.
//#define SIM_GAUGE_SIGNALS // use lptim, on gpio3, as a pwm signal you can connect to the tach or speed inputs

/*
 * USB-based diagnostics
 *
 * Print to USB: what it sounds like, prints text to the  USB port
 * USB_DISPLAY: sends the display buffer over USB, use the script in ./python to view it.
 *
 * Note, you can use both of these at the same time.
 */
//#define PRINT_TO_USB
//#define USB_DISPLAY

/*
 * Optional debug / diagnostic print on the screen
 */
//#define DIAG_SQUARE

#endif /* INC_MAIN_CPP_H_ */
