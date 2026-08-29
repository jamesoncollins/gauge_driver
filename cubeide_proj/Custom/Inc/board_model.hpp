#ifndef INC_BOARD_MODEL_HPP_
#define INC_BOARD_MODEL_HPP_

#include "platform_api.h"

#ifdef __cplusplus
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include "ugfx_widgets.h"
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
struct SharedRenderCtx;

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
  BOARD_PANEL_OBJECT_STYLE_TEXT = 0,
  BOARD_PANEL_OBJECT_STYLE_IMAGE = 1
} BoardPanelObjectStyle;

static constexpr BoardPanelObjectStyle BOARD_WARNING_STYLE_TEXT = BOARD_PANEL_OBJECT_STYLE_TEXT;
static constexpr BoardPanelObjectStyle BOARD_WARNING_STYLE_IMAGE = BOARD_PANEL_OBJECT_STYLE_IMAGE;

typedef BoardPanelObjectStyle BoardWarningStyle;

struct BoardPanelObject
{
  const char *key = nullptr;
  const char *label = nullptr;
  BoardPanelObjectStyle style = BOARD_PANEL_OBJECT_STYLE_TEXT;
  bool supported = false;
  bool active = false;
  bool good = false;
  bool fresh = false;
  bool ack = true;
  color_t color = GFX_RED;
  int x = 0;
  int y = 0;
  gImage *image = nullptr;

  void configure_text(const char *object_key, const char *object_label, int draw_x, int draw_y, color_t draw_color)
  {
    key = object_key;
    label = object_label;
    style = BOARD_PANEL_OBJECT_STYLE_TEXT;
    x = draw_x;
    y = draw_y;
    color = draw_color;
    supported = true;
    good = true;
  }

  void configure_image(const char *object_key, int draw_x, int draw_y, gImage *draw_image)
  {
    key = object_key;
    label = nullptr;
    style = BOARD_PANEL_OBJECT_STYLE_IMAGE;
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

using BoardWarningLight = BoardPanelObject;

struct BoardObjectPanel
{
  static constexpr std::size_t MAX_OBJECTS = 12;

  BoardPanelObject *add_text(const char *key, const char *label, int x, int y, color_t color)
  {
    if (object_count >= MAX_OBJECTS)
      return nullptr;
    BoardPanelObject &object = objects[object_count++];
    object.configure_text(key, label, x, y, color);
    return &object;
  }

  BoardPanelObject *add_image(const char *key, int x, int y, gImage *image)
  {
    if (object_count >= MAX_OBJECTS)
      return nullptr;
    BoardPanelObject &object = objects[object_count++];
    object.configure_image(key, x, y, image);
    return &object;
  }

  void mark_all_read()
  {
    for (std::size_t i = 0; i < object_count; ++i)
      objects[i].mark_read();
  }

  void render(SharedRenderCtx &ctx) const;

private:
  static bool is_active(const BoardPanelObject &object)
  {
    return object.supported && object.good && object.active;
  }

  BoardPanelObject objects[MAX_OBJECTS];
  std::size_t object_count = 0;
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
  static constexpr std::size_t MAX_STRING_VALUES = 8;

  BoardValue<float> rpm;
  BoardValue<float> speed_mph;
  BoardValue<uint32_t> elapsed_ms;
  BoardValue<uint32_t> loop_count;
  BoardValue<uint32_t> loop_period_ms;
  BoardValue<uint32_t> worst_loop_period_ms;
  BoardValue<uint32_t> startup_init_error_code;
  BoardValue<BoardAccelerationVector> acceleration_mps2;
  BoardValue<bool> startup_init_error;
  BoardValue<bool> lamp_on;
  BoardValue<bool> high_beam;
  BoardValue<button_e> btn;

  bool ecu_supported = false;
  ECUK *ecu = nullptr;
  int ecu_param_tps_index = 0;
  int ecu_param_wb_index = 0;
  int ecu_param_map_index = 0;
  int ecu_param_knock_index = 0;
  int ecu_param_timing_index = 0;
  int ecu_param_afr_target_index = 0;
  int ecu_param_fuel_trim_front_low_index = 0;
  int ecu_param_fuel_trim_front_med_index = 0;
  int ecu_param_fuel_trim_front_high_index = 0;
  int ecu_param_fuel_trim_rear_low_index = 0;
  int ecu_param_fuel_trim_rear_med_index = 0;
  int ecu_param_fuel_trim_rear_high_index = 0;
  flasher_t *ecu_flasher = nullptr;

  BoardObjectPanel warning_panel;
  BoardStringValue string_values[MAX_STRING_VALUES];
  std::size_t string_value_count = 0;

  BoardWarningLight *add_warning_light(const char *key, const char *label, int x, int y, color_t color)
  {
    return warning_panel.add_text(key, label, x, y, color);
  }

  BoardWarningLight *add_warning_image(const char *key, int x, int y, gImage *image)
  {
    return warning_panel.add_image(key, x, y, image);
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
    startup_init_error_code.mark_read();
    acceleration_mps2.mark_read();
    startup_init_error.mark_read();
    lamp_on.mark_read();
    high_beam.mark_read();
    btn.mark_read();
    warning_panel.mark_all_read();
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
  uint32_t startup_init_error_code;
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
  int ecu_param_timing_index;
  int ecu_param_afr_target_index;
  int ecu_param_fuel_trim_front_low_index;
  int ecu_param_fuel_trim_front_med_index;
  int ecu_param_fuel_trim_front_high_index;
  int ecu_param_fuel_trim_rear_low_index;
  int ecu_param_fuel_trim_rear_med_index;
  int ecu_param_fuel_trim_rear_high_index;
  flasher_t *ecu_flasher;
  button_e btn;
} RuntimeState;

struct SharedRenderCtx
{
  // Render-facing references: fonts/images/screen geometry are platform-provided;
  // amber, gimbal, and plots are owned by shared runtime state.
  color_t *amber_ptr;
  font_t font10;
  font_t font20;
  font_t fontLCD;
  font_t fontValue;
  coord_t screen_width;
  coord_t screen_height;
  gImage *batt_img;
  gImage *beam_img;
  Gimball_t *gimball;
  LinePlot_t *line_plot_tps;
  int *line_plot_tps_data;
  LinePlot_t *line_plot_knock;
  int *line_plot_knock_data;
  bool render_cycle_complete;
};
inline void BoardObjectPanel::render(SharedRenderCtx &ctx) const
{
  std::size_t active_count = 0;
  for (std::size_t i = 0; i < object_count; ++i)
  {
    if (is_active(objects[i]))
      ++active_count;
  }

  if (active_count == 0)
    return;

  const coord_t columns = active_count > 2 ? 2 : (coord_t)active_count;
  const coord_t rows = (active_count + columns - 1) / columns;
  const coord_t slot_w = 86;
  const coord_t slot_h = 42;
  const coord_t panel_pad_x = 6;
  const coord_t panel_pad_y = 6;
  coord_t panel_w = columns * slot_w + 2 * panel_pad_x;
  coord_t panel_h = rows * slot_h + 2 * panel_pad_y;
  const coord_t max_panel_w = ctx.screen_width > 12 ? ctx.screen_width - 12 : ctx.screen_width;
  if (panel_w > max_panel_w)
    panel_w = max_panel_w;
  const coord_t panel_center_y = 164 + (panel_h * 30) / 100;
  const coord_t panel_x = (ctx.screen_width - panel_w) / 2;
  const coord_t panel_y = panel_center_y - panel_h / 2;
  gdispFillArea(panel_x, panel_y, panel_w, panel_h, GFX_BLACK);
  gdispDrawBox(panel_x, panel_y, panel_w, panel_h, GFX_AMBER_YEL);

  const coord_t content_x = panel_x + panel_pad_x;
  const coord_t content_y = panel_y + panel_pad_y;
  const coord_t actual_slot_w = (panel_w - 2 * panel_pad_x) / columns;
  const coord_t actual_slot_h = (panel_h - 2 * panel_pad_y) / rows;
  std::size_t active_index = 0;
  for (std::size_t i = 0; i < object_count; ++i)
  {
    const BoardPanelObject &object = objects[i];
    if (!is_active(object))
      continue;

    const coord_t slot_x = content_x + (coord_t)(active_index % columns) * actual_slot_w;
    const coord_t slot_y = content_y + (coord_t)(active_index / columns) * actual_slot_h;
    const coord_t slot_center_x = slot_x + actual_slot_w / 2;
    const coord_t slot_center_y = slot_y + actual_slot_h / 2;

    if (object.style == BOARD_PANEL_OBJECT_STYLE_IMAGE && object.image != nullptr)
    {
      const coord_t image_x = slot_center_x - object.image->width / 2;
      const coord_t image_y = slot_center_y - object.image->height / 2;
      gdispImageDraw(object.image, image_x, image_y, object.image->width, object.image->height, 0, 0);
    }
    else if (object.label != nullptr)
    {
      const coord_t text_pad_y = 4;
      gdispFillStringBox(slot_x + 2,
                         slot_y + text_pad_y,
                         actual_slot_w - 4,
                         actual_slot_h - 2 * text_pad_y,
                         object.label,
                         ctx.font20,
                         object.color,
                         GFX_BLACK,
                         (gJustify)(gJustifyCenter | gJustifyNoWordWrap));
    }

    ++active_index;
  }
}
#endif

#endif /* INC_BOARD_MODEL_HPP_ */
