#include "gauge_layouts.hpp"

#include <cstdio>

#include "gui_layout.hpp"
#include "build_config.hpp"
#include "gfx.h"
#include "ugfx_widgets.h"
#include "../ECUK-lib/ECUK.hpp"

namespace
{
enum GaugeDataSourceId : GuiDataSourceId
{
  GAUGE_SOURCE_NONE = 0,
  GAUGE_SOURCE_WB_AFR = 1,
  GAUGE_SOURCE_MAP_PSI,
  GAUGE_SOURCE_TPS,
  GAUGE_SOURCE_KNOCK,
  GAUGE_SOURCE_TIMING,
  GAUGE_SOURCE_FUEL_TRIM_FRONT_LOW,
  GAUGE_SOURCE_FUEL_TRIM_FRONT_MED,
  GAUGE_SOURCE_FUEL_TRIM_FRONT_HIGH,
  GAUGE_SOURCE_FUEL_TRIM_REAR_LOW,
  GAUGE_SOURCE_FUEL_TRIM_REAR_MED,
  GAUGE_SOURCE_FUEL_TRIM_REAR_HIGH
};

struct TextBarConfig
{
  const char *label;
  const char *units;
  GaugeDataSourceId source;
  float min_value;
  float max_value;
  uint8_t decimals;
  UgfxTextBarMeterMode mode;
  float reference_value;
  coord_t bar_height;
  coord_t segment_size;
  const UgfxMeterBand *bands;
  std::size_t band_count;
};

struct PlotConfig
{
  GaugeDataSourceId source;
  LinePlot_t *SharedRenderCtx::*plot;
  int x;
  int y;
};

struct PostWotSummary
{
  bool valid = false;
  float max_knock = 0.0f;
  float afr = 0.0f;
  float timing = 0.0f;
  float tps = 0.0f;
  float map = 0.0f;
  float rpm = 0.0f;
  uint32_t timestamp_ms = 0;
};

static PostWotSummary g_post_wot_summary;

static const UgfxMeterBand kAfrBands[] = {
    {10.0f, 12.0f, GFX_GREEN},
    {12.0f, 15.0f, GFX_AMBER_YEL},
    {15.0f, 20.0f, GFX_RED},
};

static const UgfxMeterBand kMapBands[] = {
    {-20.0f, 0.0f, GFX_AMBER_YEL},
    {0.0f, 15.0f, GFX_GREEN},
    {15.0f, 20.0f, GFX_RED},
};

static const UgfxMeterBand kTrimBands[] = {
    {-25.0f, -10.0f, GFX_RED},
    {-10.0f, 10.0f, GFX_GREEN},
    {10.0f, 25.0f, GFX_RED},
};

static const UgfxMeterBand kKnockBands[] = {
    {0.0f, 3.0f, GFX_GREEN},
    {3.0f, 7.0f, GFX_AMBER_YEL},
    {7.0f, 15.0f, GFX_RED},
};

static const TextBarConfig kAfrMeter = {"O2", "AFR", GAUGE_SOURCE_WB_AFR, 10.0f, 16.0f, 1, UGFX_TEXT_BAR_METER_SEGMENT, 0.0f, 18, 16, kAfrBands, sizeof(kAfrBands) / sizeof(kAfrBands[0])};
static const TextBarConfig kMapMeter = {"MAP", "PSI", GAUGE_SOURCE_MAP_PSI, -15.0f, 20.0f, 1, UGFX_TEXT_BAR_METER_BIPOLAR, 0.0f, 16, 0, kMapBands, sizeof(kMapBands) / sizeof(kMapBands[0])};
static const TextBarConfig kTpsMeter = {"TPS", "%", GAUGE_SOURCE_TPS, 0.0f, 100.0f, 0, UGFX_TEXT_BAR_METER_FILLED, 0.0f, 12, 0, nullptr, 0};
static const TextBarConfig kKnockMeter = {"KNK", "CNT", GAUGE_SOURCE_KNOCK, 0.0f, 15.0f, 1, UGFX_TEXT_BAR_METER_MARKER, 0.0f, 14, 0, kKnockBands, sizeof(kKnockBands) / sizeof(kKnockBands[0])};
static const TextBarConfig kTimingMeter = {"TIM", "DEG", GAUGE_SOURCE_TIMING, -10.0f, 40.0f, 0, UGFX_TEXT_BAR_METER_MARKER, 0.0f, 12, 0, nullptr, 0};

static const TextBarConfig kTrimFfl = {"FF L", "%", GAUGE_SOURCE_FUEL_TRIM_FRONT_LOW, -25.0f, 25.0f, 1, UGFX_TEXT_BAR_METER_BIPOLAR, 0.0f, 10, 0, kTrimBands, sizeof(kTrimBands) / sizeof(kTrimBands[0])};
static const TextBarConfig kTrimFfm = {"FF M", "%", GAUGE_SOURCE_FUEL_TRIM_FRONT_MED, -25.0f, 25.0f, 1, UGFX_TEXT_BAR_METER_BIPOLAR, 0.0f, 10, 0, kTrimBands, sizeof(kTrimBands) / sizeof(kTrimBands[0])};
static const TextBarConfig kTrimFfh = {"FF H", "%", GAUGE_SOURCE_FUEL_TRIM_FRONT_HIGH, -25.0f, 25.0f, 1, UGFX_TEXT_BAR_METER_BIPOLAR, 0.0f, 10, 0, kTrimBands, sizeof(kTrimBands) / sizeof(kTrimBands[0])};
static const TextBarConfig kTrimRfl = {"RF L", "%", GAUGE_SOURCE_FUEL_TRIM_REAR_LOW, -25.0f, 25.0f, 1, UGFX_TEXT_BAR_METER_BIPOLAR, 0.0f, 10, 0, kTrimBands, sizeof(kTrimBands) / sizeof(kTrimBands[0])};
static const TextBarConfig kTrimRfm = {"RF M", "%", GAUGE_SOURCE_FUEL_TRIM_REAR_MED, -25.0f, 25.0f, 1, UGFX_TEXT_BAR_METER_BIPOLAR, 0.0f, 10, 0, kTrimBands, sizeof(kTrimBands) / sizeof(kTrimBands[0])};
static const TextBarConfig kTrimRfh = {"RF H", "%", GAUGE_SOURCE_FUEL_TRIM_REAR_HIGH, -25.0f, 25.0f, 1, UGFX_TEXT_BAR_METER_BIPOLAR, 0.0f, 10, 0, kTrimBands, sizeof(kTrimBands) / sizeof(kTrimBands[0])};

static bool ecu_param_is_fresh(const ECUK::ecuParam_t *param, uint32_t now_ms)
{
  return param != nullptr && (now_ms - param->lastTime_ms) <= 1000U;
}

static ECUK::ecuParam_t *param_for_source(const RuntimeState &state, GaugeDataSourceId source)
{
  if (!platform_state_has(state.data_mask, PLATFORM_DATA_ECU) || state.ecu == nullptr)
    return nullptr;

  switch (source)
  {
    case GAUGE_SOURCE_WB_AFR:
      return state.ecu->getParam(state.ecu_param_wb_index);
    case GAUGE_SOURCE_MAP_PSI:
      return state.ecu->getParam(state.ecu_param_map_index);
    case GAUGE_SOURCE_TPS:
      return state.ecu->getParam(state.ecu_param_tps_index);
    case GAUGE_SOURCE_KNOCK:
      return state.ecu->getParam(state.ecu_param_knock_index);
    case GAUGE_SOURCE_TIMING:
      return state.ecu->getParam(state.ecu_param_timing_index);
    case GAUGE_SOURCE_FUEL_TRIM_FRONT_LOW:
      return state.ecu->getParam(state.ecu_param_fuel_trim_front_low_index);
    case GAUGE_SOURCE_FUEL_TRIM_FRONT_MED:
      return state.ecu->getParam(state.ecu_param_fuel_trim_front_med_index);
    case GAUGE_SOURCE_FUEL_TRIM_FRONT_HIGH:
      return state.ecu->getParam(state.ecu_param_fuel_trim_front_high_index);
    case GAUGE_SOURCE_FUEL_TRIM_REAR_LOW:
      return state.ecu->getParam(state.ecu_param_fuel_trim_rear_low_index);
    case GAUGE_SOURCE_FUEL_TRIM_REAR_MED:
      return state.ecu->getParam(state.ecu_param_fuel_trim_rear_med_index);
    case GAUGE_SOURCE_FUEL_TRIM_REAR_HIGH:
      return state.ecu->getParam(state.ecu_param_fuel_trim_rear_high_index);
    default:
      return nullptr;
  }
}

static float source_value(const RuntimeState &state, GaugeDataSourceId source, bool &valid)
{
  valid = false;
  ECUK::ecuParam_t *param = param_for_source(state, source);
  if (param == nullptr)
    return 0.0f;

  const uint32_t now_ms = HAL_GetTick();
  valid = state.ecu->isConnected() && ecu_param_is_fresh(param, now_ms);
  return param->val;
}

static void render_meter_at(const GuiLayoutRenderCtx &ctx, const void *config, coord_t x, coord_t y, coord_t w, coord_t h)
{
  const TextBarConfig *meter_config = static_cast<const TextBarConfig *>(config);
  if (meter_config == nullptr)
    return;

  bool valid = false;
  const float value = source_value(ctx.state, meter_config->source, valid);
  UgfxTextBarMeter meter;
  color_t amber = (ctx.render.amber_ptr != nullptr) ? *ctx.render.amber_ptr : GFX_AMBER_YEL;
  meter.setBounds(x, y, w, h);
  meter.setColors(amber, GFX_RED, GFX_BLACK);
  meter.configure(meter_config->label,
                  meter_config->units,
                  meter_config->min_value,
                  meter_config->max_value,
                  meter_config->decimals,
                  ctx.render.font20,
                  ctx.render.fontValue);
  meter.setBands(meter_config->bands, meter_config->band_count);
  meter.setMode(meter_config->mode);
  meter.setReferenceValue(meter_config->reference_value);
  meter.setBarHeight(meter_config->bar_height);
  meter.setSegmentSize(meter_config->segment_size);
  meter.setValue(value, valid);
  meter.draw();
}

static void render_default_afr(const GuiLayoutRenderCtx &ctx, const void *)
{
  render_meter_at(ctx, &kAfrMeter, 24, 8, 192, 62);
}

static void render_default_map(const GuiLayoutRenderCtx &ctx, const void *)
{
  render_meter_at(ctx, &kMapMeter, 24, 93, 192, 62);
}

static void render_ecu_error(const GuiLayoutRenderCtx &ctx, const void *)
{
  if (!platform_state_has(ctx.state.data_mask, PLATFORM_DATA_ECU) || ctx.state.ecu == nullptr)
    return;

  bool show_error = !ctx.state.ecu->isConnected();
  if (ctx.state.ecu_flasher != nullptr)
    show_error = flasher_fun(ctx.state.ecu_flasher);
  if (!ctx.state.ecu->isConnected() && show_error)
  {
    const coord_t error_x = 44;
    const coord_t error_y = 52;
    const coord_t error_w = 152;
    const coord_t error_h = 62;
    gdispFillArea(error_x, error_y, error_w, error_h, GFX_BLACK);
    gdispDrawBox(error_x, error_y, error_w, error_h, GFX_AMBER_YEL);
    gdispFillStringBox(error_x + 4,
                       error_y + 4,
                       error_w - 8,
                       error_h - 8,
                       "ECU ERR",
                       ctx.render.font20,
                       GFX_RED,
                       GFX_BLACK,
                       (gJustify)(gJustifyCenter | gJustifyNoWordWrap));
  }
}

static void render_plots(const GuiLayoutRenderCtx &ctx, const void *)
{
  ECUK::ecuParam_t *tps_p = param_for_source(ctx.state, GAUGE_SOURCE_TPS);
  ECUK::ecuParam_t *knock_p = param_for_source(ctx.state, GAUGE_SOURCE_KNOCK);

  if (ctx.render.line_plot_tps != nullptr && tps_p != nullptr && tps_p->isNew)
  {
    linePlotPush(ctx.render.line_plot_tps, (int)tps_p->val);
    tps_p->isNew = false;
  }
  if (ctx.render.line_plot_tps != nullptr)
    linePlot(77, 240, ctx.render.line_plot_tps);

  if (ctx.render.line_plot_knock != nullptr && knock_p != nullptr && knock_p->isNew)
  {
    linePlotPush(ctx.render.line_plot_knock, (int)knock_p->val);
    knock_p->isNew = false;
  }
  if (ctx.render.line_plot_knock != nullptr)
    linePlot(77, 240, ctx.render.line_plot_knock);
}

static void render_status_and_colors(const GuiLayoutRenderCtx &ctx, const void *)
{
  if (platform_state_has(ctx.state.data_mask, PLATFORM_DATA_STARTUP_ERROR) && ctx.state.startup_init_error)
  {
    char err_string[16];
    (void)std::snprintf(err_string, sizeof(err_string), "ERR %02lX", (unsigned long)(ctx.state.startup_init_error_code & 0xFFU));
    gdispFillString((ctx.render.screen_width >> 1) - 72, (ctx.render.screen_height >> 1), err_string, ctx.render.fontLCD, GFX_RED, GFX_BLACK);
  }

  if (platform_state_has(ctx.state.data_mask, PLATFORM_DATA_WARN_LAMP) && ctx.state.warn_lamp_on)
  {
    if (ctx.render.amber_ptr != nullptr)
      *ctx.render.amber_ptr = GFX_AMBER_SAE;
    setColors(GFX_AMBER_SAE, GFX_RED, GFX_BLACK);
  }
  else
  {
    if (ctx.render.amber_ptr != nullptr)
      *ctx.render.amber_ptr = GFX_AMBER_YEL;
    setColors(GFX_AMBER_YEL, GFX_RED, GFX_BLACK);
  }
}

static void render_gimbal(const GuiLayoutRenderCtx &ctx, const void *)
{
  if (ctx.render.gimball != nullptr && platform_state_has(ctx.state.data_mask, PLATFORM_DATA_GIMBAL))
    drawGimball(ctx.render.gimball, 50, 210, 34, ctx.state.gimbal_x, ctx.state.gimbal_y);
}

static void render_high_beam_telltale(const GuiLayoutRenderCtx &ctx, const void *)
{
  if (!platform_state_has(ctx.state.data_mask, PLATFORM_DATA_WARN_HIGH_BEAM) || !ctx.state.warn_high_beam || ctx.render.beam_img == nullptr)
    return;

  gdispImageDraw(ctx.render.beam_img, 190, 170, ctx.render.beam_img->width, ctx.render.beam_img->height, 0, 0);
}

static void render_high_beam_badge(const GuiLayoutRenderCtx &ctx, const void *)
{
  if (!platform_state_has(ctx.state.data_mask, PLATFORM_DATA_WARN_HIGH_BEAM) || !ctx.state.warn_high_beam)
    return;

  gdispFillString(200, 6, "HB", ctx.render.font10, GFX_BLUE, GFX_BLACK);
}

static void render_shift_warning(const GuiLayoutRenderCtx &ctx)
{
  const int WARN_SIZE = 20;
  const int WARN_FINAL_SIZE = 70;
  const int SHIFT_SIZE = 100;
  const VehicleConfig &vehicle = get_build_config().vehicle;
  const int range = vehicle.rpm_alert_final - vehicle.rpm_alert_init;
  int over = (int)ctx.state.rpm - vehicle.rpm_alert_init;
  int percent = (64 * over) / (range > 0 ? range : 1);
  int current_warn_size = WARN_SIZE + (((WARN_FINAL_SIZE - WARN_SIZE) * percent) >> 6);

  if (ctx.state.rpm_mode >= 2)
  {
    gdispFillCircle((ctx.render.screen_width >> 1), (ctx.render.screen_height >> 1), SHIFT_SIZE, GFX_RED);
  }
  else if (ctx.state.rpm_mode >= 1 && current_warn_size > 0)
  {
    gdispFillDualCircle((ctx.render.screen_width >> 1), (ctx.render.screen_height >> 1), WARN_FINAL_SIZE, GFX_BLACK, WARN_FINAL_SIZE, GFX_GREEN);
    gdispFillCircle((ctx.render.screen_width >> 1), (ctx.render.screen_height >> 1), current_warn_size, GFX_YELLOW);
  }
}

static void render_shift_warning_and_panel(const GuiLayoutRenderCtx &ctx, const void *)
{
  render_high_beam_telltale(ctx, nullptr);
  render_shift_warning(ctx);

  ctx.data.warning_panel.render(ctx.render);
}

static void render_alt_indicators(const GuiLayoutRenderCtx &ctx, const void *)
{
  render_high_beam_badge(ctx, nullptr);
  render_shift_warning(ctx);
}

static void render_trim_header(const GuiLayoutRenderCtx &ctx, const void *)
{
  gdispFillString(58, 4, "FUEL TRIMS", ctx.render.font20, GFX_AMBER_YEL, GFX_BLACK);
}

static void render_post_wot_summary(const GuiLayoutRenderCtx &ctx, const void *)
{
  char line[32];
  gdispFillString(28, 6, "POST WOT", ctx.render.font20, GFX_AMBER_YEL, GFX_BLACK);
  if (!g_post_wot_summary.valid)
  {
    gdispFillString(26, 72, "NO PULL DATA", ctx.render.font20, GFX_RED, GFX_BLACK);
    return;
  }

  (void)std::snprintf(line, sizeof(line), "KNK %4.1f", g_post_wot_summary.max_knock);
  gdispFillString(18, 42, line, ctx.render.font20, GFX_RED, GFX_BLACK);
  (void)std::snprintf(line, sizeof(line), "AFR %4.1f TIM %2.0f", g_post_wot_summary.afr, g_post_wot_summary.timing);
  gdispFillString(18, 72, line, ctx.render.font10, GFX_AMBER_YEL, GFX_BLACK);
  (void)std::snprintf(line, sizeof(line), "TPS %3.0f MAP %4.1f", g_post_wot_summary.tps, g_post_wot_summary.map);
  gdispFillString(18, 90, line, ctx.render.font10, GFX_AMBER_YEL, GFX_BLACK);
  (void)std::snprintf(line, sizeof(line), "RPM %4.0f T %lus", g_post_wot_summary.rpm, (unsigned long)(g_post_wot_summary.timestamp_ms / 1000U));
  gdispFillString(18, 108, line, ctx.render.font10, GFX_AMBER_YEL, GFX_BLACK);
}

static void render_wot_title(const GuiLayoutRenderCtx &ctx, const void *)
{
  gdispFillString(86, 4, "WOT", ctx.render.font20, GFX_AMBER_YEL, GFX_BLACK);
}

static void render_cruise_title(const GuiLayoutRenderCtx &ctx, const void *)
{
  gdispFillString(72, 4, "CRUISE", ctx.render.font20, GFX_AMBER_YEL, GFX_BLACK);
}

static GuiLayoutItem item(uint8_t group, GuiLayoutRenderFn render, const void *config = nullptr)
{
  GuiLayoutItem out = {};
  out.kind = GUI_LAYOUT_ITEM_CUSTOM;
  out.render_step = group;
  out.render = render;
  out.config = config;
  return out;
}

static const GuiLayoutItem kDefaultItems[] = {
    item(1, render_default_afr),
    item(1, render_default_map),
    item(1, render_ecu_error),
    item(2, render_plots),
    item(3, render_status_and_colors),
    item(4, render_gimbal),
    item(5, render_shift_warning_and_panel),
};

static const GuiLayoutItem kFuelTrimItems[] = {
    item(1, render_trim_header),
    item(1, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kTrimFfl, 8, 28, 106, 42); }),
    item(1, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kTrimRfl, 126, 28, 106, 42); }),
    item(2, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kTrimFfm, 8, 82, 106, 42); }),
    item(2, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kTrimRfm, 126, 82, 106, 42); }),
    item(3, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kTrimFfh, 8, 136, 106, 42); }),
    item(3, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kTrimRfh, 126, 136, 106, 42); }),
    item(5, render_alt_indicators),
};

static const GuiLayoutItem kWotItems[] = {
    item(1, render_wot_title),
    item(1, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kAfrMeter, 8, 30, 106, 50); }),
    item(1, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kMapMeter, 126, 30, 106, 50); }),
    item(2, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kTpsMeter, 8, 92, 106, 42); }),
    item(2, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kKnockMeter, 126, 92, 106, 42); }),
    item(3, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kTimingMeter, 8, 146, 106, 42); }),
    item(4, render_plots),
    item(5, render_alt_indicators),
};

static const GuiLayoutItem kCruiseItems[] = {
    item(1, render_cruise_title),
    item(1, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kAfrMeter, 8, 30, 106, 50); }),
    item(1, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kMapMeter, 126, 30, 106, 50); }),
    item(2, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kTrimFfl, 8, 92, 106, 42); }),
    item(2, [](const GuiLayoutRenderCtx &ctx, const void *) { render_meter_at(ctx, &kTrimRfl, 126, 92, 106, 42); }),
    item(5, render_alt_indicators),
};

static const GuiLayoutItem kPostWotItems[] = {
    item(1, render_post_wot_summary),
    item(5, render_alt_indicators),
};

static const GuiLayoutDescriptor kDefaultLayout = {GaugeLayouts::APP_LAYOUT_DEFAULT, kDefaultItems, sizeof(kDefaultItems) / sizeof(kDefaultItems[0]), 6};
static const GuiLayoutDescriptor kFuelTrimLayout = {GaugeLayouts::APP_LAYOUT_FUEL_TRIMS, kFuelTrimItems, sizeof(kFuelTrimItems) / sizeof(kFuelTrimItems[0]), 6};
static const GuiLayoutDescriptor kWotLayout = {GaugeLayouts::APP_LAYOUT_WOT, kWotItems, sizeof(kWotItems) / sizeof(kWotItems[0]), 6};
static const GuiLayoutDescriptor kCruiseLayout = {GaugeLayouts::APP_LAYOUT_CRUISE, kCruiseItems, sizeof(kCruiseItems) / sizeof(kCruiseItems[0]), 6};
static const GuiLayoutDescriptor kPostWotLayout = {GaugeLayouts::APP_LAYOUT_POST_WOT_ANALYSIS, kPostWotItems, sizeof(kPostWotItems) / sizeof(kPostWotItems[0]), 6};
}

void gauge_layouts_register()
{
  gui_layout_reset_registry();
  (void)gui_layout_register(kDefaultLayout);
  (void)gui_layout_register(kWotLayout);
  (void)gui_layout_register(kCruiseLayout);
  (void)gui_layout_register(kPostWotLayout);
  (void)gui_layout_register(kFuelTrimLayout);
  (void)gui_layout_set_active(GaugeLayouts::APP_LAYOUT_DEFAULT);
}

GaugeLayouts::SetModeStatus gauge_layouts_set_mode(uint8_t mode)
{
  switch (mode)
  {
    case GaugeLayouts::APP_LAYOUT_DEFAULT:
    case GaugeLayouts::APP_LAYOUT_WOT:
    case GaugeLayouts::APP_LAYOUT_CRUISE:
    case GaugeLayouts::APP_LAYOUT_POST_WOT_ANALYSIS:
    case GaugeLayouts::APP_LAYOUT_FUEL_TRIMS:
      return gui_layout_set_active((GuiLayoutId)mode) ? GaugeLayouts::SET_MODE_OK : GaugeLayouts::SET_MODE_BAD_VALUE;
    default:
      return GaugeLayouts::SET_MODE_BAD_VALUE;
  }
}

void gauge_layouts_update(const RuntimeState &state)
{
  bool tps_valid = false;
  bool knock_valid = false;
  bool afr_valid = false;
  bool timing_valid = false;
  bool map_valid = false;
  const float tps = source_value(state, GAUGE_SOURCE_TPS, tps_valid);
  const float knock = source_value(state, GAUGE_SOURCE_KNOCK, knock_valid);
  const float afr = source_value(state, GAUGE_SOURCE_WB_AFR, afr_valid);
  const float timing = source_value(state, GAUGE_SOURCE_TIMING, timing_valid);
  const float map = source_value(state, GAUGE_SOURCE_MAP_PSI, map_valid);

  if (!tps_valid || !knock_valid || tps < 80.0f)
    return;

  if (!g_post_wot_summary.valid || knock > g_post_wot_summary.max_knock)
  {
    g_post_wot_summary.valid = true;
    g_post_wot_summary.max_knock = knock;
    g_post_wot_summary.afr = afr_valid ? afr : 0.0f;
    g_post_wot_summary.timing = timing_valid ? timing : 0.0f;
    g_post_wot_summary.tps = tps;
    g_post_wot_summary.map = map_valid ? map : 0.0f;
    g_post_wot_summary.rpm = state.rpm;
    g_post_wot_summary.timestamp_ms = HAL_GetTick();
  }
}
