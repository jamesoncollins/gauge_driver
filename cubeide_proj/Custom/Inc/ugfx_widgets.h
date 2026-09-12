/*
 * ugfx_widgets.h
 *
 *  Created on: Mar 2, 2024
 *      Author: user
 */

#ifndef INC_UGFX_WIDGETS_H_
#define INC_UGFX_WIDGETS_H_


#include "gfx.h"
#ifdef __cplusplus
#include <cstddef>
#include <cstdint>

class UgfxWidget
{
public:
  virtual ~UgfxWidget() = default;

  void setBounds(coord_t x, coord_t y, coord_t width, coord_t height);
  void setVisible(bool visible);
  void setColors(color_t primary, color_t secondary, color_t background);

  virtual void clear();
  virtual void draw() = 0;

protected:
  coord_t x_ = 0;
  coord_t y_ = 0;
  coord_t width_ = 0;
  coord_t height_ = 0;
  bool visible_ = true;
  color_t primary_ = GFX_AMBER_YEL;
  color_t secondary_ = GFX_RED;
  color_t background_ = GFX_BLACK;
};

struct UgfxMeterBand
{
  float min_value;
  float max_value;
  color_t color;
};

typedef enum
{
  UGFX_TEXT_BAR_METER_FILLED = 0,
  UGFX_TEXT_BAR_METER_MARKER,
  UGFX_TEXT_BAR_METER_BIPOLAR,
  UGFX_TEXT_BAR_METER_SEGMENT
} UgfxTextBarMeterMode;

class UgfxTextBarMeter : public UgfxWidget
{
public:
  void setBounds(coord_t x, coord_t y, coord_t width, coord_t height);
  void setColors(color_t primary, color_t secondary, color_t background);
  void configure(const char *label, const char *units,
                 float min_value, float max_value,
                 uint8_t decimals,
                 font_t label_font, font_t value_font);
  void setBands(const UgfxMeterBand *bands, std::size_t band_count);
  void setMode(UgfxTextBarMeterMode mode);
  void setReferenceValue(float reference_value);
  void setBarHeight(coord_t bar_height);
  void setSegmentSize(coord_t segment_size);
  void setValue(float value, bool valid = true);
  void draw() override;

private:
  color_t valueColor() const;
  float clampedValue() const;
  coord_t valueToBarX(float value, coord_t bar_x, coord_t inner_w) const;
  void drawBandRail(coord_t bar_x, coord_t bar_y, coord_t inner_w, coord_t inner_h, bool enabled);
  void drawFilledBar(coord_t bar_x, coord_t bar_y, coord_t bar_w, coord_t bar_h, color_t bar_color);
  void drawMarkerBar(coord_t bar_x, coord_t bar_y, coord_t bar_w, coord_t bar_h, color_t marker_color);
  void drawBipolarBar(coord_t bar_x, coord_t bar_y, coord_t bar_w, coord_t bar_h, color_t bar_color);
  void drawSegmentBar(coord_t bar_x, coord_t bar_y, coord_t bar_w, coord_t bar_h, color_t segment_color);
  void markLayoutDirty();
  void updateLayout();
  void updateValueText();

  static constexpr std::size_t kMaxCachedSegments = 24;

  const char *label_ = "";
  const char *units_ = "";
  float min_value_ = 0.0f;
  float max_value_ = 1.0f;
  float value_ = 0.0f;
  uint8_t decimals_ = 1;
  font_t label_font_ = nullptr;
  font_t value_font_ = nullptr;
  const UgfxMeterBand *bands_ = nullptr;
  std::size_t band_count_ = 0;
  UgfxTextBarMeterMode mode_ = UGFX_TEXT_BAR_METER_FILLED;
  float reference_value_ = 0.0f;
  coord_t bar_height_ = 7;
  coord_t segment_size_ = 0;
  bool valid_ = false;
  bool layout_dirty_ = true;
  bool value_text_dirty_ = true;
  float last_text_value_ = 0.0f;
  bool last_text_valid_ = false;
  char value_text_[16] = "";
  coord_t value_text_x_ = 0;
  coord_t bar_x_ = 0;
  coord_t bar_y_ = 0;
  coord_t bar_w_ = 0;
  coord_t bar_h_ = 0;
  coord_t bar_inner_w_ = 0;
  coord_t bar_inner_h_ = 0;
  coord_t label_y_ = 0;
  coord_t units_y_ = 0;
  coord_t segment_count_ = 0;
  coord_t segment_w_ = 0;
  coord_t segment_h_ = 0;
  coord_t segment_y_ = 0;
  coord_t segment_x_[kMaxCachedSegments] = {};
};

struct UgfxMeterMarker
{
  float value;
  bool valid;
  color_t color;
  const char *label;
};

typedef enum
{
  UGFX_MULTI_MARKER_TEXT_BOTH = 0,
  UGFX_MULTI_MARKER_TEXT_AVERAGE,
  UGFX_MULTI_MARKER_TEXT_NONE
} UgfxMultiMarkerTextMode;

class UgfxMultiMarkerMeter : public UgfxWidget
{
public:
  void setBounds(coord_t x, coord_t y, coord_t width, coord_t height);
  void setColors(color_t primary, color_t secondary, color_t background);
  void configure(const char *label, const char *units,
                 float min_value, float max_value,
                 uint8_t decimals,
                 font_t label_font, font_t value_font);
  void setBands(const UgfxMeterBand *bands, std::size_t band_count);
  void setMarkers(const UgfxMeterMarker *markers, std::size_t marker_count);
  void setTextMode(UgfxMultiMarkerTextMode text_mode);
  void setBarHeight(coord_t bar_height);
  void draw() override;

private:
  bool anyMarkerValid() const;
  coord_t valueToBarX(float value, coord_t bar_x, coord_t inner_w) const;
  void drawBandRail(bool enabled);
  void drawMarkers();
  void markLayoutDirty();
  void updateLayout();
  void updateValueText();

  const char *label_ = "";
  const char *units_ = "";
  float min_value_ = 0.0f;
  float max_value_ = 1.0f;
  uint8_t decimals_ = 1;
  font_t label_font_ = nullptr;
  font_t value_font_ = nullptr;
  const UgfxMeterBand *bands_ = nullptr;
  std::size_t band_count_ = 0;
  const UgfxMeterMarker *markers_ = nullptr;
  std::size_t marker_count_ = 0;
  UgfxMultiMarkerTextMode text_mode_ = UGFX_MULTI_MARKER_TEXT_BOTH;
  coord_t bar_height_ = 10;
  bool layout_dirty_ = true;
  bool value_text_dirty_ = true;
  char value_text_[32] = "";
  coord_t bar_x_ = 0;
  coord_t bar_y_ = 0;
  coord_t bar_w_ = 0;
  coord_t bar_h_ = 0;
  coord_t bar_inner_w_ = 0;
  coord_t bar_inner_h_ = 0;
  coord_t label_y_ = 0;
  coord_t value_text_y_ = 0;
};
#endif

void setColors(uint32_t,uint32_t,uint32_t);

/*
 *
 * TODO: detect GDISP_HARDWARE_FILLS and fill boxes isntead of drawing lines
 */
void drawBarGraph (
    int x, int y,
    int width, int height,
    float max, float min,
    float val,
    bool vert
    );

void drawVertBarGraph (
    int x, int y,
    int width, int height,
    float max, float min,
    float val
    );

void drawHorzBarGraph (
    int x, int y,
    int width, int height,
    float max, float min,
    float val
    );

bool dissolve(int x, int y, int width, int height, int iter);


typedef struct Gimball_t
{
  int r2Max, xMax, yMax;
  uint32_t peakHold_ms = 2000;
  uint32_t peakHold_ms_last = 0;
}
Gimball_t;
void drawGimball ( Gimball_t*, int x, int y, int r, int xv, int yv);


/* a routine for flashing a widget
 * dont use the flasher_fun() function, use the macro and
 * pass the function call you want to execute as the second argument.
 */
typedef struct
{
  int rate_ms;
  int last_ms;
  bool state;
}
flasher_t;
bool flasher_fun(flasher_t*);
#define flasher(flasher_struct, arg) if(flasher_fun(flasher_struct)) arg;


typedef struct LinePlot_t
{
  int *data;
  int len = -1;
  int indLatest = -1;
  int indFirst = -1;
  float scalex, scaley;
  bool isInit = false;
  uint32_t color_mode = 0; // 0 means use default, otherwise its a color
  int lineWidth = 2;
  //uint32_t color = GFX_AMBER;
}
LinePlot_t;
void linePlotInit(
    LinePlot_t*linePlot,
    int *data_buffer_ptr, int data_buffer_len,
    int width, int height, int maxVal,
    uint32_t color_mode);
void linePlotPush(LinePlot_t *, int val);
void linePlot(int x, int y, LinePlot_t *);

#endif /* INC_UGFX_WIDGETS_H_ */
