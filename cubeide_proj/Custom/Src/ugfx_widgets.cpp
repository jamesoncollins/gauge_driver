#include "platform_api.h"

#include "ugfx_widgets.h"
#include "gfx.h"
#include "math.h"
#include <cstdio>

uint32_t COLOR_PRIMARY = GFX_AMBER_YEL;
uint32_t COLOR_SECONDARY = GFX_RED;
uint32_t COLOR_BG = GFX_BLACK;

void setColors(uint32_t primary, uint32_t secondary, uint32_t background)
{
  COLOR_PRIMARY = primary;
  COLOR_SECONDARY = secondary;
  COLOR_BG = background;
}

namespace
{
float clamp_float(float value, float min_value, float max_value)
{
  if (value < min_value)
    return min_value;
  if (value > max_value)
    return max_value;
  return value;
}

coord_t clamp_coord(coord_t value, coord_t min_value, coord_t max_value)
{
  if (value < min_value)
    return min_value;
  if (value > max_value)
    return max_value;
  return value;
}

coord_t decimal_aligned_x(const char *text, font_t font, coord_t decimal_x, coord_t fallback_x)
{
  if (text == nullptr || font == nullptr)
    return fallback_x;

  char prefix[16];
  std::size_t prefix_len = 0;
  while (text[prefix_len] != '\0' && text[prefix_len] != '.' && prefix_len < (sizeof(prefix) - 1))
  {
    prefix[prefix_len] = text[prefix_len];
    ++prefix_len;
  }

  if (text[prefix_len] != '.')
    return fallback_x;

  prefix[prefix_len] = '\0';
  return decimal_x - gdispGetStringWidth(prefix, font);
}
}

void UgfxWidget::setBounds(coord_t x, coord_t y, coord_t width, coord_t height)
{
  x_ = x;
  y_ = y;
  width_ = width;
  height_ = height;
}

void UgfxWidget::setVisible(bool visible)
{
  visible_ = visible;
}

void UgfxWidget::setColors(color_t primary, color_t secondary, color_t background)
{
  primary_ = primary;
  secondary_ = secondary;
  background_ = background;
}

void UgfxWidget::clear()
{
  if (width_ <= 0 || height_ <= 0)
    return;

  gdispFillArea(x_, y_, width_, height_, background_);
}

void UgfxTextBarMeter::configure(const char *label, const char *units,
                                 float min_value, float max_value,
                                 uint8_t decimals,
                                 font_t label_font, font_t value_font)
{
  label_ = label != nullptr ? label : "";
  units_ = units != nullptr ? units : "";
  min_value_ = min_value;
  max_value_ = max_value;
  decimals_ = decimals;
  label_font_ = label_font;
  value_font_ = value_font;
}

void UgfxTextBarMeter::setBands(const UgfxMeterBand *bands, std::size_t band_count)
{
  bands_ = bands;
  band_count_ = band_count;
}

void UgfxTextBarMeter::setMode(UgfxTextBarMeterMode mode)
{
  mode_ = mode;
}

void UgfxTextBarMeter::setReferenceValue(float reference_value)
{
  reference_value_ = reference_value;
}

void UgfxTextBarMeter::setBarHeight(coord_t bar_height)
{
  bar_height_ = bar_height > 2 ? bar_height : 3;
}

void UgfxTextBarMeter::setSegmentSize(coord_t segment_size)
{
  segment_size_ = segment_size > 1 ? segment_size : 0;
}

void UgfxTextBarMeter::setValue(float value, bool valid)
{
  value_ = value;
  valid_ = valid;
}

color_t UgfxTextBarMeter::valueColor() const
{
  for (std::size_t i = 0; i < band_count_; ++i)
  {
    const UgfxMeterBand &band = bands_[i];
    if (value_ >= band.min_value && value_ <= band.max_value)
      return band.color;
  }

  return primary_;
}

float UgfxTextBarMeter::clampedValue() const
{
  if (max_value_ <= min_value_)
    return min_value_;

  return clamp_float(value_, min_value_, max_value_);
}

coord_t UgfxTextBarMeter::valueToBarX(float value, coord_t bar_x, coord_t inner_w) const
{
  const float range = max_value_ - min_value_;
  if (range <= 0.0f || inner_w <= 0)
    return bar_x;

  const float clamped = clamp_float(value, min_value_, max_value_);
  const float percent = (clamped - min_value_) / range;
  return bar_x + 1 + clamp_coord((coord_t)round((float)inner_w * percent), 0, inner_w);
}

void UgfxTextBarMeter::drawBandRail(coord_t bar_x, coord_t bar_y, coord_t inner_w, coord_t inner_h, bool enabled)
{
  if (!enabled || bands_ == nullptr || band_count_ == 0)
  {
    gdispFillArea(bar_x + 1, bar_y + 1, inner_w, inner_h, GFX_GRAY);
    return;
  }

  for (std::size_t i = 0; i < band_count_; ++i)
  {
    const coord_t start_x = valueToBarX(bands_[i].min_value, bar_x, inner_w);
    const coord_t end_x = valueToBarX(bands_[i].max_value, bar_x, inner_w);
    const coord_t segment_w = end_x > start_x ? end_x - start_x : 1;
    gdispFillArea(start_x, bar_y + 1, segment_w, inner_h, bands_[i].color);
  }
}

void UgfxTextBarMeter::drawFilledBar(coord_t bar_x, coord_t bar_y, coord_t bar_w, coord_t bar_h, color_t bar_color)
{
  const coord_t inner_w = bar_w - 2;
  const coord_t inner_h = bar_h - 2;
  const coord_t fill_x = valueToBarX(clampedValue(), bar_x, inner_w);
  const coord_t fill_w = fill_x - (bar_x + 1);
  if (fill_w > 0)
    gdispFillArea(bar_x + 1, bar_y + 1, fill_w, inner_h, bar_color);
}

void UgfxTextBarMeter::drawMarkerBar(coord_t bar_x, coord_t bar_y, coord_t bar_w, coord_t bar_h, color_t marker_color)
{
  const coord_t inner_w = bar_w - 2;
  const coord_t inner_h = bar_h - 2;
  drawBandRail(bar_x, bar_y, inner_w, inner_h, valid_);

  const coord_t marker_x = valueToBarX(clampedValue(), bar_x, inner_w);
  const coord_t tick_x = clamp_coord(marker_x - 1, bar_x + 1, bar_x + bar_w - 3);
  gdispFillArea(tick_x, bar_y - 2, 3, bar_h + 4, marker_color);
}

void UgfxTextBarMeter::drawBipolarBar(coord_t bar_x, coord_t bar_y, coord_t bar_w, coord_t bar_h, color_t bar_color)
{
  const coord_t inner_w = bar_w - 2;
  const coord_t inner_h = bar_h - 2;
  const coord_t zero_x = valueToBarX(reference_value_, bar_x, inner_w);
  const coord_t value_x = valueToBarX(clampedValue(), bar_x, inner_w);
  const coord_t start_x = value_x < zero_x ? value_x : zero_x;
  const coord_t end_x = value_x < zero_x ? zero_x : value_x;
  const coord_t fill_w = end_x - start_x;

  if (fill_w > 0)
    gdispFillArea(start_x, bar_y + 1, fill_w, inner_h, bar_color);

  gdispFillArea(clamp_coord(zero_x - 1, bar_x + 1, bar_x + bar_w - 2), bar_y - 1, 2, bar_h + 2, GFX_SILVER);
}

void UgfxTextBarMeter::drawSegmentBar(coord_t bar_x, coord_t bar_y, coord_t bar_w, coord_t bar_h, color_t segment_color)
{
  const coord_t inner_w = bar_w - 2;
  const coord_t inner_h = bar_h - 2;
  if (inner_w <= 0 || inner_h <= 0)
    return;

  const coord_t requested_segment_w = segment_size_ > 0 ? segment_size_ : inner_h;
  const coord_t segment_w = clamp_coord(requested_segment_w, 1, inner_w);
  const coord_t vertical_margin = inner_h > 2 ? 1 : 0;
  const coord_t max_segment_h = inner_h - (vertical_margin * 2);
  const coord_t segment_h = clamp_coord(segment_w, 1, max_segment_h);
  const coord_t min_segment_gap = 2;
  const color_t inactive_color = valid_ ? HTML2COLOR(0x202020) : GFX_GRAY;

  gdispFillArea(bar_x + 1, bar_y + 1, inner_w, inner_h, background_);

  const coord_t segment_count = clamp_coord((inner_w + min_segment_gap) / (segment_w + min_segment_gap), 1, inner_w);
  if (segment_count <= 0)
    return;

  const coord_t used_w = segment_count * segment_w;
  const coord_t free_w = inner_w > used_w ? inner_w - used_w : 0;
  const coord_t gap_slots = segment_count + 1;
  const coord_t base_gap = free_w / gap_slots;
  const coord_t extra_gap = free_w % gap_slots;
  const coord_t draw_y = bar_y + 1 + ((inner_h - segment_h) >> 1);

  auto segment_x = [&](coord_t index) -> coord_t {
    const coord_t leading_gap = base_gap + (extra_gap > 0 ? 1 : 0);
    const coord_t prior_extra_gaps = clamp_coord(extra_gap - 1, 0, index);
    return bar_x + 1 + leading_gap + index * (segment_w + base_gap) + prior_extra_gaps;
  };

  for (coord_t i = 0; i < segment_count; ++i)
    gdispFillArea(segment_x(i), draw_y, segment_w, segment_h, inactive_color);

  const float range = max_value_ - min_value_;
  const float percent = range > 0.0f ? (clampedValue() - min_value_) / range : 0.0f;
  const coord_t active_index = clamp_coord((coord_t)round(percent * (float)(segment_count - 1)), 0, segment_count - 1);
  gdispFillArea(segment_x(active_index), draw_y, segment_w, segment_h, segment_color);
}

void UgfxTextBarMeter::draw()
{
  if (!visible_)
    return;

  clear();

  if (width_ <= 0 || height_ <= 0)
    return;

  const color_t text_color = valid_ ? primary_ : GFX_GRAY;
  const color_t bar_color = valid_ ? valueColor() : GFX_GRAY;
  const color_t frame_color = valid_ ? primary_ : GFX_GRAY;

  char value_text[16];
  const int value_width = (int)decimals_ + (decimals_ > 0 ? 4 : 3);
  (void)std::snprintf(value_text, sizeof(value_text), "%*.*f", value_width, (int)decimals_, value_);

  if (width_ < 6 || height_ < 12)
    return;

  const coord_t bar_x = x_ + 2;
  const coord_t bar_w = width_ - 4;
  const coord_t requested_bar_h = bar_height_ > 2 ? bar_height_ : 3;
  const coord_t bar_h = requested_bar_h < (height_ - 4) ? requested_bar_h : (height_ - 4);
  const coord_t bar_y = y_ + 2;

  if (value_font_ != nullptr)
    gdispFillString(decimal_aligned_x(value_text, value_font_, x_ + 122, x_), bar_y + bar_h - 2, value_text, value_font_, text_color, background_);

  gdispDrawBox(bar_x, bar_y, bar_w, bar_h, frame_color);

  switch (mode_)
  {
    case UGFX_TEXT_BAR_METER_MARKER:
      drawMarkerBar(bar_x, bar_y, bar_w, bar_h, bar_color);
      break;
    case UGFX_TEXT_BAR_METER_BIPOLAR:
      drawBipolarBar(bar_x, bar_y, bar_w, bar_h, bar_color);
      break;
    case UGFX_TEXT_BAR_METER_SEGMENT:
      drawSegmentBar(bar_x, bar_y, bar_w, bar_h, bar_color);
      break;
    case UGFX_TEXT_BAR_METER_FILLED:
    default:
      drawFilledBar(bar_x, bar_y, bar_w, bar_h, bar_color);
      break;
  }

  if (label_font_ != nullptr)
  {
    gdispFillString(x_ + 2, bar_y + bar_h + 2, label_, label_font_, text_color, background_);
    gdispFillString(x_ + 2, bar_y + bar_h + 22, units_, label_font_, text_color, background_);
  }
}
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
    )
{
  int b = 1; // boarder gap
  int t = 1; // border thickness
  int g = b + t; // the gap from the outer edge of the box to the edge of the bar
  bool alert = val>=max ? true : false;
  val = val > max ? max : val;
  val = val < min ? min : val;
  float percent = (val - min) / (max - min);

  int barMax = vert ? height - 2 * g : width - 2 * g;
  int barMag = round (barMax * percent);

  for (int i = 0; i < t; i++)
    gdispDrawBox(
        x + i,
        y + i,
        width - i * 2,
        height - i * 2,
        COLOR_PRIMARY);

  if(vert)
  {
    gdispFillArea(
        x + g,
        y + g + (barMax - barMag),
        width - 2 * g,
        barMag,
        alert ? COLOR_SECONDARY : COLOR_PRIMARY );
  }
  else
  {
    gdispFillArea(
        x + g,
        y + g,
        barMag,
        height - 2 * g,
        alert ? COLOR_SECONDARY : COLOR_PRIMARY );
  }
}

void drawVertBarGraph (
    int x, int y,
    int width, int height,
    float max, float min,
    float val
    )
{
  drawBarGraph(x,y,width,height,max,min,val,true);
}

void drawHorzBarGraph (
    int x, int y,
    int width, int height,
    float max, float min,
    float val
    )
{
  drawBarGraph(x,y,width,height,max,min,val,false);
}

//bool dissolve(int x, int y, int width, int height, int iter)
//{
//  for(int i=0; i<width; i+=iter)
//  {
//    for(int j=0; j<height; j+=iter)
//    {
//      gdispFillArea(x+i, y+j, iter, iter, GFX_BLACK);
//    }
//  }
//}


/*
 * xy, coordinates of center
 * r, radius of outtermost ring
 * xy and yv, wher ethe gimbal is
 */
void drawGimball ( Gimball_t *gimball, int x, int y, int r, int xv, int yv)
{
  const int w = 1;
  gdispDrawThickLine( x, y, x+r, y+0, COLOR_PRIMARY, w, false);
  gdispDrawThickLine( x, y, x+0, y+r, COLOR_PRIMARY, w, false);
  gdispDrawThickLine( x, y, x-r, y+0, COLOR_PRIMARY, w, false);
  gdispDrawThickLine( x, y, x-0, y-r, COLOR_PRIMARY, w, false);
  while(r>(w<<2))
  {
    gdispDrawCircle (x, y, r, COLOR_PRIMARY);
    r = r>>1;
  }

  int r2 = xv*xv+yv*yv;
  if(r2>gimball->r2Max || HAL_GetTick() - gimball->peakHold_ms_last > gimball->peakHold_ms)
  {
    gimball->r2Max = r2;
    gimball->xMax = xv;
    gimball->yMax = yv;
    gimball->peakHold_ms_last = HAL_GetTick();
  }

  int ball_r = w*8;
  gdispFillDualCircle (x+gimball->xMax, y+gimball->yMax, ball_r, GFX_ORANGE, ball_r>>1, GFX_BLACK);
  gdispFillCircle (x+xv, y+yv, ball_r, COLOR_SECONDARY);

}

bool flasher_fun(flasher_t *flasher)
{
 int now = HAL_GetTick();
  if( now >= flasher->last_ms+flasher->rate_ms )
  {
    flasher->last_ms = now;
    flasher->state = !flasher->state;
  }
  return flasher->state;

}




void linePlotInit(
    LinePlot_t*linePlot,
    int *data_buffer_ptr, int data_buffer_len,
    int width, int height, int maxVal,
    uint32_t color_mode)
{
  linePlot->data = data_buffer_ptr;
  linePlot->len = data_buffer_len;
  linePlot->indFirst = 0;
  linePlot->indLatest = 0;
  linePlot->scalex = (float)width / (float)(data_buffer_len-1);
  linePlot->scaley = (float)height / (float)maxVal;
  linePlot->isInit = true;
  linePlot->color_mode = color_mode;

  for(int i=0; i<data_buffer_len; i++)
    data_buffer_ptr[i] = 0;
}

void linePlotPush(LinePlot_t *linePlot, int val)
{
  if(!linePlot->isInit)
    return;

  linePlot->indLatest++;
  linePlot->indLatest =  (linePlot->indLatest>=linePlot->len) ? 0 : linePlot->indLatest;
  if(linePlot->indLatest == linePlot->indFirst)
    linePlot->indFirst = (linePlot->indLatest+1) % linePlot->len;
  linePlot->data[linePlot->indLatest] = val;
}

void linePlot(int x, int y, LinePlot_t *linePlot)
{
  if(!linePlot->isInit)
    return;
  uint32_t color = COLOR_PRIMARY;
  if(linePlot->color_mode )
    color = linePlot->color_mode ;
  int ind = linePlot->indFirst;
  int from = linePlot->data[ind];
  for(int i=1; i<linePlot->len; i++)
  {
    ind = (ind+1) % linePlot->len;
    int to = linePlot->data[ind];
    gdispDrawThickLine(
        x+i*linePlot->scalex,     y-from*linePlot->scaley,
        x+(i+1)*linePlot->scalex, y-to*linePlot->scaley,
        color,
        linePlot->lineWidth,
        false);
    from = to;
  }
}
