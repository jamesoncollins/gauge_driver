#ifndef INC_GUI_LAYOUT_HPP_
#define INC_GUI_LAYOUT_HPP_

#include <cstddef>
#include <cstdint>

#include "board_model.hpp"

typedef uint16_t GuiLayoutId;
typedef uint16_t GuiDataSourceId;

static constexpr uint8_t GUI_LAYOUT_CLEAR_STEP = 0;
static constexpr uint8_t GUI_LAYOUT_FIRST_RENDER_STEP = 1;

enum GuiLayoutItemKind : uint8_t
{
  GUI_LAYOUT_ITEM_CUSTOM = 0,
  GUI_LAYOUT_ITEM_TEXT_BAR,
  GUI_LAYOUT_ITEM_LINE_PLOT,
  GUI_LAYOUT_ITEM_GIMBAL,
  GUI_LAYOUT_ITEM_WARNING_PANEL
};

struct GuiRect
{
  coord_t x;
  coord_t y;
  coord_t w;
  coord_t h;
};

struct GuiLayoutStyle
{
  color_t primary = GFX_AMBER_YEL;
  color_t secondary = GFX_RED;
  color_t background = GFX_BLACK;
};

struct GuiLayoutRenderCtx
{
  const RuntimeState &state;
  const BoardSharedData &data;
  SharedRenderCtx &render;
  int step;
};

typedef void (*GuiLayoutRenderFn)(const GuiLayoutRenderCtx &ctx, const void *config);

struct GuiLayoutItem
{
  GuiLayoutItemKind kind = GUI_LAYOUT_ITEM_CUSTOM;
  GuiDataSourceId source = 0;
  GuiRect bounds = {};
  GuiLayoutStyle style = {};
  int8_t layer = 0;
  uint8_t render_step = GUI_LAYOUT_FIRST_RENDER_STEP;
  GuiLayoutRenderFn render = nullptr;
  const void *config = nullptr;
};

struct GuiLayoutDescriptor
{
  GuiLayoutId id = 0;
  const GuiLayoutItem *items = nullptr;
  std::size_t item_count = 0;
  uint8_t render_step_count = 1;
};

void gui_layout_reset_registry();
bool gui_layout_register(const GuiLayoutDescriptor &layout);
bool gui_layout_set_active(GuiLayoutId id);
GuiLayoutId gui_layout_active_id();
const GuiLayoutDescriptor *gui_layout_active();
bool gui_layout_consume_transition();
uint8_t gui_layout_render_step_count();
void gui_layout_render_step(const RuntimeState &state,
                            const BoardSharedData &data,
                            SharedRenderCtx &ctx,
                            int render_step);

#endif /* INC_GUI_LAYOUT_HPP_ */
