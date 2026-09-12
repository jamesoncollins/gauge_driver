#include "gui_layout.hpp"

namespace
{
static constexpr std::size_t kMaxLayouts = 8;

const GuiLayoutDescriptor *g_layouts[kMaxLayouts] = {};
std::size_t g_layout_count = 0;
const GuiLayoutDescriptor *g_active_layout = nullptr;
bool g_transition_pending = false;

const GuiLayoutDescriptor *find_layout(GuiLayoutId id)
{
  for (std::size_t i = 0; i < g_layout_count; ++i)
  {
    if (g_layouts[i] != nullptr && g_layouts[i]->id == id)
      return g_layouts[i];
  }
  return nullptr;
}

bool validate_layout(const GuiLayoutDescriptor &layout)
{
  if (layout.items == nullptr || layout.item_count == 0 || layout.render_step_count <= GUI_LAYOUT_FIRST_RENDER_STEP)
    return false;

  for (std::size_t i = 0; i < layout.item_count; ++i)
  {
    const GuiLayoutItem &item = layout.items[i];
    if (item.render == nullptr)
      return false;
    if (item.render_step <= GUI_LAYOUT_CLEAR_STEP || item.render_step >= layout.render_step_count)
      return false;
  }
  return true;
}
}

void gui_layout_reset_registry()
{
  for (std::size_t i = 0; i < kMaxLayouts; ++i)
    g_layouts[i] = nullptr;
  g_layout_count = 0;
  g_active_layout = nullptr;
  g_transition_pending = false;
}

bool gui_layout_register(const GuiLayoutDescriptor &layout)
{
  if (!validate_layout(layout) || find_layout(layout.id) != nullptr)
    return false;

  if (g_layout_count >= kMaxLayouts)
    return false;

  g_layouts[g_layout_count++] = &layout;
  if (g_active_layout == nullptr)
  {
    g_active_layout = &layout;
    g_transition_pending = true;
  }
  return true;
}

bool gui_layout_set_active(GuiLayoutId id)
{
  const GuiLayoutDescriptor *layout = find_layout(id);
  if (layout == nullptr)
    return false;

  if (g_active_layout != layout)
  {
    g_active_layout = layout;
    g_transition_pending = true;
  }
  return true;
}

GuiLayoutId gui_layout_active_id()
{
  return g_active_layout != nullptr ? g_active_layout->id : 0;
}

const GuiLayoutDescriptor *gui_layout_active()
{
  return g_active_layout;
}

bool gui_layout_consume_transition()
{
  const bool was_pending = g_transition_pending;
  g_transition_pending = false;
  return was_pending;
}

uint8_t gui_layout_render_step_count()
{
  if (g_active_layout == nullptr || g_active_layout->render_step_count == 0)
    return 1;
  return g_active_layout->render_step_count;
}

void gui_layout_render_step(const RuntimeState &state,
                             const BoardSharedData &data,
                             SharedRenderCtx &ctx,
                             int render_step)
{
  const GuiLayoutDescriptor *layout = g_active_layout;
  if (layout == nullptr)
    return;

  for (std::size_t i = 0; i < layout->item_count; ++i)
  {
    const GuiLayoutItem &item = layout->items[i];
    if (item.render_step != render_step || item.render == nullptr)
      continue;

    GuiLayoutRenderCtx render_ctx = {state, data, ctx, render_step};
    item.render(render_ctx, item.config);
  }
}
