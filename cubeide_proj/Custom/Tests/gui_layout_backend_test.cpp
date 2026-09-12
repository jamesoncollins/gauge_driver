#include <cstdio>

#include "gui_layout.hpp"

namespace
{
int g_step1_render_count = 0;
int g_step2_render_count = 0;

void noop_render(const GuiLayoutRenderCtx &, const void *)
{
}

void count_step1_render(const GuiLayoutRenderCtx &, const void *)
{
  ++g_step1_render_count;
}

void count_step2_render(const GuiLayoutRenderCtx &, const void *)
{
  ++g_step2_render_count;
}

bool expect_true(bool value, const char *message)
{
  if (value)
    return true;

  std::printf("FAIL: %s\n", message);
  return false;
}
}

int main()
{
  GuiLayoutItem item = {};
  item.render_step = GUI_LAYOUT_FIRST_RENDER_STEP;
  item.render = noop_render;

  GuiLayoutItem clear_step_item = item;
  clear_step_item.render_step = GUI_LAYOUT_CLEAR_STEP;
  GuiLayoutItem out_of_range_item = item;
  out_of_range_item.render_step = 3;
  const GuiLayoutDescriptor invalid_clear_step = {10, &clear_step_item, 1, 3};
  const GuiLayoutDescriptor invalid_out_of_range_step = {11, &out_of_range_item, 1, 3};

  const GuiLayoutDescriptor first = {101, &item, 1, 3};
  const GuiLayoutDescriptor second = {202, &item, 1, 2};

  bool ok = true;
  gui_layout_reset_registry();
  ok = expect_true(!gui_layout_register(invalid_clear_step), "reject item in clear step") && ok;
  ok = expect_true(!gui_layout_register(invalid_out_of_range_step), "reject item outside render step count") && ok;
  ok = expect_true(gui_layout_register(first), "register first layout") && ok;
  ok = expect_true(gui_layout_register(second), "register second layout") && ok;
  ok = expect_true(!gui_layout_register(second), "reject duplicate layout id") && ok;
  ok = expect_true(gui_layout_active_id() == 101, "first registered layout becomes active") && ok;
  ok = expect_true(gui_layout_consume_transition(), "initial active layout reports transition") && ok;
  ok = expect_true(!gui_layout_consume_transition(), "transition is consumed once") && ok;
  ok = expect_true(!gui_layout_set_active(303), "reject unknown active layout id") && ok;
  ok = expect_true(gui_layout_set_active(202), "switch to second layout") && ok;
  ok = expect_true(gui_layout_active_id() == 202, "active id updates after switch") && ok;
  ok = expect_true(gui_layout_render_step_count() == 2, "active layout render step count") && ok;
  ok = expect_true(gui_layout_consume_transition(), "layout switch reports transition") && ok;
  ok = expect_true(gui_layout_set_active(202), "setting active layout to current id succeeds") && ok;
  ok = expect_true(!gui_layout_consume_transition(), "unchanged active layout does not report transition") && ok;

  GuiLayoutItem stepped_items[2] = {};
  stepped_items[0].render_step = 1;
  stepped_items[0].render = count_step1_render;
  stepped_items[1].render_step = 2;
  stepped_items[1].render = count_step2_render;
  const GuiLayoutDescriptor stepped = {404, stepped_items, 2, 3};
  RuntimeState state = {};
  BoardSharedData data = {};
  SharedRenderCtx render = {};

  gui_layout_reset_registry();
  ok = expect_true(gui_layout_register(stepped), "register stepped layout") && ok;
  gui_layout_render_step(state, data, render, 1);
  ok = expect_true(g_step1_render_count == 1, "step 1 renders step 1 item") && ok;
  ok = expect_true(g_step2_render_count == 0, "step 1 does not render step 2 item") && ok;
  gui_layout_render_step(state, data, render, 2);
  ok = expect_true(g_step1_render_count == 1, "step 2 does not re-render step 1 item") && ok;
  ok = expect_true(g_step2_render_count == 1, "step 2 renders step 2 item") && ok;

  return ok ? 0 : 1;
}