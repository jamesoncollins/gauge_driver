#ifndef INC_GAUGE_LAYOUTS_HPP_
#define INC_GAUGE_LAYOUTS_HPP_

#include <cstdint>

#include "board_model.hpp"

namespace GaugeLayouts
{
enum AppLayoutId : uint16_t
{
  APP_LAYOUT_DEFAULT = 1,
  APP_LAYOUT_WOT = 2,
  APP_LAYOUT_CRUISE = 3,
  APP_LAYOUT_POST_WOT_ANALYSIS = 4,
  APP_LAYOUT_FUEL_TRIMS = 5
};

enum SetModeStatus : uint8_t
{
  SET_MODE_OK = 0,
  SET_MODE_BAD_VALUE = 1
};
}

void gauge_layouts_register();
GaugeLayouts::SetModeStatus gauge_layouts_set_mode(uint8_t mode);
void gauge_layouts_update(const RuntimeState &state);

#endif /* INC_GAUGE_LAYOUTS_HPP_ */
