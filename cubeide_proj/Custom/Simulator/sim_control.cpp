#include "sim_control.hpp"

#include <ctype.h>
#include <stddef.h>
#include <string.h>

#if defined(__EMSCRIPTEN__)
#include <emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

#include "gauge_layouts.hpp"

namespace
{
bool equals_ignore_case(const char *lhs, const char *rhs)
{
  if (lhs == nullptr || rhs == nullptr)
    return false;

  while (*lhs != '\0' && *rhs != '\0')
  {
    if (tolower((unsigned char)*lhs) != tolower((unsigned char)*rhs))
      return false;
    ++lhs;
    ++rhs;
  }
  return *lhs == '\0' && *rhs == '\0';
}

const char *skip_space(const char *text)
{
  while (text != nullptr && *text != '\0' && isspace((unsigned char)*text))
    ++text;
  return text;
}

void trim_token(char *token)
{
  if (token == nullptr)
    return;

  char *end = token + strlen(token);
  while (end > token && isspace((unsigned char)*(end - 1)))
    *--end = '\0';
}

bool parse_layout_mode(const char *text, uint8_t &mode)
{
  if (text == nullptr)
    return false;

  char token[24] = {};
  const char *value = skip_space(text);
  size_t len = 0;
  while (value[len] != '\0' && !isspace((unsigned char)value[len]) && len < sizeof(token) - 1)
  {
    token[len] = value[len];
    ++len;
  }
  trim_token(token);

  if (len == 1 && token[0] >= '1' && token[0] <= '5')
  {
    mode = (uint8_t)(token[0] - '0');
    return true;
  }

  if (equals_ignore_case(token, "default"))
    mode = GaugeLayouts::APP_LAYOUT_DEFAULT;
  else if (equals_ignore_case(token, "wot"))
    mode = GaugeLayouts::APP_LAYOUT_WOT;
  else if (equals_ignore_case(token, "cruise"))
    mode = GaugeLayouts::APP_LAYOUT_CRUISE;
  else if (equals_ignore_case(token, "post-wot") || equals_ignore_case(token, "post_wot") || equals_ignore_case(token, "postwot"))
    mode = GaugeLayouts::APP_LAYOUT_POST_WOT_ANALYSIS;
  else if (equals_ignore_case(token, "fuel-trims") || equals_ignore_case(token, "fuel_trims") || equals_ignore_case(token, "trims"))
    mode = GaugeLayouts::APP_LAYOUT_FUEL_TRIMS;
  else
    return false;

  return true;
}
}

extern "C" EMSCRIPTEN_KEEPALIVE int sim_control_set_layout_mode(uint8_t mode)
{
  return gauge_layouts_set_mode(mode) == GaugeLayouts::SET_MODE_OK ? 1 : 0;
}

extern "C" int sim_control_handle_line(const char *line)
{
  const char *command = skip_space(line);
  if (command == nullptr || *command == '\0')
    return 0;

  char verb[16] = {};
  size_t verb_len = 0;
  while (command[verb_len] != '\0' && !isspace((unsigned char)command[verb_len]) && verb_len < sizeof(verb) - 1)
  {
    verb[verb_len] = command[verb_len];
    ++verb_len;
  }

  if (!equals_ignore_case(verb, "layout") && !equals_ignore_case(verb, "mode"))
    return 0;

  uint8_t mode = 0;
  if (!parse_layout_mode(command + verb_len, mode))
    return 0;

  return sim_control_set_layout_mode(mode);
}
