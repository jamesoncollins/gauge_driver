#include "sim_control.hpp"

#include <ctype.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

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

  if (equals_ignore_case(token, "1") || equals_ignore_case(token, "cruise"))
    mode = GaugeLayouts::APP_LAYOUT_CRUISE;
  else if (equals_ignore_case(token, "2") || equals_ignore_case(token, "wot"))
    mode = GaugeLayouts::APP_LAYOUT_WOT;
  else if (equals_ignore_case(token, "3") || equals_ignore_case(token, "post-wot") || equals_ignore_case(token, "post_wot") || equals_ignore_case(token, "postwot"))
    mode = GaugeLayouts::APP_LAYOUT_POST_WOT_ANALYSIS;
  else
    return false;

  return true;
}
}

extern "C" EMSCRIPTEN_KEEPALIVE int sim_control_set_layout_mode(uint8_t mode)
{
  return gauge_layouts_set_mode(mode) == GaugeLayouts::SET_MODE_OK ? 1 : 0;
}

extern "C" const char *sim_control_help_text(void)
{
  return "Simulator console commands:\n"
         "  help, ?                         Show this menu\n"
         "  layout <mode>, mode <mode>      Select display layout\n"
         "  sensor <scenario> [speed|tach|both]  clean/missing/dropout/extra/jitter/stall\n"
         "  sensor reset                   Restart current sensor scenario\n"
         "  sensor trace [on|off]          Dump retained CSV or toggle streaming\n"
         "\n"
         "Layout modes:\n"
         "  1 cruise        WB/MAP with fuel trims\n"
         "  2 wot           WB/MAP with fuel trims and knock/TPS plots\n"
         "  3 post-wot      Post-WOT quick analysis\n"
         "\n"
         "Mode names also accepted: cruise, wot, post-wot\n";
}

extern "C" int sim_control_line_is_help(const char *line)
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

  return equals_ignore_case(verb, "help") || equals_ignore_case(verb, "?");
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

  if (equals_ignore_case(verb,"sensor")) {
    char action[24]={}, channel[16]={}, trailing[2]={};
    const int count=sscanf(command+verb_len,"%23s %15s %1s",action,channel,trailing);
    if (count<1 || count>2) return 0;
    if (equals_ignore_case(action,"reset")) {
      if(count!=1) return 0;
      sim_control_sensor_reset(); return 1;
    }
    if (equals_ignore_case(action,"trace")) {
      if(count==1) sim_control_sensor_trace(-1);
      else if(equals_ignore_case(channel,"on")) sim_control_sensor_trace(1);
      else if(equals_ignore_case(channel,"off")) sim_control_sensor_trace(0);
      else return 0;
      return 1;
    }
    unsigned mask=3;
    if(count==2) {
      if(equals_ignore_case(channel,"speed")) mask=1;
      else if(equals_ignore_case(channel,"tach")) mask=2;
      else if(!equals_ignore_case(channel,"both")) return 0;
    }
    const char *names[]={"clean","missing","dropout","extra","jitter","stall"};
    for(unsigned i=0;i<6;++i)
      if(equals_ignore_case(action,names[i])) return sim_control_sensor_scenario(i,mask);
    return 0;
  }

  if (!equals_ignore_case(verb, "layout") && !equals_ignore_case(verb, "mode"))
    return 0;

  uint8_t mode = 0;
  if (!parse_layout_mode(command + verb_len, mode))
    return 0;

  return sim_control_set_layout_mode(mode);
}
