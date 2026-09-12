#ifndef CUSTOM_SIMULATOR_SIM_CONTROL_HPP_
#define CUSTOM_SIMULATOR_SIM_CONTROL_HPP_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int sim_control_set_layout_mode(uint8_t mode);
const char *sim_control_help_text(void);
int sim_control_line_is_help(const char *line);
int sim_control_handle_line(const char *line);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_SIMULATOR_SIM_CONTROL_HPP_ */
