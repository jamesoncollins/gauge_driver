#ifndef CUSTOM_SIMULATOR_SIM_CONTROL_HPP_
#define CUSTOM_SIMULATOR_SIM_CONTROL_HPP_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int sim_control_set_layout_mode(uint8_t mode);
int sim_control_handle_line(const char *line);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_SIMULATOR_SIM_CONTROL_HPP_ */
