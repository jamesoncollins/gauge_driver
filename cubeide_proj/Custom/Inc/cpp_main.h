#ifndef INC_MAIN_CPP_H_
#define INC_MAIN_CPP_H_

#include "board_model.hpp"

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C"
{
void main_cpp();
void main_cpp_step();
void main_cpp_shutdown();
}
#else
void main_cpp(void);
void main_cpp_step(void);
void main_cpp_shutdown(void);
#endif

/*
 * Optional build flags used for diagnostics/test behaviors.
 * Runtime tunables are configured in build_config.
 */
//#define SWEEP_GAUGES  // sweep needles forever
//#define SIM_GAUGES       // generate simulated rpm and mph

// WARNING
// this feature shares a pin with teh PCD backlight.  i dont have any code yet to deconflict this.
//#define SIM_GAUGE_SIGNALS // use lptim, on gpio3, as a pwm signal you can connect to the tach or speed inputs

/*
 * USB-based diagnostics
 *
 * Print to USB: what it sounds like, prints text to the  USB port
 * USB_DISPLAY: sends the display buffer over USB, use the script in ./python to view it.
 *
 * Note, you can use both of these at the same time.
 */
//#define PRINT_TO_USB
//#define USB_DISPLAY

/*
 * Optional debug / diagnostic print on the screen
 */
//#define DIAG_SQUARE

#endif /* INC_MAIN_CPP_H_ */
