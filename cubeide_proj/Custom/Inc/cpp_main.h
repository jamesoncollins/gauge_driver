
#ifndef INC_MAIN_CPP_H_
#define INC_MAIN_CPP_H_

#include "platform_api.h"
#ifdef __cplusplus
#include "ugfx_widgets.h"
#endif

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C"
{
void main_cpp();
}
#else
void main_cpp(void);
#endif

/*
 * button type for bluetooth UI
 */
typedef enum
{
  BTN_OK = 0,
  BTN_U, BTN_D, BTN_L, BTN_R,
  BTN_INV = 255
}
button_e;

#ifdef __cplusplus
class ECUK;
#endif

#ifdef __cplusplus
typedef struct
{
  float rpm;
  float speed_mph;
  uint32_t elapsed_ms;
  uint32_t loop_count;
  uint32_t loop_period_ms;
  uint32_t worst_loop_period_ms;
  int gimbal_x;
  int gimbal_y;
  bool startup_init_error;
  bool warn_batt;
  bool warn_brake;
  bool warn_4ws;
  bool warn_lamp_on;
  bool warn_high_beam;
  ECUK *ecu;
  int ecu_param_tps_index;
  int ecu_param_wb_index;
  int ecu_param_map_index;
  int ecu_param_knock_index;
  flasher_t *ecu_flasher;
  button_e btn;
} PlatformSample;

typedef struct
{
  float rpm;
  float speed_mph;
  uint32_t elapsed_ms;
  uint32_t loop_count;
  uint32_t loop_period_ms;
  uint32_t worst_loop_period_ms;
  int rpm_mode;
  int gimbal_x;
  int gimbal_y;
  bool startup_init_error;
  bool warn_batt;
  bool warn_brake;
  bool warn_4ws;
  bool warn_lamp_on;
  bool warn_high_beam;
  ECUK *ecu;
  int ecu_param_tps_index;
  int ecu_param_wb_index;
  int ecu_param_map_index;
  int ecu_param_knock_index;
  flasher_t *ecu_flasher;
  button_e btn;
} RuntimeState;

typedef struct
{
  color_t *amber_ptr;
  font_t font10;
  font_t font20;
  font_t fontLCD;
  coord_t screen_width;
  coord_t screen_height;
  gImage *batt_img;
  gImage *beam_img;
  Gimball_t *gimball;
  LinePlot_t *line_plot_tps;
  int *line_plot_tps_data;
  LinePlot_t *line_plot_knock;
  int *line_plot_knock_data;
} SharedRenderCtx;
#endif

/*
 * function declarations
 */
int get_x12_ticks_speed( float  );
int get_x12_ticks_rpm( float  );
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA_to_SPI(TIM_HandleTypeDef *htim, const uint32_t *pData, uint16_t Length);

#if defined(CUSTOM_PLATFORM_ARM) && defined(__cplusplus)
class PI4IOE5V6416;
class SwitecX12;
void platform_poll_bulb_inputs(PI4IOE5V6416 &ioexp_screen, uint16_t &bulbVals);
void platform_process_ble_and_lowrate(uint32_t &timerLED, uint32_t &loopCnt, uint32_t loopPeriod, uint32_t worstLoopPeriod);
void platform_drain_bt_budget();
void platform_update_inertial(float cosPitch, float sinPitch);
void platform_maybe_usb_print(uint32_t &timerPrint, int &logBufInd, char *logBuf, int bufLen);
bool platform_should_exit(uint32_t &timerIGN);
void platform_update_loop_diag(uint32_t &loopCnt, uint32_t &loopPeriod, uint32_t &worstLoopPeriod, uint32_t &timerLoop);
#endif


/*
 * #defnes and constants that control operation
 */

// main loop timers
#define SAMPLE_TIME_MS_LED       100
#define SAMPLE_TIME_MS_PRINT     50
#define TARGET_FPS               20
#define SAMPLE_TIME_MS_DRAW     (1000/TARGET_FPS) // it takes 60ms to refresh the screen
                                                   // with -O2 you can draw in about 10.
                                                   // so 70ms seems to be ablout the best you can do here
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

/*
 * RPM/Shift alert points
 */
const int RPM_ALERT_RESET = 5500; // soft alert OFF, provides some hysteresis
const int RPM_ALERT_INIT = 5700; // soft alert / early warn
const int RPM_ALERT_FINAL = 6500; // SHIFT


/*
 * dots per inch for this screen
 */
const int DPI = 240. / 1.4456693; //166
const int DPMM = 240. / 36.72; // 6.53 dots per mm

/*
 * gauge measurement settings
 */
const float MPH_PER_HZ = ( 0.8425872f * 1.015625f ); //(1.11746031667) //( 1.07755102 )
const float  RPM_PER_HZ = ( 20. ); // 3 ticks per revolution


/*
 * frequency measurement settings for rpm and speed
 *
 * i originally measured that every 3 ticks of the speedo, the odo was stepped once.
 * and with our stepper i think a full step is actually 12 micro steps.
 * so the numbers below should be 3 and 12.  but those aren't looking right.
 * so i tweaked it.  well, i will tweak it once i get some measurements again.
 */

#define SPEED_TICKS_PER_ODO_TICK (3)
#define ODO_STEPS_PER_TICK (12/2)


#endif /* INC_MAIN_CPP_H_ */
