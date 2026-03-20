
#ifndef INC_MAIN_CPP_H_
#define INC_MAIN_CPP_H_

#include "platform_api.h"

extern "C"
{
void main_cpp();
}

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

/*
 * function declarations
 */
int get_x12_ticks_speed( float  );
int get_x12_ticks_rpm( float  );
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA_to_SPI(TIM_HandleTypeDef *htim, const uint32_t *pData, uint16_t Length);


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


/*
 * Decalre globals that are later defined in cpp_main.cpp
 */
#include "../ECUK-lib/MUTII.hpp"
extern MUTII ecu;
extern volatile bool ecuTxDone;
extern volatile bool ecuRxDone;
extern volatile uint32_t odo_ticks;
extern volatile bool needles_ready;
#include "../SwitecX12-lib/SwitecX12.hpp"
extern SwitecX12 *x12[3];

extern volatile bool acc_int_rdy;
extern volatile bool pendingInertial;

extern volatile bool i2cPendingIrq[4];

extern volatile button_e btnCmd;

#include "HzSensorKalmanFilter.hpp"
extern HzSensorKalmanFilter<16> g_speed;
extern HzSensorKalmanFilter<16> g_tach;



#endif /* INC_MAIN_CPP_H_ */
