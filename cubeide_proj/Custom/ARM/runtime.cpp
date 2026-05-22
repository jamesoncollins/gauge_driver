#include <array>

#include "platform_api.h"
#include "cpp_main.h"
#include "runtime_context.hpp"
#include "main_shared.h"
#if !defined(CUSTOM_PLATFORM_X86)
#include "usb_device.h"
#include "usbd_cdc_if.h"
extern "C" {
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
}
#endif
extern "C" {
#include "../MCP4725-lib/MCP4725.h"
#include "../BMI088-lib/BMI088.h"

// functions inside board_s6e63d6.h
extern bool bus_busy();
extern void setAutoClear(bool);
}
#include "filters.h"
#include "utils.h"
#include "gfx.h"
#include "ugfx_widgets.h"
#include "../Quaternion/Quaternion.hpp"
#include "../../res/mitslogoanim_128.c"
#include "../../res/brake.c"
#include "../PI4IOE5V6416/PI4IOE5V6416.hpp"
#include "../ECUK-lib/MUTII.hpp"
#include "../BTbuffer-lib/BTBuffer.hpp"
#ifdef SIM_GAUGE_SIGNALS
#include "tachTest.hpp"
#endif

extern I2C_HandleTypeDef hi2c1, hi2c3;
extern SPI_HandleTypeDef hspi1;
extern LPTIM_HandleTypeDef hlptim2; // sim signals for tach/rpm on GPIO3 / PA8
extern TIM_HandleTypeDef htim1; // speaker SPI-DMA control i think, (64MHz counting to 6399+1 = 10khz)
extern TIM_HandleTypeDef htim2; // speed/tach measurement (capture control), 64MHz / (63+1) = 1MHz
extern TIM_HandleTypeDef htim16; // handle ecu at 2khz, 64MHz counting to 31999+1
extern TIM_HandleTypeDef htim17; // stepper motor ticks.  64MHz/64.  we update the ARR on the fly to change the duty cycle.
extern UART_HandleTypeDef huart1;
extern RTC_HandleTypeDef hrtc;

uint16_t screenWidth;
uint16_t screenHeight;

BMI088 imu;
uint8_t regAddr;

uint32_t startupInitError = 0;

volatile bool ecuTxDone = false;
volatile bool ecuRxDone = false;
MUTII ecu(&huart1, &ecuTxDone, &ecuRxDone);

HzSensorKalmanFilter<16> g_speed;
HzSensorKalmanFilter<16> g_tach;
volatile float rpm, speed;

/*
 * one per i2c channel
 * used to flag if someone is waiting for an RX interrupt.
 * These should only be set 'true' by this thread (no interrupts).
 *
 * i2c1 == speedo board stuff (i.e. acceleromter, and an io expander)
 * i2c2 == nothing
 * i2c3 == display board (i.e. io expander, and the oled psu)
 */
volatile bool i2cPendingIrq[4] = {0,0,0,0};

volatile bool bulbReadWaiting = false;

// flags used by accelerometer in IT mode
volatile bool acc_int_rdy = false;       // we got exti saying data ready
volatile bool pendingInertial = false;


/*
 * frequency measurement settings for rpm and speed
 *
 * i originally measured that every 3 ticks of the speedo, the odo was stepped once.
 * and with our stepper i think a full step is actually 12 micro steps.
 * so the numbers below should be 3 and 12.  but those aren't looking right.
 * so i tweaked it.  well, i will tweak it once i get some measurements again.
 */
//#define SPEED_TICKS_PER_ODO_TICK (3)
//#define ODO_STEPS_PER_TICK (12/2)
volatile uint32_t odo_ticks = 0;

/*
 * needle and odo steppers plus
 * control variables to start the measurements.
 */
SwitecX12 *x12[3];
volatile bool needles_ready = false;
volatile bool measure_freq = false;

/*
 * holds the button value sent by bluetooth
 */
volatile button_e btnCmd = BTN_INV;

/*
 * dac tones
 */
const int MAX_TONE_LEN = 256;
int current_tone_len = MAX_TONE_LEN;
uint16_t tone_buffer[MAX_TONE_LEN];
float fs;
const float mid = 1800;
float current_freq = 0;
void set_tone( float f, float amp )
{
  if(f==current_freq)
    return;
  current_freq = f;
  fs = 64000000.0f / (htim1.Instance->ARR+1); // timer1 is a 64mhz clock counting 0 - 6399
  float n_period = (int)(fs / f);
  current_tone_len = n_period-1;
  for(int i=0; i<n_period-1; i++)
    tone_buffer[i] = (amp * std::sin( 2. * M_PI * (float)i / (n_period-1) )) + 0x1000 + mid;
}


int get_x12_ticks_speed(float speed)
{
    constexpr float MIN_MPH = 10.0f;
    constexpr float ZERO_ANGLE = 3.0f;    // degrees beyond the stopper to get to 0
    constexpr float MIN_MPH_ANGLE = 0.0f; // degrees from zero to MIN_MPH
    constexpr float DEGREES_PER_MPH = 1.35f;
    constexpr float MICROSTEPS_PER_DEGREE = 12.0f;

    float angle;

    if (speed <= 1.0f)
    {
        // Below 1 MPH, clamp to zero position
        angle = ZERO_ANGLE;
    }
    else if (speed <= MIN_MPH)
    {
        // Between 1 and MIN_MPH, hold at the minimum angle
        angle = ZERO_ANGLE + MIN_MPH_ANGLE;
    }
    else
    {
        // Map speed to angle linearly beyond MIN_MPH
        angle = ZERO_ANGLE + MIN_MPH_ANGLE + (speed - MIN_MPH) * DEGREES_PER_MPH;
    }

    // Convert angle to microsteps and round
    return static_cast<int>(angle * MICROSTEPS_PER_DEGREE + 0.5f);
}



int get_x12_ticks_rpm(float rpm)
{
    constexpr float MIN_RPM = 500.0f;
    constexpr float ZERO_ANGLE = 5.0f;    // degrees beyond the stopper to get to 0
    constexpr float MIN_RPM_ANGLE = 0.0f; // degrees from zero to MIN_RPM
    constexpr float DEGREES_PER_RPM = 22.1f / 1000.0f;
    constexpr float MICROSTEPS_PER_DEGREE = 12.0f;

    float angle;

    if (rpm <= 1.0f)
    {
        // Below 1 RPM, clamp to zero position
        angle = ZERO_ANGLE;
    }
    else if (rpm <= MIN_RPM)
    {
        // Between 1 and MIN_RPM, hold at the minimum angle
        angle = ZERO_ANGLE + MIN_RPM_ANGLE;
    }
    else
    {
        // Map rpm to angle linearly beyond MIN_RPM
        angle = ZERO_ANGLE + MIN_RPM_ANGLE + (rpm - MIN_RPM) * DEGREES_PER_RPM;
    }

    // Convert angle to microsteps and round
    return static_cast<int>(angle * MICROSTEPS_PER_DEGREE + 0.5f);
}


