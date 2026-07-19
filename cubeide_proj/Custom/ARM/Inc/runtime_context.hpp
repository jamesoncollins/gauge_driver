#ifndef CUSTOM_ARM_RUNTIME_CONTEXT_HPP_
#define CUSTOM_ARM_RUNTIME_CONTEXT_HPP_

#ifndef __arm__
#error "THIS HEADER IS FOR ARM"
#endif

#include "cpp_main.h"
#include "../ECUK-lib/MUTII.hpp"
#include "../SwitecX12-lib/SwitecX12.hpp"
#include "HzSensorKalmanFilter.hpp"

extern MUTII ecu;
extern volatile bool ecuTxDone;
extern volatile bool ecuRxDone;
extern volatile uint32_t odo_ticks;
extern volatile bool needles_ready;
extern SwitecX12 *x12[3];

extern volatile bool acc_int_rdy;
extern volatile bool pendingInertial;
extern volatile bool i2cPendingIrq[4];
extern volatile bool bulbReadWaiting;
extern volatile button_e btnCmd;

extern HzSensorKalmanFilter<16> g_speed;
extern HzSensorKalmanFilter<16> g_tach;
extern volatile float rpm, speed;
extern uint16_t screenWidth;
extern uint16_t screenHeight;
extern uint32_t startupInitError;

extern volatile bool measure_freq;
extern int current_tone_len;
extern uint16_t tone_buffer[];
void set_tone(float f, float amp);

#endif /* CUSTOM_ARM_RUNTIME_CONTEXT_HPP_ */
