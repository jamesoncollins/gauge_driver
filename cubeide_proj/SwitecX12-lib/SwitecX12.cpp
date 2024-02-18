/*
 *  SwitecX12 Arduino Library
 *  Guy Carpenter, Clearwater Software - 2017
 *
 *  Licensed under the BSD2 license, see license.txt for details.
 *
 *  All text above must be included in any redistribution.
 */

#include "SwitecX12.hpp"
#include "utils.h"

// This table defines the acceleration curve.
// 1st value is the speed step, 2nd value is delay in microseconds
// 1st value in each row must be > 1st value in subsequent row
// 1st value in last row should be == maxVel, must be <= maxVel
//static unsigned short defaultAccelTable[][2] =
//{
//{ 20, 1200 },
//{ 50, 700 },
//{ 100, 400 },
//};
static unsigned short defaultAccelTable[][2] =
{
{ 20, 1400 },
{ 50, 700 },
{ 100, 400 },
{ 150, 300 },
{ 300, 200 } };

//static unsigned short defaultAccelTable[][2] =
//{
//{ 20, 800 },
//{ 50, 400 },
//{ 100, 200 },
//{ 150, 150 },
//{ 300, 90 } };

const int stepPulseMicrosec = 1;
const int resetStepMicrosec = 300;
#define DEFAULT_ACCEL_TABLE_SIZE (sizeof(defaultAccelTable)/sizeof(*defaultAccelTable))

#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET

// microseconds since program exec
inline uint64_t micros ()
{
  return get_ticks_us();
  //return DWT->CYCCNT * US_PER_SYS_TICK;
}

inline void delay(uint32_t us)
{
  DWT_Delay(us);
}

SwitecX12::SwitecX12 (uint32_t steps,
		      GPIO_TypeDef* portStep, int pinStep,
		      GPIO_TypeDef* portDir, int pinDir)
{
  this->steps = steps;
  this->pinStep = pinStep;
  this->pinDir = pinDir;
  this->portStep = portStep;
  this->portDir = portDir;

  HAL_GPIO_WritePin (portStep, pinStep, LOW);
  HAL_GPIO_WritePin (portDir, pinDir, LOW);

  dir = 0;
  vel = 0;
  stopped = true;
  currentStep = 0;
  targetStep = 0;

  accelTable = defaultAccelTable;
  maxVel = defaultAccelTable[DEFAULT_ACCEL_TABLE_SIZE - 1][0]; // last value in table.
}

void SwitecX12::step (int dir)
{
  HAL_GPIO_WritePin (portDir, pinDir, dir > 0 ? LOW : HIGH);
  //digitalWrite(13, vel == maxVel ? HIGH : LOW);
  HAL_GPIO_WritePin (portStep, pinStep, HIGH);
  delay (stepPulseMicrosec);
  HAL_GPIO_WritePin (portStep, pinStep, LOW);
  currentStep += dir;
}

void SwitecX12::stepTo (uint32_t position)
{
  int count;
  int dir;
  if (position > currentStep)
  {
    dir = 1;
    count = position - currentStep;
  }
  else
  {
    dir = -1;
    count = currentStep - position;
  }
  for (int i = 0; i < count; i++)
  {
    step (dir);
    delay (resetStepMicrosec);
  }
}

void SwitecX12::zero ()
{
  currentStep = steps - 1;
  stepTo (0);
  targetStep = 0;
  vel = 0;
  dir = 0;
}

void SwitecX12::advance ()
{
  // detect stopped state
  if (currentStep == targetStep && vel == 0)
  {
    stopped = true;
    dir = 0;
    time0 = micros ();
    return;
  }

  // if stopped, determine direction
  if (vel == 0)
  {
    dir = currentStep < targetStep ? 1 : -1;
    // do not set to 0 or it could go negative in case 2 below
    vel = 1;
  }

  step (dir);

  // determine delta, number of steps in current direction to target.
  // may be negative if we are headed away from target
  int delta = dir > 0 ? targetStep - currentStep : currentStep - targetStep;

  if (delta > 0)
  {
    // case 1 : moving towards target (maybe under accel or decel)
    if (delta < vel)
    {
      // time to declerate
      vel = delta;
    }
    else if (vel < maxVel)
    {
      // accelerating
      vel++;
    }
    else
    {
      // at full speed - stay there
    }
  }
  else
  {
    // case 2 : at or moving away from target (slow down!)
    vel--;
  }

  // vel now defines delay
  unsigned char i = 0;
  // this is why vel must not be greater than the last vel in the table.
  while (accelTable[i][0] < vel)
  {
    i++;
  }
  microDelay = accelTable[i][1];
  time0 = micros ();
}

void SwitecX12::setPosition (uint32_t pos)
{
  // pos is unsigned so don't need to check for <0
  if (pos >= steps)
    pos = steps - 1;
  targetStep = pos;
  if (stopped)
  {
    // reset the timer to avoid possible time overflow giving spurious deltas
    stopped = false;
    time0 = micros ();
    microDelay = 0;
  }
}

void SwitecX12::update ()
{
  if (!stopped)
  {
    unsigned long delta = micros () - time0;
    if (delta >= microDelay)
    {
      advance ();
    }
  }
}
