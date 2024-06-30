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
static uint32_t ticks_per_us =  ( 64000000 * 1e-6);
static uint32_t defaultAccelTable[][2] =
{
{ 20,  (uint32_t) 1.1 * 2000 * ticks_per_us },
{ 50,  (uint32_t) 1.1 * 1000 * ticks_per_us },
{ 100, (uint32_t) 1.1 * 700  * ticks_per_us },
{ 150, (uint32_t) 1.1 * 400  * ticks_per_us },
{ 300, (uint32_t) 1.1 * 300  * ticks_per_us }
};

//static unsigned short defaultAccelTable[][2] =
//{
//{ 20, 800 },
//{ 50, 400 },
//{ 100, 200 },
//{ 150, 150 },
//{ 300, 90 } };

const int stepPulseTicks = 1 * ticks_per_us;
const int resetStepTicks = 2000 * ticks_per_us;
#define DEFAULT_ACCEL_TABLE_SIZE (sizeof(defaultAccelTable)/sizeof(*defaultAccelTable))

#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET

// clock ticks since startm overflows at 32-bits
inline uint32_t ticks ()
{
  //return get_ticks_us();
  return get_ticks_32();
}

inline void delay(uint32_t us)
{
  DWT_Delay(us);
}

bool SwitecX12::atTarget()
{
  return (targetStep==currentStep) && stopped;
}

SwitecX12::SwitecX12 (uint32_t steps,
		      GPIO_TypeDef* portStep, int pinStep,
		      GPIO_TypeDef* portDir, int pinDir,
		      uint32_t _accelTable[][2],
                      int tableLen)
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

  if(tableLen > 0)
  {
    accelTable = defaultAccelTable;
    maxVel = defaultAccelTable[tableLen-1][0];
  }
}

void SwitecX12::step (int dir)
{
  // the chip is actually active-high = CW as the pin is labeled CW-/CCW
  // but its flipped here becuase the schematic is flipped
  HAL_GPIO_WritePin (portDir, pinDir, dir > 0 ? LOW : HIGH);
  HAL_GPIO_WritePin (portStep, pinStep, HIGH);
  delay (stepPulseTicks);
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
    delay (resetStepTicks);
  }
}

void SwitecX12::reset()
{
  currentStep = 0;
  targetStep = 0;
  vel = 0;
  dir = 0;
  stopped = true;
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
    time0 = ticks ();
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
  while ( (int)accelTable[i][0] < vel)
  {
    i++;
  }
  microDelay = accelTable[i][1];
  time0 = ticks ();
}

void SwitecX12::setPosition (uint32_t pos)
{
  dontUpdate = true;
  // pos is unsigned so don't need to check for <0
  if (pos >= steps)
    pos = steps - 1;
  targetStep = pos;
  if (stopped)
  {
    // reset the timer to avoid possible time overflow giving spurious deltas
    stopped = false;
    time0 = ticks ();
    microDelay = 0;
  }
  dontUpdate = false;
}

void SwitecX12::update ()
{
  if (!dontUpdate && !stopped)
  {
    unsigned long delta = ticks () - time0;
    if (delta >= microDelay)
    {
      advance ();
    }
  }
}
