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
static const uint32_t defaultAccelTable[][2] =
{
{  200, 1000 }, // 200d/s, 2400 ticks per second. 416us/tick
{  250,  900 },
{  300,  750 },
{  350,  700 },
{ 1000,  700 },
//{ 450,  500 },
//{ 500,  400 },
};


const int stepPulseTicks = 1;
const int resetStepTicks = defaultAccelTable[0][1]*10;
#define DEFAULT_ACCEL_TABLE_SIZE (sizeof(defaultAccelTable)/sizeof(*defaultAccelTable))

#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET

// clock ticks since startm overflows at 32-bits
inline uint32_t elapsed_us ()
{
  //return get_ticks_us();
  //return get_ticks_32();
  return get_us_32();
}

inline void delay(uint32_t us)
{
  DWT_Delay(us);
}

bool SwitecX12::atTarget()
{
  return (targetStep==currentStep) && (targetStepNext==targetStep) && stopped;
}

SwitecX12::SwitecX12 (uint32_t steps,
		      GPIO_TypeDef* portStep, int pinStep,
		      GPIO_TypeDef* portDir, int pinDir,
		      const uint32_t _accelTable[][2], int tableLen,
		      bool reverseDir)
{
  this->steps = steps;
  this->pinStep = pinStep;
  this->pinDir = pinDir;
  this->portStep = portStep;
  this->portDir = portDir;
  this->reverseDir = reverseDir;

  HAL_GPIO_WritePin (portStep, pinStep, LOW);
  HAL_GPIO_WritePin (portDir, pinDir, LOW);

  dir = 0;
  vel = 0;
  stopped = true;
  currentStep = 0;
  targetStep = 0;
  targetStepNext = 0;
  worstMiss = 0;

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
  HAL_GPIO_WritePin (portDir, pinDir, (dir > 0) && !reverseDir ? LOW : HIGH);
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

  targetStep = position;
  targetStepNext = position;
}

void SwitecX12::reset()
{
  currentStep = 0;
  targetStep = 0;
  targetStepNext = 0;
  vel = 0;
  dir = 0;
  stopped = true;
}

void SwitecX12::zero ()
{
  currentStep = steps - 1;
  stepTo (0);
  targetStep = 0;
  targetStepNext = 0;
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
    time0 = elapsed_us ();
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
      vel = vel;
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
  time0 = elapsed_us ();
}

void SwitecX12::setPosition (uint32_t pos)
{
  targetStepNext = pos;
}

void SwitecX12::setPositionNow ()
{
  uint32_t pos = targetStepNext;

  // pos is unsigned so don't need to check for <0
  if (pos >= steps)
    pos = steps - 1;
  targetStep = pos;
  targetStepNext = pos;
  if (stopped)
  {
    // reset the timer to avoid possible time overflow giving spurious deltas
    stopped = false;
    time0 = elapsed_us ();
    microDelay = 0;
  }

}

uint32_t SwitecX12::getTargetPosition()
{
  return targetStep;
}

void SwitecX12::update ()
{

  /*
   * check if the user had given us a new target pos or not.
   * if so, update that now before we check on moving the needles.
   */
  if(targetStepNext != targetStep)
    setPositionNow();


  if (!stopped)
  {
    uint32_t delta = elapsed_us () - time0;
    if (delta >= microDelay)
    {
      uint32_t over = delta - microDelay;
      if(over > worstMiss)
        worstMiss = over;
      advance ();
    }
  }

}
