#ifndef SwitecX12_h
#define SwitecX12_h

#include <stddef.h>
#include <atomic>
#include <stm32wb55xx.h>

class SwitecX12
{
public:
  int pinStep;
  int pinDir;
  GPIO_TypeDef* portStep;
  GPIO_TypeDef* portDir;
  bool reverseDir;
  uint32_t currentStep;      // step we are currently at
  uint32_t targetStep;       // target we are moving to
  uint32_t targetStepNext;   // queued target step, the update function will actually set it
  uint32_t steps;            // total steps available
  uint32_t time0;           // time when we entered this state
  uint32_t microDelay;       // microsecs until next state
  volatile int32_t worstMiss;
  const uint32_t (*accelTable)[2]; // accel table can be modified.
  int maxVel;           // fastest vel allowed
  int vel;              // steps travelled under acceleration
  int dir;                      // direction -1,0,1
  bool stopped;               // true if stopped
  SwitecX12 (uint32_t steps,
	     GPIO_TypeDef*, int,
	     GPIO_TypeDef*, int,
	     const uint32_t accelTable[][2] = NULL,
	     int tableLen = 0,
	     bool reverse_step = false);
  bool atTarget();

  /*
   * step right now
   */
  void stepNow (int dir);

public:
  /*
   * step to 0, right now
   */
  void zero ();

  // like zero() but doesnt actually move
  void reset ();

  /*
   * step to position, right now
   */
  void stepTo (uint32_t position);

  /*
   * set desired position, every call to update() will
   * move us to closer to that desired state
   */
  void setPosition (uint32_t pos);
  uint32_t getTargetPosition();

  /*
   * step the motor now if currentStep != targetStep
   *
   * returns an amount of time, in microseonds, that you should try to
   * call this function again after.
   */
  uint32_t update ();

private:
  void setPositionNow ();
  void advance ();
  void step (int dir); // perform a step, but dont UNstep
  void stepEnd ();      // call this often to check if a step should end
  bool inStep = false;
  uint32_t steppedAt;
};

#endif
