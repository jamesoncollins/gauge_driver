#ifndef SwitecX12_h
#define SwitecX12_h

#include "main.h"

class SwitecX12
{
public:
  int pinStep;
  int pinDir;
  GPIO_TypeDef* portStep;
  GPIO_TypeDef* portDir;
  int currentStep;      // step we are currently at
  int targetStep;       // target we are moving to
  int steps;            // total steps available
  long time0;           // time when we entered this state
  unsigned int microDelay;       // microsecs until next state
  unsigned short (*accelTable)[2]; // accel table can be modified.
  int maxVel;           // fastest vel allowed
  int vel;              // steps travelled under acceleration
  int dir;                      // direction -1,0,1
  bool stopped;               // true if stopped
  SwitecX12 (unsigned int steps,
	     GPIO_TypeDef*, int,
	     GPIO_TypeDef*, int);

  /*
   * step right now
   */
  void step (int dir);

  /*
   * step to 0, right now
   */
  void zero ();

  /*
   * step to position, right now
   */
  void stepTo (int position);

  /*
   * set desired position, every call to update() will
   * move us to closer to that desired state
   */
  void setPosition (int pos);

  /*
   * step the motor now if currentStep != targetStep
   */
  void update ();

private:
  void advance ();
};

#endif
