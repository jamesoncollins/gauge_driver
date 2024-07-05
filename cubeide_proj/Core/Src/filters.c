
#include "filters.h"

/*
 * IIR moving average filter
 *
 * https://circuitcellar.com/research-design-hub/basics-of-design/introduction-to-iir-filters/
 */
float iir_ma(iir_ma_state_t *state, float currentReading)
{
  const float a0 = 0.2;
  const float b1 = 0.8;
  float y = a0*currentReading + b1*state->yz1;
  state->yz1 = y;
  return y;
}
