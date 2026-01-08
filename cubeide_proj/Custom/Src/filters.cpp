
#include "filters.h"

/*
 * IIR moving average filter
 *
 * https://circuitcellar.com/research-design-hub/basics-of-design/introduction-to-iir-filters/
 *
 * https://www.wavewalkerdsp.com/2022/08/10/single-pole-iir-filter-substitute-for-moving-average-filter/
 */
float iir_ma(iir_ma_state_t *state, float currentReading)
{
  float a0 = state->alpha;
  float b1 = 1.f-a0;
  float y = a0*currentReading + b1*state->yz1;
  state->yz1 = y;
  return y;
}
