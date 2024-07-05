/*
 * filter.h
 *
 *  Created on: Jul 5, 2024
 *      Author: user
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

typedef struct
{
  float yz1;
}
iir_ma_state_t;

extern float iir_ma(iir_ma_state_t *state, float currentReading);

#endif /* INC_FILTER_H_ */
