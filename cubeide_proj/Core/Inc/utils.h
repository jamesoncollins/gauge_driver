/*
 * utils.h
 *
 *  Created on: Jan 28, 2024
 *      Author: user
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "stdbool.h"
#include "stdint.h"

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void DWT_Delay(uint32_t us);
extern uint64_t get_micros_64();
extern uint32_t get_us_32 ();
extern uint32_t get_ticks_32();
extern bool tic_toc(uint32_t *tic, uint32_t delay);
extern void tic();
extern uint32_t toc();
extern void init_get_cycle_count();
extern uint64_t get_cycle_count();

extern bool lock_mutex(volatile unsigned *lock);
extern void unlock_mutex(volatile unsigned *lock);

#ifdef __cplusplus
}
#endif

#endif /* INC_UTILS_H_ */
