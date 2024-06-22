#include "utils.h"
#include "common.h"
#include "gfx.h"
#include "stm32wbxx_hal.h"
#include "main.h"
#include "assert.h"
#include "stdbool.h"

bool tic_toc (uint32_t *tic, uint32_t delay)
{
  uint32_t toc = HAL_GetTick ();
  if (*tic == 0)
  {
    *tic = toc;
    return false;
  }

  if (toc - *tic > delay)
  {
    *tic = toc;
    return true;
  }

  return false;

}

/*
 * Call at least every 2^32 cycles (every 59.6 seconds @ 72 MHz).
 *
 * masking interrupts is only necessary if this function will be called
 * FROM an interrupt
 *
 * alteratively look into atomic increment options for the m4
 */
inline uint64_t get_cycle_count ()
{
  volatile static uint64_t last_cycle_count_64 = 0;

//  uint32_t prim;
//  prim = __get_PRIMASK();
//  __disable_irq();

//  static unsigned lock = 0;
//  while (!lock_mutex (&lock))
//  {
//  };

  int64_t r = last_cycle_count_64;
  r += DWT->CYCCNT - (uint32_t) (r);
  last_cycle_count_64 = r;

//  unlock_mutex (&lock);

//  if (!prim) {
//      __enable_irq();
//  }

  return r;
}

void init_get_cycle_count ()
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * Delay routine itself.
 * Time is in microseconds (1/1000000th of a second), not to be
 * confused with millisecond (1/1000th).
 *
 * No need to check an overflow. Let it just tick :)
 *
 * @param uint32_t us  Number of microseconds to delay for
 */
void DWT_Delay(uint32_t us)
{
    uint32_t startTick = DWT->CYCCNT,
             delayTicks = us * (SystemCoreClock/1000000);

    while (DWT->CYCCNT - startTick < delayTicks);
}

uint64_t get_ticks_us ()
{
  return get_cycle_count () * US_PER_SYS_TICK ;
}

gTicks gfxSystemTicks(void)
{
    return HAL_GetTick();
}

gTicks gfxMillisecondsToTicks(gDelay ms)
{
    return ms;
}

bool lock_mutex (volatile unsigned *lock)
{
  static unsigned compare = 0, exchange = 1;
  if(*lock==1)
    compare = 0;
  return __atomic_compare_exchange_n (lock, &compare, exchange, false,
                                      __ATOMIC_SEQ_CST,
                                      __ATOMIC_SEQ_CST);
}

void unlock_mutex (volatile unsigned *lock)
{
  static unsigned compare = 1, exchange = 0;
  if (*lock == 0)
    compare = 1;
  assert(
      __atomic_compare_exchange_n(lock, &compare, exchange, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)!=0);
}
