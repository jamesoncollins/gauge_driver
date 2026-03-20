#ifndef INC_PLATFORM_API_H_
#define INC_PLATFORM_API_H_

/*
 * Central platform include for shared custom code.
 * - ARM/default build: use STM headers
 * - x86 build: use local shim header
 */
#if defined(CUSTOM_PLATFORM_X86)
#include "x86_hal.h"
#else
#include "stm32wbxx_hal.h"
#include "main.h"
#endif

#endif /* INC_PLATFORM_API_H_ */
