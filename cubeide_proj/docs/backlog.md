# Backlog

Local source of truth for one-time TODOs, plans, fixes, and verification follow-ups that should stay close to the codebase.

## Next

- [x] Fix the docs/preset mismatch: docs mention `x86-ucrt-debug`, while `CMakePresets.json` exposes `x86-debug` as the UCRT64 simulator preset.
- [ ] Review the latest `web-debug` capture contact sheet for persistent clipping or overlap after layout changes.

## Technical Debt

- [x] Clean up C++ anonymous `typedef struct` warnings in `ugfx_widgets.h` and `ECUK.hpp`.
- [ ] Move hardware-specific USART init/deinit calls out of `ECUK`.
- [ ] Clarify `ECUK::isConnected()` semantics so "never attempted connection" is not treated as a connection error.
- [x] Replace the hard-coded `BTBufferData::data[64-8]` sizing with a named constant or protocol-derived size.
- [ ] Decide whether `get_us_32()` should use `TIM2->CNT` instead of `DWT->CYCCNT`.
- [ ] Check MCP4725 voltage conversion rounding accuracy, including whether a `+0.5` adjustment is appropriate.

## Verification

- [ ] Investigate the ARM linker warning: `gauge_driver.elf has a LOAD segment with RWX permissions`.

## Later

- [ ] Optimize `drawBarGraph` to use fills when `GDISP_HARDWARE_FILLS` is available.
