# Architecture

This project is a STM32WB55 gauge driver with shared application logic, hardware-specific platform code, host simulator backends, and uGFX display rendering.

## Source Ownership

- `Core`, `Drivers`, `Middlewares`, `STM32_WPAN`, `USB_Device`, `Utilities`: STM32Cube-generated and vendor-provided code. Keep generated edits isolated and expect CubeMX/CubeIDE regeneration to affect these trees.
- `Custom/Src` and `Custom/Inc`: shared gauge application code. This is the preferred place for vehicle logic, render state, widgets, and target-independent behavior.
- `Custom/ARM`: hardware runtime glue for the STM32 target.
- `Custom/Simulator`: simulator services and sample data used by host builds.
- `Custom/Host/win32`: native Windows host entrypoint and uGFX configuration.
- `Custom/Host/emscripten`: web host entrypoint and uGFX configuration.
- `ugfx`: vendored graphics framework plus selected display drivers.
- `res`: project resources.

## Runtime Shape

Hardware builds start in STM32 startup code and generated `Core/Src/main.c`, then cross into custom C/C++ code through the custom platform/runtime layer. Simulator builds skip STM32-generated source trees and compile the shared app logic with simulator services and a selected host backend.

The main shared rendering and state translation path lives in:

- `Custom/Src/main_shared.cpp`
- `Custom/Inc/main_shared.h`
- `Custom/Src/build_config.cpp`
- `Custom/Inc/build_config.hpp`
- `Custom/Src/ugfx_widgets.cpp`
- `Custom/Inc/ugfx_widgets.h`

`Custom/Inc/platform_api.h` is the central include boundary between shared code and platform headers. ARM builds use STM32 HAL headers; host builds use `Custom/Inc/host_hal.h`.

## Build Axes

CMake composes targets from explicit axes:

- `PLATFORM_KIND`: `hardware` or `simulator`
- `DISPLAY_BACKEND`: `s6e63d6`, `st7789vi`, `win32`, or `sdl`
- `SIM_PROFILE`: `none` or `3000gt_soft`
- `HOST_BACKEND`: `none`, `win32`, or `emscripten`

`BUILD_TARGET` is the friendly preset-level selector. See `Custom/CONFIGURATION.md` for the current target matrix and compatibility rules.

## Current Targets

- `3000gt_oled`: STM32 hardware, S6E63D6 display.
- `3000gt_lcd`: STM32 hardware, ST7789VI display.
- `sim_3000gt_soft`: Win32 simulator, Win32 uGFX display backend.
- `web_3000gt_soft`: Emscripten simulator, SDL/uGFX web backend.

## Dependency Direction

Shared custom code should depend on project abstractions and uGFX APIs, not directly on host or STM32 details. Platform-specific code may depend on STM32 HAL, Windows APIs, Emscripten APIs, simulator services, or backend-specific `gfxconf.h`.

Keep new target-specific behavior behind one of these boundaries:

- build axes in `CMakeLists.txt`
- runtime values in `build_config`
- platform services in `Custom/ARM`, `Custom/Simulator`, or `Custom/Host/<backend>`
- display driver selection through `DISPLAY_BACKEND`

## Change Guidelines

1. Put portable behavior in `Custom/Src` and `Custom/Inc`.
2. Put hardware-only behavior in `Custom/ARM`.
3. Put simulator behavior shared by host targets in `Custom/Simulator`.
4. Put native host or web entrypoint behavior in `Custom/Host/<backend>`.
5. Add new display backends through CMake driver selection and the matching uGFX include path.
6. Build at least one simulator target and the affected hardware target after changing shared application code.
