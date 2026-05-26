# Target Configuration Model

This project now composes builds from four axes:

1. `platform`: CPU/toolchain/runtime (`x86_sim`, `stm32wb55`)
2. `board`: pinout/peripherals (`sim_host`, `board_3000gt_rev_a`)
3. `display`: panel/driver/timing (`software`, `s6e63d6_oled`, `st7789vi_lcd`)
4. `vehicle`: calibration and app thresholds (`3000gt`)

## Current composed targets

1. `sim_3000gt_soft`
2. `3000gt_oled`
3. `3000gt_lcd`

These are selected by CMake cache variable `BUILD_TARGET`.

## Presets

1. `x86-debug` -> `BUILD_TARGET=sim_3000gt_soft`
2. `s6e63d6-debug` -> `BUILD_TARGET=3000gt_oled`
3. `st7789vi-debug` -> `BUILD_TARGET=3000gt_lcd`

## Where values live

All runtime tunables for those axes are defined in:

- `Custom/Inc/build_config.hpp`
- `Custom/Src/build_config.cpp`

Examples:

1. RPM alert thresholds
2. Display FPS and timing intervals
3. Sensor conversion scales (`mph_per_hz`, `rpm_per_hz`)
4. Odometer tick stepping

## Adding a new target

1. Add a new branch in `get_build_config()` in `Custom/Src/build_config.cpp`.
2. Add a compile definition in `CMakeLists.txt` for that `BUILD_TARGET`.
3. Add/adjust `CUSTOM_PLATFORM` + `UGFX_DRIVER` mapping in `CMakeLists.txt`.
4. Add a matching configure/build preset in `CMakePresets.json`.
5. Build at least one ARM target and x86 sim target before merging.
