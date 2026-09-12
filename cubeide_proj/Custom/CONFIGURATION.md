# Target Configuration Model

This project composes builds from explicit CMake axes. `BUILD_TARGET` remains the
friendly preset name, while the axes describe what that preset means internally.

## Build axes

1. `PLATFORM_KIND`: runtime kind (`hardware`, `simulator`)
2. `DISPLAY_BACKEND`: uGFX/display driver (`s6e63d6`, `st7789vi`, `win32`, `sdl`)
3. `SIM_PROFILE`: simulator data profile (`none`, `3000gt_soft`)
4. `HOST_BACKEND`: host runtime wrapper (`none`, `win32`, `emscripten`)

`CUSTOM_PLATFORM` still exists as a compatibility selector for older configure commands (`arm` or `x86`), but source layout is now driven by `PLATFORM_KIND` and `HOST_BACKEND` rather than an `x86` folder.

## Source layout

- `Custom/ARM`: hardware-only platform code.
- `Custom/Simulator`: simulator code shared by all host backends.
- `Custom/Host/win32`: native Win32 host entrypoint and `gfxconf.h`.
- `Custom/Host/emscripten`: Emscripten host entrypoint and `gfxconf.h`.

Simulator builds compile `Custom/Simulator` plus `Custom/Host/${HOST_BACKEND}`.
Hardware builds exclude both host and simulator directories.

## Current composed targets

| `BUILD_TARGET` | `PLATFORM_KIND` | `DISPLAY_BACKEND` | `SIM_PROFILE` | `HOST_BACKEND` |
| --- | --- | --- | --- | --- |
| `sim_3000gt_soft` | `simulator` | `win32` | `3000gt_soft` | `win32` |
| `web_3000gt_soft` | `simulator` | `sdl` | `3000gt_soft` | `emscripten` |
| `3000gt_oled` | `hardware` | `s6e63d6` | `none` | `none` |
| `3000gt_lcd` | `hardware` | `st7789vi` | `none` | `none` |

These are selected by CMake cache variable `BUILD_TARGET`.

## Compatibility rules

Not every axis combination is valid. CMake validates combinations early:

- `PLATFORM_KIND=hardware` requires `HOST_BACKEND=none`, `SIM_PROFILE=none`, and a hardware display backend (`s6e63d6` or `st7789vi`).
- `PLATFORM_KIND=simulator` requires a host backend, a simulator profile, and a host display backend (`win32` or `sdl`).
- `HOST_BACKEND=win32` currently requires `DISPLAY_BACKEND=win32`.
- `HOST_BACKEND=emscripten` currently requires `DISPLAY_BACKEND=sdl`.

## Presets

1. `x86-debug` -> `BUILD_TARGET=sim_3000gt_soft`
2. `web-debug` -> `BUILD_TARGET=web_3000gt_soft`
3. `s6e63d6-debug` -> `BUILD_TARGET=3000gt_oled`
4. `st7789vi-debug` -> `BUILD_TARGET=3000gt_lcd`

## Where values live

All runtime tunables for those axes are defined in:

- `Custom/Inc/build_config.hpp`
- `Custom/Src/build_config.cpp`

Examples:

1. RPM alert thresholds
2. Display FPS and timing intervals
3. Sensor conversion scales (`mph_per_hz`, `rpm_per_hz`)
4. Odometer tick stepping

## Hardware FPS baseline

On 2026-08-30, the ST7789VI release build reported 11 FPS on hardware with the
runtime FPS diagnostic overlay. A comparison build from commit `065e1f6` (`Clean
up LCD render layering`, immediately after `4cbda8f` `Recover LCD render
performance`) also reported 11 FPS when built as `st7789vi-release` and flashed
over the current BLE OTA loader.

The baseline binary used for that check was built from a side worktree at:

```text
../cubeide_proj-fps-baseline-065e1f6/cubeide_proj/build/st7789vi-release/gauge_driver.bin
```

## Adding a new target

1. Add a new `BUILD_TARGET` branch in `CMakeLists.txt` that sets the four axes.
2. Add validation support if the new target needs a new axis value.
3. Add/update display backend source selection in `CMakeLists.txt` if needed.
4. Add compile definitions for target-specific code paths if needed.
5. Add a matching configure/build preset in `CMakePresets.json`; Emscripten targets use `toolchain-emcc.cmake`.
6. Build at least one hardware target and one simulator target before merging.

## Simulator render verification

When comparing simulator output to a still reference image, first make sure the reference image is available as an actual local file path. Images pasted into chat are useful for visual discussion, but the agent may not have a stable filesystem path for automated pixel comparison. If a precise comparison is needed, save the reference under a repo-local path such as `docs/reference/` or provide the path explicitly.

For web renders, build `web-debug` and serve `build/web-debug` over HTTP; opening `index.html` directly is not reliable for Emscripten artifacts. In this environment the in-app browser automation can fail before startup with a sandbox metadata error, and Chrome/Edge headless may terminate the Emscripten renderer before writing a screenshot. If that happens, use the visible browser/web page for manual confirmation, or install/use an MSYS2 browser automation package that can capture the canvas reliably.
