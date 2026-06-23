# Development Guide

## Common Commands

List configured presets:

```powershell
cmake --list-presets
```

Build the recommended native simulator:

```powershell
.\scripts\build-preset.ps1 -Preset x86-ucrt-debug
```

Build hardware display variants:

```powershell
.\scripts\build-preset.ps1 -Preset s6e63d6-debug
.\scripts\build-preset.ps1 -Preset st7789vi-debug
```

Build the web simulator:

```powershell
.\scripts\build-preset.ps1 -Preset web-debug
```

## Working Rules

- Treat Cube-generated and vendor trees as generated input unless a hardware integration change requires touching them.
- Keep cross-target gauge behavior in `Custom/Src` and `Custom/Inc`.
- Keep target-specific side effects behind platform services or host backend code.
- Update `Custom/CONFIGURATION.md` when adding or changing build axes.
- Update `architecture.md` when changing ownership boundaries or startup flow.
- Update `build-environments.md` when adding build prerequisites or presets.

## Adding a Target

1. Add a `BUILD_TARGET` branch in `CMakeLists.txt`.
2. Set all four axes: `PLATFORM_KIND`, `DISPLAY_BACKEND`, `SIM_PROFILE`, and `HOST_BACKEND`.
3. Add validation for new axis values if needed.
4. Add any new uGFX driver source and include selection.
5. Add a configure preset and build preset in `CMakePresets.json`.
6. Build the new target and one existing target of the same platform kind.

## Debugging Build Problems

- If CMake cannot find `arm-none-eabi-gcc`, set `ARM_GCC_DIR` to the ARM GCC `bin` directory.
- If CMake cannot find UCRT64 GCC, run `scripts/setup-msys2-ucrt.ps1` or check `C:\msys64\ucrt64\bin`.
- If CMake cannot find Emscripten, activate emsdk in the same shell or set `EMSDK`.
- If a simulator build starts compiling STM32-generated trees, check `CUSTOM_PLATFORM`, `PLATFORM_KIND`, and `BUILD_TARGET`.
