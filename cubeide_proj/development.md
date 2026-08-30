# Development Guide

## Common Commands

List configured presets:

```sh
cmake --list-presets
```

Install or update the MSYS2 UCRT64 build tools from the package manifest:

```sh
./scripts/setup-msys2-ucrt.sh
```

Use the UCRT64 tool environment in the current shell:

```sh
. ./scripts/env-msys2-ucrt.sh
```

Build the recommended native simulator:

```sh
./scripts/build-preset.sh x86-debug
```

Build hardware display variants:

```sh
./scripts/build-preset.sh s6e63d6-debug
./scripts/build-preset.sh st7789vi-debug
```

Flash a hardware debug build over BLE OTA from VS Code:

1. Install the Python BLE dependency into the Python configured by `gaugeDriver.pythonCommand`:

   ```sh
   ${userHome}/miniconda3/python.exe -m pip install -r scripts/requirements-ble-ota.txt
   ```

2. Open the VS Code command palette with `Ctrl+Shift+P`.
3. Run `Tasks: Run Task`.
4. Run `BLE OTA: scan` to confirm the board is visible.
5. Run one of:
   - `BLE OTA: flash st7789vi-debug`
   - `BLE OTA: flash s6e63d6-debug`

The flash tasks build the selected CMake preset first, then upload
`build/<preset>/gauge_driver.bin` with `scripts/ble_ota_loader.py`.

The VS Code tasks use generic executable settings instead of assuming a specific
platform shell. `cmake.cmakePath` is the CMake executable source of truth for
both the CMake Tools extension and the repo tasks; the repo default is
MSYS2/UCRT CMake, but other systems can override it. `gaugeDriver.buildToolPathPrefix`
prepends any toolchain directory needed by the build, and `gaugeDriver.pythonCommand`
selects the Python executable used for BLE OTA. On systems where CMake and the
build tools are already on `PATH`, set `gaugeDriver.buildToolPathPrefix` to an
empty string.

The shared VS Code defaults are in `.vscode/settings.json`:

```json
{
  "cmake.sourceDirectory": "${workspaceFolder}",
  "cmake.cmakePath": "C:/msys64/ucrt64/bin/cmake.exe",
  "gaugeDriver.buildToolPathPrefix": "C:/msys64/ucrt64/bin",
  "gaugeDriver.pythonCommand": "${userHome}/miniconda3/python.exe",
  "gaugeDriver.stm32CubeIdeRoot": "C:/ST/STM32CubeIDE_1.14.0/STM32CubeIDE",
  "gaugeDriver.stm32CubeProgrammerRoot": "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer"
}
```

Override those settings locally if your build tools, Python, or ST install live
somewhere else.

For CLI use without VS Code:

```sh
${userHome}/miniconda3/python.exe scripts/ble_ota_loader.py --file build/st7789vi-debug/gauge_driver.bin
```

Build the web simulator:

```sh
./scripts/build-preset.sh web-debug
```

## Working Rules

- Treat Cube-generated and vendor trees as generated input unless a hardware integration change requires touching them.
- Keep cross-target gauge behavior in `Custom/Src` and `Custom/Inc`.
- Keep target-specific side effects behind platform services or host backend code.
- Update `Custom/CONFIGURATION.md` when adding or changing build axes.
- Update `architecture.md` when changing ownership boundaries or startup flow.
- Update `build-environments.md` when adding build prerequisites or presets.
- Update `env/msys2-ucrt-packages.txt` when MSYS2 package dependencies change.

## Adding a Target

1. Add a `BUILD_TARGET` branch in `CMakeLists.txt`.
2. Set all four axes: `PLATFORM_KIND`, `DISPLAY_BACKEND`, `SIM_PROFILE`, and `HOST_BACKEND`.
3. Add validation for new axis values if needed.
4. Add any new uGFX driver source and include selection.
5. Add a configure preset and build preset in `CMakePresets.json`.
6. Build the new target and one existing target of the same platform kind.

## Debugging Build Problems

- If CMake cannot find UCRT64 GCC, run `scripts/setup-msys2-ucrt.sh`, then source `scripts/env-msys2-ucrt.sh` from MSYS2 UCRT64.
- If CMake cannot find `arm-none-eabi-gcc`, install the MSYS2 UCRT64 package manifest and check `command -v arm-none-eabi-gcc`.
- If CMake cannot find Emscripten, install the MSYS2 UCRT64 package manifest and check `command -v emcc`.
- If a simulator build starts compiling STM32-generated trees, check `CUSTOM_PLATFORM`, `PLATFORM_KIND`, and `BUILD_TARGET`.
