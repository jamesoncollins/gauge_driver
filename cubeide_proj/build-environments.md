# Build Environments

This repo supports three practical build environments through MSYS2 UCRT64:

- ARM firmware builds through `arm-none-eabi-gcc`.
- Native Windows simulator builds through UCRT64 GCC.
- Web simulator builds through Emscripten.

## Recommended Windows Host

The supported Windows workflow assumes:

- MSYS2 installed with a UCRT64 shell.
- Packages listed in `env/msys2-ucrt-packages.txt` installed with `pacman`.
- CMake presets run from MSYS2 UCRT64 bash or a shell that has sourced `scripts/env-msys2-ucrt.sh`.

Package names live in `env/msys2-ucrt-packages.txt`; update that manifest when build dependencies change instead of embedding package lists in scripts.

## Setup

Install or update the UCRT64 environment from the package manifest:

```sh
./scripts/setup-msys2-ucrt.sh
```

To skip a full MSYS2 system update and only install missing packages:

```sh
./scripts/setup-msys2-ucrt.sh --no-system-update
```

Source the UCRT64 tool environment in the current shell:

```sh
. ./scripts/env-msys2-ucrt.sh
```

## Native Simulator

Configure and build the recommended native simulator:

```sh
./scripts/build-preset.sh x86-debug
```

`x86-debug` targets MSYS2 UCRT64 through `C:/msys64/ucrt64`.

## ARM Firmware

The ARM toolchain expects `arm-none-eabi-gcc` on `PATH`, supplied by the MSYS2 UCRT64 package manifest. `ARM_GCC_DIR` remains available for manually pointing at another toolchain `bin` directory.

Build examples:

```sh
./scripts/build-preset.sh s6e63d6-debug
./scripts/build-preset.sh st7789vi-debug
```

ARM builds emit the ELF plus `.map`, `.list`, `.hex`, and `.bin` artifacts in the preset build directory.

## Web Simulator

The web toolchain expects `emcc` and `em++` on `PATH`, supplied by the MSYS2 UCRT64 package manifest. `EMSDK` remains available as a fallback for external Emscripten SDK installs.

```sh
./scripts/build-preset.sh web-debug
```

The web toolchain sets the executable suffix to `.html`.

## CMake Direct Use

The wrapper is only a convenience. After sourcing the environment, direct CMake commands also work:

```sh
cmake --preset x86-debug
cmake --build --preset x86-debug
```

For a clean configure through the wrapper:

```sh
./scripts/build-preset.sh --fresh x86-debug
```
