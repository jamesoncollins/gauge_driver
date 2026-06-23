# Environment Automation

This folder contains machine-readable inputs for local build environment setup.

- `msys2-ucrt-packages.txt`: MSYS2 UCRT64 packages needed for native simulator, ARM firmware, and web/Emscripten builds.

Keep package names here rather than embedding them in scripts. The shell entrypoints in `scripts/` read this manifest:

- `setup-msys2-ucrt.sh`: install/update MSYS2 UCRT64 packages from the manifest.
- `env-msys2-ucrt.sh`: sourceable MSYS2 UCRT64 environment setup.
- `build-preset.sh`: configure and build a CMake preset.
