# Environment Automation

This folder contains machine-readable inputs for local build environment setup.

- `msys2-ucrt-packages.txt`: MSYS2 UCRT64 packages needed for the native Windows simulator build.

The PowerShell entrypoints live in `scripts/`:

- `setup-msys2-ucrt.ps1`: install/update MSYS2 UCRT64 packages.
- `enter-msys2-ucrt.ps1`: open a PowerShell environment with UCRT64 tools on `PATH`.
- `build-preset.ps1`: configure and build a CMake preset.
