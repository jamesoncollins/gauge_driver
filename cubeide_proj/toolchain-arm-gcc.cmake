set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_PREFIX arm-none-eabi)

# Prefer the active toolchain on PATH. In the supported Windows workflow, that
# is the MSYS2 UCRT64 arm-none-eabi package from env/msys2-ucrt-packages.txt.
find_program(ARM_CC ${ARM_PREFIX}-gcc)

if(NOT ARM_CC AND DEFINED ENV{ARM_GCC_DIR})
  find_program(ARM_CC ${ARM_PREFIX}-gcc HINTS "$ENV{ARM_GCC_DIR}" PATH_SUFFIXES bin NO_DEFAULT_PATH)
endif()

if(NOT ARM_CC AND WIN32)
  # Fallback for older local setups that still use STM32CubeIDE's bundled GCC.
  file(GLOB_RECURSE _arm_bins
    "C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/**/tools/bin/${ARM_PREFIX}-gcc.exe"
  )
  foreach(p IN LISTS _arm_bins)
    get_filename_component(_bin_dir "${p}" DIRECTORY)
    list(APPEND _arm_hints "${_bin_dir}")
  endforeach()
  find_program(ARM_CC ${ARM_PREFIX}-gcc HINTS ${_arm_hints} PATH_SUFFIXES bin NO_DEFAULT_PATH)
endif()

if(NOT ARM_CC)
  message(FATAL_ERROR "arm-none-eabi-gcc not found. Run scripts/setup-msys2-ucrt.sh and source scripts/env-msys2-ucrt.sh from MSYS2 UCRT64, or set ARM_GCC_DIR to the toolchain bin directory.")
endif()

get_filename_component(ARM_TOOLCHAIN_DIR ${ARM_CC} DIRECTORY)
find_program(ARM_CXX ${ARM_PREFIX}-g++ HINTS ${ARM_TOOLCHAIN_DIR} PATH_SUFFIXES bin NO_DEFAULT_PATH)
find_program(ARM_AR  ${ARM_PREFIX}-ar  HINTS ${ARM_TOOLCHAIN_DIR} PATH_SUFFIXES bin NO_DEFAULT_PATH)
find_program(ARM_OBJCOPY ${ARM_PREFIX}-objcopy HINTS ${ARM_TOOLCHAIN_DIR} PATH_SUFFIXES bin NO_DEFAULT_PATH)
find_program(ARM_OBJDUMP ${ARM_PREFIX}-objdump HINTS ${ARM_TOOLCHAIN_DIR} PATH_SUFFIXES bin NO_DEFAULT_PATH)
find_program(ARM_SIZE ${ARM_PREFIX}-size HINTS ${ARM_TOOLCHAIN_DIR} PATH_SUFFIXES bin NO_DEFAULT_PATH)

set(CMAKE_C_COMPILER   ${ARM_CC})
set(CMAKE_CXX_COMPILER ${ARM_CXX})
set(CMAKE_ASM_COMPILER ${ARM_CC})
set(CMAKE_AR           ${ARM_AR})
set(CMAKE_OBJCOPY      ${ARM_OBJCOPY})
set(CMAKE_OBJDUMP      ${ARM_OBJDUMP})
set(CMAKE_SIZE         ${ARM_SIZE})
