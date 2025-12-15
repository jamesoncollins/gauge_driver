set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_PREFIX arm-none-eabi)

# Try to locate the toolchain if not on PATH. You can also set ARM_GCC_DIR env to the bin folder.
set(_arm_hints "")
# Preferred default (CubeIDE 1.14, GCC 13.3). Added first so it wins if present.
set(_arm_default_hint "C:/ST/STM32CubeIDE_1.14.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin")
if(EXISTS "${_arm_default_hint}")
  list(APPEND _arm_hints "${_arm_default_hint}")
endif()
if(DEFINED ENV{ARM_GCC_DIR})
  list(APPEND _arm_hints "$ENV{ARM_GCC_DIR}")
endif()
if(WIN32)
  # Examples:
  # C:/ST/STM32CubeIDE_1.14.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406191623/tools/bin/arm-none-eabi-gcc.exe
  file(GLOB_RECURSE _arm_bins
    "C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/**/tools/bin/${ARM_PREFIX}-gcc.exe"
  )
  foreach(p IN LISTS _arm_bins)
    get_filename_component(_bin_dir "${p}" DIRECTORY)
    list(APPEND _arm_hints "${_bin_dir}")
  endforeach()
endif()

find_program(ARM_CC ${ARM_PREFIX}-gcc HINTS ${_arm_hints} PATH_SUFFIXES bin)
if(NOT ARM_CC)
  message(FATAL_ERROR "arm-none-eabi-gcc not found. Add it to PATH or set ARM_GCC_DIR to the toolchain bin directory.")
endif()
get_filename_component(ARM_TOOLCHAIN_DIR ${ARM_CC} DIRECTORY)
find_program(ARM_CXX ${ARM_PREFIX}-g++ HINTS ${ARM_TOOLCHAIN_DIR} PATH_SUFFIXES bin)
find_program(ARM_AR  ${ARM_PREFIX}-ar  HINTS ${ARM_TOOLCHAIN_DIR} PATH_SUFFIXES bin)
find_program(ARM_OBJCOPY ${ARM_PREFIX}-objcopy HINTS ${ARM_TOOLCHAIN_DIR} PATH_SUFFIXES bin)
find_program(ARM_OBJDUMP ${ARM_PREFIX}-objdump HINTS ${ARM_TOOLCHAIN_DIR} PATH_SUFFIXES bin)
find_program(ARM_SIZE ${ARM_PREFIX}-size HINTS ${ARM_TOOLCHAIN_DIR} PATH_SUFFIXES bin)

set(CMAKE_C_COMPILER   ${ARM_CC})
set(CMAKE_CXX_COMPILER ${ARM_CXX})
set(CMAKE_ASM_COMPILER ${ARM_CC})
set(CMAKE_AR           ${ARM_AR})
set(CMAKE_OBJCOPY      ${ARM_OBJCOPY})
set(CMAKE_OBJDUMP      ${ARM_OBJDUMP})
set(CMAKE_SIZE         ${ARM_SIZE})
