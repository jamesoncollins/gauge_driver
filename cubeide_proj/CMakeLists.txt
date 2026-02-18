cmake_minimum_required(VERSION 3.20)
project(gauge_driver C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# Toolchain/CPU flags (matches CubeIDE: STM32WB55, FPU fpv4-sp-d16 hard)
# ---------------------------------------------------------------------------
set(MCU_FLAGS
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
)
add_compile_options(
  ${MCU_FLAGS}
  -ffunction-sections -fdata-sections
  -Wall
  -fstack-usage
  -fcyclomatic-complexity
  --specs=nano.specs
  $<$<COMPILE_LANGUAGE:C>:-MMD>
  $<$<COMPILE_LANGUAGE:C>:-MP>
  $<$<COMPILE_LANGUAGE:CXX>:-MMD>
  $<$<COMPILE_LANGUAGE:CXX>:-MP>
)
# Apply C++-only flags safely (separate generator expressions to avoid stray characters)
add_compile_options(
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(DEBUG)
  add_compile_options(-O0 -g3)
else()
  add_compile_options(-O2)
endif()

add_compile_definitions(
  USE_HAL_DRIVER
  STM32WB55xx
)

# ---------------------------------------------------------------------------
# Linker
# ---------------------------------------------------------------------------
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32WB55RGVX_FLASH.ld)
add_link_options(${MCU_FLAGS}
  -T${LINKER_SCRIPT}
  -Wl,--gc-sections
  -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map
  -static
  --specs=nosys.specs
  --specs=nano.specs
  -Wl,-u,_printf_float
)

# ---------------------------------------------------------------------------
# uGFX driver selection (one at a time)
# ---------------------------------------------------------------------------
set(UGFX_DRIVER s6e63d6 CACHE STRING "uGFX gdisp driver (s6e63d6 or st7789vi)")
if(UGFX_DRIVER STREQUAL "st7789vi")
  set(UGFX_DRIVER_SRCS ugfx/drivers/gdisp/ST7789VI/gdisp_lld_ST7789VI.c)
  set(UGFX_DRIVER_INCS ugfx/drivers/gdisp/ST7789VI)
elseif(UGFX_DRIVER STREQUAL "s6e63d6")
  set(UGFX_DRIVER_SRCS ugfx/drivers/gdisp/s6e63d6/gdisp_lld_s6e63d6.c)
  set(UGFX_DRIVER_INCS ugfx/drivers/gdisp/s6e63d6)
else()
  message(FATAL_ERROR "Unknown UGFX_DRIVER=${UGFX_DRIVER}")
endif()

# ---------------------------------------------------------------------------
# Custom code layout selection
# - Custom shared code lives under Custom
# - Platform-specific code lives under ARM or x86 subfolders
# ---------------------------------------------------------------------------
set(CUSTOM_ROOT Custom)
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/${CUSTOM_ROOT}")
  message(FATAL_ERROR "Could not find custom code folder (expected ./Custom)")
endif()

set(CUSTOM_PLATFORM arm CACHE STRING "Custom platform code to compile (arm or x86)")
if(CUSTOM_PLATFORM STREQUAL "arm")
  add_compile_definitions(CUSTOM_PLATFORM_ARM)
  if(EXISTS "${CMAKE_SOURCE_DIR}/${CUSTOM_ROOT}/ARM/platform.cpp")
    set(CUSTOM_PLATFORM_SRC ${CUSTOM_ROOT}/ARM/platform.cpp)
  else()
    message(FATAL_ERROR "Could not find ARM platform source in ${CUSTOM_ROOT}/ARM")
  endif()
elseif(CUSTOM_PLATFORM STREQUAL "x86")
  add_compile_definitions(CUSTOM_PLATFORM_X86)
  if(EXISTS "${CMAKE_SOURCE_DIR}/${CUSTOM_ROOT}/x86/platform.cpp")
    set(CUSTOM_PLATFORM_SRC ${CUSTOM_ROOT}/x86/platform.cpp)
  else()
    message(FATAL_ERROR "Could not find x86 platform source in ${CUSTOM_ROOT}/x86")
  endif()
else()
  message(FATAL_ERROR "Unknown CUSTOM_PLATFORM=${CUSTOM_PLATFORM} (valid values: arm, x86)")
endif()

# ---------------------------------------------------------------------------
# Sources
# - Glob all non-uGFX C/C++ sources
# - Pick uGFX manually (core + selected driver)
# ---------------------------------------------------------------------------
file(GLOB_RECURSE PROJECT_SOURCES
  Core/*.c Core/*.cpp
  Drivers/*.c Drivers/*.cpp
  Middlewares/*.c Middlewares/*.cpp
  Utilities/*.c Utilities/*.cpp
  USB_Device/*.c USB_Device/*.cpp
  STM32_WPAN/*.c STM32_WPAN/*.cpp
)

file(GLOB_RECURSE CUSTOM_SOURCES
  ${CUSTOM_ROOT}/*.c ${CUSTOM_ROOT}/*.cpp
)
list(FILTER CUSTOM_SOURCES EXCLUDE REGEX ".*/${CUSTOM_ROOT}/(ARM|x86)/.*")
list(APPEND PROJECT_SOURCES ${CUSTOM_SOURCES} ${CUSTOM_PLATFORM_SRC})

# Drop anything under ugfx from the generic glob
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/ugfx/.*")

# uGFX core: CubeIDE project compiles only gfx_mk.c
set(UGFX_CORE_SRCS ugfx/src/gfx_mk.c)

set(STARTUP_FILE Core/Startup/startup_stm32wb55rgvx.s)

add_executable(${PROJECT_NAME}
  ${STARTUP_FILE}
  ${PROJECT_SOURCES}
  ${UGFX_CORE_SRCS}
  ${UGFX_DRIVER_SRCS}
)

# ---------------------------------------------------------------------------
# Includes (mirrors CubeIDE .cproject entries)
# ---------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME} PRIVATE
  Core/Inc
  ${CUSTOM_ROOT}/Inc
  Drivers/STM32WBxx_HAL_Driver/Inc
  Drivers/STM32WBxx_HAL_Driver/Inc/Legacy
  Drivers/CMSIS/Device/ST/STM32WBxx/Include
  Drivers/CMSIS/Include
  ugfx
  ${UGFX_DRIVER_INCS}
  ugfx/drivers/gdisp/SSD1351  # legacy include path used by some C++ files
  STM32_WPAN/App
  Utilities/lpm/tiny_lpm
  Middlewares/ST/STM32_WPAN
  Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread
  Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl
  Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci
  Middlewares/ST/STM32_WPAN/utilities
  Middlewares/ST/STM32_WPAN/ble/core
  Middlewares/ST/STM32_WPAN/ble/core/auto
  Middlewares/ST/STM32_WPAN/ble/core/template
  Middlewares/ST/STM32_WPAN/ble/svc/Inc
  Middlewares/ST/STM32_WPAN/ble/svc/Src
  Middlewares/ST/STM32_WPAN/ble
  STM32_WPAN/Target
  Utilities/sequencer
  USB_Device/App
  USB_Device/Target
  Middlewares/ST/STM32_USB_Device_Library/Core/Inc
  Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
)

target_link_libraries(${PROJECT_NAME} c m stdc++ supc++)

# Post-build artifacts to mirror CubeIDE: .list, .hex, .bin, and size
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
  COMMAND ${CMAKE_OBJDUMP} -h -S $<TARGET_FILE:${PROJECT_NAME}> > ${PROJECT_NAME}.list
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
