cmake_minimum_required(VERSION 3.20)
project(gauge_driver C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
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
  -Wall -Wextra
  -fstack-usage
  -fcyclomatic-complexity
  $<$<COMPILE_LANGUAGE:C>:-MMD>
  $<$<COMPILE_LANGUAGE:C>:-MP>
  $<$<COMPILE_LANGUAGE:CXX>:-MMD>
  $<$<COMPILE_LANGUAGE:CXX>:-MP>
)
# Apply C++-only flags safely (separate generator expressions to avoid stray characters)
add_compile_options(
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
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
  --specs=nano.specs --specs=nosys.specs
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
# Drop anything under ugfx from the generic glob
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/ugfx/.*")

# uGFX core: take all src files but exclude gfx.c to avoid duplicate gfxInit/gfxDeinit
file(GLOB UGFX_CORE_SRCS ugfx/src/*.c)
list(FILTER UGFX_CORE_SRCS EXCLUDE REGEX ".*/gfx\\.c$")

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

target_link_libraries(${PROJECT_NAME} m c gcc)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
)
