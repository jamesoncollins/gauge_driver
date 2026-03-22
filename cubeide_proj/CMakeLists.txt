cmake_minimum_required(VERSION 3.20)
project(gauge_driver C CXX)
if(NOT MSVC)
  enable_language(ASM)
endif()
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
# Toolchain/CPU flags
# ---------------------------------------------------------------------------
if(CUSTOM_PLATFORM STREQUAL "arm")
  # Matches CubeIDE STM32WB55 flags
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
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
  )
  add_compile_definitions(
    USE_HAL_DRIVER
    STM32WB55xx
  )
else()
  # Host/x86 flags
  if(MSVC)
    add_compile_options(/W3)
  else()
    add_compile_options(-Wall -Wextra)
  endif()
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(DEBUG)
  if(MSVC)
    add_compile_options(/Od /Zi)
  else()
    add_compile_options(-O0 -g3)
  endif()
else()
  if(MSVC)
    add_compile_options(/O2)
  else()
    add_compile_options(-O2)
  endif()
endif()

# ---------------------------------------------------------------------------
# Linker
# ---------------------------------------------------------------------------
if(CUSTOM_PLATFORM STREQUAL "arm")
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
endif()

# ---------------------------------------------------------------------------
# uGFX driver selection (one at a time)
# ---------------------------------------------------------------------------
if(CUSTOM_PLATFORM STREQUAL "x86")
  # Host preview: use the uGFX Win32 multiple driver
  set(UGFX_DRIVER_SRCS ugfx/drivers/multiple/Win32/gdisp_lld_Win32.c)
  set(UGFX_DRIVER_INCS ugfx/drivers/multiple/Win32)
else()
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
endif()

# ---------------------------------------------------------------------------
# Sources
# - Glob all non-uGFX C/C++ sources
# - Pick uGFX manually (core + selected driver)
# ---------------------------------------------------------------------------
if(CUSTOM_PLATFORM STREQUAL "arm")
  file(GLOB_RECURSE PROJECT_SOURCES
    Core/*.c Core/*.cpp
    Drivers/*.c Drivers/*.cpp
    Middlewares/*.c Middlewares/*.cpp
    Utilities/*.c Utilities/*.cpp
    USB_Device/*.c USB_Device/*.cpp
    STM32_WPAN/*.c STM32_WPAN/*.cpp
  )
else()
  # x86 build: do not compile STM-generated source trees by default
  set(PROJECT_SOURCES)
endif()

file(GLOB_RECURSE CUSTOM_SOURCES
  ${CUSTOM_ROOT}/*.c ${CUSTOM_ROOT}/*.cpp
)
if(CUSTOM_PLATFORM STREQUAL "arm")
  list(FILTER CUSTOM_SOURCES EXCLUDE REGEX ".*/${CUSTOM_ROOT}/(ARM|x86)/.*")
  list(APPEND PROJECT_SOURCES ${CUSTOM_SOURCES} ${CUSTOM_PLATFORM_SRC})
else()
  # x86 build: use host platform files plus the shared app entrypoint.
  list(FILTER CUSTOM_SOURCES INCLUDE REGEX ".*/${CUSTOM_ROOT}/x86/.*")
  list(FILTER CUSTOM_SOURCES EXCLUDE REGEX ".*/${CUSTOM_ROOT}/x86/cpp_main\\.cpp$")
  list(APPEND PROJECT_SOURCES ${CUSTOM_SOURCES})
  list(APPEND PROJECT_SOURCES ${CUSTOM_ROOT}/Src/cpp_main.cpp)
endif()

# Drop anything under ugfx from the generic glob
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/ugfx/.*")

# uGFX core: CubeIDE project compiles only gfx_mk.c
set(UGFX_CORE_SRCS ugfx/src/gfx_mk.c)

if(CUSTOM_PLATFORM STREQUAL "x86")
  set(STARTUP_FILE)
else()
  set(STARTUP_FILE Core/Startup/startup_stm32wb55rgvx.s)
endif()

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
  ${CUSTOM_ROOT}/Inc
  ugfx
  ${UGFX_DRIVER_INCS}
)

if(CUSTOM_PLATFORM STREQUAL "arm")
  target_include_directories(${PROJECT_NAME} BEFORE PRIVATE
    ${CUSTOM_ROOT}/ARM/Inc
  )
  target_include_directories(${PROJECT_NAME} PRIVATE
    Core/Inc
    Drivers/STM32WBxx_HAL_Driver/Inc
    Drivers/STM32WBxx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32WBxx/Include
    Drivers/CMSIS/Include
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
else()
  # x86 platform headers (including gfxconf.h).
  target_include_directories(${PROJECT_NAME} BEFORE PRIVATE
    ${CUSTOM_ROOT}/x86/Inc
    ${CUSTOM_ROOT}/x86
  )
endif()

if(CUSTOM_PLATFORM STREQUAL "arm")
  target_link_libraries(${PROJECT_NAME} c m stdc++ supc++)
else()
  target_link_libraries(${PROJECT_NAME} gdi32)
endif()

# Post-build artifacts to mirror CubeIDE: .list, .hex, .bin, and size
if(CUSTOM_PLATFORM STREQUAL "arm")
  add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMAND ${CMAKE_OBJDUMP} -h -S $<TARGET_FILE:${PROJECT_NAME}> > ${PROJECT_NAME}.list
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endif()
