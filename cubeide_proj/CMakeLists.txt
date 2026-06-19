cmake_minimum_required(VERSION 3.20)
project(gauge_driver C CXX)
if(NOT MSVC)
  enable_language(ASM)
endif()
include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)
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

set(BUILD_TARGET "" CACHE STRING "Composed target profile (sim_3000gt_soft, 3000gt_oled, 3000gt_lcd)")

# Build axes. BUILD_TARGET presets below fill these in for known-good builds,
# while still making the underlying choices explicit and independently
# validated. CUSTOM_PLATFORM remains as a compatibility shim for existing code.
set(PLATFORM_KIND "hardware" CACHE STRING "Runtime kind (hardware or simulator)")
set(DISPLAY_BACKEND "s6e63d6" CACHE STRING "Display backend (s6e63d6, st7789vi, win32, sdl)")
set(SIM_PROFILE "none" CACHE STRING "Simulator data profile (none or 3000gt_soft)")
set(HOST_BACKEND "none" CACHE STRING "Host runtime backend (none, win32, emscripten)")
set(CUSTOM_PLATFORM arm CACHE STRING "Compatibility platform code selector (arm or x86)")

set_property(CACHE PLATFORM_KIND PROPERTY STRINGS hardware simulator)
set_property(CACHE DISPLAY_BACKEND PROPERTY STRINGS s6e63d6 st7789vi win32 sdl)
set_property(CACHE SIM_PROFILE PROPERTY STRINGS none 3000gt_soft)
set_property(CACHE HOST_BACKEND PROPERTY STRINGS none win32 emscripten)
set_property(CACHE CUSTOM_PLATFORM PROPERTY STRINGS arm x86)

if(BUILD_TARGET STREQUAL "sim_3000gt_soft")
  set(PLATFORM_KIND simulator CACHE STRING "Runtime kind (hardware or simulator)" FORCE)
  set(DISPLAY_BACKEND win32 CACHE STRING "Display backend (s6e63d6, st7789vi, win32, sdl)" FORCE)
  set(SIM_PROFILE 3000gt_soft CACHE STRING "Simulator data profile (none or 3000gt_soft)" FORCE)
  set(HOST_BACKEND win32 CACHE STRING "Host runtime backend (none, win32, emscripten)" FORCE)
  set(CUSTOM_PLATFORM x86 CACHE STRING "Compatibility platform code selector (arm or x86)" FORCE)
  set(UGFX_DRIVER win32 CACHE STRING "uGFX display backend token" FORCE)
  add_compile_definitions(TARGET_SIM_3000GT_SOFT)
elseif(BUILD_TARGET STREQUAL "3000gt_oled")
  set(PLATFORM_KIND hardware CACHE STRING "Runtime kind (hardware or simulator)" FORCE)
  set(DISPLAY_BACKEND s6e63d6 CACHE STRING "Display backend (s6e63d6, st7789vi, win32, sdl)" FORCE)
  set(SIM_PROFILE none CACHE STRING "Simulator data profile (none or 3000gt_soft)" FORCE)
  set(HOST_BACKEND none CACHE STRING "Host runtime backend (none, win32, emscripten)" FORCE)
  set(CUSTOM_PLATFORM arm CACHE STRING "Compatibility platform code selector (arm or x86)" FORCE)
  set(UGFX_DRIVER s6e63d6 CACHE STRING "uGFX display backend token" FORCE)
  add_compile_definitions(TARGET_3000GT_OLED)
elseif(BUILD_TARGET STREQUAL "3000gt_lcd")
  set(PLATFORM_KIND hardware CACHE STRING "Runtime kind (hardware or simulator)" FORCE)
  set(DISPLAY_BACKEND st7789vi CACHE STRING "Display backend (s6e63d6, st7789vi, win32, sdl)" FORCE)
  set(SIM_PROFILE none CACHE STRING "Simulator data profile (none or 3000gt_soft)" FORCE)
  set(HOST_BACKEND none CACHE STRING "Host runtime backend (none, win32, emscripten)" FORCE)
  set(CUSTOM_PLATFORM arm CACHE STRING "Compatibility platform code selector (arm or x86)" FORCE)
  set(UGFX_DRIVER st7789vi CACHE STRING "uGFX display backend token" FORCE)
  add_compile_definitions(TARGET_3000GT_LCD)
elseif(NOT BUILD_TARGET STREQUAL "")
  message(FATAL_ERROR "Unknown BUILD_TARGET=${BUILD_TARGET} (valid values: sim_3000gt_soft, 3000gt_oled, 3000gt_lcd)")
endif()

# Backward compatibility for older ad-hoc configure commands that selected only
# CUSTOM_PLATFORM=x86. The composed BUILD_TARGET presets above are preferred.
if(BUILD_TARGET STREQUAL "" AND CUSTOM_PLATFORM STREQUAL "x86" AND PLATFORM_KIND STREQUAL "hardware")
  set(PLATFORM_KIND simulator CACHE STRING "Runtime kind (hardware or simulator)" FORCE)
  set(DISPLAY_BACKEND win32 CACHE STRING "Display backend (s6e63d6, st7789vi, win32, sdl)" FORCE)
  set(SIM_PROFILE 3000gt_soft CACHE STRING "Simulator data profile (none or 3000gt_soft)" FORCE)
  set(HOST_BACKEND win32 CACHE STRING "Host runtime backend (none, win32, emscripten)" FORCE)
endif()

set(HARDWARE_DISPLAY_BACKENDS s6e63d6 st7789vi)
set(HOST_DISPLAY_BACKENDS win32 sdl)
set(SUPPORTED_SIM_PROFILES none 3000gt_soft)
set(SUPPORTED_HOST_BACKENDS none win32 emscripten)
set(SUPPORTED_PLATFORM_KINDS hardware simulator)

if(NOT PLATFORM_KIND IN_LIST SUPPORTED_PLATFORM_KINDS)
  message(FATAL_ERROR "Unknown PLATFORM_KIND=${PLATFORM_KIND} (valid values: ${SUPPORTED_PLATFORM_KINDS})")
endif()
if(NOT DISPLAY_BACKEND IN_LIST HARDWARE_DISPLAY_BACKENDS AND NOT DISPLAY_BACKEND IN_LIST HOST_DISPLAY_BACKENDS)
  message(FATAL_ERROR "Unknown DISPLAY_BACKEND=${DISPLAY_BACKEND} (valid values: ${HARDWARE_DISPLAY_BACKENDS};${HOST_DISPLAY_BACKENDS})")
endif()
if(NOT SIM_PROFILE IN_LIST SUPPORTED_SIM_PROFILES)
  message(FATAL_ERROR "Unknown SIM_PROFILE=${SIM_PROFILE} (valid values: ${SUPPORTED_SIM_PROFILES})")
endif()
if(NOT HOST_BACKEND IN_LIST SUPPORTED_HOST_BACKENDS)
  message(FATAL_ERROR "Unknown HOST_BACKEND=${HOST_BACKEND} (valid values: ${SUPPORTED_HOST_BACKENDS})")
endif()

if(PLATFORM_KIND STREQUAL "hardware")
  if(NOT HOST_BACKEND STREQUAL "none")
    message(FATAL_ERROR "PLATFORM_KIND=hardware requires HOST_BACKEND=none (got ${HOST_BACKEND})")
  endif()
  if(NOT SIM_PROFILE STREQUAL "none")
    message(FATAL_ERROR "PLATFORM_KIND=hardware requires SIM_PROFILE=none (got ${SIM_PROFILE})")
  endif()
  if(NOT DISPLAY_BACKEND IN_LIST HARDWARE_DISPLAY_BACKENDS)
    message(FATAL_ERROR "PLATFORM_KIND=hardware requires a hardware DISPLAY_BACKEND (${HARDWARE_DISPLAY_BACKENDS}; got ${DISPLAY_BACKEND})")
  endif()
  set(CUSTOM_PLATFORM arm CACHE STRING "Compatibility platform code selector (arm or x86)" FORCE)
elseif(PLATFORM_KIND STREQUAL "simulator")
  if(HOST_BACKEND STREQUAL "none")
    message(FATAL_ERROR "PLATFORM_KIND=simulator requires HOST_BACKEND to name the host runtime")
  endif()
  if(SIM_PROFILE STREQUAL "none")
    message(FATAL_ERROR "PLATFORM_KIND=simulator requires SIM_PROFILE to name the simulator data profile")
  endif()
  if(NOT DISPLAY_BACKEND IN_LIST HOST_DISPLAY_BACKENDS)
    message(FATAL_ERROR "PLATFORM_KIND=simulator requires a host DISPLAY_BACKEND (${HOST_DISPLAY_BACKENDS}; got ${DISPLAY_BACKEND})")
  endif()
  set(CUSTOM_PLATFORM x86 CACHE STRING "Compatibility platform code selector (arm or x86)" FORCE)
endif()

# Keep legacy UGFX_DRIVER consumers aligned with the explicit display backend.
set(UGFX_DRIVER ${DISPLAY_BACKEND} CACHE STRING "uGFX display backend token" FORCE)

message(STATUS "Build axes: PLATFORM_KIND=${PLATFORM_KIND}, DISPLAY_BACKEND=${DISPLAY_BACKEND}, SIM_PROFILE=${SIM_PROFILE}, HOST_BACKEND=${HOST_BACKEND}, CUSTOM_PLATFORM=${CUSTOM_PLATFORM}")

if(PLATFORM_KIND STREQUAL "hardware")
  add_compile_definitions(GAUGE_PLATFORM_HARDWARE)
elseif(PLATFORM_KIND STREQUAL "simulator")
  add_compile_definitions(GAUGE_PLATFORM_SIMULATOR)
endif()

if(DISPLAY_BACKEND STREQUAL "s6e63d6")
  add_compile_definitions(GAUGE_DISPLAY_S6E63D6)
elseif(DISPLAY_BACKEND STREQUAL "st7789vi")
  add_compile_definitions(GAUGE_DISPLAY_ST7789VI)
elseif(DISPLAY_BACKEND STREQUAL "win32")
  add_compile_definitions(GAUGE_DISPLAY_WIN32)
elseif(DISPLAY_BACKEND STREQUAL "sdl")
  add_compile_definitions(GAUGE_DISPLAY_SDL)
endif()

if(SIM_PROFILE STREQUAL "none")
  add_compile_definitions(GAUGE_SIM_PROFILE_NONE)
elseif(SIM_PROFILE STREQUAL "3000gt_soft")
  add_compile_definitions(GAUGE_SIM_PROFILE_3000GT_SOFT)
endif()

if(HOST_BACKEND STREQUAL "none")
  add_compile_definitions(GAUGE_HOST_BACKEND_NONE)
elseif(HOST_BACKEND STREQUAL "win32")
  add_compile_definitions(GAUGE_HOST_BACKEND_WIN32)
elseif(HOST_BACKEND STREQUAL "emscripten")
  add_compile_definitions(GAUGE_HOST_BACKEND_EMSCRIPTEN)
endif()
if(CUSTOM_PLATFORM STREQUAL "arm")
  add_compile_definitions(CUSTOM_PLATFORM_ARM)
  if(EXISTS "${CMAKE_SOURCE_DIR}/${CUSTOM_ROOT}/ARM/platform.cpp" AND
     EXISTS "${CMAKE_SOURCE_DIR}/${CUSTOM_ROOT}/ARM/platform_services_arm.cpp" AND
     EXISTS "${CMAKE_SOURCE_DIR}/${CUSTOM_ROOT}/ARM/runtime.cpp")
    set(CUSTOM_PLATFORM_SRC
      ${CUSTOM_ROOT}/ARM/platform.cpp
      ${CUSTOM_ROOT}/ARM/btbuffer_backend_arm.cpp
      ${CUSTOM_ROOT}/ARM/platform_services_arm.cpp
      ${CUSTOM_ROOT}/ARM/runtime.cpp
    )
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
  check_c_compiler_flag("-fcyclomatic-complexity" ARM_C_HAS_CYCLOMATIC_COMPLEXITY)
  check_cxx_compiler_flag("-fcyclomatic-complexity" ARM_CXX_HAS_CYCLOMATIC_COMPLEXITY)
  set(ARM_CYCLOMATIC_COMPLEXITY_FLAG "")
  if(ARM_C_HAS_CYCLOMATIC_COMPLEXITY AND ARM_CXX_HAS_CYCLOMATIC_COMPLEXITY)
    set(ARM_CYCLOMATIC_COMPLEXITY_FLAG -fcyclomatic-complexity)
  else()
    message(STATUS "Compiler does not support -fcyclomatic-complexity; disabling complexity report output")
  endif()
  add_compile_options(
    ${MCU_FLAGS}
    -ffunction-sections -fdata-sections
    -Wall
    -fstack-usage
    ${ARM_CYCLOMATIC_COMPLEXITY_FLAG}
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
  # Host/simulator flags
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
if(DISPLAY_BACKEND STREQUAL "win32")
  set(UGFX_DRIVER_SRCS ugfx/drivers/multiple/Win32/gdisp_lld_Win32.c)
  set(UGFX_DRIVER_INCS ugfx/drivers/multiple/Win32)
elseif(DISPLAY_BACKEND STREQUAL "sdl")
  set(UGFX_DRIVER_SRCS ugfx/drivers/multiple/SDL/gdisp_lld_SDL.c)
  set(UGFX_DRIVER_INCS ugfx/drivers/multiple/SDL)
elseif(DISPLAY_BACKEND STREQUAL "st7789vi")
  set(UGFX_DRIVER_SRCS ugfx/drivers/gdisp/ST7789VI/gdisp_lld_ST7789VI.c)
  set(UGFX_DRIVER_INCS ugfx/drivers/gdisp/ST7789VI)
elseif(DISPLAY_BACKEND STREQUAL "s6e63d6")
  set(UGFX_DRIVER_SRCS ugfx/drivers/gdisp/s6e63d6/gdisp_lld_s6e63d6.c)
  set(UGFX_DRIVER_INCS ugfx/drivers/gdisp/s6e63d6)
else()
  message(FATAL_ERROR "Unknown DISPLAY_BACKEND=${DISPLAY_BACKEND}")
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
  # Simulator build: do not compile STM-generated source trees by default
  set(PROJECT_SOURCES)
endif()

file(GLOB_RECURSE CUSTOM_SOURCES
  ${CUSTOM_ROOT}/*.c ${CUSTOM_ROOT}/*.cpp
)
if(CUSTOM_PLATFORM STREQUAL "arm")
  list(FILTER CUSTOM_SOURCES EXCLUDE REGEX ".*/${CUSTOM_ROOT}/(ARM|x86)/.*")
  list(APPEND PROJECT_SOURCES ${CUSTOM_SOURCES} ${CUSTOM_PLATFORM_SRC})
else()
  # Simulator build: use host platform files plus the shared app entrypoint.
  list(FILTER CUSTOM_SOURCES INCLUDE REGEX ".*/${CUSTOM_ROOT}/x86/.*")
  list(APPEND PROJECT_SOURCES ${CUSTOM_SOURCES})
  list(APPEND PROJECT_SOURCES ${CUSTOM_ROOT}/Src/main_shared.cpp)
  list(APPEND PROJECT_SOURCES ${CUSTOM_ROOT}/Src/build_config.cpp)
  list(APPEND PROJECT_SOURCES ${CUSTOM_ROOT}/Src/ugfx_widgets.cpp)
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
  # Simulator platform headers (including gfxconf.h).
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
