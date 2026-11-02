#ifndef __arm__
#error "THIS CODE IS FOR ARM"
#endif

#include <cstdio>
#include <cmath>

#include "cpp_main.h"
#include "main_shared.h"
#include "build_config.hpp"
#include "platform_services.hpp"
#include "runtime_context.hpp"
#include "arm_runtime.hpp"
#include "gfx.h"
#include "utils.h"
#include "app_entry.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
extern "C" {
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
}
#include "../PI4IOE5V6416/PI4IOE5V6416.hpp"
#include "../SwitecX12-lib/SwitecX12.hpp"
#include "../BTBuffer-lib/BTBuffer.hpp"
#include "btbuffer_backend_arm.hpp"
extern "C" {
#include "../MCP4725-lib/MCP4725.h"
#include "../BMI088-lib/BMI088.h"
extern bool bus_busy();
extern void setAutoClear(bool);
}
#include "../../res/mitslogoanim_128.c"
#include "../../res/batt.c"
#include "../../res/beam.c"

extern I2C_HandleTypeDef hi2c1, hi2c3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern RTC_HandleTypeDef hrtc;

extern BMI088 imu;
extern uint8_t regAddr;

struct PlatformSample
{
  uint64_t data_mask;
  float rpm;
  float speed_mph;
  uint32_t elapsed_ms;
  uint32_t loop_count;
  uint32_t loop_period_ms;
  uint32_t worst_loop_period_ms;
  int gimbal_x;
  int gimbal_y;
  bool startup_init_error;
  bool warn_batt;
  bool warn_brake;
  bool warn_4ws;
  bool warn_lamp_on;
  bool warn_high_beam;
  ECUK *ecu;
  int ecu_param_tps_index;
  int ecu_param_wb_index;
  int ecu_param_map_index;
  int ecu_param_knock_index;
  int ecu_param_timing_index = 0;
  int ecu_param_afr_target_index = 0;
  int ecu_param_fuel_trim_front_low_index;
  int ecu_param_fuel_trim_front_med_index;
  int ecu_param_fuel_trim_front_high_index;
  int ecu_param_fuel_trim_rear_low_index;
  int ecu_param_fuel_trim_rear_med_index;
  int ecu_param_fuel_trim_rear_high_index;
  flasher_t *ecu_flasher;
  button_e btn;
};

struct EcuSignalMap
{
  int tps_index;
  int wb_index;
  int map_index;
  int knock_index;
  int timing_index;
  int afr_target_index;
  int fuel_trim_front_low_index;
  int fuel_trim_front_med_index;
  int fuel_trim_front_high_index;
  int fuel_trim_rear_low_index;
  int fuel_trim_rear_med_index;
  int fuel_trim_rear_high_index;
};

static ECUK *g_ecu = &ecu;
static BoardSharedData *g_board_data = nullptr;
static const EcuSignalMap g_ecu_signal_map = {
    MUTII::ECU_PARAM_TPS,
    MUTII::ECU_PARAM_WB,
    MUTII::ECU_PARAM_MAP,
    MUTII::ECU_PARAM_KNOCK,
    MUTII::ECU_PARAM_TIMING,
    MUTII::ECU_PARAM_AFR_TARGET,
    MUTII::ECU_PARAM_FFTL,
    MUTII::ECU_PARAM_FFTM,
    MUTII::ECU_PARAM_FFTH,
    MUTII::ECU_PARAM_RFTL,
    MUTII::ECU_PARAM_RFTM,
    MUTII::ECU_PARAM_RFTH,
};

static BoardAccelerationVector g_acceleration_mps2 = {};
static constexpr int X27_STEPS = 240 * 12;
static constexpr uint16_t bulb_mask(unsigned bit)
{
  return (uint16_t)(1U << bit);
}
static constexpr uint16_t BULB_4WS_MASK = bulb_mask(0);
static constexpr uint16_t BULB_BRAKE_MASK = bulb_mask(1);
static constexpr uint16_t BULB_LAMP_MASK = bulb_mask(2);
static constexpr uint16_t BULB_HIGH_BEAM_MASK = bulb_mask(3);
static constexpr uint16_t BULB_BATT_MASK = bulb_mask(7);
static constexpr uint16_t BULB_INPUT_MASK = 0x00FF;
static constexpr uint16_t BULB_PULLUP_MASK = BULB_BRAKE_MASK | BULB_BATT_MASK;
static constexpr uint32_t STARTUP_ERR_IOEXP_SCREEN = 1U << 0;
static constexpr uint32_t STARTUP_ERR_IOEXP_SPEEDO = 1U << 1;
static constexpr uint32_t STARTUP_ERR_IMU = 1U << 2;
static constexpr uint32_t STARTUP_ERR_TIM17 = 1U << 3;
static constexpr uint32_t STARTUP_ERR_TIM2_CH3 = 1U << 4;
static constexpr uint32_t STARTUP_ERR_TIM2_CH4 = 1U << 5;
static constexpr uint32_t STARTUP_ERR_TIM16 = 1U << 6;

typedef enum
{
  RPM_MODE_LOW = 0,
  RPM_MODE_EARLY_WARN = 1,
  RPM_MODE_SHIFT = 2
} rpmMode_e;

struct ArmMainCtx
{
  bool cleanPwr = false;
  font_t font10 = nullptr;
  font_t font20 = nullptr;
  font_t fontLCD = nullptr;
  font_t fontValue = nullptr;
  uint16_t bulbVals = 0;

  PI4IOE5V6416 *ioexp_speedo = nullptr;
  PI4IOE5V6416 *ioexp_screen = nullptr;
  SwitecX12 *tachX12 = nullptr;
  SwitecX12 *speedX12 = nullptr;
  SwitecX12 *odoX12 = nullptr;
  gImage battImg;
  gImage beamImg;
  BoardWarningLight *warn_batt = nullptr;
  BoardWarningLight *warn_brake = nullptr;
  BoardWarningLight *warn_4ws = nullptr;
  flasher_t ecuGoodFlasher = {.rate_ms = 500, .last_ms = 0};

  uint32_t timerLoop = 0;
  uint32_t timerLED = 0;
  uint32_t timerIGN = 0;
  uint32_t timerPrint = 0;
  uint32_t loopPeriod = 0;
  uint32_t worstLoopPeriod = 0;
  uint32_t loopCnt = 0;
  rpmMode_e rpm_mode = RPM_MODE_LOW;
  rpmMode_e last_rpm_mode = RPM_MODE_LOW;
  bool audio_started = false;
  int toggle_mode = 0;
  uint32_t toggleTime_last = 0;

  const int bufLen = 384;
  int logBufInd = 0;
  char logBuf[384];
};

static ArmMainCtx g_arm_main;
static volatile uint32_t g_startup_diag_marker = 0;

static void startup_diag_mark(uint32_t marker)
{
  g_startup_diag_marker = marker;
}

static void record_startup_error(int status, uint32_t bit)
{
  if (status != 0)
    startupInitError |= bit;
}

static void arm_reset_motor_driver()
{
  g_sensor_pipeline.driver_reset();
  HAL_GPIO_WritePin(RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_SET);
}

static void arm_run_startup_animation_and_needle_dance()
{
  startup_diag_mark(0x1000U);
  gImage startup_anim;
  if (gdispImageOpenMemory(&startup_anim, mitslogoanim_128) != GDISP_IMAGE_ERR_OK)
  {
    startup_diag_mark(0x1001U);
    g_arm_main.tachX12->setPosition(get_x12_ticks_rpm(9000));
    g_arm_main.speedX12->setPosition(get_x12_ticks_speed(180));
    while (!g_arm_main.tachX12->atTarget() || !g_arm_main.speedX12->atTarget())
      HAL_Delay(1);

    g_arm_main.tachX12->setPosition(get_x12_ticks_rpm(0));
    g_arm_main.speedX12->setPosition(get_x12_ticks_speed(0));
    while (!g_arm_main.tachX12->atTarget() || !g_arm_main.speedX12->atTarget())
      HAL_Delay(1);
    return;
  }

  startup_diag_mark(0x1010U);
  gDelay delay = 0;
  int display_count = 42;
  gdispClear(GFX_BLACK);
  startup_diag_mark(0x1011U);
  gdispImageDraw(&startup_anim,
                 (screenWidth >> 1) - (startup_anim.width >> 1),
                 75,
                 startup_anim.width, startup_anim.height,
                 0, 0);
  for (int i = 0; i < 17; ++i)
  {
    startup_diag_mark(0x1020U + (uint32_t)i);
    gdispImageNext(&startup_anim);
    --display_count;
  }

  int startup_state = 0;
  uint32_t timer_anim = HAL_GetTick();
  bool startup_done = false;
  while (!startup_done)
  {
    switch (startup_state)
    {
      case 0:
        startup_diag_mark(0x1030U);
        g_arm_main.tachX12->setPosition(get_x12_ticks_rpm(9000));
        g_arm_main.speedX12->setPosition(get_x12_ticks_speed(180));
        ++startup_state;
        break;
      case 1:
        startup_diag_mark(0x1031U);
        if (g_arm_main.tachX12->atTarget() && g_arm_main.speedX12->atTarget())
          ++startup_state;
        break;
      case 2:
        startup_diag_mark(0x1032U);
        g_arm_main.tachX12->setPosition(get_x12_ticks_rpm(0));
        g_arm_main.speedX12->setPosition(get_x12_ticks_speed(0));
        ++startup_state;
        break;
      case 3:
        startup_diag_mark(0x1033U);
        if (g_arm_main.tachX12->atTarget() && g_arm_main.speedX12->atTarget())
          ++startup_state;
        break;
      default:
        if (display_count < 0)
          startup_done = true;
        break;
    }

    if ((HAL_GetTick() - timer_anim) > delay && display_count >= 0)
    {
      startup_diag_mark(0x1040U);
      gdispImageDraw(&startup_anim,
                     (screenWidth >> 1) - (startup_anim.width >> 1),
                     75,
                     startup_anim.width, startup_anim.height,
                     0, 0);
      delay = gdispImageNext(&startup_anim);
      timer_anim = HAL_GetTick();
      startup_diag_mark(0x1041U);
      gdispFlush();
      --display_count;
    }

    HAL_Delay(1);
  }

  startup_diag_mark(0x1050U);
  gdispImageClose(&startup_anim);
  gdispClear(GFX_BLACK);
  setAutoClear(true);
  gdispClear(GFX_BLACK);
  gdispFlush();
  gdispFlush();
  startup_diag_mark(0x10FFU);
}

static void platform_poll_bulb_inputs()
{
  if (g_arm_main.ioexp_screen == nullptr)
    return;

  if (!bulbReadWaiting && !i2cPendingIrq[3])
  {
    if (g_arm_main.ioexp_screen->get_IT(&g_arm_main.bulbVals) == HAL_OK)
    {
      i2cPendingIrq[3] = true;
      bulbReadWaiting = true;
    }
  }

  if (bulbReadWaiting && !i2cPendingIrq[3])
    bulbReadWaiting = false;
}

static PlatformSample arm_collect_platform_sample()
{
  PlatformSample sample = {};
  sample.data_mask =
      PLATFORM_DATA_RPM |
      PLATFORM_DATA_SPEED_MPH |
      PLATFORM_DATA_TIMING_DIAG |
      PLATFORM_DATA_GIMBAL |
      PLATFORM_DATA_STARTUP_ERROR |
      PLATFORM_DATA_WARN_BATT |
      PLATFORM_DATA_WARN_BRAKE |
      PLATFORM_DATA_WARN_4WS |
      PLATFORM_DATA_WARN_LAMP |
      PLATFORM_DATA_WARN_HIGH_BEAM |
      PLATFORM_DATA_ECU |
      PLATFORM_DATA_BTN;
  sample.rpm = rpm;
  sample.speed_mph = speed;
  sample.elapsed_ms = HAL_GetTick();
  sample.btn = btnCmd;
  sample.startup_init_error = (startupInitError != 0);
  sample.warn_batt = ((g_arm_main.bulbVals & BULB_BATT_MASK) == 0U);
  sample.warn_brake = ((g_arm_main.bulbVals & BULB_BRAKE_MASK) == 0U);
  sample.warn_4ws = ((g_arm_main.bulbVals & BULB_4WS_MASK) != 0U);
  sample.warn_lamp_on = ((g_arm_main.bulbVals & BULB_LAMP_MASK) != 0U);
  sample.warn_high_beam = sample.warn_lamp_on && ((g_arm_main.bulbVals & BULB_HIGH_BEAM_MASK) == 0U);
  sample.ecu = g_ecu;
  sample.ecu_param_tps_index = g_ecu_signal_map.tps_index;
  sample.ecu_param_wb_index = g_ecu_signal_map.wb_index;
  sample.ecu_param_map_index = g_ecu_signal_map.map_index;
  sample.ecu_param_knock_index = g_ecu_signal_map.knock_index;
  sample.ecu_param_timing_index = g_ecu_signal_map.timing_index;
  sample.ecu_param_afr_target_index = g_ecu_signal_map.afr_target_index;
  sample.ecu_param_fuel_trim_front_low_index = g_ecu_signal_map.fuel_trim_front_low_index;
  sample.ecu_param_fuel_trim_front_med_index = g_ecu_signal_map.fuel_trim_front_med_index;
  sample.ecu_param_fuel_trim_front_high_index = g_ecu_signal_map.fuel_trim_front_high_index;
  sample.ecu_param_fuel_trim_rear_low_index = g_ecu_signal_map.fuel_trim_rear_low_index;
  sample.ecu_param_fuel_trim_rear_med_index = g_ecu_signal_map.fuel_trim_rear_med_index;
  sample.ecu_param_fuel_trim_rear_high_index = g_ecu_signal_map.fuel_trim_rear_high_index;
  sample.ecu_flasher = &g_arm_main.ecuGoodFlasher;
  sample.loop_count = g_arm_main.loopCnt;
  sample.loop_period_ms = g_arm_main.loopPeriod;
  sample.worst_loop_period_ms = g_arm_main.worstLoopPeriod;
  return sample;
}

static void arm_bringup_hardware(SharedRenderCtx &arm_render_ctx)
{
  HAL_GPIO_WritePin(PWREN_GPIO_Port, PWREN_Pin, GPIO_PIN_SET);

  HAL_PWR_EnableBkUpAccess();
  g_arm_main.cleanPwr = (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0xBEEF);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x0);
  HAL_PWR_DisableBkUpAccess();

  gfxInit();
  setAutoClear(false);
  gdispClear(GFX_BLACK);

  screenWidth = gdispGetWidth();
  screenHeight = gdispGetHeight();

  g_arm_main.font10 = gdispOpenFont("DejaVuSans10");
  g_arm_main.font20 = gdispOpenFont("DejaVuSans20");
  g_arm_main.fontLCD = gdispOpenFont("lcddot_tr80");
  g_arm_main.fontValue = gdispOpenFont("BITSUMIS84_Numbers");

  gdispImageOpenMemory(&g_arm_main.battImg, batt);
  gdispImageOpenMemory(&g_arm_main.beamImg, beam);
  record_startup_error(g_arm_main.ioexp_screen->init(BULB_PULLUP_MASK, BULB_INPUT_MASK), STARTUP_ERR_IOEXP_SCREEN);
  record_startup_error(g_arm_main.ioexp_speedo->init(0x0000, 0x0000), STARTUP_ERR_IOEXP_SPEEDO);
  g_arm_main.bulbVals = g_arm_main.ioexp_screen->get();

  record_startup_error(BMI088_Init(&imu, &hi2c1), STARTUP_ERR_IMU);
  regAddr = BMI_ACC_DATA;

  // Initialize before enabling capture IRQs. Preserve the actual TIM2 modulus.
  init_get_cycle_count();
  g_sensor_pipeline.init(get_build_config().vehicle, HAL_GetTick()*1000U,
                         htim2.Instance->ARR, RCC->CSR);

  arm_reset_motor_driver();
  {
    int step_down = 100;
    if (!g_arm_main.cleanPwr)
    {
      gdispClear(GFX_BLACK);
      gdispFillString((screenWidth >> 1) - 77, (screenHeight >> 1), "RESET", g_arm_main.fontLCD, GFX_AMBER_YEL, GFX_BLACK);
      gdispFlush();
      step_down = X27_STEPS;
    }

    for (int i = 0; i < step_down; ++i)
    {
      g_arm_main.tachX12->stepNow(-1);
      g_arm_main.speedX12->stepNow(-1);
      DWT_Delay(2000);
    }
    g_arm_main.tachX12->reset();
    g_arm_main.speedX12->reset();
    HAL_Delay(200);
  }
  arm_reset_motor_driver();

  static ArmBTBufferBackend bt_backend(nullptr, 0);
  BTBuffer::CreateInstance(&bt_backend);

  x12[0] = g_arm_main.tachX12;
  x12[1] = g_arm_main.speedX12;
  x12[2] = g_arm_main.odoX12;
  g_arm_main.tachX12->reset();
  g_arm_main.speedX12->reset();
  g_arm_main.odoX12->reset();
  needles_ready = true;
  record_startup_error(HAL_TIM_Base_Start_IT(&htim17), STARTUP_ERR_TIM17);
  arm_run_startup_animation_and_needle_dance();
  measure_freq = true;

  record_startup_error(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3), STARTUP_ERR_TIM2_CH3);
  record_startup_error(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4), STARTUP_ERR_TIM2_CH4);
  record_startup_error(HAL_TIM_Base_Start_IT(&htim16), STARTUP_ERR_TIM16);

  arm_render_ctx = {};
  arm_render_ctx.font10 = g_arm_main.font10;
  arm_render_ctx.font20 = g_arm_main.font20;
  arm_render_ctx.fontLCD = g_arm_main.fontLCD;
  arm_render_ctx.fontValue = g_arm_main.fontValue;
  arm_render_ctx.screen_width = (coord_t)screenWidth;
  arm_render_ctx.screen_height = (coord_t)screenHeight;
  arm_render_ctx.batt_img = &g_arm_main.battImg;
  arm_render_ctx.beam_img = &g_arm_main.beamImg;
  arm_render_ctx.render_cycle_complete = false;
}

static void platform_process_ble_and_lowrate()
{
  // Required for BLE/HCI scheduling; old ARM main loop called this each iteration.
  MX_APPE_Process();

  const uint32_t now = HAL_GetTick();
  if ((now - g_arm_main.timerLED) >= get_led_interval_ms())
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    g_arm_main.timerLED = now;
  }
}

static void platform_drain_bt_budget()
{
  for (int i = 0; i < 4; ++i)
  {
    if (!BTBuffer::popBuffer())
      break;
  }
}

static void platform_update_inertial()
{
  static uint32_t last_accel_poll_ms = 0;
  const uint32_t now = HAL_GetTick();
  const bool poll_due = ((now - last_accel_poll_ms) >= 20U);

  if (!acc_int_rdy && !poll_due)
    return;

  if (i2cPendingIrq[1] || HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    return;

  acc_int_rdy = false;
  last_accel_poll_ms = now;
  if (BMI088_ReadAccelerometer(&imu) != 0)
    return;

  const float x = imu.acc_mps2[0];
  const float y = imu.acc_mps2[1];
  const float z = imu.acc_mps2[2];

  g_acceleration_mps2 = {x, y, z};
}

static void platform_maybe_usb_print()
{
#ifdef PRINT_TO_USB
  const uint32_t now = HAL_GetTick();
  if ((now - g_arm_main.timerPrint) < get_print_interval_ms())
    return;

  g_arm_main.timerPrint = now;
  static uint32_t next_event=0;
  const uint32_t seq=g_sensor_pipeline.history_count;
  const uint32_t oldest=seq>16 ? seq-16 : 0;
  if(next_event<oldest) next_event=oldest;
  if(next_event>=seq) return;
  const auto &event=g_sensor_pipeline.history[next_event%16];
  const auto &f=event.frame;
  const auto &t=f.channel[GaugeSensors::Tach];
  const auto &s=f.channel[GaugeSensors::Speed];
  const int len=std::snprintf(g_arm_main.logBuf,g_arm_main.bufLen,
    "sensor us=%lu seq=%lu flags=%lu reset=%08lx loop=%lu worst=%lu "
    "tach=%d/%d/%u/%lu/%lu/%lu/%lu/%lu speed=%d/%d/%u/%lu/%lu/%lu/%lu/%lu\r\n",
    (unsigned long)f.now_us,(unsigned long)next_event,(unsigned long)event.flags,
    (unsigned long)event.reset_context,(unsigned long)f.loop_us,(unsigned long)f.worst_loop_us,
    (int)t.measured,t.target,(unsigned)t.filter.stale,
    (unsigned long)t.filter.tick_age_us,(unsigned long)t.filter.measurement_age_us,
    (unsigned long)t.filter.dropped_measurements,
    (unsigned long)t.filter.rejected_ticks,(unsigned long)t.filter.raw_tick_age_us,
    (int)s.measured,s.target,(unsigned)s.filter.stale,
    (unsigned long)s.filter.tick_age_us,(unsigned long)s.filter.measurement_age_us,
    (unsigned long)s.filter.dropped_measurements,
    (unsigned long)s.filter.rejected_ticks,(unsigned long)s.filter.raw_tick_age_us);
  if (len>0 && CDC_Transmit_FS((uint8_t *)g_arm_main.logBuf,
      (uint16_t)(len<g_arm_main.bufLen ? len : g_arm_main.bufLen-1))==0) ++next_event;
#endif
}

static bool platform_should_exit()
{
  if (HAL_GPIO_ReadPin(IGN_GPIO_Port, IGN_Pin) == GPIO_PIN_SET)
  {
    g_arm_main.timerIGN = HAL_GetTick();
    return false;
  }

  return ((HAL_GetTick() - g_arm_main.timerIGN) > 10);
}

static void platform_update_loop_diag()
{
  const uint32_t now = HAL_GetTick();
  g_arm_main.loopPeriod = now - g_arm_main.timerLoop;
  g_arm_main.timerLoop = now;
  if (g_arm_main.loopPeriod > g_arm_main.worstLoopPeriod)
    g_arm_main.worstLoopPeriod = g_arm_main.loopPeriod;
  g_arm_main.loopCnt++;
}

void board_reset_loop_diag()
{
  const uint32_t now = HAL_GetTick();
  g_arm_main.timerLoop = now;
  g_arm_main.loopPeriod = 0;
  g_arm_main.worstLoopPeriod = 0;
  g_arm_main.loopCnt = 0;
}

static void arm_publish_current_data()
{
  if (g_board_data == nullptr)
    return;

  PlatformSample sample = arm_collect_platform_sample();
  const uint32_t now = HAL_GetTick();

  g_board_data->rpm.publish(sample.rpm, now);
  g_board_data->speed_mph.publish(sample.speed_mph, now);
  g_board_data->elapsed_ms.publish(sample.elapsed_ms, now);
  g_board_data->loop_count.publish(sample.loop_count, now);
  g_board_data->loop_period_ms.publish(sample.loop_period_ms, now);
  g_board_data->worst_loop_period_ms.publish(sample.worst_loop_period_ms, now);
  g_board_data->startup_init_error_code.publish(startupInitError, now);
  g_board_data->acceleration_mps2.publish(g_acceleration_mps2, now);
  g_board_data->startup_init_error.publish(sample.startup_init_error, now);
  g_board_data->lamp_on.publish(sample.warn_lamp_on, now);
  g_board_data->btn.publish(sample.btn, now);

  g_board_data->ecu_supported = true;
  g_board_data->ecu = sample.ecu;
  g_board_data->ecu_param_tps_index = sample.ecu_param_tps_index;
  g_board_data->ecu_param_wb_index = sample.ecu_param_wb_index;
  g_board_data->ecu_param_map_index = sample.ecu_param_map_index;
  g_board_data->ecu_param_knock_index = sample.ecu_param_knock_index;
  g_board_data->ecu_param_timing_index = sample.ecu_param_timing_index;
  g_board_data->ecu_param_afr_target_index = sample.ecu_param_afr_target_index;
  g_board_data->ecu_param_fuel_trim_front_low_index = sample.ecu_param_fuel_trim_front_low_index;
  g_board_data->ecu_param_fuel_trim_front_med_index = sample.ecu_param_fuel_trim_front_med_index;
  g_board_data->ecu_param_fuel_trim_front_high_index = sample.ecu_param_fuel_trim_front_high_index;
  g_board_data->ecu_param_fuel_trim_rear_low_index = sample.ecu_param_fuel_trim_rear_low_index;
  g_board_data->ecu_param_fuel_trim_rear_med_index = sample.ecu_param_fuel_trim_rear_med_index;
  g_board_data->ecu_param_fuel_trim_rear_high_index = sample.ecu_param_fuel_trim_rear_high_index;
  g_board_data->ecu_flasher = sample.ecu_flasher;

  if (g_arm_main.warn_batt != nullptr)
    g_arm_main.warn_batt->publish(sample.warn_batt);
  if (g_arm_main.warn_brake != nullptr)
    g_arm_main.warn_brake->publish(sample.warn_brake);
  if (g_arm_main.warn_4ws != nullptr)
    g_arm_main.warn_4ws->publish(sample.warn_4ws);
  g_board_data->high_beam.publish(sample.warn_high_beam, now);
}

void board_init(BoardSharedData &data, SharedRenderCtx &ctx)
{
  g_board_data = &data;

  static const uint32_t ticks_per_us = (64000000 * 1e-6);
  static const uint32_t accelTable[5][2] = {
      {1, (uint32_t)(1.1 * 40000 * ticks_per_us)},
      {5, (uint32_t)(1.1 * 20000 * ticks_per_us)},
      {10, (uint32_t)(1.1 * 15000 * ticks_per_us)},
      {20, (uint32_t)(1.1 * 10000 * ticks_per_us)},
      {100, (uint32_t)(1.1 * 2000 * ticks_per_us)},
  };

  g_arm_main.ioexp_speedo = new PI4IOE5V6416(&hi2c1);
  g_arm_main.ioexp_screen = new PI4IOE5V6416(&hi2c3);
  g_arm_main.tachX12 = new SwitecX12(
      X27_STEPS,
      STEP_TACH_GPIO_Port,
      STEP_TACH_Pin,
      DIR_TACH_GPIO_Port,
      DIR_TACH_Pin);
  g_arm_main.speedX12 = new SwitecX12(
      X27_STEPS,
      STEP_SPEED_GPIO_Port,
      STEP_SPEED_Pin,
      DIR_SPEED_GPIO_Port,
      DIR_SPEED_Pin);
  g_arm_main.odoX12 = new SwitecX12(
      0xFFFFFFFE,
      STEP_ODO_GPIO_Port,
      STEP_ODO_Pin,
      DIR_ODO_GPIO_Port,
      DIR_ODO_Pin,
      accelTable,
      5);

  arm_bringup_hardware(ctx);

  g_arm_main.warn_batt = data.add_warning_image("batt", 140, 38, &g_arm_main.battImg);
  g_arm_main.warn_brake = data.add_warning_light("brake", "BRAKE", 110, 70, GFX_RED);
  g_arm_main.warn_4ws = data.add_warning_light("4ws", "4WS", 175, 45, GFX_YELLOW);

  const uint32_t now = HAL_GetTick();
  g_arm_main.timerLoop = now;
  g_arm_main.timerLED = now;
  g_arm_main.timerIGN = now;
  g_arm_main.timerPrint = now;
  g_arm_main.loopPeriod = 0;
  g_arm_main.worstLoopPeriod = 0;
  g_arm_main.loopCnt = 0;
  g_arm_main.rpm_mode = RPM_MODE_LOW;
  g_arm_main.last_rpm_mode = RPM_MODE_LOW;
  g_arm_main.audio_started = false;
  g_arm_main.toggle_mode = 0;
  g_arm_main.toggleTime_last = now;
  arm_publish_current_data();
}

void board_update()
{
  g_arm_main.logBufInd = 0;
  platform_process_ble_and_lowrate();
  platform_drain_bt_budget();
  platform_update_inertial();
  platform_maybe_usb_print();

  platform_poll_bulb_inputs();

#ifdef SWEEP_GAUGES
  static bool set = false;
  if (set && g_arm_main.tachX12->atTarget() && g_arm_main.speedX12->atTarget())
  {
    set = !set;
    g_arm_main.tachX12->setPosition(get_x12_ticks_rpm(9000));
    g_arm_main.speedX12->setPosition(get_x12_ticks_speed(180));
  }
  else if (!set && g_arm_main.tachX12->atTarget() && g_arm_main.speedX12->atTarget())
  {
    set = !set;
    g_arm_main.tachX12->setPosition(get_x12_ticks_rpm(0));
    g_arm_main.speedX12->setPosition(get_x12_ticks_speed(0));
  }
#elif defined(SIM_GAUGES)
  static int lastTime = 0;
  int diff = HAL_GetTick() - lastTime;
  rpm += (float)diff / 1000.0f * 3000.0f;
  speed = rpm / (9000.0f / 180.0f);
  if (rpm > 9000)
  {
    rpm = 1000;
    speed = rpm / (9000.0f / 180.0f);
  }
  lastTime = HAL_GetTick();
  g_arm_main.tachX12->setPosition(get_x12_ticks_rpm(rpm));
  g_arm_main.speedX12->setPosition(get_x12_ticks_speed(speed));
#else
  if (needles_ready && measure_freq)
  {
    const auto &frame = g_sensor_pipeline.update(HAL_GetTick()*1000U);
    rpm = frame.channel[GaugeSensors::Tach].measured;
    speed = frame.channel[GaugeSensors::Speed].measured;
    g_arm_main.tachX12->setPosition(frame.channel[GaugeSensors::Tach].target);
    g_arm_main.speedX12->setPosition(frame.channel[GaugeSensors::Speed].target);
  }
#endif
  g_arm_main.odoX12->setPosition(odo_ticks);

  g_arm_main.rpm_mode = (rpmMode_e)compute_rpm_mode_shared(rpm, (int)g_arm_main.rpm_mode);

  if (g_arm_main.rpm_mode == RPM_MODE_LOW)
  {
    if (g_arm_main.audio_started)
    {
      HAL_TIM_Base_Stop_DMA(&htim1);
      g_arm_main.audio_started = false;
    }
    g_arm_main.last_rpm_mode = g_arm_main.rpm_mode;
  }
  else if (g_arm_main.rpm_mode == RPM_MODE_EARLY_WARN)
  {
    set_tone(500.0f, 0.05f);
    HAL_TIM_Base_Start_DMA_to_SPI(&htim1, (uint32_t *)tone_buffer, (uint16_t)current_tone_len);
    g_arm_main.audio_started = true;
    g_arm_main.last_rpm_mode = g_arm_main.rpm_mode;
  }
  else if (g_arm_main.rpm_mode == RPM_MODE_SHIFT)
  {
    if (g_arm_main.last_rpm_mode != RPM_MODE_SHIFT)
    {
      g_arm_main.toggle_mode = 0;
      g_arm_main.toggleTime_last = HAL_GetTick();
      g_arm_main.last_rpm_mode = RPM_MODE_SHIFT;
    }

    if ((HAL_GetTick() - g_arm_main.toggleTime_last) > 100U)
    {
      g_arm_main.toggleTime_last = HAL_GetTick();
      g_arm_main.toggle_mode = !g_arm_main.toggle_mode;
      if (g_arm_main.toggle_mode)
        set_tone(1250.0f, 0.05f);
      else
        set_tone(2500.0f, 0.05f);
      HAL_TIM_Base_Start_DMA_to_SPI(&htim1, (uint32_t *)tone_buffer, (uint16_t)current_tone_len);
      g_arm_main.audio_started = true;
    }
  }

  arm_publish_current_data();
  platform_update_loop_diag();
}

bool board_check_exit()
{
  return platform_should_exit();
}

bool board_display_ready()
{
  return !bus_busy();
}

void board_shutdown()
{
  gdispClear(GFX_BLACK);
  gdispFillString((screenWidth >> 1) - 50, (screenHeight >> 1), "PWR", g_arm_main.fontLCD, GFX_AMBER_YEL, GFX_BLACK);
  gdispFlush();

  measure_freq = false;
  g_arm_main.speedX12->setPosition(0);
  g_arm_main.tachX12->setPosition(0);
  while (1)
  {
    if (g_arm_main.speedX12->atTarget() && g_arm_main.tachX12->atTarget())
      break;
  }

  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0xBEEF);
  HAL_PWR_DisableBkUpAccess();

  HAL_GPIO_WritePin(PWREN_GPIO_Port, PWREN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);
}
