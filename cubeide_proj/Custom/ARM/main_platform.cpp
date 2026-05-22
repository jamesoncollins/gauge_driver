#ifndef __arm__
#error "THIS CODE IS FOR ARM"
#endif

#include <cstdio>
#include <cmath>

#include "cpp_main.h"
#include "main_shared.h"
#include "runtime_context.hpp"
#include "gfx.h"
#include "utils.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
extern "C" {
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
}
#include "../PI4IOE5V6416/PI4IOE5V6416.hpp"
#include "../SwitecX12-lib/SwitecX12.hpp"
#include "../BTbuffer-lib/BTBuffer.hpp"
#include "btbuffer_backend_arm.hpp"
extern "C" {
#include "../MCP4725-lib/MCP4725.h"
#include "../BMI088-lib/BMI088.h"
extern bool bus_busy();
extern void setAutoClear(bool);
}
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

struct EcuSignalMap
{
  int tps_index;
  int wb_index;
  int map_index;
  int knock_index;
};

static ECUK *g_ecu = &ecu;
static const EcuSignalMap g_ecu_signal_map = {
    MUTII::ECU_PARAM_TPS,
    MUTII::ECU_PARAM_WB,
    MUTII::ECU_PARAM_MAP,
    MUTII::ECU_PARAM_KNOCK,
};

static int gimbal_x = 0;
static int gimbal_y = 0;

void platform_poll_bulb_inputs(PI4IOE5V6416 &ioexp_screen, uint16_t &bulbVals)
{
  if (!bulbReadWaiting && !i2cPendingIrq[3])
  {
    if (ioexp_screen.get_IT(&bulbVals) == HAL_OK)
    {
      i2cPendingIrq[3] = true;
      bulbReadWaiting = true;
    }
  }

  if (bulbReadWaiting && !i2cPendingIrq[3])
    bulbReadWaiting = false;
}

static PlatformSample arm_collect_platform_sample(
    float rpm_val,
    float speed_val,
    uint16_t bulbVals,
    uint32_t loop_count,
    uint32_t loop_period_ms,
    uint32_t worst_loop_period_ms,
    flasher_t *ecu_flasher)
{
  PlatformSample sample = {};
  sample.rpm = rpm_val;
  sample.speed_mph = speed_val;
  sample.elapsed_ms = HAL_GetTick();
  sample.gimbal_x = gimbal_x;
  sample.gimbal_y = gimbal_y;
  sample.btn = btnCmd;
  sample.startup_init_error = (startupInitError != 0);
  sample.warn_batt = ((bulbVals & (uint16_t)(1U << 7)) == 0U);
  sample.warn_brake = ((bulbVals & (uint16_t)(1U << 1)) == 0U);
  sample.warn_4ws = ((bulbVals & (uint16_t)(1U << 0)) != 0U);
  sample.warn_lamp_on = ((bulbVals & (uint16_t)(1U << 2)) != 0U);
  sample.warn_high_beam = ((bulbVals & (uint16_t)(1U << 3)) == 0U);
  sample.ecu = g_ecu;
  sample.ecu_param_tps_index = g_ecu_signal_map.tps_index;
  sample.ecu_param_wb_index = g_ecu_signal_map.wb_index;
  sample.ecu_param_map_index = g_ecu_signal_map.map_index;
  sample.ecu_param_knock_index = g_ecu_signal_map.knock_index;
  sample.ecu_flasher = ecu_flasher;
  sample.loop_count = loop_count;
  sample.loop_period_ms = loop_period_ms;
  sample.worst_loop_period_ms = worst_loop_period_ms;
  return sample;
}

static void arm_bringup_hardware(
    bool &cleanPwr,
    uint32_t &GFX_AMBER,
    font_t &font10,
    font_t &font20,
    font_t &fontLCD,
    PI4IOE5V6416 &ioexp_speedo,
    PI4IOE5V6416 &ioexp_screen,
    uint16_t &bulbVals,
    SwitecX12 &tachX12,
    SwitecX12 &speedX12,
    SwitecX12 &odoX12,
    int x27_steps,
    gImage &battImg,
    gImage &beamImg,
    LinePlot_t &linePlotTPS,
    int *tpsPlotData,
    LinePlot_t &linePlotKnock,
    int *knockPlotData,
    SharedRenderCtx &arm_render_ctx,
    Gimball_t &gimball)
{
  (void)x27_steps;

  HAL_PWR_EnableBkUpAccess();
  cleanPwr = (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0xBEEF);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x0);
  HAL_PWR_DisableBkUpAccess();

  gfxInit();
  setAutoClear(false);
  gdispClear(GFX_BLACK);

  screenWidth = gdispGetWidth();
  screenHeight = gdispGetHeight();

  font10 = gdispOpenFont("DejaVuSans10");
  font20 = gdispOpenFont("DejaVuSans20");
  fontLCD = gdispOpenFont("lcddot_tr80");
  GFX_AMBER = GFX_AMBER_YEL;

  gdispImageOpenMemory(&battImg, batt);
  gdispImageOpenMemory(&beamImg, beam);
  startupInitError |= ioexp_screen.init(0x00FF, 0x00FF);
  startupInitError |= ioexp_speedo.init(0x0000, 0x0000);
  bulbVals = ioexp_screen.get();

  startupInitError |= BMI088_Init(&imu, &hi2c1);
  regAddr = BMI_ACC_DATA;

  x12[0] = &tachX12;
  x12[1] = &speedX12;
  x12[2] = &odoX12;
  tachX12.setPosition(0);
  speedX12.setPosition(0);
  odoX12.setPosition(0);
  needles_ready = true;
  measure_freq = true;

  startupInitError |= HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  startupInitError |= HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
  startupInitError |= HAL_TIM_Base_Start_IT(&htim16);
  startupInitError |= HAL_TIM_Base_Start_IT(&htim17);

  IRQn_Type bt_irqs[] = {TIM1_UP_TIM16_IRQn, TIM1_TRG_COM_TIM17_IRQn, USART1_IRQn};
  static ArmBTBufferBackend bt_backend(bt_irqs, (int)(sizeof(bt_irqs) / sizeof(bt_irqs[0])));
  BTBuffer::CreateInstance(&bt_backend);

  arm_render_ctx = {
      .amber_ptr = (color_t *)&GFX_AMBER,
      .font10 = font10,
      .font20 = font20,
      .fontLCD = fontLCD,
      .screen_width = (coord_t)screenWidth,
      .screen_height = (coord_t)screenHeight,
      .batt_img = &battImg,
      .beam_img = &beamImg,
      .gimball = &gimball,
      .line_plot_tps = &linePlotTPS,
      .line_plot_tps_data = tpsPlotData,
      .line_plot_knock = &linePlotKnock,
      .line_plot_knock_data = knockPlotData,
  };
}

void platform_process_ble_and_lowrate(uint32_t &timerLED, uint32_t &loopCnt, uint32_t loopPeriod, uint32_t worstLoopPeriod)
{
  const uint32_t now = HAL_GetTick();
  if ((now - timerLED) >= SAMPLE_TIME_MS_LED)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    timerLED = now;
  }

  (void)loopCnt;
  (void)loopPeriod;
  (void)worstLoopPeriod;
}

void platform_drain_bt_budget()
{
  for (int i = 0; i < 4; ++i)
  {
    if (!BTBuffer::popBuffer())
      break;
  }
}

void platform_update_inertial(float cosPitch, float sinPitch)
{
  if (acc_int_rdy)
  {
    acc_int_rdy = false;
    if (BMI088_ReadAccelerometer(&imu) == 0)
      return;

    const float x = imu.acc_mps2[0];
    const float y = imu.acc_mps2[1];
    const float z = imu.acc_mps2[2];

    const float yp = (y * cosPitch) - (z * sinPitch);
    gimbal_x = (int)(x * 5.0f);
    gimbal_y = (int)(yp * 5.0f);
  }
}

void platform_maybe_usb_print(uint32_t &timerPrint, int &logBufInd, char *logBuf, int bufLen)
{
#ifdef PRINT_TO_USB
  const uint32_t now = HAL_GetTick();
  if ((now - timerPrint) < SAMPLE_TIME_MS_PRINT)
    return;

  timerPrint = now;
  logBufInd += (int)std::snprintf(
      logBuf + logBufInd,
      (size_t)(bufLen - logBufInd),
      "rpm=%d speed=%d ecu=%d missed=%lu\r\n",
      (int)rpm,
      (int)speed,
      g_ecu->isConnected() ? 1 : 0,
      (unsigned long)g_ecu->getMissedReplyResetCnt());
  if (logBufInd > 0)
    CDC_Transmit_FS((uint8_t *)logBuf, (uint16_t)logBufInd);
#else
  (void)timerPrint;
  (void)logBufInd;
  (void)logBuf;
  (void)bufLen;
#endif
}

bool platform_should_exit(uint32_t &timerIGN)
{
  if (HAL_GPIO_ReadPin(IGN_GPIO_Port, IGN_Pin) == GPIO_PIN_SET)
  {
    timerIGN = HAL_GetTick();
    return false;
  }

  return ((HAL_GetTick() - timerIGN) > 10);
}

void platform_update_loop_diag(uint32_t &loopCnt, uint32_t &loopPeriod, uint32_t &worstLoopPeriod, uint32_t &timerLoop)
{
  const uint32_t now = HAL_GetTick();
  loopPeriod = now - timerLoop;
  timerLoop = now;
  if (loopPeriod > worstLoopPeriod)
    worstLoopPeriod = loopPeriod;
  loopCnt++;
}

typedef enum
{
  RPM_MODE_LOW = 0,
  RPM_MODE_EARLY_WARN = 1,
  RPM_MODE_SHIFT = 2
} rpmMode_e;

struct ArmMainCtx
{
  bool cleanPwr = false;
  uint32_t amber = GFX_AMBER_YEL;
  font_t font10 = nullptr;
  font_t font20 = nullptr;
  font_t fontLCD = nullptr;
  uint16_t bulbVals = 0;

  PI4IOE5V6416 *ioexp_speedo = nullptr;
  PI4IOE5V6416 *ioexp_screen = nullptr;
  SwitecX12 *tachX12 = nullptr;
  SwitecX12 *speedX12 = nullptr;
  SwitecX12 *odoX12 = nullptr;

  Gimball_t gimball;
  gImage battImg;
  gImage beamImg;
  LinePlot_t linePlotTPS;
  int tpsPlotData[20];
  LinePlot_t linePlotKnock;
  int knockPlotData[20];
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

  const int bufLen = 256;
  int logBufInd = 0;
  char logBuf[256];
};

static ArmMainCtx g_arm_main;

void platform_main_init(SharedRenderCtx &ctx, RuntimeState &state, int &draw_step, uint32_t &timer_draw_ms)
{
  const int X27_STEPS = 240 * 12;
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

  arm_bringup_hardware(
      g_arm_main.cleanPwr,
      g_arm_main.amber,
      g_arm_main.font10,
      g_arm_main.font20,
      g_arm_main.fontLCD,
      *g_arm_main.ioexp_speedo,
      *g_arm_main.ioexp_screen,
      g_arm_main.bulbVals,
      *g_arm_main.tachX12,
      *g_arm_main.speedX12,
      *g_arm_main.odoX12,
      X27_STEPS,
      g_arm_main.battImg,
      g_arm_main.beamImg,
      g_arm_main.linePlotTPS,
      g_arm_main.tpsPlotData,
      g_arm_main.linePlotKnock,
      g_arm_main.knockPlotData,
      ctx,
      g_arm_main.gimball);

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

  draw_step = 0;
  timer_draw_ms = now;
  state = runtime_state_from_sample(arm_collect_platform_sample(
      rpm,
      speed,
      g_arm_main.bulbVals,
      g_arm_main.loopCnt,
      g_arm_main.loopPeriod,
      g_arm_main.worstLoopPeriod,
      &g_arm_main.ecuGoodFlasher));
}

void platform_main_step(RuntimeState &state, bool &exit_requested, bool &render_requested, uint32_t timer_draw_ms)
{
  constexpr float pitch = 75.0f * M_PI / 180.0f;
  constexpr float cosPitch = cos(pitch), sinPitch = sin(pitch);

  g_arm_main.logBufInd = 0;
  platform_process_ble_and_lowrate(g_arm_main.timerLED, g_arm_main.loopCnt, g_arm_main.loopPeriod, g_arm_main.worstLoopPeriod);
  platform_drain_bt_budget();
  platform_update_inertial(cosPitch, sinPitch);
  platform_maybe_usb_print(g_arm_main.timerPrint, g_arm_main.logBufInd, g_arm_main.logBuf, g_arm_main.bufLen);

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
    auto tach = g_tach.retrieveValue();
    auto spd = g_speed.retrieveValue();
    rpm = tach.stale ? 0.0f : tach.units;
    speed = spd.stale ? 0.0f : spd.units;
  }
  g_arm_main.tachX12->setPosition(get_x12_ticks_rpm(rpm));
  g_arm_main.speedX12->setPosition(get_x12_ticks_speed(speed));
#endif
  g_arm_main.odoX12->setPosition(odo_ticks);

  g_arm_main.rpm_mode = (rpmMode_e)compute_rpm_mode_shared(rpm, (int)g_arm_main.rpm_mode);

  // Restore shift/early warning audio behavior for ARM.
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

  render_requested = ((HAL_GetTick() - timer_draw_ms) >= SAMPLE_TIME_MS_DRAW) && !bus_busy();
  if (render_requested)
  {
    platform_poll_bulb_inputs(*g_arm_main.ioexp_screen, g_arm_main.bulbVals);
    state = runtime_state_from_sample(arm_collect_platform_sample(
        rpm,
        speed,
        g_arm_main.bulbVals,
        g_arm_main.loopCnt,
        g_arm_main.loopPeriod,
        g_arm_main.worstLoopPeriod,
        &g_arm_main.ecuGoodFlasher));
    state.rpm_mode = g_arm_main.rpm_mode;
  }

  exit_requested = platform_should_exit(g_arm_main.timerIGN);
  platform_update_loop_diag(g_arm_main.loopCnt, g_arm_main.loopPeriod, g_arm_main.worstLoopPeriod, g_arm_main.timerLoop);
}

void platform_main_shutdown()
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
