
#include <array>

#include "main.h"
#include "cpp_main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
extern "C" {
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "../MCP4725-lib/MCP4725.h"
#include "../BMI088-lib/BMI088.h"
//#include "board_s6e63d6.h" // cant include this
extern bool bus_busy();
extern void setAutoClear(bool);
}
#include "filters.h"
#include "../SwitecX12-lib/SwitecX12.hpp"
#include "utils.h"
#include "gfx.h"
#include "ugfx_widgets.h"
#include "../Quaternion/Quaternion.hpp"
#include "../../res/mitslogoanim_128.c"
#include "../../res/batt.c"
#include "../../res/brake.c"
#include "../../res/beam.c"
#include "../PI4IOE5V6416/PI4IOE5V6416.hpp"
#include "../ECUK-lib/MUTII.hpp"
#include "../BTbuffer-lib/BTBuffer.hpp"


extern I2C_HandleTypeDef hi2c1, hi2c3;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart1;
extern RTC_HandleTypeDef hrtc;

static uint16_t screenWidth;
static uint16_t screenHeight;

static BMI088 imu;
static uint8_t regAddr;

static uint32_t startupInitError = 0;

volatile static bool ecuTxDone = false;
volatile static bool ecuRxDone = false;
static MUTII ecu(&huart1, &ecuTxDone, &ecuRxDone);

volatile static float rpm, speed;

/*
 * one per i2c channel
 * used to flag if someone is waiting for an RX interrupt.
 * These should only be set 'true' by this thread (no interrupts).
 *
 * i2c1 == speedo board stuff (i.e. acceleromter, and an io expander)
 * i2c2 == nothing
 * i2c3 == display board (i.e. io expander, and the oled psu)
 */
volatile bool i2cPendingIrq[4] = {0,0,0,0};

volatile static bool bulbReadWaiting = false;
static const bool force_all_lamps = true;
static const uint16_t lampMask         = 1<<2;
static const uint16_t beamMask         = 1<<3;
static const uint16_t psMask           = 1<<0;
static const uint16_t battMask         = 1<<7;
static const uint16_t brakeMask        = 1<<1;

// flags used by accelerometer in IT mode
volatile static bool acc_int_rdy = false;       // we got exti saying data ready
volatile static bool pendingInertial = false;


/*
 * frequency measurement settings for rpm and speed
 *
 * i originally measured that every 3 ticks of the speedo, the odo was stepped once.
 * and with our stepper i think a full step is actually 12 micro steps.
 * so the numbers below should be 3 and 12.  but those aren't looking right.
 * so i tweaked it.  well, i will tweak it once i get some measurements again.
 */
#define SPEED_TICKS_PER_ODO_TICK (3)
#define ODO_STEPS_PER_TICK (12/2)
volatile static uint32_t odo_ticks = 0;
volatile static int resetCnt1 = 0, resetCnt2 = 0;

/*
 * needle and odo steppers plus
 * control variables to start the measurements.
 */
SwitecX12 *x12[3];
volatile static bool needles_ready = false;
volatile static bool measure_freq = false;

/*
 * holds the button value sent by bluetooth
 */
volatile static  button_e btnCmd = BTN_INV;

/*
 * dac tones
 */
const int MAX_TONE_LEN = 256;
int current_tone_len = MAX_TONE_LEN;
uint16_t tone_buffer[MAX_TONE_LEN];
float fs;
const float mid = 1800;
float current_freq = 0;
void set_tone( float f, float amp )
{
  if(f==current_freq)
    return;
  current_freq = f;
  fs = 64000000.0f / (htim1.Instance->ARR+1); // timer1 is a 64mhz clock counting 0 - 6399
  float n_period = (int)(fs / f);
  current_tone_len = n_period-1;
  for(int i=0; i<n_period-1; i++)
    tone_buffer[i] = (amp * std::sin( 2. * M_PI * (float)i / (n_period-1) )) + 0x1000 + mid;
}


int main_cpp(void)
{
  // init our 64-bit system-tick counter based on teh DWT timer
  init_get_cycle_count ();

  /*
   * string buffer for logging and/or printing
   */
  const int bufLen = 256;
  int logBufInd = 0;
  char logBuf[bufLen];
  (void) logBufInd;

  /*
   * Turn on mcu-controlled pwr-en signal.
   */
  HAL_GPIO_WritePin ( PWREN_GPIO_Port, PWREN_Pin, GPIO_PIN_SET );

  /*
   * test if the board powered down correctly.
   *
   * clear the register so we are forced to reset it later
   */
  bool cleanPwr = false;  
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0xBEEF)
  {
    cleanPwr = true;
  }
  HAL_PWR_EnableBkUpAccess ();
  HAL_RTCEx_BKUPWrite (&hrtc, RTC_BKP_DR1, 0x0000);
  HAL_PWR_DisableBkUpAccess ();

  /*
   * Setup the bluetooth buffer.
   *
   * Used to allows interrupts to push data into a buffer
   * that the main loop can then send to the bluetooth.
   *
   * If multiple interrupts are going to add data to this then
   * we need to give the ffunction a list of those interrupts
   * so that it knows to disable them during critical sections of
   * code.
   */
  BTBuffer::CreateInstance( NULL, 0 );

  /*
   * init graphics library
   */
  gfxInit();
  gdispClear(GFX_BLACK);
  gdispFlush();
  screenWidth = gdispGetWidth();
  screenHeight = gdispGetHeight();
  uint32_t GFX_AMBER = GFX_AMBER_YEL;

  font_t font10 = gdispOpenFont("DejaVuSans10");
  (void) font10;
  font_t font20 = gdispOpenFont("DejaVuSans20");
  font_t fontLCD = gdispOpenFont("lcddot_tr80");

  /*
   * dac setup
   */
  {
    htim1.Instance->ARR = 1600-1;
    int16_t buffer = 0x1000 + 2048;
    HAL_SPI_Transmit (&hspi1, (uint8_t*)&buffer, 2 , HAL_MAX_DELAY);
    set_tone(
        3000,
        1000
       );
  }


  /*
   * speedo pcb io expander
   */
  PI4IOE5V6416 ioexp_speedo(&hi2c1);
  if(ioexp_speedo.init(
        0x0000,  // no pull-ups
        0x00FF  // port 0 are inputs, port 1 are outputs
      ))
  {
    startupInitError |= 1<<0;
  }

  /*
   * screen pcb io expander
   */
  uint16_t bulbVals = 0;
  PI4IOE5V6416 ioexp_screen(&hi2c3);
  if(ioexp_screen.init(
        0x0000
        | brakeMask     // switch pulls bulb down, need to mimic bulb voltage
        | battMask      // battery monitor goes through comparator with open-drain output
      )
    )
  {
    startupInitError |= 1<<1;
  }

  /*
   * Acc / Gyro setup
   */
  if(BMI088_Init(&imu, &hi2c1))
  {
    //exit(-1);
  }

  /*
   * start timers
   */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim17);

  /*
   * tach and speedo freq measurement setup
   */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3); // speed
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4); // tach

  /*
   * needle drivers
   *
   * reset the driver chip, then configure each motor driver
   */
  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_RESET );
  HAL_Delay(10);
  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_SET );

  //const int X27_STEPS = 315*12; // 315 degrees, 12 microsteps each?
  const int X27_STEPS = 240*12;

  SwitecX12 tachX12(
      X27_STEPS,
      STEP_TACH_GPIO_Port,
      STEP_TACH_Pin,
      DIR_TACH_GPIO_Port,
      DIR_TACH_Pin
      );

  SwitecX12 speedX12(
      X27_STEPS,
      STEP_SPEED_GPIO_Port,
      STEP_SPEED_Pin,
      DIR_SPEED_GPIO_Port,
      DIR_SPEED_Pin
      );

  static const uint32_t ticks_per_us =  ( 64000000 * 1e-6);
  static const uint32_t accelTable[5][2] =
  {
  { 1,   (uint32_t) 1.1 * 40000 * ticks_per_us },
  { 5,   (uint32_t) 1.1 * 20000 * ticks_per_us },
  { 10,  (uint32_t) 1.1 * 15000 * ticks_per_us },
  { 20,  (uint32_t) 1.1 * 10000 * ticks_per_us },
  { 100, (uint32_t) 1.1 * 2000 * ticks_per_us },
  };
  SwitecX12 odoX12(
      0xFFFFFFFE,  // hopefully infinite
      STEP_ODO_GPIO_Port,
      STEP_ODO_Pin,
      DIR_ODO_GPIO_Port,
      DIR_ODO_Pin,
      accelTable, 5,
      true      // reverse direction for odometer ticks
      );
  odoX12.currentStep = 0;
  odoX12.targetStep = 0;

  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_RESET );
  HAL_Delay(10);
  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_SET );

  /*
   * calibrate the needles by bumping them against the stops
   */
  int stepDown = 100 ;
  if(!cleanPwr)
  {
    gdispClear(GFX_BLACK);
    gdispFillString((screenWidth>>1)-77, (screenHeight>>1), "RESET", fontLCD, GFX_AMBER, GFX_BLACK);
    gdispFlush();
    stepDown = X27_STEPS;
  }
  for(int i=0; i<stepDown; i++)
  {
    tachX12.stepNow(-1);
    speedX12.stepNow(-1);
    DWT_Delay(2000);
  }
  tachX12.reset();
  speedX12.reset();
  //odoX12.reset();
  HAL_Delay(200);

  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_RESET );
  HAL_Delay(10);
  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_SET );


  /*
   * tell the timer interrupt to start updating the needles
   */
  x12[0] = &tachX12;
  x12[1] = &speedX12;
  x12[2] = &odoX12;
  needles_ready = true;  // dont turn this on if you dont want to use the timer


  /*
   * load startup animation resources
   */
  gImage startupAnim;
  gdispImageOpenMemory(&startupAnim, mitslogoanim_128);
  gDelay delay = 0;
  int displayCnt = 42; // number of logo frames
  gdispClear(GFX_BLACK);
  gdispImageDraw(&startupAnim,
                 (screenWidth>>1)-(startupAnim.width>>1),
                 75,
                 startupAnim.width, startupAnim.height,
                 0, 0);
  for(int i=0; i<17; i++)
  {
    // skip ahead before displaying.  logo anim is too long.
    gdispImageNext (&startupAnim);
    displayCnt--;
  }


  /*
   * do gauge dance and startup animation
   */
  int startup_state = 0;
  uint32_t timerAnim = HAL_GetTick ();
  bool startup_done = false;
  while(!startup_done)
  {
    switch(startup_state)
    {
      case 0:
        tachX12.setPosition (get_x12_ticks_rpm(9000));
        speedX12.setPosition (get_x12_ticks_speed(180) );
        startup_state++;
        break;
      case 1:
        if(tachX12.atTarget() && speedX12.atTarget())
          startup_state ++;
        break;
      case 2:
        tachX12.setPosition (get_x12_ticks_rpm(0));
        speedX12.setPosition (get_x12_ticks_speed(0) );
        startup_state++;
        break;
      case 3:
        if(tachX12.atTarget() && speedX12.atTarget())
          startup_state++;
        break;
      default:
        if(displayCnt<0)
          startup_done = true;
        break;
    }

    if(HAL_GetTick() - timerAnim > delay && displayCnt>=0)
    {
      // load the next image
      gdispImageDraw(&startupAnim,
                     (screenWidth>>1)-(startupAnim.width>>1),
                     75,
                     startupAnim.width, startupAnim.height,
                     0, 0);
      delay = gdispImageNext (&startupAnim);
      timerAnim = HAL_GetTick ();
      gdispFlush();
      displayCnt--;
    }
  }
  gdispImageClose (&startupAnim);
  
  /*
   * gif animations rely on persistant pixels, so we need autoClearing
   * to be off. until its done.
   * 
   * auto clearing the screen buffer will happen any time the display 
   * is flushed.
   */
  gdispClear(GFX_BLACK); // not sure if this clear fixes the temporary display of the logo over the gauges
  setAutoClear(true);
  gdispClear(GFX_BLACK); // or if its this one
  gdispFlush(); // not sure why we flush twice here, maybe becuase its required by autoflush somehow
  gdispFlush();

  /*
   * tell the interrupt it should start setting needle pos based on rpm is calcs
   */
  measure_freq = true;

  /*
   * load image resources for warning indicators
   */
  gImage battImg, beamImg; //brakeImg;
  gdispImageOpenMemory(&battImg, batt);
  gdispImageOpenMemory(&beamImg, beam);
  //gdispImageOpenMemory(&brakeImg, brake);

  /*
   * setup graphics structs
   */
  LinePlot_t linePlotTPS;
  int tpsPlotData[20];
  linePlotTPS.lineWidth = 3;
  linePlotInit(&linePlotTPS, tpsPlotData,
               20,
               200, 50,
               100,
               0);

  LinePlot_t linePlotKnock;
  int knockPlotData[20];
  linePlotKnock.lineWidth = 3;
  linePlotInit(&linePlotKnock, knockPlotData,
               20,
               200,
               50,
               15,
               GFX_RED);

  Gimball_t gimball;

  /*
   * assume that the board is mounted with a known pitch angle, but that there is no yaw or roll
   */
  constexpr float pitch = 75. * M_PI / 180.0;
  constexpr float cosPitch = cos(pitch), sinPitch = sin(pitch);

  /*
   * Main while loop.
   *
   * We stay here until the ignition turns off.
   */

  /* Timers */
  uint32_t timerLoop = HAL_GetTick ();
  uint32_t timerLED     = timerLoop;
  uint32_t timerIGN = timerLoop;
  uint32_t timerPrint = timerLoop;
  uint32_t timerDraw = timerLoop;
  int drawStep = 0; // breakup drawing into multtiple steps so avoid doing too much in one loop
  uint32_t loopPeriod = 0, worstLoopPeriod = 0, loopCnt = 0;
  while (1)
  {
    logBufInd = 0;

    /*
     * Process bluetooth transactions
     */
    MX_APPE_Process();

    /*
     * Used to just be for toggling the LED, now its used for any low rate updates
     */
    if ((HAL_GetTick () - timerLED) >= SAMPLE_TIME_MS_LED)
    {
      // needs to be called occasionally to avoid looping
      get_cycle_count();

      HAL_GPIO_TogglePin ( LED_GPIO_Port, LED_Pin );
      timerLED = HAL_GetTick ();

      static uint32_t dataBuffer[BTBuffer::dataLen / 4];
      dataBuffer[0] = loopCnt++;
      dataBuffer[1] = loopPeriod;
      dataBuffer[2] = worstLoopPeriod;
      dataBuffer[3] = 0;
      dataBuffer[4] = speed;
      dataBuffer[5] = rpm;
      BTBuffer::pushBuffer(0,0,HAL_GetTick(),(uint8_t*)dataBuffer, BTBuffer::dataLen);
    }

    /**
     * spend a fixed amount of time sending data to the bluetooth stack
     */
    uint32_t start = HAL_GetTick();
    while(
        HAL_GetTick() < start + 2
        && BTBuffer::popBuffer() )
    {

    }

    /**
     * bmi088 triggered us that data is available, or an I2C interrupt
     * has told us that new data has been collected, but needs to be converted
     */
    if( acc_int_rdy && !i2cPendingIrq[1] )
    {
        regAddr = BMI_ACC_DATA;
        uint8_t status = 1;
        status = HAL_I2C_Mem_Read_IT(
          &hi2c1,
          ACC_ADDR,
          BMI_ACC_DATA, 1,
          imu.accRxBuf, 6);
        if(status==0)
        {
          acc_int_rdy = false;
          i2cPendingIrq[1] = true;
          pendingInertial = true;
        }
    }
    else if( pendingInertial && !i2cPendingIrq[1]  )
    {
      BMI088_ConvertAccData(&imu); // converts raw buffered data to accel floats
      pendingInertial = false;
      rotateVectorKnownPitch(imu.acc_mps2, cosPitch, sinPitch);
    }

    /*
     * USB Print Logging
     */
    if ((HAL_GetTick () - timerPrint) >= SAMPLE_TIME_MS_PRINT)
    {
#ifdef PRINT_TO_USB
      timerPrint = HAL_GetTick ();
      logBufInd += snprintf (logBuf+logBufInd, bufLen-logBufInd, " tach: %d , speedo %d  \n",  (int)rpm,  (int)speed);
      CDC_Transmit_FS ((uint8_t*) logBuf, logBufInd);
#endif
    }


    /*
     * Draw to the screen
     *
     * But only if its time to draw, and if there isn't a pending action
     * between the screen buffer and the SPI interface.
     *
     * Best possible rate is (16 mbps) / (240*255*2 bytes) = 16.3Hz
     *
     * In practice you can get about 13 if you draw nothing at all
     * and currently, about 9.5fps if you draw what's here.
     *
     * SAMPLE_TIME_MS_DRAW will limit the frame frame to 1/SAMPLE_TIME_MS_DRAW
     * but in practice we're actually limited by bus_busy()
     */
    if (
         (HAL_GetTick () - timerDraw) >= SAMPLE_TIME_MS_DRAW
         && !bus_busy()
        )
    {

      /*
       * do draw operations in a bunch of smaller steps
       */
      switch(drawStep++)
      {
        case 0:
          /*
           * There are a few ways that clearing the screen can behaves.
           *
           * Some screens dont support it, and this will return immediatly.
           *
           * Other times, like the default for our screen, this will for a memset
           * of the entire display buffer, which can take awhile.
           *
           * Also for our screen, if its in auto-clear mode, this function will
           * return immediatly as long as that has already finished.  Given that
           * we dont enter this section of code uncless the display bus isn't busy
           * then we know this function call below will always return immediatly
           */
          if(rpm > RPM_ALERT_INIT)
            gdispClear(GFX_YELLOW);
          else if (rpm > RPM_ALERT_FINAL)
            gdispClear(GFX_RED);
          else
            gdispClear(GFX_BLACK);
          break;

        case 1:
          static const int gimbal_radius = 45;
          drawGimball ( &gimball, 168+10, 48-0, gimbal_radius,
                        -imu.acc_mps2[1] * 1.f / (9.8f / 1.f) * gimbal_radius,
                        -imu.acc_mps2[0] * 1.f / (9.8f / 1.f) * gimbal_radius
                       );
          break;

        case 2:
          snprintf (logBuf, bufLen, "%s", ecu.getValString(MUTII::ECU_PARAM_WB));
          gdispFillString(20, 20, logBuf, fontLCD, GFX_AMBER, GFX_BLACK);
          drawHorzBarGraph (20, 57, 80, 15, 19, 9, ecu.getVal(MUTII::ECU_PARAM_WB));

          snprintf (logBuf, bufLen, "ECU:%s-%ld", ecu.getStatus(), ecu.getMsgRate());
          gdispFillString(20, 80, logBuf, font20, GFX_AMBER, GFX_BLACK);

          if(!ecu.isConnected())
          {
            static flasher_t ecuGoodFlasher = {.rate_ms = 500, .last_ms = 0};
            flasher(&ecuGoodFlasher, gdispFillString(20, 80, "ECU ERR      ", font20, GFX_RED, GFX_BLACK));
          }
          break;

        case 3:
        {
          static char tmpString[4] = {'N',0,0,0};
          switch(btnCmd)
          {
            case BTN_OK:
              tmpString[0] = 'O';
              break;
            case BTN_U:
              tmpString[0] = 'U';
              break;
            case BTN_D:
              tmpString[0] = 'D';
              break;
            case BTN_L:
              tmpString[0] = 'L';
              break;
            case BTN_R:
              tmpString[0] = 'R';
              break;
            default:
              break;
          }
          gdispFillString(20, 100, tmpString, font20, GFX_AMBER, GFX_BLACK);
        }
        break;

        case 4:
          snprintf (logBuf, bufLen, "%d", (int)speed);
          gdispFillString(15, 110+42, logBuf, fontLCD, GFX_AMBER, GFX_BLACK);
          snprintf (logBuf, bufLen, "%d", (int)rpm);
          gdispFillString(15, 155+42, logBuf, fontLCD, GFX_AMBER, GFX_BLACK);
          break;

        case 5:
          if(ecu.getParam(MUTII::ECU_PARAM_TPS)->isNew)
          {
            linePlotPush(&linePlotTPS, (int)ecu.getVal(MUTII::ECU_PARAM_TPS));
          }
          linePlot(10, 149, &linePlotTPS);

          if(ecu.getParam(MUTII::ECU_PARAM_KNOCK)->isNew)
          {
            linePlotPush(&linePlotKnock, (int)ecu.getVal(MUTII::ECU_PARAM_KNOCK));
          }
          linePlot(10, 149, &linePlotKnock);
          break;

        case 6:
          if(startupInitError)
          {
            gdispFillString(
                (screenWidth>>1)-50, (screenHeight>>1),
                "ERR",
                fontLCD, GFX_RED, GFX_BLACK
                );
          }

          /*
           * If a bulb read is not pending, and no i2c irq is pending
           * then submit a request
           */
          if( !bulbReadWaiting && !i2cPendingIrq[3] )
          {
            if(ioexp_screen.get_IT(&bulbVals)==0) //we dont know when this will finish, dont care
            {
              i2cPendingIrq[3] = true;
              bulbReadWaiting = true;
            }
          }
          else if( bulbReadWaiting && !i2cPendingIrq[3] )
          {
            // we have new values
            bulbReadWaiting = false;
          }

          if( !(bulbVals&battMask) || force_all_lamps ) // voltage threshold
            gdispImageDraw(&battImg,  140,  200, battImg.width,  battImg.height,  0, 0);
          if( !(bulbVals&brakeMask) || force_all_lamps ) // car pulls down
            gdispFillString(120, 233, "BRAKE", font20, GFX_RED, GFX_BLACK);
           //gdispImageDraw(&brakeImg, 100,  230, brakeImg.width, brakeImg.height, 0, 0);
          if(  (bulbVals&psMask) || force_all_lamps ) // car pulls HIGH
            gdispFillString(175, 205, "4WS", font20, GFX_YELLOW, GFX_BLACK);
          if (bulbVals&lampMask || force_all_lamps )
          {
            // headlights are on
            GFX_AMBER = GFX_AMBER_SAE;
            setColors(GFX_AMBER,GFX_RED,GFX_BLACK);
            if( !(bulbVals&beamMask) || force_all_lamps )
            {
              // high beam on
              gdispImageDraw(&beamImg, 190,  223, beamImg.width,  beamImg.height,  0, 0);
            }
          }
          else
          {
            // headlights are off
            GFX_AMBER = GFX_AMBER_YEL;
            setColors( GFX_AMBER, GFX_RED, GFX_BLACK );
          }
          break;

        default:
          // debug / diag messages
//#define DIAG_SQUARE
#ifdef DIAG_SQUARE
          static uint32_t displayTime = 0;
          static const int xdiag = 30, ydiag = 192;
          gdispDrawBox(xdiag-1,ydiag-1,60,62,GFX_AMBER);

          snprintf (logBuf, bufLen, "ECU: %lu/%lu", ecu.getMsgRate(), ecu.getMissedReplyResetCnt());
          gdispFillString(xdiag, ydiag+00, logBuf, font10, GFX_AMBER, GFX_BLACK);

          snprintf (logBuf, bufLen, "%d/%d/%lu/%lu",
                    (int)loopPeriod ,
                    (int) worstLoopPeriod,
                    HAL_GetTick() - displayTime,
                    1000/(HAL_GetTick() - displayTime)    // FPS
                    );
          displayTime = HAL_GetTick();
          gdispFillString(xdiag, ydiag+10, logBuf, font10, GFX_AMBER, GFX_BLACK);

          snprintf (logBuf, bufLen, "%d", bulbVals);
          gdispFillString(xdiag, ydiag+40, logBuf, font10, GFX_AMBER, GFX_BLACK);

          snprintf (logBuf, bufLen, "%lu / %lu", x12[0]->getTargetPosition(), x12[1]->getTargetPosition());
          gdispFillString(xdiag, ydiag+50, logBuf, font10, GFX_AMBER, GFX_BLACK);
#endif
          gdispFlush();
          drawStep = 0;
          timerDraw = HAL_GetTick ();
          break;
      }
    }

#ifdef SWEEP_GAUGES
    /*
     * temporarily sweep needle back and forth
     */
    static bool set = false;
    if ( set && tachX12.atTarget() && speedX12.atTarget() )
    {
      set = !set;
      tachX12.setPosition (get_x12_ticks_rpm(9000));
      speedX12.setPosition (get_x12_ticks_speed(180) );
    }
    else if ( !set && tachX12.atTarget() && speedX12.atTarget() )
    {
      set = !set;
      tachX12.setPosition (get_x12_ticks_rpm(0));
      speedX12.setPosition (get_x12_ticks_speed(0) );
    }
#elif defined(SIM_GAUGES)
    /*
     * simulate reasonable speed and rpm signals
     */
    static int lastTime = 0;
    int diff = HAL_GetTick () - lastTime;
    rpm += (float) diff / 1000. * 3000; // 3000rpm per second
    speed = rpm / (9000./180.);
    if (rpm > 9000)
    {
      rpm = 1000;
      speed = rpm / (9000./180.);
    }
    lastTime = HAL_GetTick ();
    tachX12.setPosition( get_x12_ticks_rpm(rpm) );
    speedX12.setPosition( get_x12_ticks_speed(speed) );
#else
    tachX12.setPosition( get_x12_ticks_rpm(rpm) );
    speedX12.setPosition( get_x12_ticks_speed(speed) );
#endif
    odoX12.setPosition(odo_ticks);


    /*
     * shift alert
     */
    static int rpm_mode = -1; // 0 - no alert, 1 - early warning, 2 - critical
    static int toggle_mode = 0;
    static int toggleTime_last = 0;
    if(rpm <  RPM_ALERT_INIT)
    {
      if(rpm_mode!=0)
      {
        rpm_mode = 0;
        HAL_TIM_Base_Stop_DMA(&htim1);
      }
    }
    else if (rpm>RPM_ALERT_INIT && rpm<RPM_ALERT_FINAL && rpm_mode<=1 )
    {
      if(rpm_mode!=1)
      {
        HAL_TIM_Base_Stop_DMA(&htim1);
        htim1.Instance->ARR = 2200;
        HAL_TIM_Base_Start_DMA_to_SPI(&htim1, (uint32_t*)tone_buffer, current_tone_len);
        rpm_mode = 1;
      }
    }
    else
    {
      if(rpm_mode!=2)
      {
        toggle_mode = 0;
        toggleTime_last = HAL_GetTick ();
        HAL_TIM_Base_Stop_DMA(&htim1);
        htim1.Instance->ARR = 800;
        rpm_mode = 2;
      }

      if(HAL_GetTick () - toggleTime_last < 50 && toggle_mode == 0)
      {
        toggle_mode = 1;
        HAL_TIM_Base_Start_DMA_to_SPI(&htim1, (uint32_t*)tone_buffer, current_tone_len);
      }
      else if(HAL_GetTick () - toggleTime_last > 50 && HAL_GetTick () - toggleTime_last < 100 && toggle_mode == 1)
      {
        HAL_TIM_Base_Stop_DMA(&htim1);
        toggle_mode = 2;
      }
      else if( HAL_GetTick () - toggleTime_last > 100 && toggle_mode == 2 )
      {
        toggle_mode = 0;
        toggleTime_last = HAL_GetTick ();
      }
    }


    /*
     * control the speedo boards rpm alert pins based on the toggle mode
     * for the rpm section above.
     *
     * this pins will toggle at the same rate the the oboard speaker toggles.
     * i.e. it goes solid for the first alert, then rapidly beeps for the critical alert.
     */
    if( rpm_mode==1 || (toggle_mode==1 && rpm_mode==2) )
    {
      for(int i=11; i<=14; i++)
      {
        if(!ioexp_speedo.getOutputState(i) && !i2cPendingIrq[1] )
          ioexp_speedo.set_IT(i,1);
      }
    }
    else
    {
      for(int i=11; i<=14; i++)
      {
        if(ioexp_speedo.getOutputState(i) && !i2cPendingIrq[1] )
          ioexp_speedo.set_IT(i,0);
      }
    }



    /*
     * check ignition signal status.  we want it low for at least 10ms
     * before we decide the +car is off.
     */
    GPIO_PinState ign = HAL_GPIO_ReadPin ( IGN_GPIO_Port, IGN_Pin );
    if ( !ign  )
    {
      if( !timerIGN )
	timerIGN = HAL_GetTick ();
      else if ( (HAL_GetTick () - timerIGN) >= 10 )
	break; // break the main loop
    }

    /*
     * misc loop diagnostics
     */
    loopCnt++;
    loopPeriod = (HAL_GetTick () - timerLoop);
    worstLoopPeriod = (loopPeriod>worstLoopPeriod) ? loopPeriod : worstLoopPeriod;
    timerLoop = HAL_GetTick ();

  } // main while loop.  we leave when ignition signal goes low.


  gdispClear(GFX_BLACK);
  gdispFillString((screenWidth>>1)-50, (screenHeight>>1), "PWR", fontLCD, GFX_AMBER, GFX_BLACK);
  gdispFlush();


  /*
   * Power-down loop, do anything we need to in order to cleanup before
   * we power off.
   */
  measure_freq = false; // stop seting rpm & speed from measured data
  speedX12.setPosition(0);
  tachX12.setPosition(0);
  while(1)
  {
    if(speedX12.atTarget() && tachX12.atTarget())
      break;
  }

  /*
   * store the fact that we shutdown clean
   */
  HAL_PWR_EnableBkUpAccess ();
  HAL_RTCEx_BKUPWrite (&hrtc, RTC_BKP_DR1, 0xBEEF);
  HAL_PWR_DisableBkUpAccess ();

  /*
   * we left the main loop, we can power down.
   *
   * this command will disable the 3.3v and 5v regulators.
   */
  HAL_GPIO_WritePin ( PWREN_GPIO_Port, PWREN_Pin, GPIO_PIN_RESET );
  HAL_Delay(1000);

  return 0;
}



int get_x12_ticks_speed( float speed )
{
  const float MIN_MPH = 10;
  const float ZERO_ANGLE = 3;    // degrees beyond the stopper to get to 0
  const float MIN_MPH_ANGLE = 0; // degrees from zero to MIN_MPH
  const float DEGREES_PER_MPH = 1.35;

  if( speed <= 1 )
    return ZERO_ANGLE * 12.;
  else if( speed<=MIN_MPH )
    return (MIN_MPH_ANGLE + ZERO_ANGLE) * 12.;
  else
    return ((speed-MIN_MPH) * DEGREES_PER_MPH + ZERO_ANGLE + MIN_MPH_ANGLE ) * 12.;
}


int get_x12_ticks_rpm( float rpm )
{
  const float MIN_RPM = 500;
  const float ZERO_ANGLE = 5;    // degrees beyond the stopper to get to 0
  const float MIN_RPM_ANGLE = 0; // degrees from zero to MIN_RPM
  const float DEGREES_PER_RPM = 22.1 / 1000.;

  if( rpm <= 1 )
    return ZERO_ANGLE * 12.;
  else if( rpm<=MIN_RPM )
    return (MIN_RPM_ANGLE + ZERO_ANGLE) * 12.;
  else
    return ((rpm-MIN_RPM) * DEGREES_PER_RPM + ZERO_ANGLE + MIN_RPM_ANGLE ) * 12.;
}

/**
 *
 * THis is a copy of HAL_TIM_Base_Start_DMA that we have modified
 *
  * @brief  Starts the TIM Base generation in DMA mode.
  * @param  htim TIM Base handle
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to peripheral.
  * @retval HAL status
  */
extern "C" {
  /**
    * @brief  TIM DMA Period Elapse complete callback.
    * @param  hdma pointer to DMA handle.
    * @retval None
    */
  static void TIM_DMAPeriodElapsedCplt(DMA_HandleTypeDef *hdma)
  {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (htim->hdma[TIM_DMA_ID_UPDATE]->Init.Mode == DMA_NORMAL)
    {
      htim->State = HAL_TIM_STATE_READY;
    }

  #if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->PeriodElapsedCallback(htim);
  #else
    HAL_TIM_PeriodElapsedCallback(htim);
  #endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }

  /**
    * @brief  TIM DMA Period Elapse half complete callback.
    * @param  hdma pointer to DMA handle.
    * @retval None
    */
  static void TIM_DMAPeriodElapsedHalfCplt(DMA_HandleTypeDef *hdma)
  {
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  #if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->PeriodElapsedHalfCpltCallback(htim);
  #else
    HAL_TIM_PeriodElapsedHalfCpltCallback(htim);
  #endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
  }
}
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA_to_SPI(TIM_HandleTypeDef *htim, const uint32_t *pData, uint16_t Length)
{
  uint32_t tmpsmcr;

  /* Check the parameters */
  assert_param(IS_TIM_DMA_INSTANCE(htim->Instance));

  /* Set the TIM state */
  if (htim->State == HAL_TIM_STATE_BUSY)
  {
    return HAL_BUSY;
  }
  else if (htim->State == HAL_TIM_STATE_READY)
  {
    if ((pData == NULL) || (Length == 0U))
    {
      return HAL_ERROR;
    }
    else
    {
      htim->State = HAL_TIM_STATE_BUSY;
    }
  }
  else
  {
    return HAL_ERROR;
  }

  /* Set the DMA Period elapsed callbacks */
  htim->hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = TIM_DMAPeriodElapsedCplt;
  htim->hdma[TIM_DMA_ID_UPDATE]->XferHalfCpltCallback = TIM_DMAPeriodElapsedHalfCplt;

  /* Set the DMA error callback */
  htim->hdma[TIM_DMA_ID_UPDATE]->XferErrorCallback = TIM_DMAError ;

  /* Enable the DMA channel */
  if (HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_UPDATE], (uint32_t)pData, (uint32_t)&hspi1.Instance->DR,
                       Length) != HAL_OK)
  {
    /* Return error status */
    return HAL_ERROR;
  }

  /* Enable the TIM Update DMA request */
  __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_UPDATE);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TIM_SLAVE_INSTANCE(htim->Instance))
  {
    tmpsmcr = htim->Instance->SMCR & TIM_SMCR_SMS;
    if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __HAL_TIM_ENABLE(htim);
    }
  }
  else
  {
    __HAL_TIM_ENABLE(htim);
  }

  /* Return function status */
  return HAL_OK;
}

/*
 * These functions are all extern becuase they are generally
 * callbacks for interrupts from c.
 */
extern "C"
{

/*
 * bluetooth notification handler
 */
void handleButton(uint8_t button_char)
{
  button_e btn;
  btnCmd = (button_e)button_char;
  btn = BTN_INV;
  Custom_STM_App_Update_Char(
      CUSTOM_STM_BUTTONPRESS,
      (uint8_t*)&btn
      );
}

/*
 * call by a timer to update needle positions regularly
 */
uint32_t update_needles ()
{
  uint32_t v1 = x12[0]->update ();
  uint32_t v2 = x12[1]->update ();
  x12[2]->update ();
  return (v1<v2) ? v1 : v2;
}

/*
 * exti interrupts from IMU
 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT_ACC_Pin)
  {
    acc_int_rdy = true;
  }
  else if (GPIO_Pin == INT_GYR_Pin)
  {
    //BMI088_ReadGyroscopeDMA (&imu);
  }
}

/*
 * this callback fires when the acceleromter read finishes
 * or the iio expander on the display board.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance==hi2c3.Instance)
  {
    i2cPendingIrq[3] = false;
  }
  else if(hi2c->Instance==hi2c1.Instance)
  {
    i2cPendingIrq[1] = false;
  }
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance==hi2c3.Instance)
  {

  }
  else if(hi2c->Instance==hi2c1.Instance)
  {

  }
}

/*
 *
 * Speed and Tach exti
 *
 * at 100hz it takes 288 seconds to go 7 miles (4114 ticks / mile)
 * 4114 ticks = 1 mile, 4114 hz = 1 mile per second, 4114hz/3600s = 1.143hz = 1 mph
 *
 * the gauge face says 1025rev=1mile, so that would be 4100 ticks, that would be 1.38888hz/mph
 *
 * See https://www.3si.org/threads/speed-sensor-gear-ratio.831219/#post-1056408948
 *
 * 27 tooth variant (trans pre feb 1993?) 1.117hz/mph
 * 28 tooth 1.078 hz/mp
 * Initial testing is showing this to be way off.  I display 100mph,
 * but gps says 60mph.
 *
 * speed and RPM are measured by TIM2, which is 1MHz and ticks 100000 times (100ms) per overflow
 *
 */
enum
{
  SPEEDOIND = 0,
  TACHIND = 1,
};
enum
{
  IDLE = 0,
  DONE = 1,
};
#define F_CLK  (1000000)        // TIM2 uses 1MHz cock
#define TIM2_MAX_CNT 100000
#define OVERFLOW_MS ((int)(100)) // TIM2 counts to 100000-1, so every 100ms
const float MPH_PER_HZ = ( 0.8425872 ); //(1.11746031667) //( 1.07755102 )
const float  RPM_PER_HZ = ( 20. ); // 3 ticks per revolution
volatile static uint8_t state[2] = {IDLE, IDLE};
volatile static uint32_t T1[2] = {0,0};
volatile static uint32_t T2[2] = {0,0};
volatile static float ticks[2] = {0,0};
volatile static uint32_t rejects[2];
volatile static uint32_t TIM2_OVC[2] = {0,0};
volatile static uint32_t speed_tick_count = 0;
static std::array<SMA<8>, 2> sma = {
    SMA<8>(0.5f, 0.125f),
    SMA<8>(0.5f, 0.125f),
};

void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
  int ch = (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) ? SPEEDOIND : TACHIND;
  if (state[ch] == IDLE)
  {
    T1[ch] = (ch==SPEEDOIND) ? TIM2->CCR3 : TIM2->CCR4;
    TIM2_OVC[ch] = 0;
    state[ch] = DONE;
  }
  else if (state[ch] == DONE)
  {
    T2[ch] = (ch==SPEEDOIND) ? TIM2->CCR3 : TIM2->CCR4;
    float tmp = (T2[ch] + (TIM2_OVC[ch] * TIM2_MAX_CNT)) - T1[ch];
    state[ch] = IDLE;
    TIM2_OVC[ch] = 0;
    ticks[ch] = sma[ch].add(tmp);
  }

  /*
   * flag for an odo tick every 3 (or whatever) speedo ticks
   * as this is a unidirectional flag we dont need a mutex
   *
   * todo: this is a a single threaded devies, use an odo tick
   * count instead of a flag, and just incrment it here and decrement
   * it elsewhere
   */
  if(ch == SPEEDOIND)
  {
    speed_tick_count++;
    if(speed_tick_count == SPEED_TICKS_PER_ODO_TICK)
    {
      speed_tick_count = 0;
      odo_ticks += ODO_STEPS_PER_TICK;
    }
  }
}



/*
 * keep track of how many times the timer elapsed so we can use
 * that to calc the actual time between ticks.
 *
 * note, if this gets too high then assume no more ticks are coming
 * and we need to say the freq is 0
 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  static int worst_timing = 0;
  int end, start = HAL_GetTick();

  if(htim->Instance == TIM2)
  {
    TIM2_OVC[0]++;
    TIM2_OVC[1]++;
    if(TIM2_OVC[0]*OVERFLOW_MS > 2000)
    {
      TIM2_OVC[0] = 0;
      ticks[0] = 0;
      state[0] = IDLE;
      resetCnt1++;
    }
    if(TIM2_OVC[1]*OVERFLOW_MS > 2000)
    {
      TIM2_OVC[1] = 0;
      ticks[1] = 0;
      state[1] = IDLE;
      resetCnt2++;
    }
  }
  else if(htim->Instance == TIM16)
  {
    /*
     * 2khz
     */
    if(needles_ready)
    {
      if(measure_freq)
      {
#ifndef SWEEP_GAUGES
#ifndef SIM_GAUGES
        /*
         * convert ticks, to Hz, to RPM and Speed
         *
         * TODO: get rid of float division
         */
        float tmp = (ticks[TACHIND]==0) ? 0 : RPM_PER_HZ * (float)F_CLK / (float)ticks[TACHIND];
        if( tmp > 9000 )
          tmp = rpm;
        rpm = tmp;

        tmp = (ticks[SPEEDOIND]==0) ? 0 : MPH_PER_HZ * (float)F_CLK / (float)ticks[SPEEDOIND];
        if( tmp > 180 )
          tmp = speed;
        speed = tmp;
#endif
#endif
      }
    }
    ecu.update();
  }
  else if(htim->Instance == TIM17)
  {
    /*
     * ticks at 1us
     */
    if(needles_ready)
    {
      uint32_t delay = update_needles();
      if(delay>TIM17->ARR)
        TIM17->CNT = 0;
      else
        TIM17->CNT = TIM17->ARR - (delay-2);
    }
  }
  else if(htim->Instance == TIM1)
  {

  }

  end = HAL_GetTick();
  if(end-start>worst_timing)
    worst_timing = end-start;
}

/*
 * override the _weak definition in the hal
 * this code is used for printf and puts to do trhough teh jtag interface
 */
int _write(int32_t file, uint8_t *ptr, int32_t len)
{
    int i = 0;
    for (i = 0; i < len; i++)
    {
        ITM_SendChar((*ptr++));
    }
    return len;
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  ecuRxDone = true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  ecuTxDone = true;
}

} // extern C
