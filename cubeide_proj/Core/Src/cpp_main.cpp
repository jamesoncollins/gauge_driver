
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
#include "filters.h"
}
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

extern "C" {
extern void setAutoClear();
}

extern I2C_HandleTypeDef hi2c1, hi2c3;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart1;
extern RTC_HandleTypeDef hrtc;

static uint16_t screenWidth;
static uint16_t screenHeight;

static MCP4725 dac;
static BMI088 imu;
static uint8_t regAddr;

volatile static bool ecuTxDone = false;
volatile static bool ecuRxDone = false;
static MUTII ecu(&huart1, &ecuTxDone, &ecuRxDone);

volatile static float rpm, speed;

volatile static bool bulbReadWaiting = false;
static const uint16_t lampMask         = 1<<1;
static const uint16_t beamMask         = 1<<0;
static const uint16_t psMask           = 1<<3;
static const uint16_t battMask         = 1<<2;
static const uint16_t brakeMask        = 1<<4;

// flags used by accelerometer in IT mode
volatile static bool acc_int_rdy = false;       // we got exti saying data ready
volatile static bool do_convert = false;        // we received the raw data

volatile static bool rpm_alert = false;
volatile static bool rpm_alert_has_lock = false;
volatile static bool acc_has_lock = false;
volatile static unsigned i2c_lock = 0;

/*
 * frequency measurement settings for rpm and speed
 *
 * i originally measured that every 3 ticks of the speedo, the odo was stepped once.
 * and with our stepper i think a full step is actually 12 micro steps.
 * so the numbers below should be 3 and 12.  but those aren't looking right.
 * so i tweaked it.  well, i will tweak it once i get some measurements again.
 */
#define SPEED_TICKS_PER_ODO_TICK (3)
#define ODO_STEPS_PER_TICK (12)
volatile int odo_tick_flag = 0;
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
   * init graphics library
   */
  gfxInit();
  gdispClear(GFX_BLACK);
  gdispFlush();
  screenWidth = gdispGetWidth();
  screenHeight = gdispGetHeight();
  uint32_t GFX_AMBER = GFX_AMBER_YEL;

  //font_t font10 = gdispOpenFont("DejaVuSans10");
  font_t font20 = gdispOpenFont("DejaVuSans20");
  font_t fontLCD = gdispOpenFont("lcddot_tr80");

  /*
   * dac setup
   */
  MCP4725_init (&dac, &hi2c1, MCP4725A0_ADDR_A00, 3.30);
  if (!MCP4725_isConnected (&dac))
  {
    //exit(-1);
  }
  if( MCP4725_setVoltage(&dac, 0, MCP4725_REGISTER_MODE, MCP4725_POWER_DOWN_100KOHM) )
  {
    //exit(-1);
  }

  /*
   * dac output timer
   */
  HAL_TIM_Base_Start_IT(&htim16);


  /*
   * onboard io expander
   */
  PI4IOE5V6416 ioexp_onboard(&hi2c1);
  if(ioexp_onboard.init(0x0000))
  {
    //exit(-1);
  }

  /*
   * screen pcb io expander
   */
  uint16_t bulbVals = 0;
  PI4IOE5V6416 ioexp_screen(&hi2c3);
  if(ioexp_screen.init(
        0x0000
//        | lampMask    // not sure
        | brakeMask     // switch pulls bulb down, need to mimic bulb voltage
        | battMask    // voltage source is normally applied
//        | psMask      // switch pulls this up, so we pull down
      ))
  {
    //exit(-1);
  }

  /*
   * Acc / Gyro setup
   */
  if(BMI088_Init(&imu, &hi2c1))
  {
    //exit(-1);
  }

  /*
   * USART and ECU ISO9141 init/control
   */
  // deinit the uart to allow gpio control
  My_MX_USART1_UART_DeInit();

  /*
   * tach and speedo freq measurement setup
   */
  HAL_TIM_Base_Start_IT(&htim2);
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

  uint32_t slowTable[][2] = { 20, (uint32_t) (2100. * 64000000. * 1e-6) };
  SwitecX12 odoX12(
      0xFFFFFFFE,  // hopefully infinite
      STEP_ODO_GPIO_Port,
      STEP_ODO_Pin,
      DIR_ODO_GPIO_Port,
      DIR_ODO_Pin,
      slowTable, 1,
      true      // reverse direction for odometer ticks
      );
  odoX12.currentStep = 0xFFFFFFFE;
  odoX12.targetStep = 0xFFFFFFFE;

  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_RESET );
  HAL_Delay(10);
  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_SET );

  /*
   * calibrate the needles by bumping them against the stops
   */
  if(!cleanPwr)
  {
    for(int i=0; i<X27_STEPS>>1; i++)
    {
      tachX12.step(-1);
      speedX12.step(-1);
      DWT_Delay(500);
    }
  }
  tachX12.reset();
  speedX12.reset();
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
        tachX12.setPosition (X27_STEPS);
        speedX12.setPosition (X27_STEPS);
        startup_state++;
        break;
      case 1:
        if(tachX12.atTarget() && speedX12.atTarget())
          startup_state ++;
        break;
      case 2:
        tachX12.setPosition (0);
        speedX12.setPosition (0);
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
      // load te next image
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
  setAutoClear();
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
  uint32_t timerUpdates = timerLoop;
  bool firstAcc = false;
  uint32_t loopPeriod = 0, worstLoopPeriod = 0;
  while (1)
  {

    MX_APPE_Process();

    /* Toggle LED */
    if ((HAL_GetTick () - timerLED) >= SAMPLE_TIME_MS_LED)
    {
      // needs to be called occasionally to avoid looping
      get_cycle_count();

      HAL_GPIO_TogglePin ( LED_GPIO_Port, LED_Pin );
      timerLED = HAL_GetTick ();

      static uint8_t data = 0;
      Custom_STM_App_Update_Char(CUSTOM_STM_READNEXT, &data);
      data++;
    }

    /**
     * bmi088 triggered us that data is available, start reading it
     */
    if( do_convert )
    {
      BMI088_ConvertAccData(&imu); // converts raw buffered data to accel floats
      do_convert = false;
      rotateVectorKnownPitch(imu.acc_mps2, cosPitch, sinPitch);
      if(!firstAcc)
      {
        firstAcc = true;
      }
    }

    /*
     * fixme: mutex isnt required, just use a flag.  the reason
     * is that we only lock from this loop and we only unlock from the
     * callbacks.
     */
    if( acc_int_rdy  )
    {
      if (!acc_has_lock)
        acc_has_lock = lock_mutex (&i2c_lock);
      if (acc_has_lock)
      {
        regAddr = BMI_ACC_DATA;
        uint8_t status = 1;
        while(status)
        {
          status = HAL_I2C_Mem_Read_IT(
            &hi2c1,
            ACC_ADDR,
            BMI_ACC_DATA, 1,
            imu.accRxBuf, 6);
        }
        acc_int_rdy = false;
      }
    }

    /* Print */

    if ((HAL_GetTick () - timerPrint) >= SAMPLE_TIME_MS_PRINT)
    {
#ifdef PRINT_TO_USB
      timerPrint = HAL_GetTick ();
      logBufInd += snprintf (logBuf+logBufInd, bufLen-logBufInd, " tach: %d , speedo %d  \n",  (int)rpm,  (int)speed);
      CDC_Transmit_FS ((uint8_t*) logBuf, ind);
#endif
    }


    /*
     * Any updates we want presented to the user.
     */
    if ((HAL_GetTick () - timerUpdates) >= SAMPLE_TIME_MS_UPDATES)
    {
      timerUpdates = HAL_GetTick ();

      /*
       * display updates
       */

      // fixme: this forces a write to ram of 320x240*2 bytes.
      // instead of doing this maybe we should be using "widgets",
      // either from ugfx or our own, that track their state and clear
      // themselves if they need.
      // update: current implementation automatically clears after each
      // flush, so this clear call below basically just blocks us until
      // busy operations are done and its safe to draw.
      // but the drawing operations do that too, so wahtever.
      gdispClear(GFX_BLACK); // if the device doesnt support flushing, then this is immediate

      // fixme: unnecessary float division
      const int gimbal_radius = 35;
      drawGimball ( &gimball, 168, 48, gimbal_radius,
                    -imu.acc_mps2[1] / (9.8f / 1.f) * gimbal_radius,
                    -imu.acc_mps2[0] / (9.8f / 1.f) * gimbal_radius
                   );

      snprintf (logBuf, bufLen, "%s", ecu.getValString(MUTII::ECU_PARAM_WB));
      gdispFillString(20, 20, logBuf, fontLCD, GFX_AMBER, GFX_BLACK);
      drawHorzBarGraph (20, 57, 80, 15, 19, 9, ecu.getVal(MUTII::ECU_PARAM_WB));

      snprintf (logBuf, bufLen, "ECU:%s-%ld", ecu.getStatus(), ecu.getMsgRate());
      gdispFillString(20, 80, logBuf, font20, GFX_AMBER, GFX_BLACK);

      if(!ecu.isConnected())
      {
        static flasher_t ecuGoodFlasher = {.rate_ms = 500, .last_ms = 0};
        flasher(&ecuGoodFlasher, gdispFillString(20, 80, "ECU ERR     ", font20, GFX_RED, GFX_BLACK));
      }

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

      //Custom_STM_App_Update_Char(
      //    CUSTOM_STM_READNEXT,
      //    (uint8_t*)ecu.getParam(0)
      //    );

      snprintf (logBuf, bufLen, "%d", (int)speed);
      gdispFillString(15, 110+42, logBuf, fontLCD, GFX_AMBER, GFX_BLACK);
      snprintf (logBuf, bufLen, "%d", (int)rpm);
      gdispFillString(15, 155+42, logBuf, fontLCD, GFX_AMBER, GFX_BLACK);

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

      // check warning
      if(!bulbReadWaiting)
      {
        if(ioexp_screen.get_IT(&bulbVals)==0) //we dont know when this will finish, dont care
          bulbReadWaiting = true;
      }
      //bulbVals = ioexp_screen.get();

      if( !(bulbVals&battMask) ) // car pulls down
        gdispImageDraw(&battImg,  145,  210, battImg.width,  battImg.height,  0, 0);
      if( !(bulbVals&brakeMask) ) // car pulls down
        gdispFillString(120, 233, "BRAKE", font20, GFX_RED, GFX_BLACK);
       //gdispImageDraw(&brakeImg, 100,  230, brakeImg.width, brakeImg.height, 0, 0);
      if(  (bulbVals&psMask) ) // car pulls HIGH
        gdispFillString(145, 215, "4WS", font20, GFX_YELLOW, GFX_BLACK);
      if (bulbVals&lampMask )
      {
        // headlights are on
        GFX_AMBER = GFX_AMBER_SAE;
        setColors(GFX_AMBER,GFX_RED,GFX_BLACK);
        if( !(bulbVals&beamMask) )
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

      // debug / diag messages
//#define DIAG_SQUARE
#ifdef DIAG_SQUARE
      static uint32_t displayTime = 0;
      const int xdiag = 30, ydiag = 192;
      gdispDrawBox(xdiag-1,ydiag-1,60,62,GFX_AMBER);
      snprintf (logBuf, bufLen, "ECU: %lu/%lu", ecu.getMsgRate(), ecu.getMissedReplyResetCnt());
      gdispFillString(xdiag, ydiag+00, logBuf, font10, GFX_AMBER, GFX_BLACK);
      snprintf (logBuf, bufLen, "%d/%d/%lu", (int)loopPeriod ,(int) worstLoopPeriod, HAL_GetTick() - displayTime);
      displayTime = HAL_GetTick();
      gdispFillString(xdiag, ydiag+10, logBuf, font10, GFX_AMBER, GFX_BLACK);
      if(irq_overlap_1 || irq_overlap_2)
        gdispFillString(xdiag, ydiag+20, "IRQERR", font10, GFX_AMBER, GFX_BLACK);
      snprintf (logBuf, bufLen, "%d/%d/%lu/%lu", (int)resetCnt1 ,(int) resetCnt2,rejects[0],rejects[1]);
      gdispFillString(xdiag, ydiag+30, logBuf, font10, GFX_AMBER, GFX_BLACK);
      snprintf (logBuf, bufLen, "%d", bulbVals);
      gdispFillString(xdiag, ydiag+40, logBuf, font10, GFX_AMBER, GFX_BLACK);
      snprintf (logBuf, bufLen, "%lu / %lu", x12[0]->getTargetPosition(), x12[1]->getTargetPosition());
      gdispFillString(xdiag, ydiag+50, logBuf, font10, GFX_AMBER, GFX_BLACK);
#endif

      // some devices dont support this and instead they draw whenever you call a drawing function
      // but its always safe to call it
      gdispFlush();

    }

    /*
     * odometer ticks
     */
    if(odo_tick_flag && odoX12.atTarget())
    {
      odoX12.setPosition(odoX12.targetStep+ODO_STEPS_PER_TICK);
      odo_tick_flag--;
    }

#ifdef SWEEP_GAUGES
    /*
     * temporarily sweep needle back and forth
     */
    static bool set = false;
    if ( set && tachX12.atTarget() && speedX12.atTarget() )
    {
      set = !set;
      tachX12.setPosition (get_x12_ticks_rpm(7000));
      speedX12.setPosition (get_x12_ticks_speed(180) );
    }
    else if ( !set && tachX12.atTarget() && speedX12.atTarget() )
    {
      set = !set;
      tachX12.setPosition (0);
      speedX12.setPosition (0);
    }
#elif defined(SIM_GAUGES)
    /*
     * simulate reasonable speed and rpm signals
     */
    static int lastTime = 0;
    int diff = HAL_GetTick () - lastTime;
    rpm += (float) diff / 1000. * 3000; // 3000rpm per second
    if (rpm > 9000)
      rpm = 3000;
    speed += (float) diff / 1000. * 20; // 20 mph per second
    if (speed > 180)
    {
      speed = 0;
      rpm = 1000;
    }
    lastTime = HAL_GetTick ();
#endif

    /*
     * shift alert
     */
    if( rpm > 6500  && (!rpm_alert || !rpm_alert_has_lock))
    {
      rpm_alert = true;
      if(!rpm_alert_has_lock)
        rpm_alert_has_lock = lock_mutex(&i2c_lock);
    }

    if( rpm < 6400 && rpm_alert )
    {
      rpm_alert = false;
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
     * does nothing, just checks loop timing for diagnostics
     */
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
  const float ZERO_ANGLE = 1;    // degrees beyond the stopper to get to 0
  const float MIN_MPH_ANGLE = 3.5; // degrees from zero to MIN_MPH
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
  const float ZERO_ANGLE = 1;    // degrees beyond the stopper to get to 0
  const float MIN_RPM_ANGLE = 4.5; // degrees from zero to MIN_RPM
  const float DEGREES_PER_RPM = 21.9 / 1000.;

  if( rpm <= 1 )
    return ZERO_ANGLE * 12.;
  else if( rpm<=MIN_RPM )
    return (MIN_RPM_ANGLE + ZERO_ANGLE) * 12.;
  else
    return ((rpm-MIN_RPM) * DEGREES_PER_RPM + ZERO_ANGLE + MIN_RPM_ANGLE ) * 12.;
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
void update_needles ()
{
  x12[0]->update ();
  x12[1]->update ();
  x12[2]->update ();
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
    // this is the screen io expander
    bulbReadWaiting = false;
    return;
  }

  do_convert = true;

  bool was_locked = acc_has_lock;
  acc_has_lock = false;
  if(was_locked)
    unlock_mutex(&i2c_lock);
}

/*
 * fires when DAC write finishes.  currently the only thing using
 * master transmits
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  bool was_locked = rpm_alert_has_lock;
  rpm_alert_has_lock = false;
  if(was_locked)
    unlock_mutex(&i2c_lock);
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
volatile static float ticks_raw[2] = {1e9,1e9};
volatile static float ticks[2] = {0,0};
volatile static uint32_t rejects[2];
volatile static uint32_t TIM2_OVC[2] = {0,0};
volatile static uint32_t speed_tick_count = 0;
static iir_ma_state_t filter_state[2] = {{0.3,0}, {0.3,0}};

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

    /*
     * reject outliers that are more than X as long as the last tick
     */
    if( tmp>10*ticks_raw[ch] )
    {
      ticks[ch] = ticks[ch];
      rejects[ch]++;
    }
    else
    {
      ticks[ch] = iir_ma( &filter_state[ch], tmp );
      ticks_raw[ch] = ticks[ch];
    }
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
    if(speed_tick_count >= SPEED_TICKS_PER_ODO_TICK)
    {
      speed_tick_count = 0;
      odo_tick_flag++;
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
     * we should be here every
     * 1 / (10000 * (1/64000000 seconds)) = 6.4kHz
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
        x12[0]->setPosition( get_x12_ticks_rpm(rpm) );
        x12[1]->setPosition( get_x12_ticks_speed(speed) );
#endif
      }
      update_needles();
    }


    ecu.update();
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
