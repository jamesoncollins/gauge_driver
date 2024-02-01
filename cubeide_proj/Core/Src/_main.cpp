
#include "main.h"
#include "usb_device.h"

#include "usbd_cdc_if.h"
extern "C" {
#include "../../MCP4725-lib/MCP4725.h"
#include "../../BMI088-lib/BMI088.h"
}
#include "../../SwitecX12-lib/SwitecX12.hpp"
#include "utils.h"

#define SAMPLE_TIME_MS_LED    1000
#define SAMPLE_TIME_MS_PRINT   750
#define SAMPLE_TIME_MS_UPDATES   (1000/50)

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

MCP4725 dac;
BMI088 imu;

/*
 * disabled becuase we currently read manually, its only 100sps
 */
bool acc_intr_flase = false;
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT_ACC_Pin)
  {
    //BMI088_ReadAccelerometerDMA (&imu);
    acc_intr_flase = true;
  }
  else if (GPIO_Pin == INT_GYR_Pin)
  {
    //BMI088_ReadGyroscopeDMA (&imu);
  }
}


/*
 * at 60Hz it takes 70 seconds to go 1 mile (4200 ticks / mile)
 *
 * at 70hz it takes 117 seconds to go 2 miles (4095 ticks / mile)
 *
 * at 100hz is takes 165 seconds to go 4 miles (4125 ticks / mile)
 * at 100hz it takes 288 seconds to go 7 miles (4114 ticks / mile)
 * 4114 ticks = 1 mile, 4114 hz = 1 mile per second, 4114hz/3600s = 1.143hz = 1 mph
 */
#define SPEEDOIND 0
#define TACHIND 1
#define IDLE   0
#define DONE   1
#define F_CLK  (SystemCoreClock)
#define OVERFLOW_MS ((int)(1000*65536.f/(float)F_CLK))
#define SPEED_TICKS_PER_ODO_TICK (3)
#define MPH_PER_HZ ( 4114./3600. )
#define RPM_PER_HZ ( 20 ) // 3 ticks per revolution
#define DEGREES_PER_MPH   ( 20 )
#define DEGREES_PER_RPM ( 10. / 1000.  )
volatile uint8_t state[2] = {IDLE, IDLE};
volatile uint32_t T1[2] = {0,0};
volatile uint32_t T2[2] = {0,0};
volatile uint32_t ticks[2] = {0,0};
//volatile float freq_Hz[2] = {0,0}; // todo, dont calc freq in the callback, do it in the main loop
volatile uint32_t TIM2_OVC[2] = {0,0};
volatile uint32_t speed_tick_count = 0;
volatile bool odo_tick_flag = false;

/*
 * rpm and speed frequency measurement
 */
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
    ticks[ch] = (T2[ch] + (TIM2_OVC[ch] * 65536)) - T1[ch];
    //if(ticks[ch] == 0 )
    //  freq_Hz[ch] = 0;
    //else
    //  freq_Hz[ch] = (float)F_CLK / (float) ticks[ch];
    state[ch] = IDLE;
    TIM2_OVC[ch] = 0;
  }

  /*
   * flag for an odo tick every 3 (or whatever) speedo ticks
   * as this is a unidirectional flag we dont need a mutex
   *
   * todo: this is a a single threaded devies, use an odo tick
   * count instead of a flag, and just incrment it here and decrement
   * it elsewhere
   */
  if(ch == SPEEDOIND && !odo_tick_flag)
  {
    speed_tick_count ++;
    if(speed_tick_count >= SPEED_TICKS_PER_ODO_TICK)
    {
      speed_tick_count = 0;
      odo_tick_flag = true;
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
  //static uint32_t us0 = DWT->CYCCNT * (HAL_RCC_GetHCLKFreq() / 1000000);
  //static uint32_t us1 = DWT->CYCCNT * (HAL_RCC_GetHCLKFreq() / 1000000);
  TIM2_OVC[0]++;
  TIM2_OVC[1]++;
  if(TIM2_OVC[0]*OVERFLOW_MS > 250)
  {
    TIM2_OVC[0] = 0;
    //freq_Hz[0] = 0;
    ticks[0] = 0;
    state[0] = IDLE;
  }
  if(TIM2_OVC[1]*OVERFLOW_MS > 250)
  {
    TIM2_OVC[1] = 0;
    //freq_Hz[1] = 0;
    ticks[1] = 0;
    state[1] = IDLE;
  }
}


int main_cpp(void)
{

  // init our 64-bit system-tick counter based on teh DWT timer
  init_get_cycle_count ();

  /* Timers */
  uint32_t timerLoop = 0;
  uint32_t timerLED	= 0;
  uint32_t timerIGN = 0;
  uint32_t timerPrint = 0;
  uint32_t timerUpdates = 0;
  bool flagSlow = false;

  /* USB data buffer */
  const int bufLen = 256;
  char logBuf[bufLen];

  /*
   * Turn on mcu-controlled pwr-en signal.
   */
  HAL_GPIO_WritePin ( PWREN_GPIO_Port, PWREN_Pin, GPIO_PIN_SET );

  /*
   * dac setup
   */
  dac = MCP4725_init (&hi2c1, MCP4725A0_ADDR_A00, 3.30);
  if (MCP4725_isConnected (&dac))
  {
    sprintf (logBuf, "DAC Connected\n");
    CDC_Transmit_FS ((uint8_t*) logBuf, strlen (logBuf));
    MCP4725_getValue(&dac);
  }
  else
  {
    sprintf (logBuf, "DAC NOT Connected\n");
    CDC_Transmit_FS ((uint8_t*) logBuf, strlen (logBuf));
  }

  // Start DAC output timer
  //HAL_TIM_Base_Start_IT (&htim1);

  /*
   * Acc / Gyro setup
   */
  BMI088_Init(&imu, &hi2c1);

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

  const int X27_STEPS = 315*12; // 315 degrees, 12 microsteps each?

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

  SwitecX12 odoX12(
      0xFFFFFFFE,  // hopefully infinite
      STEP_ODO_GPIO_Port,
      STEP_ODO_Pin,
      DIR_ODO_GPIO_Port,
      DIR_ODO_Pin
      );

  // zero out the tach and speedo
  for(int i=0; i<X27_STEPS; i++)
  {
    tachX12.step(-1);
    speedX12.step(-1);
    DWT_Delay(500);
  }
  //tachX12.zero();
  //speedX12.zero();
  tachX12.currentStep = 0; tachX12.stopped = true; tachX12.vel = 0;
  speedX12.currentStep = 0; speedX12.stopped = true; speedX12.vel = 0;
  HAL_Delay(200);

  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_RESET );
  HAL_Delay(10);
  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_SET );

  tachX12.setPosition (X27_STEPS);
  speedX12.setPosition (X27_STEPS);
  while (!tachX12.stopped && !speedX12.stopped)
  {
    tachX12.update ();
    speedX12.update ();
    //DWT_Delay(200);
  }
  HAL_Delay(200);

  tachX12.setPosition (0);
  speedX12.setPosition (0);
  while  (!tachX12.stopped && !speedX12.stopped)
  {
    tachX12.update ();
    speedX12.update ();
    //DWT_Delay(200);
  }
  HAL_Delay(200);



  /*
   * delay to let usb come up
   */
  HAL_Delay(250);


  while (1)
  {


    /* Toggle LED */
    if ((HAL_GetTick () - timerLED) >= SAMPLE_TIME_MS_LED)
    {
      // needs to be called occasionally to avoid looping
      get_cycle_count();

      HAL_GPIO_TogglePin ( LED_GPIO_Port, LED_Pin );
      timerLED = HAL_GetTick ();
    }

    /**
     * get attitude measurements
     */
    if( acc_intr_flase )
    {
      BMI088_ReadAccelerometer(&imu);
      acc_intr_flase = false;
    }



    /* Print */
    if ((HAL_GetTick () - timerPrint) >= SAMPLE_TIME_MS_PRINT)
    {
#if 0
      //float RPM = freq_Hz[TACHIND] * 60.f * 3.f / 2.f;
      //float speed = freq_Hz[SPEEDOIND] / (float) HZ_PER_MPH;
      int ind = 0;
      ind += snprintf (logBuf+ind, bufLen-ind, "\033[1J");
      ind += snprintf (logBuf+ind, bufLen-ind, "tach: %d mHz, speedo %d mHz \n", (int)(1000*freq_Hz[TACHIND]), (int)(1000*freq_Hz[SPEEDOIND]));
      ind += snprintf (logBuf+ind, bufLen-ind, "acc (mps): %d %d %d \n", (int)imu.acc_mps2[0],(int)imu.acc_mps2[1],(int)imu.acc_mps2[2]);
      if(flagSlow)
      {
	ind += snprintf (logBuf+ind, bufLen-ind, "main loop running slow \n");
      }
      CDC_Transmit_FS ((uint8_t*) logBuf, ind);
#endif
      timerPrint = HAL_GetTick ();
    }

    /*
     * ANy updates we want presented to the user.
     *  Currently this is 20fps
     *
     */
#if 0
    if ((HAL_GetTick () - timerUpdates) >= SAMPLE_TIME_MS_UPDATES)
    {
      float rpm_hz   = (ticks[TACHIND]==0) ? 0 : (float)F_CLK / (float)ticks[TACHIND];
      float speed_hz = (ticks[SPEEDOIND]==0) ? 0 : (float)F_CLK / (float)ticks[SPEEDOIND];
      float rpm = rpm_hz * RPM_PER_HZ;
      float speed = speed_hz * MPH_PER_HZ;
      tachX12.setPosition( rpm * ((float)DEGREES_PER_RPM * 12.) );
      speedX12.setPosition( rpm * ((float)DEGREES_PER_MPH * 12.) );
      timerUpdates = HAL_GetTick ();
    }

    /*
     * odometer ticks
     */
    if(odo_tick_flag && odoX12.stopped)
    {
      odoX12.setPosition(odoX12.currentStep+12);
      odo_tick_flag = false;
    }
#else
    /*
     * temporarily sweep needle back and forth
     */
    static bool set = false;
    if ( set && tachX12.stopped && speedX12.stopped )
    {
      set = !set;
      tachX12.setPosition (X27_STEPS/12.*270);
      speedX12.setPosition (X27_STEPS/12.*270);
    }
    else if ( !set && tachX12.stopped && speedX12.stopped )
    {
      set = !set;
      tachX12.setPosition (0);
      speedX12.setPosition (0);
    }
#endif

    /*
     * called as fast as possible, moves the motors if they need to be moved
     */
    speedX12.update();
    tachX12.update();
    odoX12.update();



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
    uint32_t loopPeriod = (HAL_GetTick () - timerLoop);
    if(loopPeriod>5)
    {
      flagSlow = true;
    }
    timerLoop = HAL_GetTick ();

  }

  /*
   * Power-down loop, do anything we need to in order to cleanup before
   * we power off.
   */
  speedX12.setPosition(0);
  tachX12.setPosition(0);
  while(1)
  {
    speedX12.update();
    tachX12.update();
    if(speedX12.stopped && tachX12.stopped)
      break;
  }

  /*
   * we left the main loop, we can power down
   */
  HAL_GPIO_WritePin ( PWREN_GPIO_Port, PWREN_Pin, GPIO_PIN_RESET );
  HAL_Delay(1000);

  return 0;
}

