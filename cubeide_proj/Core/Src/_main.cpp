
#include "main.h"
#include "usb_device.h"

#include "usbd_cdc_if.h"
extern "C" {
#include "../../MCP4725-lib/MCP4725.h"
#include "../../BMI088-lib/BMI088.h"
}
#include "../../SwitecX12-lib/SwitecX12.hpp"
#include "utils.h"


/*
 * Milisecond timers, controlled by the main while loop, for various
 * slow functions
 */
#define SAMPLE_TIME_MS_LED    1000
#define SAMPLE_TIME_MS_PRINT   750
#define SAMPLE_TIME_MS_UPDATES   (1000/50)



/*
 * #defines used to control what happens in teh main loop
 */
//#define PRINT_TO_USB

//#define SWEEP_GAUGES  // sweep needles forever
#define SIM_GAUGES       // generate simulated rpm and mph

// enable one of these to get acceleromter data
//#define ACC_USE_BLOCK   // use blocking calls
#define ACC_USE_IT    // use interrupt calls (not wokring)



extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

MCP4725 dac;
BMI088 imu;
uint8_t regAddr;

float rpm, speed;

extern "C" {

// flags used by accelerometer in IT mode
volatile bool do_tx = false;	// we got exti saying data ready
volatile bool do_rx = false;	// we sent the register address
volatile bool do_convert = false;	// we received the raw data
volatile bool i2c_error = false;

/*
 * exti interrupts from IMU
 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT_ACC_Pin)
  {
    do_tx = true;
  }
  else if (GPIO_Pin == INT_GYR_Pin)
  {
    //BMI088_ReadGyroscopeDMA (&imu);
  }
}

/*
 * call back used for BMI088 accelerometer data
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  do_convert = true;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  do_convert = true;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  do_rx = true;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  i2c_error = true;
}

/*
 *
 * Speed and Tach exti
 *
 * at 100hz it takes 288 seconds to go 7 miles (4114 ticks / mile)
 * 4114 ticks = 1 mile, 4114 hz = 1 mile per second, 4114hz/3600s = 1.143hz = 1 mph
 *
 * See https://www.3si.org/threads/speed-sensor-gear-ratio.831219/#post-1056408948
 *
 * 27 tooth variant (trans pre feb 1993?) 1.117hz/mph
 * 28 tooth 1.078 hz/mp
 *
 */
#define SPEEDOIND 0
#define TACHIND 1
#define IDLE   0
#define DONE   1
#define F_CLK  (SystemCoreClock)
#define OVERFLOW_MS ((int)(1000*65536.f/(float)F_CLK))
#define SPEED_TICKS_PER_ODO_TICK (3)
#define MPH_PER_HZ ( 1.07755102 )
#define RPM_PER_HZ ( 20 ) // 3 ticks per revolution
volatile uint8_t state[2] = {IDLE, IDLE};
volatile uint32_t T1[2] = {0,0};
volatile uint32_t T2[2] = {0,0};
volatile uint32_t ticks[2] = {0,0};
volatile uint32_t TIM2_OVC[2] = {0,0};
volatile uint32_t speed_tick_count = 0;
volatile bool odo_tick_flag = false;

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
  TIM2_OVC[0]++;
  TIM2_OVC[1]++;
  if(TIM2_OVC[0]*OVERFLOW_MS > 250)
  {
    TIM2_OVC[0] = 0;
    ticks[0] = 0;
    state[0] = IDLE;
  }
  if(TIM2_OVC[1]*OVERFLOW_MS > 250)
  {
    TIM2_OVC[1] = 0;
    ticks[1] = 0;
    state[1] = IDLE;
  }
}

} // extern C


int get_x12_ticks_rpm( float rpm )
{
  const float MIN_RPM = 500;
  const float ZERO_ANGLE = 3;    // degrees beyond the stopper to get to 0
  const float MIN_RPM_ANGLE = 3; // degrees from zero to MIN_RPM
  const float DEGREES_PER_RPM_MIN = MIN_RPM_ANGLE / MIN_RPM;
  const float DEGREES_PER_RPM = ( 10. / 1000.  );

  if( rpm<MIN_RPM )
    return (rpm * DEGREES_PER_RPM_MIN + ZERO_ANGLE) * 12.;
  else
    return ((rpm * DEGREES_PER_RPM + ZERO_ANGLE) - MIN_RPM_ANGLE ) * 12.;
}

int get_x12_ticks_speed( float speed )
{
  const float MIN_MPH = 10;
  const float ZERO_ANGLE = 3;    // degrees beyond the stopper to get to 0
  const float MIN_MPH_ANGLE = 3; // degrees from zero to MIN_RPM
  const float DEGREES_PER_MPH_MIN = MIN_MPH_ANGLE / MIN_MPH;
  const float DEGREES_PER_MPH = ( 1 );

  if( speed<MIN_MPH )
    return (speed * DEGREES_PER_MPH_MIN + ZERO_ANGLE) * 12.;
  else
    return ((speed * DEGREES_PER_MPH + ZERO_ANGLE) - MIN_MPH_ANGLE ) * 12.;
}


/*
 * reads the two ports, sets one of them to have pull up resistors,
 * then reads again
 */
int io_exp_init()
{
  uint8_t status = 0;
  uint8_t buffer[2];
  status |= HAL_I2C_Mem_Read(
            &hi2c1,
            0b01000000,
            0, 1,
            buffer,
            2,
            1000);

  buffer[0] = 0xff; buffer[1] = 0xff;
  status |= HAL_I2C_Mem_Write(
            &hi2c1,
            0b01000000,
            0x46, 1,
            buffer,
            2,
            1000);

  buffer[0] = 0xff; buffer[1] = 0xaa;
  status |= HAL_I2C_Mem_Write(
            &hi2c1,
            0b01000000,
            0x48, 1,
            buffer,
            2,
            1000);

  buffer[0] = 0x00; buffer[1] = 0x00;
  status |= HAL_I2C_Mem_Read(
            &hi2c1,
            0b01000000,
            0, 1,
            buffer,
            2,
            1000);

  return (buffer[0]!=0xFF) | (buffer[1]!=0xAA) | status;
}

int calcEMA(float timePeriod_, float currentStock_, float lastEMA_)
{
   return lastEMA_ + 0.5 * (currentStock_ - lastEMA_);
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
  MCP4725_init (&dac, &hi2c1, MCP4725A0_ADDR_A00, 3.30);
  if (!MCP4725_isConnected (&dac))
    exit(-1);
  if( MCP4725_setVoltage(&dac, 0, MCP4725_REGISTER_MODE, MCP4725_POWER_DOWN_100KOHM) )
    exit(-1);

#if 0
#define lowByte(x)            ((uint8_t)(x%256))
#define highByte(x)             ((uint8_t)(x/256))

  int n = 0;
  uint16_t val;
  int16_t valarr[] =
  { 0, 383, 707, 924, 1000, 924, 707, 383, 0, -383, -707, -924, -1000, -924,
      -707, -383 };
  int valarrsz = sizeof(valarr) / sizeof(valarr[0]);
  uint8_t mode = 0;
  uint8_t powerType = 0;
  uint8_t buffer[4];
  my_transfer (dac.hi2c, dac._i2cAddress,  NULL, 0, 1000);
  while (1)
  {
    val = 2048 + (valarr[n] >> 2);
    n += 2;
    if (n >= valarrsz)
      n = 0;

    //MCP4725_setValue(&dac, val, MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF);

    buffer[0] = mode | (powerType << 4) | highByte(val);
    buffer[1] = lowByte(val);

    my_transfer (dac.hi2c, dac._i2cAddress, buffer, 2, 1000);

    DWT_Delay(50);

  }
  my_transfer (dac.hi2c, dac._i2cAddress, buffer, -2, 1000);
#endif


  /*
   * io expander
   */
  if(io_exp_init())
    exit(-1);

  /*
   * Acc / Gyro setup
   */
  if(BMI088_Init(&imu, &hi2c1))
    exit(-1);


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

  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_RESET );
  HAL_Delay(10);
  HAL_GPIO_WritePin ( RESET_MOTOR_GPIO_Port, RESET_MOTOR_Pin, GPIO_PIN_SET );

  // zero out the tach and speedo
  for(int i=0; i<X27_STEPS; i++)
  {
    tachX12.step(-1);
    speedX12.step(-1);
    DWT_Delay(500);
  }
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
  }
  HAL_Delay(200);

  tachX12.setPosition (0);
  speedX12.setPosition (0);
  while  (!tachX12.stopped && !speedX12.stopped)
  {
    tachX12.update ();
    speedX12.update ();
  }
  HAL_Delay(200);

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
     * bmi088 triggered us that data is available, start reading it
     */
#ifdef ACC_USE_BLOCK
    if( do_tx )
    {
      // ~245us
      BMI088_ReadAccelerometer(&imu);
    }
#elif defined ACC_USE_IT
    if( do_convert )
    {
      BMI088_ConvertAccData(&imu); // converts raw buffered data to accel floats
      do_convert = false;
    }

    if( do_rx )
    {

    }

    if( do_tx  )
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
      do_rx = true;
      do_tx = false;
    }
#endif



    /* Print */
    if ((HAL_GetTick () - timerPrint) >= SAMPLE_TIME_MS_PRINT)
    {
#ifdef PRINT_TO_USB
      //float RPM = freq_Hz[TACHIND] * 60.f * 3.f / 2.f;
      //float speed = freq_Hz[SPEEDOIND] / (float) HZ_PER_MPH;
      int ind = 0;
      ind += snprintf (logBuf+ind, bufLen-ind, "\033[1J");
      //ind += snprintf (logBuf+ind, bufLen-ind, "tach: %d mHz, speedo %d mHz \n", (int)(1000*freq_Hz[TACHIND]), (int)(1000*freq_Hz[SPEEDOIND]));
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
     * Any updates we want presented to the user.
     *  Currently this is 20fps
     *
     */
#ifndef SWEEP_GAUGES
    if ((HAL_GetTick () - timerUpdates) >= SAMPLE_TIME_MS_UPDATES)
    {
#ifdef SIM_GAUGES
      rpm += (float)SAMPLE_TIME_MS_UPDATES / 1000. * 1000;
      if(rpm>7000)
        rpm = 0;
//      speed += (float)SAMPLE_TIME_MS_UPDATES / 1000. * 20;
//      if(speed>100)
//        speed = 0;
      speed = (imu.acc_mps2[2] / 9.8) * 50 + 50;
      if(speed > 100)
        speed = 100;
      if(speed < 10)
        speed = 10;
#else
      /*
       * convert ticks, to Hz, to RPM and Speed
       */
      rpm   = (ticks[TACHIND]==0) ? 0 : (float)F_CLK / (float)ticks[TACHIND];   // Actually this is Hz
      speed = (ticks[SPEEDOIND]==0) ? 0 : (float)F_CLK / (float)ticks[SPEEDOIND]; // Actually, this is Hz
      rpm = rpm * RPM_PER_HZ;
      speed = speed * MPH_PER_HZ;
#endif
      tachX12.setPosition( get_x12_ticks_rpm(rpm) );
      speedX12.setPosition( get_x12_ticks_speed(speed) );
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

