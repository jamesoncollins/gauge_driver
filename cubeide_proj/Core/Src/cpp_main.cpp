
#include "main.h"
#include "usb_device.h"

#include "usbd_cdc_if.h"
extern "C" {
#include "../Core/MCP4725-lib/MCP4725.h"
#include "../Core/BMI088-lib/BMI088.h"
}
#include "../Core/SwitecX12-lib/SwitecX12.hpp"
#include "utils.h"
#include "gfx.h"
#include "ugfx_widgets.h"
#include "../Quaternion/Quaternion.hpp"
#include "../../res/mitslogoanim_128.c"

/*
 * Milisecond timers, controlled by the main while loop, for various
 * slow functions
 */
#define SAMPLE_TIME_MS_LED    1000
#define SAMPLE_TIME_MS_PRINT   750
#define SAMPLE_TIME_MS_UPDATES   (1000/24)

int screenWidth;
int screenHeight;

/*
 * #defines used to control what happens in teh main loop
 */
//#define PRINT_TO_USB

//#define SWEEP_GAUGES  // sweep needles forever
//#define SIM_GAUGES       // generate simulated rpm and mph

// enable one of these to get acceleromter data
//#define ACC_USE_BLOCK   // use blocking calls
#define ACC_USE_IT    // use interrupt calls (not wokring)



extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;

MCP4725 dac;
BMI088 imu;
uint8_t regAddr;

float rpm, speed;

extern "C" {

// flags used by accelerometer in IT mode
volatile bool acc_int_rdy = false;	// we got exti saying data ready
volatile bool do_convert = false;	// we received the raw data
volatile bool i2c_error = false;

bool rpm_alert = false;
bool rpm_alert_has_lock = false;
bool acc_has_lock = false;
volatile unsigned i2c_lock = 0;

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
 * this callback fires when the acceleromter read finishs,
 * its currently the only thing using IT mem reads
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  do_convert = true;

  bool was_locked = acc_has_lock;
  acc_has_lock = false;
  if(was_locked)
    unlock_mutex(&i2c_lock);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

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
 * the gauge face says 1025rev=1mile, so that would be 4100 ticks, that would be 1.38888hz/mph
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
#define MPH_PER_HZ (1.38888) //(1.11746031667) //( 1.07755102 )
#define RPM_PER_HZ ( 20 ) // 3 ticks per revolution
volatile uint8_t state[2] = {IDLE, IDLE};
volatile uint32_t T1[2] = {0,0};
volatile uint32_t T2[2] = {0,0};
volatile uint32_t ticks[2] = {0,0};
volatile uint32_t TIM2_OVC[2] = {0,0};
volatile uint32_t speed_tick_count = 0;

/* i originally measured that every 3 ticks of the speedo, the odo was stepped once.
 * and with our stepper i think a full step is actually 12 micro steps.
 * so the numbers below should be 3 and 12.  but those aren't looking right.
 * so i tweaked it.  well, i will tweak it once i get some measurements again.
 */
#define SPEED_TICKS_PER_ODO_TICK (3)
#define ODO_STEPS_PER_TICK (12)
volatile bool odo_tick_flag = false;

void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
  if(htim->Instance != TIM2)
    return;

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
  if(htim->Instance == TIM2)
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
  else if(rpm_alert && rpm_alert_has_lock && htim->Instance == TIM16)
  {
    // tim16?
//    static uint16_t valarr[16] =
//    { 0+2048, 383+2048, 707+2048, 924+2048, 1000+2048, 924+2048, 707+2048, 383+2048, 0+2048, -383+2048, -707+2048, -924+2048, -1000+2048, -924+2048,
//        -707+2048, -383+2048 };
//    static int16_t valarr[16] =
//    { -1000, 1000, -1000, 1000, -1000, 1000, -1000, 1000, -1000, 1000, -1000, 1000, -1000, 1000, -1000, 1000,  };
    static int16_t valarr[16] =
    { 0, 707, 1000, 707, 0, -707, -1000, -707,  0, 707, 1000, 707, 0, -707, -1000, -707, };
    static uint16_t valarr_ctr = 0;
    int16_t val = ( valarr[valarr_ctr & 0xf]  * 2 + 2048 ) & 0x0fff;
    valarr_ctr++;
#define lowByte(x)            ((uint8_t)(x%256))
#define highByte(x)             ((uint8_t)(x/256))
    static uint8_t arr[2];
    arr[1] = lowByte(val);
    arr[0] = highByte(val);
    HAL_I2C_Master_Transmit_IT(
      &hi2c1,
      dac._i2cAddress,
      arr,
      2);
  }
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

} // extern C


int get_x12_ticks_rpm( float rpm )
{
  const float MIN_RPM = 500;
  const float ZERO_ANGLE = 5;    // degrees beyond the stopper to get to 0
  const float MIN_RPM_ANGLE = 3; // degrees from zero to MIN_RPM
  const float DEGREES_PER_RPM_MIN = MIN_RPM_ANGLE / MIN_RPM;
  const float DEGREES_PER_RPM = ( 21.5 / 1000.  );

  if( rpm<MIN_RPM )
    return (rpm * DEGREES_PER_RPM_MIN + ZERO_ANGLE) * 12.;
  else
    return ((rpm * DEGREES_PER_RPM + ZERO_ANGLE) - MIN_RPM_ANGLE ) * 12.;
}

int get_x12_ticks_speed( float speed )
{
  const float MIN_MPH = 10;
  const float ZERO_ANGLE = 5;    // degrees beyond the stopper to get to 0
  const float MIN_MPH_ANGLE = 3; // degrees from zero to MIN_RPM
  const float DEGREES_PER_MPH_MIN = MIN_MPH_ANGLE / MIN_MPH;
  const float DEGREES_PER_MPH = ( 1.31 );

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


int movingAvg(int *ptrArrNumbers, long *ptrSum, int *pos, int len, int nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[*pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[*pos] = nextNum;

  (*pos)++;
  if((*pos)>=len)
    (*pos) = 0;

  //return the average
  return *ptrSum / len;
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
   * init graphics library
   */
  gfxInit();
  gdispClear(GFX_BLACK);
  screenWidth = gdispGetWidth();
  screenHeight = gdispGetHeight();
  font_t font = gdispOpenFont("DejaVuSans10");
//  font_t fontMits20 = gdispOpenFont("BITSUMIS20");
//  font_t fontMits40 = gdispOpenFont("BITSUMIS40");
  gImage myImage;
  gdispImageOpenMemory(&myImage,mitslogoanim_128);
//  gdispFillString(80, 200, "3000GT", fontMits40, GFX_AMBER, GFX_BLACK);
  gdispFlush();
  gDelay delay;
  int displayLogo = 45; // number of logo frames
  while (displayLogo--)
  {
    //gdispImageCache(&myImage);
    gdispImageDraw(&myImage, 55, 90, myImage.width, myImage.height, 0, 0);
    delay = gdispImageNext (&myImage);
    gdispFlush();
    gfxSleepMilliseconds(delay);
  }
  gdispImageClose (&myImage);
  gdispFlush();
  gdispFlush();


  /*
   * dac setup
   */
  MCP4725_init (&dac, &hi2c1, MCP4725A0_ADDR_A00, 3.30);
  if (!MCP4725_isConnected (&dac))
    exit(-1);
  if( MCP4725_setVoltage(&dac, 0, MCP4725_REGISTER_MODE, MCP4725_POWER_DOWN_100KOHM) )
    exit(-1);


  /*
   * dac output timer
   */
  HAL_TIM_Base_Start_IT(&htim16);

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


  /*
   * assume that the board is mounted with a known pitch angle, but that there is no yaw or roll
   */
  constexpr float pitch = 0. * M_PI / 180.0;
  constexpr float cosPitch = cos(pitch), sinPitch = sin(pitch);


  /*
   * Main while loop.
   *
   * We stay here until the ignition turns off.
   */
  bool firstAcc = false;
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
    if( acc_int_rdy )
    {
      // ~245us
      BMI088_ReadAccelerometer(&imu);
    }
#elif defined ACC_USE_IT
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
#endif



    /* Print */
    if ((HAL_GetTick () - timerPrint) >= SAMPLE_TIME_MS_PRINT)
    {
      int ind = 0;
      ind += snprintf (logBuf+ind, bufLen-ind, "\033[1J");
      ind += snprintf (logBuf+ind, bufLen-ind, "tach: %.1f , speedo %.1f  \n",  rpm,  speed);
      ind += snprintf (logBuf+ind, bufLen-ind, "acc: %.1f, %.1f, %.1f", imu.acc_mps2[0],imu.acc_mps2[1],imu.acc_mps2[2]);
      if(flagSlow)
      {
	ind += snprintf (logBuf+ind, bufLen-ind, "main loop running slow \n");
      }
#ifdef PRINT_TO_USB
      CDC_Transmit_FS ((uint8_t*) logBuf, ind);
#else
      printf("%s", logBuf);
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
      rpm += (float)SAMPLE_TIME_MS_UPDATES / 1000. * 3000;
      if(rpm>7000)
        rpm = 0;
      speed += (float)SAMPLE_TIME_MS_UPDATES / 1000. * 40;
      if(speed>100)
        speed = 0;
      //speed = (imu.acc_mps2[2] / 9.8) * 50 + 50;
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
      if(rpm < 0) rpm = 0;
      else if(rpm > 9000) rpm = 9000;
      speed = speed * MPH_PER_HZ;
      if(speed < 0) speed = 0;
      else if(speed > 180) speed = 180;
#endif
      tachX12.setPosition( get_x12_ticks_rpm(rpm) );
      speedX12.setPosition( get_x12_ticks_speed(speed) );
      timerUpdates = HAL_GetTick ();


      /*
       * display updates
       */
      gdispClear(GFX_BLACK); // if the device doesnt support flushing, then this is immediate

      snprintf (logBuf, bufLen, "acc:%.1f,%.1f,%.1f", imu.acc_mps2[0],
                imu.acc_mps2[1], imu.acc_mps2[2]);
      gdispFillString(1, 16, logBuf, font, GFX_AMBER, GFX_BLACK);
      drawVertBarGraph (90, 17, 7, 45, 9.7, 0, imu.acc_mps2[0]);
      //drawHorzBarGraph(5, 50, 45, 7, 10, 0, imu.acc_mps2[2]);

      snprintf (logBuf, bufLen, "rpm: %.1f", rpm);
      gdispFillString(1, 25, logBuf, font, GFX_AMBER, GFX_BLACK);
      drawVertBarGraph (99, 17, 7, 45, 30, 0, rpm);

      snprintf (logBuf, bufLen, "spd: %.1f", speed);
      gdispFillString(1, 35, logBuf, font, GFX_AMBER, GFX_BLACK);
      drawVertBarGraph (109, 17, 7, 45, 30, 0, speed);

      // make x be -x, flip x and y
      drawGimball (65, 45, 20, -imu.acc_mps2[1] / 9.8 * 20,
                   imu.acc_mps2[0] / 9.8 * 20);

      // this box is exactly the size of the top yellow area on the
      // common amazon ssd1306, 0.96" displays
      gdispDrawBox(0, 0, 128, 16, GFX_AMBER);


      // some devices dont support this and instead they draw whenever you call a drawing function
      // but its always safe to call it
      gdispFlush();


    }

    /*
     * odometer ticks
     */
    if(odo_tick_flag && odoX12.stopped)
    {
      odoX12.setPosition(odoX12.targetStep+ODO_STEPS_PER_TICK);
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
      tachX12.setPosition (get_x12_ticks_rpm(7000));
      speedX12.setPosition (get_x12_ticks_speed(180) );
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

