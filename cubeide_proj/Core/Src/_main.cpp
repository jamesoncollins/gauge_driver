
#include "main.h"
#include "usb_device.h"

#include "usbd_cdc_if.h"
extern "C" {
#include "../../MCP4725-lib/MCP4725.h"
#include "../../BMI088-lib/BMI088.h"
}
#include "../../SwitecX12-lib/SwitecX12.hpp"

#define SAMPLE_TIME_MS_USB  250
#define SAMPLE_TIME_MS_LED  500
#define SAMPLE_TIME_MS_ATT   50

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

MCP4725 dac;
BMI088 imu;

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT_ACC_Pin)
  {
    BMI088_ReadAccelerometerDMA (&imu);
  }
  else if (GPIO_Pin == INT_GYR_Pin)
  {
    BMI088_ReadGyroscopeDMA (&imu);
  }
}

#define SPEEDOIND 0
#define TACHIND 1
#define IDLE   0
#define DONE   1
#define F_CLK  64000000UL
#define TICKS_PER_MILE (30)
#define SPEED_TICKS_PER_ODO_TICK (TICKS_PER_MILE/10)
#define HZ_PER_MPH ((float) TICKS_PER_MILE / 3600.f)
volatile uint8_t state[2] = {IDLE, IDLE};
volatile uint32_t T1[2] = {0,0};
volatile uint32_t T2[2] = {0,0};
volatile uint32_t ticks[2] = {0,0};
volatile float freq_Hz[2] = {0,0};
volatile uint32_t TIM2_OVC[2] = {0,0};
volatile uint32_t speed_tick_count = 0;
volatile bool odo_tick_flag = false;

void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
  int ch = (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) ? SPEEDOIND : TACHIND;
  if (state[ch] == IDLE)
  {
    T1[ch] = TIM2->CCR1;
    TIM2_OVC[ch] = 0;
    state[ch] = DONE;
  }
  else if (state[ch] == DONE)
  {
    T2[ch] = TIM2->CCR1;
    ticks[ch] = (T2[ch] + (TIM2_OVC[ch] * 65536)) - T1[ch];
    if(ticks[ch] == 0 )
      freq_Hz[ch] = 0;
    else
      freq_Hz[ch] = (float)F_CLK / (float) ticks[ch];
    state[ch] = IDLE;
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

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  TIM2_OVC[0]++;
  TIM2_OVC[1]++;
}

/**
 * @brief  This function provides a delay (in microseconds)
 * @param  microseconds: delay in microseconds
 */
void DWT_Delay_us(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main_cpp(void)
{

  /* Timers */
  //uint32_t timerBAR = 0;
  uint32_t timerUSB = 0;
  uint32_t timerLED	= 0;
  uint32_t timerIGN = 0;
  //uint32_t timerATT = 0;

  /* USB data buffer */
  char logBuf[128];

  /*
   * dac setup
   */
  dac = MCP4725_init (&hi2c1, MCP4725A0_ADDR_A00, 3.30);
  if (MCP4725_isConnected (&dac))
  {
    sprintf (logBuf, "DAC Connected\n");
    CDC_Transmit_FS ((uint8_t*) logBuf, strlen (logBuf));
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
  BMI088_Init(&imu, NULL, &hi2c1, NULL, 0, NULL, 0);

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

  SwitecX12 tachX12(
      100,
      STEP_TACH_GPIO_Port,
      STEP_TACH_Pin,
      DIR_TACH_GPIO_Port,
      DIR_TACH_Pin
      );

  SwitecX12 speedX12(
      100,
      STEP_SPEED_GPIO_Port,
      STEP_SPEED_Pin,
      DIR_SPEED_GPIO_Port,
      DIR_SPEED_Pin
      );

  SwitecX12 odoX12(
      0xFFFFFFFE,
      STEP_ODO_GPIO_Port,
      STEP_ODO_Pin,
      DIR_ODO_GPIO_Port,
      DIR_ODO_Pin
      );


  /*
   * Turn on mcu-controlled pwr-en signal.
   */
  HAL_GPIO_WritePin ( PWREN_GPIO_Port, PWREN_Pin, GPIO_PIN_SET );


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Log data via USB */
    if ((HAL_GetTick () - timerUSB) >= SAMPLE_TIME_MS_USB)
    {
      float PPR = 3;
      float GR = 1; // tach gear ratio
      float RPM = (float)freq_Hz[TACHIND]*60 / PPR * GR;
      sprintf (logBuf, "tach: %g Hz, speedo %g Hz \n", freq_Hz[0], freq_Hz[1]);
      CDC_Transmit_FS ((uint8_t*) logBuf, strlen (logBuf));
      //enable ITM Stimulus Port 0.
      printf("SWV console %g %g %g\n", (float)freq_Hz[0], (float)freq_Hz[1], RPM);
      timerUSB = HAL_GetTick ();
    }

    /* Toggle LED */
    if ((HAL_GetTick () - timerLED) >= SAMPLE_TIME_MS_LED)
    {
      HAL_GPIO_TogglePin ( LED_GPIO_Port, LED_Pin );
      timerLED = HAL_GetTick ();
    }

    /**
     * get attitude measurements
     */
    uint8_t data;
    BMI088_ReadAccRegister(&imu, BMI_ACC_INT_STAT_1, &data);
    //if ((HAL_GetTick () - timerATT) >= SAMPLE_TIME_MS_ATT)
    if( data&0b1000000 )
    {
      BMI088_ReadAccelerometer(&imu);
      sprintf(logBuf, "acc (mps): %g\t%g\t%g \n", imu.acc_mps2[0],imu.acc_mps2[1],imu.acc_mps2[2]);
      CDC_Transmit_FS ((uint8_t*) logBuf, strlen (logBuf));
      //timerATT = HAL_GetTick ();
    }

    /*
     * odometer ticks
     */
    if(odo_tick_flag)
    {
      odoX12.step(1);
      tachX12.step(1);
      speedX12.step(1);
      odo_tick_flag = false;
    }

    /*
     * check ignition signal status.  we want it low for at least 10ms
     * before we decide the car is off.
     */
    GPIO_PinState ign = HAL_GPIO_ReadPin ( IGN_GPIO_Port, IGN_Pin );
    if ( !ign  )
    {
      if( !timerIGN )
	timerIGN = HAL_GetTick ();
      else if ( (HAL_GetTick () - timerIGN) >= 10 )
	break; // break the main loop
    }
  }

  /*
   * Power-down loop, do anything we need to in order to cleanup before
   * we power off.
   */
  speedX12.stepTo(0);
  tachX12.stepTo(0);
  while(1)
  {
    speedX12.update();
    tachX12.update();
    if(speedX12.currentStep==0 && tachX12.currentStep==0)
      break;
  }

  /*
   * we left the main loop, we can power down
   */
  HAL_GPIO_WritePin ( PWREN_GPIO_Port, PWREN_Pin, GPIO_PIN_RESET );
  HAL_Delay(1000);

  return 0;
}

