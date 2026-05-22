/*
 * This is going to be where we place all weak overrides of STM interrupts
 */

#ifndef __arm__
#error "THIS CODE IS FOR ARM"
#endif

#include <array>

#include "main.h"
#include "cpp_main.h"
#include "runtime_context.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"
extern "C" {
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
}
extern I2C_HandleTypeDef hi2c1, hi2c3;
extern SPI_HandleTypeDef hspi1;
extern LPTIM_HandleTypeDef hlptim2; // sim signals for tach/rpm on GPIO3 / PA8
extern TIM_HandleTypeDef htim1; // speaker SPI-DMA control i think, (64MHz counting to 6399+1 = 10khz)
extern TIM_HandleTypeDef htim2; // speed/tach measurement (capture control), 64MHz / (63+1) = 1MHz
extern TIM_HandleTypeDef htim16; // handle ecu at 2khz, 64MHz counting to 31999+1
extern TIM_HandleTypeDef htim17; // stepper motor ticks.  64MHz/64.  we update the ARR on the fly to change the duty cycle.
extern UART_HandleTypeDef huart1;
extern RTC_HandleTypeDef hrtc;

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
 * speed and RPM are measured by TIM2, at 1MHz, using capture-compare registers.
 *
 */
volatile static uint32_t speed_tick_count = 0;
void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2) return;

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
	static uint32_t last = 0, diff = 0;
	diff = TIM2->CCR3 - last;
	last = TIM2->CCR3;
	g_speed.tick(diff);

    // Odometer ticks with speed
    speed_tick_count++;
    if (speed_tick_count == SPEED_TICKS_PER_ODO_TICK)
    {
      speed_tick_count = 0;
      odo_ticks += ODO_STEPS_PER_TICK;
    }
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
	static uint32_t last, diff = 0;
	diff = TIM2->CCR4 - last;
	last = TIM2->CCR4;
	g_tach.tick(diff);
  }
}



/*
 * Timer-period-elapsed callback.
 *
 * Used for several housekeeping functions.
 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  static int worst_timing = 0;
  int end, start = HAL_GetTick();

  if (htim->Instance == TIM16)
  {
    /*
     * 2 kHz
     */
    ecu.update();
  }
  else if(htim->Instance == TIM17)
  {
    /*
     * Stepper motor ticks/steps.
     *
     * ticks at 1us.
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
