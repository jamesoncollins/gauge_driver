/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.io/license.html
 */

#ifndef GDISP_LLD_BOARD_H
#define GDISP_LLD_BOARD_H

#include "utils.h"
#include "main.h"
#include "common.h"
#include "gfx.h"

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern LPTIM_HandleTypeDef hlptim2;
extern DMA_HandleTypeDef hdma_memtomem_dma2_channel1;
#define dma_memtomem hdma_memtomem_dma2_channel1

#define SPIDEV     hspi2

#define CS_PIN     GPIO_PIN_12   // SPI_LCD_nCS (reuse OLED wiring)
#define CS_PORT    GPIOB

#define RST_PIN    GPIO_PIN_12
#define RST_PORT   GPIOC

#define DC_PIN     GPIO_PIN_4    // was OLED PWR_EN, now D/C
#define DC_PORT    GPIOC

// Backlight MOSFET gate on PA8 via LPTIM2_OUT
#define BL_PIN     GPIO_PIN_8
#define BL_PORT    GPIOA

#define CLR_RST CLEAR_BIT(RST_PORT->ODR, RST_PIN)
#define SET_RST SET_BIT(RST_PORT->ODR, RST_PIN)

#define CLR_DC CLEAR_BIT(DC_PORT->ODR, DC_PIN)
#define SET_DC SET_BIT(DC_PORT->ODR, DC_PIN)
#define GET_DC READ_BIT(DC_PORT->IDR, DC_PIN)

#define CLR_CS CLEAR_BIT(CS_PORT->ODR, CS_PIN)
#define SET_CS SET_BIT(CS_PORT->ODR, CS_PIN)
#define GET_CS READ_BIT(CS_PORT->IDR, CS_PIN)

// Backlight polarity: active HIGH (MOSFET gate high turns LED on)
#define BL_ACTIVE_LOW 0

bool busy = false;
uint8_t *data_ptr;
uint32_t size_left = 0;
uint32_t xfer_len = 0;
bool autoClear = false;
uint32_t clear_int = 0;
bool isLastClear = false;
bool txPending = false;
static bool bl_started = false;
static bool bl_gpio_prepped = false;
// 20 kHz PWM from ~500 kHz LPTIM2 clock -> ARR = 25
static const uint32_t BL_PERIOD_TICKS = 25U;

void DMA_TxCpltCallback (DMA_HandleTypeDef *);

bool getAutoClear()
{
  return autoClear;
}

/*
 * pack two into this, because it's 32-bit instead of 16
 */
void setClearColor(uint32_t color)
{
  clear_int = color;
}

void setAutoClear(bool cl)
{
  autoClear = cl;
}

bool bus_busy()
{
  return busy;
}

// Ensure backlight gate starts OFF as GPIO before switching to AF/PWM
static void prep_backlight_pin_off(void)
{
  if (bl_gpio_prepped)
    return;
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = BL_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BL_PORT, &gpio);
  // Drive gate OFF (active-high gate = low)
  HAL_GPIO_WritePin(BL_PORT, BL_PIN, GPIO_PIN_RESET);
  bl_gpio_prepped = true;
}

static void ensure_backlight_pwm_started(void)
{
  if (bl_started)
    return;

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = BL_PIN;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF14_LPTIM2;
  HAL_GPIO_Init(BL_PORT, &gpio);

  uint32_t arr = (BL_PERIOD_TICKS > 1U) ? (BL_PERIOD_TICKS - 1U) : 1U;
  uint32_t cmp = 0U; // start OFF (active-high gate)
  HAL_LPTIM_PWM_Start(&hlptim2, arr, cmp);
  bl_started = true;
}

static GFXINLINE void init_board(GDisplay *g)
{
	(void) g;

	  //while(HAL_ERROR==HAL_SPI_RegisterCallback(&SPIDEV, HAL_SPI_TX_COMPLETE_CB_ID, &clear_cs));

	  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	  GPIO_InitStruct.Pin = CS_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CS_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RST_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DC_PORT, &GPIO_InitStruct);

  // Hold backlight OFF immediately on boot
  prep_backlight_pin_off();

  SET_DC; // defaults to 'data'
  CLR_RST;
  SET_CS;

    HAL_DMA_RegisterCallback(
        &dma_memtomem,
        HAL_DMA_XFER_CPLT_CB_ID,
        DMA_TxCpltCallback);

    return;
}

static GFXINLINE void post_init_board(GDisplay *g)
{
	(void) g;
}

static GFXINLINE void setpin_reset(GDisplay *g, gBool state)
{
  (void) g;
  if(state)
      CLR_RST;
  else
      SET_RST;
}

static GFXINLINE void set_backlight(GDisplay *g, gU8 percent)
{
	(void) g;
  if (percent > 100U)
    percent = 100U;

  percent = 100 - percent;

  uint32_t arr = (BL_PERIOD_TICKS > 1U) ? (BL_PERIOD_TICKS - 1U) : 1U;
  uint32_t cmp = (percent * arr) / 100U;

  if (percent == 100U) {
    // Some hardware still leaks light at CMP=0; force gate low via GPIO.
    HAL_LPTIM_PWM_Stop(&hlptim2);
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = BL_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BL_PORT, &gpio);
    HAL_GPIO_WritePin(BL_PORT, BL_PIN, GPIO_PIN_RESET); // off
    bl_started = false; // force re-init of PWM next time
    return;
  }

  // Re-init PWM if we stopped it above
  if (!bl_started) {
    ensure_backlight_pwm_started();
  }

  // Avoid stopping the PWM; CMP sets duty directly.
  __HAL_LPTIM_AUTORELOAD_SET(&hlptim2, arr);
  __HAL_LPTIM_COMPARE_SET(&hlptim2, cmp);
}

static GFXINLINE void acquire_bus(GDisplay *g)
{
	(void) g;
}

static GFXINLINE void release_bus(GDisplay *g)
{
	(void) g;
}

static GFXINLINE void write_index(GDisplay *g, uint8_t index)
{
  (void) g;
  while(busy);
  while (HAL_SPI_GetState(&SPIDEV) != HAL_SPI_STATE_READY);
  CLR_DC;
  CLR_CS;
  HAL_SPI_Transmit(&SPIDEV, (uint8_t *)&index, 1, HAL_MAX_DELAY);
  SET_CS;
}

static GFXINLINE void write_data_one(GDisplay *g, uint8_t data)
{
  while(busy);
  while (HAL_SPI_GetState(&SPIDEV) != HAL_SPI_STATE_READY);
  SET_DC;
  CLR_CS;
  HAL_SPI_Transmit(
          &SPIDEV,
          (uint8_t *)&data,
          1,
          HAL_MAX_DELAY);
  SET_CS;
}



/*
 * WARNING: This is a DMA transfer. It will not set CS, that gets done in the
 * callback.  DOn't use this for anything you need to do in a loop in the
 * driver.
 */
static GFXINLINE void write_data(GDisplay *g, gU8* data, unsigned int length)
{
    (void) g;
    busy = true;
    while (HAL_SPI_GetState(&SPIDEV) != HAL_SPI_STATE_READY);
    while (HAL_DMA_GetState(&hdma_spi2_tx) != HAL_DMA_STATE_READY);
    SET_DC;
    CLR_CS;
    if(length>65536)
      size_left = length-65535;
    else
      size_left = 0;
    xfer_len = (length>65535) ? 65535 : length;
    data_ptr = data + xfer_len;
    HAL_SPI_Transmit_DMA(
            &SPIDEV,
            (uint8_t *)data,
            xfer_len );

}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance==hspi2.Instance)
  {
    /*
     * start mem2mem dma transfer to clear source memory that's
     * already been transferred.
     */
    if(autoClear)
    {
      if(size_left)
        isLastClear = false;
      else
        isLastClear = true;
      HAL_DMA_Start_IT(
          &dma_memtomem,
          (uint32_t)&clear_int,
          ((uint32_t)data_ptr - xfer_len),
          xfer_len>>2 // 32-bit transfers
          );
    }

    if(size_left)
    {
      write_data(NULL, data_ptr, size_left);
    }
    else
    {
      SET_CS;
      if(!autoClear)
      {
        busy = false;
      }
    }
  }
}

/*
 * a clearing job is done
 */
void DMA_TxCpltCallback(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  if(isLastClear)
  {
    busy = false;
  }
}

static GFXINLINE void setreadmode(GDisplay *g)
{
	(void) g;
}

static GFXINLINE void setwritemode(GDisplay *g)
{
	(void) g;
}

static GFXINLINE gU16 read_data(GDisplay *g)
{
	(void) g;
	
	return 0x9341;
}

#endif /* GDISP_LLD_BOARD_H */
