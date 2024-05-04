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

#define SPIDEV     hspi2

#define CS_PIN     GPIO_PIN_6
#define CS_PORT    GPIOB

#define RST_PIN    GPIO_PIN_12
#define RST_PORT   GPIOC

#define DC_PIN     GPIO_PIN_4
#define DC_PORT    GPIOC

#define CLR_RST CLEAR_BIT(RST_PORT->ODR, RST_PIN)
#define SET_RST SET_BIT(RST_PORT->ODR, RST_PIN)

#define CLR_DC CLEAR_BIT(DC_PORT->ODR, DC_PIN)
#define SET_DC SET_BIT(DC_PORT->ODR, DC_PIN)
#define GET_DC READ_BIT(DC_PORT->IDR, DC_PIN)

#define CLR_CS CLEAR_BIT(CS_PORT->ODR, CS_PIN)
#define SET_CS SET_BIT(CS_PORT->ODR, CS_PIN)
#define GET_CS READ_BIT(CS_PORT->IDR, CS_PIN)

bool busy = false;
uint8_t *data_ptr;
uint32_t size_left = 0;

bool bus_busy ()
{
  return busy;
}

static GFXINLINE void init_board (GDisplay *g)
{
  (void) g;

  //while(HAL_ERROR==HAL_SPI_RegisterCallback(&SPIDEV, HAL_SPI_TX_COMPLETE_CB_ID, &clear_cs));

  GPIO_InitTypeDef GPIO_InitStruct =
  { 0 };

  GPIO_InitStruct.Pin = CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (CS_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (RST_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (DC_PORT, &GPIO_InitStruct);

  SET_DC; // defaults to 'data'
  CLR_RST;
  SET_CS;
}

static GFXINLINE void post_init_board (GDisplay *g)
{
  (void) g;
}

static GFXINLINE void setpin_reset (GDisplay *g, gBool state)
{
  (void) g;
  if (state)
    CLR_RST;
  else
    SET_RST;
}

static GFXINLINE void set_backlight (GDisplay *g, gU8 percent)
{
  (void) g;
  (void) percent;
}

static GFXINLINE void acquire_bus (GDisplay *g)
{
  (void) g;
}

static GFXINLINE void release_bus (GDisplay *g)
{
  (void) g;
}


/*
 * Each transfer is performed as:
 * 1. chip-select active
 * 2. send 8-bit start code
 * 3. send 16-bit data
 * 4. chip-select inactive
 */
static void send_word (uint8_t rs, uint16_t word, int leavelow)
{

  while (busy);
  while (HAL_SPI_GetState (&SPIDEV) != HAL_SPI_STATE_READY);
  /*
   * The start byte looks like (binary):
   * 01110<ID><RS><R/W>
   * RS is 0 for index or 1 for data, and R/W is 0 for write.
   */
  uint32_t id = 0;
  uint32_t buf8 = 0x70 | id | (rs & 2);
  uint32_t buf16 =  ((word>>8) | (word<<8)); // byteswap

  CLR_CS;
  HAL_SPI_Transmit (&SPIDEV, (uint8_t*) &buf8, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit (&SPIDEV, (uint8_t*) &buf16, 2, HAL_MAX_DELAY);
  if(!leavelow) SET_CS;
}


static GFXINLINE void write_index (GDisplay *g, uint8_t idx)
{
  (void) g;
  send_word(0, idx, false);
}

static GFXINLINE void write_data_one (GDisplay *g, uint16_t param)
{
  (void) g;
  send_word(2, param, false);
}

static GFXINLINE void write_data_one_leave (GDisplay *g, uint16_t param)
{
  (void) g;
  send_word(2, param, true);
}

/*
 * WARNING: This is a DMA transfer. It will not set CS, that gets done in the
 * callback.  DOn't use this for anything you need to do in a loop in the
 * driver.
 */
static GFXINLINE void write_data (GDisplay *g, gU8 *data, unsigned int length)
{
  (void) g;
  busy = true;
  while (HAL_SPI_GetState (&SPIDEV) != HAL_SPI_STATE_READY);
  while (HAL_DMA_GetState (&hdma_spi2_tx) != HAL_DMA_STATE_READY);
//  SET_DC;
  CLR_CS;
  if (length > 65536)
    size_left = length - 65535;
  else
    size_left = 0;
  uint16_t xfer_len = (length > 65535) ? 65535 : length;
  data_ptr = data + xfer_len;
  HAL_SPI_Transmit_DMA (&SPIDEV, (uint8_t*) data, xfer_len);

}

void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == hspi2.Instance)
  {
    if (size_left)
    {
      write_data (NULL, data_ptr, size_left);
    }
    else
    {
      SET_CS;
      busy = false;
    }
  }
}

static GFXINLINE void setreadmode (GDisplay *g)
{
  (void) g;
}

static GFXINLINE void setwritemode (GDisplay *g)
{
  (void) g;
}

static GFXINLINE gU16 read_data (GDisplay *g)
{
  (void) g;

  return 0x9341;
}

#endif /* GDISP_LLD_BOARD_H */
