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

#define RST_PIN    GPIO_PIN_0 // labeled scl on connector
#define RST_PORT   GPIOC

#define DC_PIN     GPIO_PIN_1 // labeled sda on connector
#define DC_PORT    GPIOC

#define CLR_RST CLEAR_BIT(RST_PORT->ODR, RST_PIN)
#define SET_RST SET_BIT(RST_PORT->ODR, RST_PIN)

#define CLR_DC CLEAR_BIT(DC_PORT->ODR, DC_PIN)
#define SET_DC SET_BIT(DC_PORT->ODR, DC_PIN)
#define GET_DC READ_BIT(DC_PORT->IDR, DC_PIN)

#define CLR_CS CLEAR_BIT(CS_PORT->ODR, CS_PIN)
#define SET_CS SET_BIT(CS_PORT->ODR, CS_PIN)
#define GET_CS READ_BIT(CS_PORT->IDR, CS_PIN)

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

	  SET_DC; // defaults to 'data'
	  CLR_RST;
	  SET_CS;
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
	(void) percent;
}

static GFXINLINE void acquire_bus(GDisplay *g)
{
	(void) g;
}

static GFXINLINE void release_bus(GDisplay *g)
{
	(void) g;
}

static GFXINLINE void write_index(GDisplay *g, gU16 index)
{
  (void) g;

  while (HAL_SPI_GetState(&SPIDEV) != HAL_SPI_STATE_READY);
  CLR_DC;
  CLR_CS;
  HAL_SPI_Transmit(&SPIDEV, (uint8_t *)&index, 1, HAL_MAX_DELAY);
  SET_CS;
}

static GFXINLINE void write_data(GDisplay *g, gU16 data)
{
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
