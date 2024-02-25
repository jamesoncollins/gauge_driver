/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.io/license.html
 */

#ifndef _GDISP_LLD_BOARD_H
#define _GDISP_LLD_BOARD_H

//#define SSD1306_PAGE_PREFIX        0x40
//#define SSD1306_SH1106 1


#include "utils.h"
#include "main.h"
#include "common.h"
#include "gfx.h"

extern SPI_HandleTypeDef hspi2;

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

    GPIO_InitTypeDef GPIO_InitStruct =
    {   0};

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
    //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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
    (void) state;
    if(state)
        CLR_RST;
    else
        SET_RST;
}

static GFXINLINE void acquire_bus(GDisplay *g)
{
    (void) g;
    //while(!lock_mutex(&spi2_lock)){};
//    while (HAL_SPI_GetState(&SPIDEV) != HAL_SPI_STATE_READY);
//    while (HAL_DMA_GetState(&hdma_spi2_tx) != HAL_DMA_STATE_READY);
//    if(HAL_SPI_DeInit(&SPIDEV) != HAL_OK )
//        Error_Handler();
//    hspi2.Instance = SPI2;
//    hspi2.Init.Mode = SPI_MODE_MASTER;
//    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
//    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
//    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
//    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
//    hspi2.Init.NSS = SPI_NSS_SOFT;
//    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
//    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//    hspi2.Init.CRCPolynomial = 10;
//    if (HAL_SPI_Init(&SPIDEV) != HAL_OK)
//    {
//      Error_Handler();
//    }
//    CLR_CS;
}

static GFXINLINE void release_bus(GDisplay *g)
{
    (void) g;
    //unlock_mutex(&spi2_lock);
//    SET_CS;
}

static GFXINLINE void write_cmd(GDisplay *g, gU8 cmd)
{
    (void) g;
    (void) cmd;

    while (HAL_SPI_GetState(&SPIDEV) != HAL_SPI_STATE_READY);
    //while (HAL_DMA_GetState(&hdma_spi2_tx) != HAL_DMA_STATE_READY);
    //while ( !GET_CS );
    CLR_DC;
    CLR_CS;
    HAL_SPI_Transmit(&SPIDEV, (uint8_t *)&cmd, 1, HAL_MAX_DELAY);
    SET_CS;

}

static GFXINLINE void write_data(GDisplay *g, gU8* data, gU16 length)
{
    (void) g;
    (void) data;
    (void) length;


    while (HAL_SPI_GetState(&SPIDEV) != HAL_SPI_STATE_READY);
    //while (HAL_DMA_GetState(&hdma_spi2_tx) != HAL_DMA_STATE_READY);
   // while ( !GET_CS );
    SET_DC;
    CLR_CS;
    HAL_SPI_Transmit(
            &SPIDEV,
            (uint8_t *)data,
            length ,
            HAL_MAX_DELAY);
    SET_CS;

}

#endif /* _GDISP_LLD_BOARD_H */
