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
extern I2C_HandleTypeDef hi2c3;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_memtomem_dma2_channel1;

#define SPIDEV     hspi2

#define CS_PIN     GPIO_PIN_6
#define CS_PORT    GPIOB

#define RST_PIN    GPIO_PIN_12
#define RST_PORT   GPIOC

#define DC_PIN     GPIO_PIN_4
#define DC_PORT    GPIOC

#define PWR_EN_PIN     GPIO_PIN_4
#define PWR_EN_PORT    GPIOA

#define CLR_RST CLEAR_BIT(RST_PORT->ODR, RST_PIN)
#define SET_RST SET_BIT(RST_PORT->ODR, RST_PIN)

#define CLR_DC CLEAR_BIT(DC_PORT->ODR, DC_PIN)
#define SET_DC SET_BIT(DC_PORT->ODR, DC_PIN)
#define GET_DC READ_BIT(DC_PORT->IDR, DC_PIN)

#define CLR_CS CLEAR_BIT(CS_PORT->ODR, CS_PIN)
#define SET_CS SET_BIT(CS_PORT->ODR, CS_PIN)
#define GET_CS READ_BIT(CS_PORT->IDR, CS_PIN)

#define CLR_PWR_EN CLEAR_BIT(PWR_EN_PORT->ODR, PWR_EN_PIN)
#define SET_PWR_EN SET_BIT(PWR_EN_PORT->ODR, PWR_EN_PIN)
#define GET_PWR_EN READ_BIT(PWR_EN_PORT->IDR, PWR_EN_PIN)

void DMA_TxCpltCallback (DMA_HandleTypeDef *);

bool busy = false;
uint8_t *data_ptr;
uint32_t size_left = 0;
uint16_t xfer_len;
bool autoClear = false;
uint32_t clear_int = 0;

/*
 * pack two into this, becuase its 32bit instead of 16
 */
void setClearColor(uint32_t color)
{
  clear_int = color;
}

void setAutoClear()
{
  autoClear = true;
}

bool bus_busy ()
{
  return busy;
}

static void setup_regulator()
{

  /*
   * configure the off-board programable regulator for an oled display
   * to be +4.6V and -4.4V
   */
  const uint8_t PWR_ADDR = 0x3E<<1;
  uint8_t data_desired[4] =
  {
      0b00110, // 4.6v
      0b00100, // -4.4v
      0b00000000,
      0b01000011,
  };
  uint8_t data_have[4];
  HAL_I2C_Mem_Read(
      &hi2c3,
      PWR_ADDR,
      0x00, 1,
      data_have, 4,
      1000
      );
  bool isSame = true;
  for(int i=0; i<4; i++)
  {
    isSame &= (data_desired[i] == data_have[i]);
  }

  if(!isSame)
  {
    /*
     * load the values we want and flash eeprom
     */
    HAL_I2C_Mem_Write(
        &hi2c3,
        PWR_ADDR,
        0x00, 1,
        data_desired, 4,
        1000
        );

    /*
     * flash them to eeprom
     */
    uint8_t val = 0b10000000;
    HAL_I2C_Mem_Write(
        &hi2c3,
        PWR_ADDR,
        0xFF, 1,
        &val, 1,
        1000
        );

    /*
     * need to wait 50ms
     */
    HAL_Delay(50);

    /*
     * verify
     */
    HAL_I2C_Mem_Read(
        &hi2c3,
        PWR_ADDR,
        0x00, 1,
        data_have, 4,
        1000
        );
  }
}

static GFXINLINE void init_board (GDisplay *g)
{
  (void) g;

  //while(HAL_ERROR==HAL_SPI_RegisterCallback(&SPIDEV, HAL_SPI_TX_COMPLETE_CB_ID, &clear_cs));

  setup_regulator();

  GPIO_InitTypeDef GPIO_InitStruct =  { 0 };

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

  GPIO_InitStruct.Pin = PWR_EN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (PWR_EN_PORT, &GPIO_InitStruct);

  SET_DC; // defaults to 'data'
  CLR_PWR_EN;
  CLR_RST;
  SET_CS;

  HAL_DMA_RegisterCallback(
      &hdma_memtomem_dma2_channel1,
      HAL_DMA_XFER_CPLT_CB_ID,
      DMA_TxCpltCallback);

  // dummy transmit, makes MOSI idle high.  seems to be required
  // in order for the display to pick the SPI interface.
  // without this the screen starts pink
  uint8_t dat = 0xff;
  HAL_SPI_Transmit (&SPIDEV, &dat, 1, HAL_MAX_DELAY);
}

static GFXINLINE void pwr_en(bool on)
{
  // in this board rev we are using DC to both set the screen data mode and turn on
  // on the adjustable regulator.  is this a good idea?  probably not
  if(on)
    SET_PWR_EN;
  else
    CLR_PWR_EN;
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

static uint16_t read_index (uint16_t index)
{

  while (busy);
  while (HAL_SPI_GetState (&SPIDEV) != HAL_SPI_STATE_READY);

  uint32_t id = 0;
  uint8_t buffer[3], buffer_rx[3];

  buffer[1] = ((index & 0xff00) >> 8);
  buffer[2] = ((index & 0x00ff) >> 0);

  // send start byte and register
  CLR_CS;
  buffer[0] = 0x70 | id | (0 << 1) | 0; // set index register
  HAL_SPI_Transmit (&SPIDEV, buffer, 3, HAL_MAX_DELAY);
  SET_CS;
  //HAL_Delay (1);

  // send start byte, but with the data bit and read bit
  CLR_CS;
  buffer[0] = 0x70 | id | (1 << 1) | 1; // read register data
  HAL_SPI_Transmit (&SPIDEV, buffer, 1, HAL_MAX_DELAY);
  SET_CS;
  //HAL_Delay (1);


  // get teh read data, including a dummy byte
  // this is only a requirement in this version of the manual: http://www.avr-developers.com/liquidwaredocs/pdfs/S6E63D6-320x240-OLEDcontroller.pdf
  CLR_CS;
  HAL_SPI_Receive (&SPIDEV, buffer_rx, 3, HAL_MAX_DELAY);
  SET_CS;
 // HAL_Delay (1);

  return (uint16_t) (buffer_rx[1]<<8 | buffer_rx[2]);

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
  uint8_t buffer[3];
  buffer[0] = 0x70 | id | (rs<<1) | 0;
  buffer[1] = ((word&0xff00)>>8);
  buffer[2] = ((word&0x00ff)>>0);

  CLR_CS;
  if( rs<=1 )
    HAL_SPI_Transmit (&SPIDEV, buffer, 3, HAL_MAX_DELAY);
  else
    HAL_SPI_Transmit (&SPIDEV, &buffer[1], 2, HAL_MAX_DELAY);
  if(!leavelow)
    SET_CS;
}


static GFXINLINE void write_index (GDisplay *g, uint8_t idx)
{
  (void) g;
  send_word(0, idx, false);
}

static GFXINLINE void write_data_one (GDisplay *g, uint16_t param)
{
  (void) g;
  send_word(1, param, false);
}

static GFXINLINE void write_data_one_leave_low (GDisplay *g, uint16_t param, bool first, bool leaveLow)
{
  (void) g;
  if(first)
    send_word(1, param, leaveLow);
  else
    send_word(2, param, leaveLow);
}

/*
 * WARNING: This is a DMA transfer. It will not set CS, that gets done in the
 * callback.  DOn't use this for anything you need to do in a loop in the
 * driver.
 */
static GFXINLINE void write_data (GDisplay *g, uint8_t *data, unsigned int length)
{
  (void) g;
  busy = true;

  /*
   * dont spinlock in a function that gets called from an interrupt
   */
  //while (HAL_SPI_GetState (&SPIDEV) != HAL_SPI_STATE_READY);
  //while (HAL_DMA_GetState (&hdma_spi2_tx) != HAL_DMA_STATE_READY);
  //while (HAL_DMA_GetState (&hdma_memtomem_dma2_channel1) != HAL_DMA_STATE_READY);
//  SET_DC;
  CLR_CS;
  if (length > 65536)
    size_left = length - 65535;
  else
    size_left = 0;
  xfer_len = (length > 65535) ? 65535 : length;
  data_ptr = data + xfer_len;
  HAL_SPI_Transmit_DMA (&SPIDEV, data, xfer_len);

}

void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == hspi2.Instance)
  {
    /*
     * start mem2mem dma transfer to clear source memory thats
     * already been transfered.
     *
     * we should be polling to make sure the job was done, but thats
     * dangerous in an interrupt, so i guess we just live with a possible
     * temporary screen curruption.
     *
     * note, we're always a buffer behind.  we clear the buffer that just finished
     * going out the SPI.
     *
     * becuase the other transfer is SPI I cant imagine we'll ever hit this.
     */
    if(autoClear)
    {
      HAL_DMA_Start_IT(
          &hdma_memtomem_dma2_channel1,
          (uint32_t)&clear_int,
          (uint32_t)(uint8_t*)(data_ptr - xfer_len),
          xfer_len
          );
    }

    if (size_left)
    {
      write_data (NULL, data_ptr, size_left);
    }
    else
    {
      SET_CS;

      /*
       * we're assuming that the xfer always finishes after
       * the clearing
       */
      busy = false;
    }
  }
}

/*
 * a clearing job is done
 */
void DMA_TxCpltCallback(DMA_HandleTypeDef *hdma)
{

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
