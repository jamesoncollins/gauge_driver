
#include "PI4IOE5V6416.hpp"

PI4IOE5V6416::PI4IOE5V6416(I2C_HandleTypeDef *i2c)
{
  i2cdev = i2c;
}

/*
 * Configure all ports to use pull or or pull down resistors.
 *
 * Default is pull-down.
 */
int PI4IOE5V6416::init( uint16_t pullup_mask )
{
  status = 0;

  /*
   * note sure what this is here for, doesnt do anthing
   */
  status |= HAL_I2C_Mem_Read(
            i2cdev,
            ADDR,       // dev address
            0x00, 1,       // mem address, mem address size
            (uint8_t*)&buffer,     // data buffer
            2,          // data read len
            1000);

  // set IO mode
  // nevermind, this defaults to high-impedance input

  // enable pull-up / pull-down resistor
  buffer = 0xffff;
  status |= HAL_I2C_Mem_Write(
            i2cdev,
            ADDR,
            0x46, 1,
            (uint8_t*)&buffer,
            2,
            1000);

  // select pull-up or pull-down resistor
  buffer = pullup_mask;
  status |= HAL_I2C_Mem_Write(
            i2cdev,
            ADDR,
            0x48, 1,
            (uint8_t*)&buffer,
            2,
            1000);



  return status;
}

uint16_t PI4IOE5V6416::get()
{
  status = 0;
  buffer = 0x00000;
  status |= HAL_I2C_Mem_Read(
            i2cdev,
            ADDR,
            0, 1,
            (uint8_t*)&buffer,
            2,
            1000);

  return buffer;
}

int PI4IOE5V6416::get_IT(uint16_t *user_buffer)
{
  status = 0;
  *user_buffer = 0x0000;
  status = 0;
  status |= HAL_I2C_Mem_Read_IT(
            i2cdev,
            ADDR,
            0, 1,
            (uint8_t*)user_buffer,
            2);
  return status;
}
