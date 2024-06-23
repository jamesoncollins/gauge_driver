
#ifndef PI4IOE5V6416_PI4IOE5V6416_HPP_
#define PI4IOE5V6416_PI4IOE5V6416_HPP_

#include "stm32wbxx_hal.h"

const uint8_t ADDR = 0b01000000;

class PI4IOE5V6416
{
public:
  PI4IOE5V6416(I2C_HandleTypeDef *i2c);
  int init(uint16_t pullup_mask);
  uint16_t get();
  int get_IT(uint16_t *buffer);

  I2C_HandleTypeDef *i2cdev;
  uint8_t status = 0;
  uint16_t buffer;
};


#endif /* PI4IOE5V6416_PI4IOE5V6416_HPP_ */
