#include "BMI088.h"




uint8_t txBuffer[2];
/*
 *
 * INITIALISATION
 *
 */
uint8_t BMI088_Init (BMI088 *imu,  I2C_HandleTypeDef *i2cHandle)
{

  /* Store interface parameters in struct */

  imu->i2cHandle = i2cHandle;

  /* Clear DMA flags */
  imu->readingAcc = 0;
  imu->readingGyr = 0;

  uint8_t status = 0;

  /*
   *
   * ACCELEROMETER
   *
   */


  /* Perform accelerometer soft reset */
  status |= BMI088_WriteAccRegister (imu, BMI_ACC_SOFTRESET, 0xB6);
  HAL_Delay (50);

  /* Check chip ID */
  uint8_t chipID;
  status |= BMI088_ReadAccRegister (imu, BMI_ACC_CHIP_ID, &chipID);

  if (chipID != 0x1E)
  {
    return 1;
  }
  HAL_Delay (10);

  /* Configure accelerometer  */
  //status |= BMI088_WriteAccRegister (imu, BMI_ACC_CONF, 0xA8); /* (no oversampling, ODR = 100 Hz, BW = 40 Hz) */
  status |= BMI088_WriteAccRegister (imu, BMI_ACC_CONF, 0xA7); /* (no oversampling, ODR = 50 Hz, BW = 20 Hz) */
  HAL_Delay (10);

  status |= BMI088_WriteAccRegister (imu, BMI_ACC_RANGE, 0x00); /* +- 3g range */
  HAL_Delay (10);

  /* Enable accelerometer data ready interrupt */
  status |= BMI088_WriteAccRegister (imu, BMI_INT1_IO_CONF, 0x0A); /* INT1 = push-pull output, active high */
  HAL_Delay (10);

  status |= BMI088_WriteAccRegister (imu, BMI_INT1_INT2_MAP_DATA, 0x04);
  HAL_Delay (10);

  /* Put accelerometer into active mode */
  status |= BMI088_WriteAccRegister (imu, BMI_ACC_PWR_CONF, 0x00);
  HAL_Delay (10);

  /* Turn accelerometer on */
  status |= BMI088_WriteAccRegister (imu, BMI_ACC_PWR_CTRL, 0x04);
  HAL_Delay (10);

  /* Pre-compute accelerometer conversion constant (raw to m/s^2) */
  imu->accConversion = 9.81f / 32768.0f * 2.0f * 1.5f; /* Datasheet page 27 */

  /* Set accelerometer TX buffer for DMA */
  imu->accTxBuf[0] = BMI_ACC_DATA | 0x80;

  /*
   *
   * GYROSCOPE
   *
   */

//  /* Perform gyro soft reset */
//  status |= BMI088_WriteGyrRegister (imu, BMI_GYR_SOFTRESET, 0xB6);
//  HAL_Delay (250);
//
//  /* Check chip ID */
//  status |= BMI088_ReadGyrRegister (imu, BMI_GYR_CHIP_ID, &chipID);
//
//  if (chipID != 0x0F)
//  {
//    return 1;
//  }
//  HAL_Delay (10);
//
//  /* Configure gyroscope */
//  status |= BMI088_WriteGyrRegister (imu, BMI_GYR_RANGE, 0x01); /* +- 1000 deg/s */
//  HAL_Delay (10);
//
//  status |= BMI088_WriteGyrRegister (imu, BMI_GYR_BANDWIDTH, 0x07); /* ODR = 100 Hz, Filter bandwidth = 32 Hz */
//  HAL_Delay (10);
//
//  /* Enable gyroscope data ready interrupt */
//  status |= BMI088_WriteGyrRegister (imu, BMI_GYR_INT_CTRL, 0x80); /* New data interrupt enabled */
//  HAL_Delay (10);
//
//  status |= BMI088_WriteGyrRegister (imu, BMI_INT3_INT4_IO_CONF, 0x01); /* INT3 = push-pull, active high */
//  HAL_Delay (10);
//
//  status |= BMI088_WriteGyrRegister (imu, BMI_INT3_INT4_IO_MAP, 0x01); /* Data ready interrupt mapped to INT3 pin */
//  HAL_Delay (10);
//
//  /* Pre-compute gyroscope conversion constant (raw to rad/s) */
//  imu->gyrConversion = 0.01745329251f * 1000.0f / 32768.0f; /* Datasheet page 39 */
//
//  /* Set gyroscope TX buffer for DMA */
//  imu->gyrTxBuf[0] = BMI_GYR_DATA | 0x80;

  return status;

}

uint8_t BMI088_ReadAccIntr(BMI088 *imu)
{
  uint8_t data = 0;
  BMI088_ReadAccRegister(imu, BMI_ACC_INT_STAT_1, &data);
  if( data&0b10000000 )
    return 1;
  return 0;
}

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */

/* ACCELEROMETER READS ARE DIFFERENT TO GYROSCOPE READS. SEND ONE BYTE ADDRESS, READ ONE DUMMY BYTE, READ TRUE DATA !!! */
uint8_t BMI088_ReadAccRegister (BMI088 *imu, uint8_t regAddr, uint8_t *data)
{
  uint8_t status = 0;
  status |= HAL_I2C_Master_Transmit (
	imu->i2cHandle, ACC_ADDR,
	&regAddr, 1, 1000);
  status |= HAL_I2C_Master_Receive (
	imu->i2cHandle, ACC_ADDR,
	data, 1, 1000);
  return status;
}

uint8_t BMI088_ReadGyrRegister (BMI088 *imu, uint8_t regAddr, uint8_t *data)
{
  uint8_t status = 0;
  status |= HAL_I2C_Master_Transmit (
	imu->i2cHandle, GYR_ADDR,
	&regAddr, 1, 1000);
  status |= HAL_I2C_Master_Receive (
	imu->i2cHandle, GYR_ADDR,
	data, 1, 1000);
  return status;
}

uint8_t BMI088_WriteAccRegister (BMI088 *imu, uint8_t regAddr, uint8_t data)
{
  uint8_t status = 0;
  txBuffer[0] = regAddr;
  txBuffer[1] = data;
  status |= HAL_I2C_Master_Transmit (
	imu->i2cHandle, ACC_ADDR,
	txBuffer, 2, 1000);
  return status;

}

uint8_t BMI088_WriteGyrRegister (BMI088 *imu, uint8_t regAddr, uint8_t data)
{
  uint8_t status = 0;
  txBuffer[0] = regAddr;
  txBuffer[1] = data;
  status |= HAL_I2C_Master_Transmit (
	imu->i2cHandle, GYR_ADDR,
	txBuffer, 2, 1000);
  return status;
}


/*
 * convert received bytes to acc measurements
 */
void BMI088_ConvertAccData( BMI088 *imu )
{
  /* Form signed 16-bit integers */
  int16_t accX = (int16_t) ((imu->accRxBuf[1] << 8) | imu->accRxBuf[0]);
  int16_t accY = (int16_t) ((imu->accRxBuf[3] << 8) | imu->accRxBuf[2]);
  int16_t accZ = (int16_t) ((imu->accRxBuf[5] << 8) | imu->accRxBuf[4]);

  /* Convert to m/s^2 */
  imu->acc_mps2[0] = imu->accConversion * accX;
  imu->acc_mps2[1] = imu->accConversion * accY;
  imu->acc_mps2[2] = imu->accConversion * accZ;
}

void BMI088_ConvertGyrData( BMI088 *imu )
{
  /* Form signed 16-bit integers */
  int16_t gyrX = (int16_t) ((imu->gyrRxBuf[1] << 8) | imu->gyrRxBuf[0]);
  int16_t gyrY = (int16_t) ((imu->gyrRxBuf[3] << 8) | imu->gyrRxBuf[2]);
  int16_t gyrZ = (int16_t) ((imu->gyrRxBuf[5] << 8) | imu->gyrRxBuf[4]);

  /* Convert to rad/s */
  imu->gyr_rps[0] = imu->gyrConversion * gyrX;
  imu->gyr_rps[1] = imu->gyrConversion * gyrY;
  imu->gyr_rps[2] = imu->gyrConversion * gyrZ;
}

/*
 *
 * POLLING
 *
 */
uint8_t BMI088_ReadAccelerometer (BMI088 *imu)
{
//  uint8_t regAddr = BMI_ACC_DATA;
  uint8_t status = 0;

//  status |= HAL_I2C_Master_Transmit (
//	imu->i2cHandle, ACC_ADDR,
//	&regAddr, 1, 1000);
//  status |= HAL_I2C_Master_Receive (
//	imu->i2cHandle, ACC_ADDR,
//	imu->accRxBuf, 6, 1000);

  status |= HAL_I2C_Mem_Read(
     imu->i2cHandle,
     ACC_ADDR,
     BMI_ACC_DATA, 1,
     imu->accRxBuf, 6,
     1000);

  BMI088_ConvertAccData(imu);

  return status;

}

uint8_t BMI088_ReadGyroscope (BMI088 *imu)
{
  uint8_t regAddr = BMI_GYR_DATA;
  uint8_t status = 0;
  status |= HAL_I2C_Master_Transmit (
	imu->i2cHandle, GYR_ADDR,
	&regAddr, 1, 1000);
  status |= HAL_I2C_Master_Receive (
	imu->i2cHandle, GYR_ADDR,
	imu->gyrRxBuf, 6, 1000);

  BMI088_ConvertGyrData(imu);

  return status;

}

// not converted to iic
#if 0
/*
 *
 * DMA
 *
 */
uint8_t
BMI088_ReadAccelerometerDMA (BMI088 *imu)
{

  HAL_GPIO_WritePin (imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive_DMA (imu->spiHandle, imu->accTxBuf,
				   (uint8_t*) imu->accRxBuf, 8) == HAL_OK)
  {

    imu->readingAcc = 1;
    return 1;

  }
  else
  {

    HAL_GPIO_WritePin (imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
    return 0;

  }

}

void
BMI088_ReadAccelerometerDMA_Complete (BMI088 *imu)
{

  HAL_GPIO_WritePin (imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
  imu->readingAcc = 0;

  /* Form signed 16-bit integers */
  int16_t accX = (int16_t) ((imu->accRxBuf[3] << 8) | imu->accRxBuf[2]);
  int16_t accY = (int16_t) ((imu->accRxBuf[5] << 8) | imu->accRxBuf[4]);
  int16_t accZ = (int16_t) ((imu->accRxBuf[7] << 8) | imu->accRxBuf[6]);

  /* Convert to m/s^2 */
  imu->acc_mps2[0] = imu->accConversion * accX;
  imu->acc_mps2[1] = imu->accConversion * accY;
  imu->acc_mps2[2] = imu->accConversion * accZ;

}

uint8_t
BMI088_ReadGyroscopeDMA (BMI088 *imu)
{

  HAL_GPIO_WritePin (imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive_DMA (imu->spiHandle, imu->gyrTxBuf,
				   (uint8_t*) imu->gyrRxBuf, 7) == HAL_OK)
  {

    imu->readingGyr = 1;
    return 1;

  }
  else
  {

    HAL_GPIO_WritePin (imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);
    return 0;

  }

}

void
BMI088_ReadGyroscopeDMA_Complete (BMI088 *imu)
{

  HAL_GPIO_WritePin (imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);
  imu->readingGyr = 0;

  /* Form signed 16-bit integers */
  int16_t gyrX = (int16_t) ((imu->gyrRxBuf[2] << 8) | imu->gyrRxBuf[1]);
  int16_t gyrY = (int16_t) ((imu->gyrRxBuf[4] << 8) | imu->gyrRxBuf[3]);
  int16_t gyrZ = (int16_t) ((imu->gyrRxBuf[6] << 8) | imu->gyrRxBuf[5]);

  /* Convert to deg/s */
  imu->gyr_rps[0] = imu->gyrConversion * gyrX;
  imu->gyr_rps[1] = imu->gyrConversion * gyrY;
  imu->gyr_rps[2] = imu->gyrConversion * gyrZ;

}
#endif
