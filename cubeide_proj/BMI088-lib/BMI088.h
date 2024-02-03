#ifndef BMI088_IMU_H
#define BMI088_IMU_H

//#include "stm32f4xx_hal.h"

// i2c addresses
#define ACC_ADDR	0b00110000
#define GYR_ADDR 	0b11010000

/* Register defines */
#define BMI_ACC_CHIP_ID 		0x00
#define BMI_ACC_STATUS			0x03
#define BMI_ACC_DATA 			0x12
#define BMI_ACC_INT_STAT_1		0x1D
#define BMI_TEMP_DATA 			0x22
#define BMI_ACC_CONF 			0x40
#define BMI_ACC_RANGE 			0x41
#define BMI_INT1_IO_CONF 	   	0x53
#define BMI_INT1_INT2_MAP_DATA		0x58
#define BMI_ACC_PWR_CONF 		0x7C
#define BMI_ACC_PWR_CTRL 		0x7D
#define BMI_ACC_SOFTRESET 		0x7E

#define BMI_GYR_CHIP_ID			0x00
#define BMI_GYR_DATA			0x02
#define	BMI_GYR_RANGE			0x0F
#define	BMI_GYR_BANDWIDTH		0x10
#define	BMI_GYR_SOFTRESET		0x14
#define	BMI_GYR_INT_CTRL		0x15
#define	BMI_INT3_INT4_IO_CONF	0x16
#define BMI_INT3_INT4_IO_MAP	0x18


typedef enum
{
  MEASURE_MODE_ACC, MEASURE_MODE_GYR
} measureMode_e;

typedef struct
{
  I2C_HandleTypeDef *i2cHandle;

  /* DMA */
  uint8_t readingAcc;
  uint8_t readingGyr;
  uint8_t accTxBuf[8];
  uint8_t gyrTxBuf[7];
  uint8_t accRxBuf[8];
  uint8_t gyrRxBuf[7];

  /* Conversion constants (raw to m/s^2 and raw to rad/s) */
  float accConversion;
  float gyrConversion;

  /* x-y-z measurements */
  float acc_mps2[3];
  float gyr_rps[3];

} BMI088;

/*
 *
 * INITIALISATION
 *
 */
uint8_t
BMI088_Init (BMI088 *imu, I2C_HandleTypeDef *i2cHandle );

/*
 * get the state of the acceleromter interrupt, via i2c
 */
uint8_t BMI088_ReadAccIntr(BMI088 *imu);

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */
uint8_t
BMI088_ReadAccRegister (BMI088 *imu, uint8_t regAddr, uint8_t *data);
uint8_t
BMI088_ReadGyrRegister (BMI088 *imu, uint8_t regAddr, uint8_t *data);

uint8_t
BMI088_WriteAccRegister (BMI088 *imu, uint8_t regAddr, uint8_t data);
uint8_t
BMI088_WriteGyrRegister (BMI088 *imu, uint8_t regAddr, uint8_t data);


void BMI088_ConvertAccData( BMI088 *imu );
void BMI088_ConvertGyrData( BMI088 *imu );

/*
 *
 * POLLING
 *
 */
uint8_t
BMI088_ReadAccelerometer (BMI088 *imu);
uint8_t
BMI088_ReadGyroscope (BMI088 *imu);

/*
 *
 * DMA
 *
 */
uint8_t
BMI088_ReadAccelerometerDMA (BMI088 *imu);
void
BMI088_ReadAccelerometerDMA_Complete (BMI088 *imu);

uint8_t
BMI088_ReadGyroscopeDMA (BMI088 *imu);
void
BMI088_ReadGyroscopeDMA_Complete (BMI088 *imu);

#endif
