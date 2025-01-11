/*
 * mpu9250.h
 *
 *  Created on: Dec 26, 2021
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit
 */

#ifndef INC_9255_H_
#define INC_9255_H_

#endif /* INC_9255_H_ */

#include <stdint.h>
#include "spi.h"
#include "mpu9250_defs.h"


// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

enum Mscale {
  MFS_14BITS, // 0.6 mG per LSB. 참고로 mpu9250은 14bit 밖에 없음.
  MFS_16BITS      // 0.15 mG per LSB
};

typedef enum{
  MADWICK,
  comFilter
}FusionMethod;


typedef struct
{
    float AccelX;
    float AccelY;
    float AccelZ;

    float GyroX;
    float GyroY;
    float GyroZ;

    float MagX;
    float MagY;
    float MagZ;

    float pitch;
    float roll;
    float yaw;

    struct SensorData
    {
        float aScaleFactor, gScaleFactor;
    } sensorData;

    struct Settings
    {
      uint8_t aFullScaleRange, gFullScaleRange;
      GPIO_TypeDef *CS_PORT;
      uint16_t CS_PIN;
    } settings;

} MPU9250_t;

uint8_t MPU9250_Init(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);

void readAll(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);

void getMres();
void getGres();
void getAres();

void readAccelData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, int16_t * destination);
void readGyroData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, int16_t * destination);
void readMagData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, int16_t * destination);

void initAK8963(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, float * destination);
void initMPU9250(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);

void calibrateMPU9250(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, float * dest1, float * dest2);
void calibrateMag(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, float * dest1, float * dest2);

void MPU9250SelfTest(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, float * destination);

void QuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void MPU_REG_READ(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t addr, uint8_t *pRxData, uint16_t RxSize);
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t pAddr, uint8_t *pVal);
void MPU_writeGyroFullScaleRange(SPI_HandleTypeDef *SPIx,  MPU9250_t *pMPU9250, uint8_t gScale);
void MPU_writeAccFullScaleRange(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t aScale);
void MPU_CS(MPU9250_t *pMPU9250, uint8_t state);

void AK8963_Write(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t reg, uint8_t data);
void AK8963_Read(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t reg, uint8_t *data, uint8_t RxSize);

void replaceBlockAK(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t address, uint8_t block, uint8_t at, uint8_t sz);

/* filter select*/
void update(FusionMethod method, MPU9250_t *pMPU9250);
void comFilter_func(MPU9250_t *pMPU9250, float dt);
