/*
 * mpu9250.c
 *
 *  Created on: Dec 26, 2021
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit   
 */

//Includes
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "tim.h"
#include "mpu9250.h"

const uint16_t SPI_TIMOUT_MS = 100;


//Specify sensor full scale
uint8_t Gscale = GFSR_2000DPS;
uint8_t Ascale = AFSR_16G;
uint8_t Mscale = MFS_14BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read



float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}; // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};      // Bias corrections for gyro and accelerometer
float   SelfTest[6];    // holds results of gyro and accelerometer self test

float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

//float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
//float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

float beta_mad = 0.604599;   //beta value for madgwick
float zeta = 0.0;

#define DT 0.01f
#define fc 0.5f
#define Tc (1./(2.*M_PI*fc))
#define ALPHA (Tc/(Tc+DT))
#define BETA (1-ALPHA)

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components

//===================================================================================================================
//====== MAIN INIT FUNCTION
//===================================================================================================================


uint8_t MPU9250_Init(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250){


	//pre-def. vars
	uint8_t readData;

	printf("**************************** \r\n");
	printf("MPU9250 STM32 Implementation \r\n");
	printf("**************************** \r\n");


	MPU_writeAccFullScaleRange(SPIx, pMPU9250, pMPU9250->settings.aFullScaleRange);
	MPU_writeGyroFullScaleRange(SPIx, pMPU9250, pMPU9250->settings.gFullScaleRange);


	//read MPU9250 WHOAMI
	MPU_REG_READ(SPIx, pMPU9250, WHO_AM_I_MPU9250, &readData, 1);

	if (SerialDebugA) {printf("MPU -WHO AM I- is: (Must return 113) %d\r\n", readData);}

	if (readData == 113) {

		//Start by performing self test and reporting values
		MPU9250SelfTest(SPIx, pMPU9250, SelfTest);

		//Calibrate gyro and accelerometers, load biases in bias registers
		calibrateMPU9250(SPIx, pMPU9250, gyroBias, accelBias);
		HAL_Delay(1000);

		//init Gyro and Accelerometer and start using internal clock
		initMPU9250(SPIx, pMPU9250);



		//Read the WHO_AM_I register of the magnetometer
		AK8963_Read(SPIx, pMPU9250, AK8963_WHO_AM_I, &readData, 1); // Read WHO_AM_I register for AK8963


		if (SerialDebugA) {printf("MAG -WHO AM I- is: (Must return 72) %d\r\n", readData);}
		HAL_Delay(1000);

		//Get magnetometer calibration from AK8963 ROM
		initAK8963(SPIx, pMPU9250, magCalibration);  // Initialize device for active mode read of magnetometer

		calibrateMag(SPIx, pMPU9250, magBias, magScale);

		HAL_Delay(1000);
		return 0;
	}
	return 1; // Loop forever if communication doesn't happen
}

void readAll(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250) {
	uint8_t Data;
	float pitch=0.0f, roll=0.0f, yaw=0.0f;

	// If intPin goes high, all data registers have new data
	MPU_REG_READ(SPIx, pMPU9250, INT_STATUS, &Data, 1);

	if (Data & 0x01) {  // On interrupt, check if data ready interrupt
		readAccelData(SPIx, pMPU9250, accelCount);  // Read the x/y/z adc values

		getAres();

		// Now we'll calculate the accleration value into actual g's
		ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
		ay = (float)accelCount[1]*aRes; // - accelBias[1];
		az = (float)accelCount[2]*aRes; // - accelBias[2];

		pMPU9250->AccelX = ax;
		pMPU9250->AccelY = ay;
		pMPU9250->AccelZ = az;

		readGyroData(SPIx, pMPU9250, gyroCount);  // Read the x/y/z adc values
		getGres();

		// Calculate the gyro value into actual degrees per second
		gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
		gy = (float)gyroCount[1]*gRes;
		gz = (float)gyroCount[2]*gRes;

		pMPU9250->GyroX = gx;
		pMPU9250->GyroY = gy;
		pMPU9250->GyroZ = gz;

		readMagData(SPIx, pMPU9250, magCount);  // Read the x/y/z adc values

//		if(Mmode == 0x02) HAL_Delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
//		if(Mmode == 0x06) HAL_Delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
// instead of add delay on here, add HAL_Delay in AK8963_Read function.

		getMres();

		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
		my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
		mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];
		mx *= magScale[0];
		my *= magScale[1];
		mz *= magScale[2];

		pMPU9250->MagX = mx;
		pMPU9250->MagY = my;
		pMPU9250->MagZ = mz;
	}

	//for middle test
	//printf("a_xyz : %f %f %f g_xyz : %f %f %f \n", ax, ay, az, gx, gy, gz);

	//Now = __HAL_TIM_GET_COUNTER(&htim1);
	Now = HAL_GetTick();
	deltat = ((Now - lastUpdate)/1000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;
	sum += deltat; // sum for averaging filter update rate

	// Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
	// the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro.

	// Calculate quaternions based on Madgwick's filter
	//Since MPU9250's mag. and IMU modules are different and seperate (AK8963 and MPU6050), their...
	//...coordinate systems also different. So, to compensate this, order should be my - mx - mz
	//QuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);
	QuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

	// Convert quaternions to Euler angles
	a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
	a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
	a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
	a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	pitch = -asinf(a32);
	roll  = atan2f(a31, a33);
	yaw   = atan2f(a12, a22);
	pitch *= 180.0f / PI;
	yaw   *= 180.0f / PI;
	yaw   += 5.53f; // Declination

	if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
	roll  *= 180.0f / PI;
	lin_ax = ax + a31;
	lin_ay = ay + a32;
	lin_az = az - a33;


	pMPU9250->yaw = yaw;
	pMPU9250->pitch = pitch;
	pMPU9250->roll = roll;



	sum = 0;
}

//==========================================================================================================
//====== FUNCTIONS TO READ AND WRITE DATA FROM REGISTERS AND ALSO INITS KALMAN AND QUATERNION FILTERS ======
//==========================================================================================================

void getMres() {
  switch (Mscale)
  {
  // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFSR_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFSR_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFSR_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFSR_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFSR_2G:
          aRes = 2.0/32768.0;
          break;
    case AFSR_4G:
          aRes = 4.0/32768.0;
          break;
    case AFSR_8G:
          aRes = 8.0/32768.0;
          break;
    case AFSR_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

//read raw Accelerometer values from registers
void readAccelData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, int16_t * destination){
  uint8_t rawAccelData[6];  // x/y/z accel register data stored here
  MPU_REG_READ(SPIx, pMPU9250, ACCEL_XOUT_H, &rawAccelData[0], 6); // Read the six raw data registers into data array

  destination[0] = ((int16_t)rawAccelData[0] << 8) | rawAccelData[1];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawAccelData[2] << 8) | rawAccelData[3];
  destination[2] = ((int16_t)rawAccelData[4] << 8) | rawAccelData[5];

  if(SerialDebugB){
	printf("Acc X: %d\r\n", destination[0]);
	printf("Acc Y: %d\r\n", destination[1]);
	printf("Acc Z: %d\r\n", destination[2]);
	printf("-------------------------\r\n");
  }
}

//read raw Gyro values from registers
void readGyroData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, int16_t * destination){
  uint8_t rawGyroData[6];  // x/y/z gyro register data stored here
  MPU_REG_READ(SPIx, pMPU9250, GYRO_XOUT_H, &rawGyroData[0], 6);  // Read the six raw data registers sequentially into data array

  destination[0] = ((int16_t)rawGyroData[0] << 8) | rawGyroData[1];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawGyroData[2] << 8) | rawGyroData[3];
  destination[2] = ((int16_t)rawGyroData[4] << 8) | rawGyroData[5];

  if(SerialDebugB){
	printf("Gyro X: %d\r\n", destination[0]);
	printf("Gyro Y: %d\r\n", destination[1]);
	printf("Gyro Z: %d\r\n", destination[2]);
	printf("---------------------------\r\n");
  }
}

void readMagData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, int16_t * destination){

	uint8_t readData;

	AK8963_Read(SPIx, pMPU9250, AK8963_ST1, &readData, 1);

	if( (readData & 0x01) == 0x01 ){
		uint8_t rawMagData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
		AK8963_Read(SPIx, pMPU9250, AK8963_XOUT_L, rawMagData, 7);  // Read the six raw data and ST2 registers sequentially into data array

		uint8_t c = rawMagData[6];
		if(!(c & 0x08)) {
			destination[0] = ((int16_t)rawMagData[1] << 8) | rawMagData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)rawMagData[3] << 8) | rawMagData[2] ;  // Data stored as little Endian
			destination[2] = ((int16_t)rawMagData[5] << 8) | rawMagData[4] ;

			AK8963_Read(SPIx, pMPU9250, AK8963_ST2, &readData, 1);    //It is essential for checking that reading is done !!!!

			if(SerialDebugB){
				printf("Mag X: %d\r\n", destination[0]);
				printf("Mag Y: %d\r\n", destination[1]);
				printf("Mag Z: %d\r\n", destination[2]);
				printf("-------------------------\r\n");
			}
		}
	}
}

void initAK8963(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, float * destination){
  //pre def. vars
  uint8_t writeData;
  //uint8_t readData_temp = 0; for debug

  //First extract the factory calibration for each magnetometer axis
  // x/y/z gyro calibration data stored here
  uint8_t rawMagCalData[3];

  //Power down magnetometer
  writeData = 0x00;
  AK8963_Write(SPIx, pMPU9250, AK8963_CNTL, writeData);
  //AK8963_Read(SPIx, pMPU9250, AK8963_CNTL, &readData_temp, 1); for debug

  HAL_Delay(100);

  writeData = 0x0F;
  AK8963_Write(SPIx, pMPU9250, AK8963_CNTL, writeData); // Enter Fuse ROM access mode
  //AK8963_Read(SPIx, pMPU9250, AK8963_CNTL, &readData_temp, 1); for debug

  HAL_Delay(100);


  AK8963_Read(SPIx, pMPU9250, AK8963_ASAX, rawMagCalData, 3);// Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawMagCalData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawMagCalData[1] - 128)/256. + 1.;
  destination[2] =  (float)(rawMagCalData[2] - 128)/256. + 1.;

  if(SerialDebugA){
	printf("Mag cal off X: %f\r\n", destination[0]);
	printf("Mag cal off Y: %f\r\n", destination[1]);
	printf("Mag cal off Z: %f\r\n", destination[2]);
	printf("-------------------------\r\n");
  }

  writeData = 0x00;
  AK8963_Write(SPIx, pMPU9250, AK8963_CNTL, writeData);// Power down magnetometer
  HAL_Delay(100);

  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeData = Mscale << 4 | Mmode;
  AK8963_Write(SPIx, pMPU9250, AK8963_CNTL, writeData); // Set magnetometer data resolution and sample ODR


  //writeData = 0x16;
  //HAL_I2C_Mem_Write(I2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeout);
  HAL_Delay(10);

  if(SerialDebugA)
  {
    printf("MAG Init Succesful! \r\n");
  }
}

void calibrateMag(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, float * dest1, float * dest2){

  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

//  uint8_t temp_cntl;  //for checking that AK8963 is read normally
//  uint8_t readData;

  if (SerialDebugA){printf("Mag Calibration: Wave device in a figure eight until done!\r\n");}
  HAL_Delay(4000);

    // shoot for ~fifteen seconds of mag data
    if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
   for(ii = 0; ii < sample_count; ii++) {
    readMagData(SPIx, pMPU9250, mag_temp);  // Read the mag data

//    printf("mx : %d my : %d mz : %d \n", mag_temp[0], mag_temp[1], mag_temp[2]);   //for checking that AK8963 is read normally
//    AK8963_Read(SPIx, pMPU9250, AK8963_CNTL, &temp_cntl, 1);
//    printf("check the value of AK8963_CNTL : %d\n", temp_cntl);
//
//    AK8963_Read(SPIx, pMPU9250, AK8963_ST1, &readData, 1);
//    printf("check the value of AK8963_ST1 : %d\n", readData);


    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if(Mmode == 0x02) HAL_Delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) HAL_Delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms


    }





    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    getMres();

    dest1[0] = 0;
    dest1[1] = 0;
    dest1[2] = 0;

    dest1[0] = (float)mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float)mag_bias[1]*mRes*magCalibration[1];
    dest1[2] = (float)mag_bias[2]*mRes*magCalibration[2];

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = (float)(mag_scale[0] + mag_scale[1] + mag_scale[2]);
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

    if (SerialDebugA){printf("Mag Calibration done!\r\n");}

}


void initMPU9250(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250){
	//pre def. vars
	uint8_t readData;
	uint8_t writeData;

	//Wake up device & start using PLL
	writeData = 0x00;
	MPU_REG_WRITE(SPIx, pMPU9250, PWR_MGMT_1, &writeData);
	HAL_Delay(100);

	writeData = 0x01;
	MPU_REG_WRITE(SPIx, pMPU9250, PWR_MGMT_1, &writeData);
	HAL_Delay(100);

	writeData = 0x20;
	MPU_REG_WRITE(SPIx, pMPU9250, USER_CTRL, &writeData);
	HAL_Delay(100);

  writeData = 0x0D;
  MPU_REG_WRITE(SPIx, pMPU9250, I2C_MST_CTRL, &writeData);
  HAL_Delay(100);

  replaceBlockAK(SPIx, pMPU9250, AK8963_CNTL, 0, 0, 4);
  HAL_Delay(100);

  writeData = 0x80;
  MPU_REG_WRITE(SPIx, pMPU9250, PWR_MGMT_1, &writeData);
  HAL_Delay(100);

  writeData = 0x01;
  MPU_REG_WRITE(SPIx, pMPU9250, PWR_MGMT_1, &writeData);
  HAL_Delay(100);

	writeData = 0x03;
	MPU_REG_WRITE(SPIx, pMPU9250, CONFIG, &writeData);   // need adjustment
	HAL_Delay(100);

	writeData = 0x04;
	MPU_REG_WRITE(SPIx, pMPU9250, SMPLRT_DIV, &writeData); // need adjustment
	HAL_Delay(100);

	writeData = 0x20;
	MPU_REG_WRITE(SPIx, pMPU9250, INT_PIN_CFG, &writeData); // Latch_int_en
	HAL_Delay(100);

  writeData = 0x01;
  MPU_REG_WRITE(SPIx, pMPU9250, INT_ENABLE, &writeData); // RAW_RDY_EN
  HAL_Delay(100);

  writeData = 0x20;
  MPU_REG_WRITE(SPIx, pMPU9250, USER_CTRL, &writeData);
  HAL_Delay(100);

  writeData = 0x0D;
  MPU_REG_WRITE(SPIx, pMPU9250, I2C_MST_CTRL, &writeData);
  HAL_Delay(100);

  // This is where the gyro and accel configuration starts.

	MPU_REG_READ(SPIx, pMPU9250, GYRO_CONFIG, &readData, 1);
	readData = readData & ~0x03; // Clear Fchoice bits [1:0]
	readData = readData & ~0x18; // Clear GFS bits [4:3]
	readData = readData | Gscale << 3; // Set full scale range for the gyro
	HAL_Delay(100);

	writeData = readData;
	MPU_REG_WRITE(SPIx, pMPU9250, GYRO_CONFIG, &writeData);
	HAL_Delay(100);

	MPU_REG_READ(SPIx, pMPU9250, ACCEL_CONFIG, &readData, 1);
	readData = readData & ~0x18;  // Clear AFS bits [4:3]
	readData = readData | Ascale << 3; // Set full scale range for the accelerometer

	writeData = readData;
	MPU_REG_WRITE(SPIx, pMPU9250, ACCEL_CONFIG, &writeData);

	HAL_Delay(100);
	//**
	MPU_REG_READ(SPIx, pMPU9250, ACCEL_CONFIG2, &readData, 1);

	readData = readData & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	readData = readData | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	writeData = readData;
	MPU_REG_WRITE(SPIx, pMPU9250, ACCEL_CONFIG2, &writeData);
	HAL_Delay(100);



	if(SerialDebugA){printf("MPU Init Succesful! \r\n");}
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, float * dest1, float * dest2){
  //pre def. vars
  uint8_t writeData;

	uint8_t calibData[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device
	writeData = 0x80;
	MPU_REG_WRITE(SPIx, pMPU9250, PWR_MGMT_1, &writeData);// Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeData = 0x01;
  MPU_REG_WRITE(SPIx, pMPU9250, PWR_MGMT_1, &writeData);
	writeData = 0x00;
	MPU_REG_WRITE(SPIx, pMPU9250, PWR_MGMT_2, &writeData);
	HAL_Delay(200);

	// Configure device for bias calculation
	writeData = 0x00;
	MPU_REG_WRITE(SPIx, pMPU9250, INT_ENABLE, &writeData);// Disable all interrupts

	writeData = 0x00;
	MPU_REG_WRITE(SPIx, pMPU9250, FIFO_EN, &writeData);// Disable FIFO

	writeData = 0x01;
	MPU_REG_WRITE(SPIx, pMPU9250, PWR_MGMT_1, &writeData);// Turn on clock 1 Auto selects the best available clock source – PLL if ready, else
	//use the Internal oscillator

//	writeData = 0x00;
//	MPU_REG_WRITE(SPIx, pMPU9250, I2C_MST_CTRL, &writeData);// Disable I2C master


	writeData = 0x20;
	MPU_REG_WRITE(SPIx, pMPU9250, USER_CTRL, &writeData); // Disable FIFO

	writeData = 0x0C;
	MPU_REG_WRITE(SPIx, pMPU9250, USER_CTRL, &writeData);// Reset FIFO and DMP

	HAL_Delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeData = 0x01;
	MPU_REG_WRITE(SPIx, pMPU9250, CONFIG, &writeData);// Set low-pass filter to 188 Hz

	writeData = 0x00;
	MPU_REG_WRITE(SPIx, pMPU9250, SMPLRT_DIV, &writeData);// Set sample rate to 1 kHz

	writeData = 0x00;
	MPU_REG_WRITE(SPIx, pMPU9250, GYRO_CONFIG, &writeData);// Set gyro full-scale to 250 degrees per second, maximum sensitivity

	writeData = 0x00;
	MPU_REG_WRITE(SPIx, pMPU9250, ACCEL_CONFIG, &writeData);// Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeData = 0x40;
	MPU_REG_WRITE(SPIx, pMPU9250, USER_CTRL, &writeData);// Enable FIFO

	writeData = 0x78;
	MPU_REG_WRITE(SPIx, pMPU9250, FIFO_EN, &writeData);// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)

	HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeData = 0x00;
	MPU_REG_WRITE(SPIx, pMPU9250, FIFO_EN, &writeData);// Disable gyro and accelerometer sensors for FIFO

	MPU_REG_READ(SPIx, pMPU9250, FIFO_COUNTH, &calibData[0], 2);// read FIFO sample count

	fifo_count = ((uint16_t)calibData[0] << 8) | calibData[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		MPU_REG_READ(SPIx, pMPU9250, FIFO_R_W, &calibData[0], 12);

		//Form signed 16-bit integer for each sample in FIFO
		accel_temp[0] = (int16_t) (((int16_t)calibData[0] << 8) | calibData[1]  ) ;
		accel_temp[1] = (int16_t) (((int16_t)calibData[2] << 8) | calibData[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)calibData[4] << 8) | calibData[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)calibData[6] << 8) | calibData[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)calibData[8] << 8) | calibData[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)calibData[10] << 8) | calibData[11]) ;

		//Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[0] += (int32_t) accel_temp[0];
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	//Normalize sums to get average count biases
	accel_bias[0] /= (int32_t) packet_count;
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	//Remove gravity from the z-axis accelerometer bias calculation
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	//Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	calibData[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	calibData[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	calibData[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	calibData[3] = (-gyro_bias[1]/4)       & 0xFF;
	calibData[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	calibData[5] = (-gyro_bias[2]/4)       & 0xFF;

	//Push gyro biases to hardware registers
	writeData = calibData[0];
	MPU_REG_WRITE(SPIx, pMPU9250, XG_OFFSET_H, &writeData);

	writeData = calibData[1];
	MPU_REG_WRITE(SPIx, pMPU9250, XG_OFFSET_L, &writeData);

	writeData = calibData[2];
	MPU_REG_WRITE(SPIx, pMPU9250, YG_OFFSET_H, &writeData);

	writeData = calibData[3];
	MPU_REG_WRITE(SPIx, pMPU9250, YG_OFFSET_L, &writeData);

	writeData = calibData[4];
	MPU_REG_WRITE(SPIx, pMPU9250, ZG_OFFSET_H, &writeData);

	writeData = calibData[5];
	MPU_REG_WRITE(SPIx, pMPU9250, ZG_OFFSET_L, &writeData);

	//Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	if(SerialDebugA){
		float gyroBiasX = (float) gyro_bias[0]/(float) gyrosensitivity;
		float gyroBiasY = (float) gyro_bias[1]/(float) gyrosensitivity;
		float gyroBiasZ = (float) gyro_bias[2]/(float) gyrosensitivity;

		printf("Gyro bias X: %f\r\n", gyroBiasX);
		printf("Gyro bias Y: %f\r\n", gyroBiasY);
		printf("Gyro bias Z: %f\r\n", gyroBiasZ);

		printf("-------------------------\r\n");
	}

	//Construct the accelerometer biases for push to the hardware accelerometer bias registers.
	int32_t accel_bias_reg[3] = {0, 0, 0}; //A place to hold the factory accelerometer trim biases
	MPU_REG_READ(SPIx, pMPU9250, XA_OFFSET_H, &calibData[0], 2); //Read factory accelerometer trim values

	accel_bias_reg[0] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);
	MPU_REG_READ(SPIx, pMPU9250, YA_OFFSET_H, &calibData[0], 2);

	accel_bias_reg[1] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);
	MPU_REG_READ(SPIx, pMPU9250, ZA_OFFSET_H, &calibData[0], 2);

	accel_bias_reg[2] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);

	//Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint32_t mask = 1uL;
	//Define array to hold mask bit for each accelerometer bias axis
	uint8_t mask_bit[3] = {0, 0, 0};

	for(ii = 0; ii < 3; ii++) {
		//If temperature compensation bit is set, record that fact in mask_bit
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01;
	}

	//Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); //Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	calibData[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	calibData[1] = (accel_bias_reg[0])      & 0xFF;
	calibData[1] = calibData[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	calibData[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	calibData[3] = (accel_bias_reg[1])      & 0xFF;
	calibData[3] = calibData[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	calibData[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	calibData[5] = (accel_bias_reg[2])      & 0xFF;
	calibData[5] = calibData[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	//Push accelerometer biases to hardware registers
	writeData = calibData[0];
	MPU_REG_WRITE(SPIx, pMPU9250, XA_OFFSET_H, &writeData);

	writeData = calibData[1];
	MPU_REG_WRITE(SPIx, pMPU9250, XA_OFFSET_L, &writeData);

	writeData = calibData[2];
	MPU_REG_WRITE(SPIx, pMPU9250, YA_OFFSET_H, &writeData);

	writeData = calibData[3];
	MPU_REG_WRITE(SPIx, pMPU9250, YA_OFFSET_L, &writeData);

	writeData = calibData[4];
	MPU_REG_WRITE(SPIx, pMPU9250, ZA_OFFSET_H, &writeData);

	writeData = calibData[5];
	MPU_REG_WRITE(SPIx, pMPU9250, ZA_OFFSET_L, &writeData);

	//Output scaled gyro biases for display in the main program
	dest2[0] = (float) accel_bias[0]/(float) accelsensitivity;
	dest2[1] = (float) accel_bias[1]/(float) accelsensitivity;
	dest2[2] = (float) accel_bias[2]/(float) accelsensitivity;

	if(SerialDebugA){
		float accelBiasX = (float) accel_bias[0]/(float) accelsensitivity;
		float accelBiasY = (float) accel_bias[1]/(float) accelsensitivity;
		float accelBiasZ = (float) accel_bias[2]/(float) accelsensitivity;

		printf("Accel bias X: %f\r\n", accelBiasX);
		printf("Accel bias Y: %f\r\n", accelBiasY);
		printf("Accel bias Z: %f\r\n", accelBiasZ);

		printf("-------------------------\r\n");
	}
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, float * destination) {
	uint8_t writeData;

	uint8_t rawTestData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;

	writeData = 0x00;
	MPU_REG_WRITE(SPIx, pMPU9250, SMPLRT_DIV, &writeData); // Set gyro sample rate to 1 kHz
	writeData = 0x02;
	MPU_REG_WRITE(SPIx, pMPU9250, CONFIG, &writeData); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeData = FS<<3;
	MPU_REG_WRITE(SPIx, pMPU9250, GYRO_CONFIG, &writeData); // Set full scale range for the gyro to 250 dps
	writeData = 0x02;
	MPU_REG_WRITE(SPIx, pMPU9250, ACCEL_CONFIG2, &writeData); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeData = FS<<3;
	MPU_REG_WRITE(SPIx, pMPU9250, ACCEL_CONFIG, &writeData);// Set full scale range for the accelerometer to 2 g

	//get average current values of gyro and acclerometer
	for( int ii = 0; ii < 200; ii++) {

	  MPU_REG_READ(SPIx, pMPU9250, ACCEL_XOUT_H, rawTestData, 6);// Read the six raw data registers into data array


		aAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;

		MPU_REG_READ(SPIx, pMPU9250, GYRO_XOUT_H, &rawTestData[0], 6); // Read the six raw data registers sequentially into data array

		gAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;
	}

	//Get average of 200 values and store as average current readings
	for (int ii =0; ii < 3; ii++) {
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	//Configure the accelerometer for self-test
	writeData = 0xE0;
	MPU_REG_WRITE(SPIx, pMPU9250, ACCEL_CONFIG, &writeData);// Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeData = 0xE0;
	MPU_REG_WRITE(SPIx, pMPU9250, GYRO_CONFIG, &writeData);// Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	HAL_Delay(25);  // Delay a while to let the device stabilize

	//get average self-test values of gyro and acclerometer
	for( int ii = 0; ii < 200; ii++) {

	  MPU_REG_READ(SPIx, pMPU9250, ACCEL_XOUT_H, &rawTestData[0], 6);// Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;

		MPU_REG_READ(SPIx, pMPU9250, GYRO_XOUT_H, &rawTestData[0], 6);// Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;
	}

	//Get average of 200 values and store as average self-test readings
	for (int ii =0; ii < 3; ii++) {
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	//Configure the gyro and accelerometer for normal operation
	writeData = 0x11 << 3;
	MPU_REG_WRITE(SPIx, pMPU9250, ACCEL_CONFIG, &writeData);
	writeData = 0x11 << 3;
	MPU_REG_WRITE(SPIx, pMPU9250, GYRO_CONFIG, &writeData);
	HAL_Delay(25);  // Delay a while to let the device stabilize

	//Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  MPU_REG_READ(SPIx, pMPU9250, SELF_TEST_X_ACCEL, &selfTest[0], 1);// X-axis accel self-test results
  MPU_REG_READ(SPIx, pMPU9250, SELF_TEST_Y_ACCEL, &selfTest[1], 1);// Y-axis accel self-test results
  MPU_REG_READ(SPIx, pMPU9250, SELF_TEST_Z_ACCEL, &selfTest[2], 1);// Z-axis accel self-test results
  MPU_REG_READ(SPIx, pMPU9250, SELF_TEST_X_GYRO, &selfTest[3], 1);// X-axis gyro self-test results
  MPU_REG_READ(SPIx, pMPU9250, SELF_TEST_Y_GYRO, &selfTest[4], 1);// Y-axis gyro self-test results
  MPU_REG_READ(SPIx, pMPU9250, SELF_TEST_Z_GYRO, &selfTest[5], 1);// Z-axis gyro self-test results

	//Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	float testResults[6];

	//Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	//To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		testResults[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0;   // Report percent differences
		testResults[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0; // Report percent differences
	}

	if(SerialDebugA){
		float testResultAccelX = testResults[0];
		float testResultAccelY = testResults[1];
		float testResultAccelZ = testResults[2];
		float testResultGyroX = testResults[3];
		float testResultGyroY = testResults[4];
		float testResultGyroZ = testResults[5];

		printf("Accel Test X: %f\r\n", testResultAccelX);
		printf("Accel Test Y: %f\r\n", testResultAccelY);
		printf("Accel Test Z: %f\r\n", testResultAccelZ);
		printf("Gyro Test X: %f\r\n", testResultGyroX);
		printf("Gyro Test Y: %f\r\n", testResultGyroY);
		printf("Gyro Test Z: %f\r\n", testResultGyroZ);
		printf("-------------------------\r\n");
	}

   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
   }
}

void QuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){

  //madgwick
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta_mad * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta_mad * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta_mad * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta_mad * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;


    //no filter
//    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // variable for readability
//    q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * deltat;
//    q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * deltat;
//    q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * deltat;
//    q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * deltat;
//    float recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
//    q[0] *= recipNorm;
//    q[1] *= recipNorm;
//    q[2] *= recipNorm;
//    q[3] *= recipNorm;


    //mahony filter
//          float recipNorm;
//          float vx, vy, vz;
//          float ex, ey, ez;  //error terms
//          float qa, qb, qc;
//          static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
//          float tmp;
//
//          // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//          tmp = ax * ax + ay * ay + az * az;
//          if (tmp > 0.0) {
//              // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
//              recipNorm = 1.0 / sqrt(tmp);
//              ax *= recipNorm;
//              ay *= recipNorm;
//              az *= recipNorm;
//
//              // Estimated direction of gravity in the body frame (factor of two divided out)
//              vx = q[1] * q[3] - q[0] * q[2];
//              vy = q[0] * q[1] + q[2] * q[3];
//              vz = q[0] * q[0] - 0.5f + q[3] * q[3];
//
//              // Error is cross product between estimated and measured direction of gravity in body frame
//              // (half the actual magnitude)
//              ex = (ay * vz - az * vy);
//              ey = (az * vx - ax * vz);
//              ez = (ax * vy - ay * vx);
//
//              // Compute and apply to gyro term the integral feedback, if enabled
//              if (Ki > 0.0f) {
//                  ix += Ki * ex * deltat;  // integral error scaled by Ki
//                  iy += Ki * ey * deltat;
//                  iz += Ki * ez * deltat;
//                  gx += ix;  // apply integral feedback
//                  gy += iy;
//                  gz += iz;
//              }
//
//              // Apply proportional feedback to gyro term
//              gx += Kp * ex;
//              gy += Kp * ey;
//              gz += Kp * ez;
//          }
//
//          // Integrate rate of change of quaternion, q cross gyro term
//          deltat = 0.5 * deltat;
//          gx *= deltat;  // pre-multiply common factors
//          gy *= deltat;
//          gz *= deltat;
//          qa = q[0];
//          qb = q[1];
//          qc = q[2];
//          q[0] += (-qb * gx - qc * gy - q[3] * gz);
//          q[1] += (qa * gx + qc * gz - q[3] * gy);
//          q[2] += (qa * gy - qb * gz + q[3] * gx);
//          q[3] += (qa * gz + qb * gy - qc * gx);
//
//          // renormalise quaternion
//          recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
//          q[0] = q[0] * recipNorm;
//          q[1] = q[1] * recipNorm;
//          q[2] = q[2] * recipNorm;
//          q[3] = q[3] * recipNorm;
//
//



}

/// @brief Read a specific registry address
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param pAddr Pointer to address to be written to
/// @param pVal Pointer of value to write to given address
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t pAddr, uint8_t *pVal)
{
    MPU_CS(pMPU9250, CS_SELECT);
    HAL_SPI_Transmit(SPIx, (uint8_t *)&pAddr, 1, SPI_TIMOUT_MS);
    HAL_SPI_Transmit(SPIx, (uint8_t *)pVal, 1, SPI_TIMOUT_MS);
    MPU_CS(pMPU9250, CS_DESELECT);
}

/// @brief Read a specific registry address
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param addr Address to start reading at
/// @param pRxData Pointer to data buffer
/// @param RxSize Size of data buffer
void MPU_REG_READ(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t addr, uint8_t *pRxData, uint16_t RxSize)
{
    MPU_CS(pMPU9250, CS_SELECT);
    uint8_t writeAddr = addr | READWRITE;
    HAL_SPI_Transmit(SPIx, (uint8_t *)&writeAddr, 1, SPI_TIMOUT_MS);
    HAL_SPI_Receive(SPIx, pRxData, RxSize, SPI_TIMOUT_MS);
    //printf("*pRxData : %d\n", *pRxData);
    MPU_CS(pMPU9250, CS_DESELECT);
}

/// @brief Set CS state to either start or end transmissions
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param state Set low to select, high to deselect
void MPU_CS(MPU9250_t *pMPU9250, uint8_t state)
{
    HAL_GPIO_WritePin(pMPU9250->settings.CS_PORT, pMPU9250->settings.CS_PIN, state);
}

/// @brief Set the accelerometer full scale range
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g
void MPU_writeAccFullScaleRange(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t aScale)
{
    // Variable init
    uint8_t addr = ACCEL_CONFIG;
    uint8_t val;

    // Set the value
    switch (aScale)
    {
    case AFSR_2G:
        pMPU9250->sensorData.aScaleFactor = 16384.0;
        val = 0x00;
        MPU_REG_WRITE(SPIx, pMPU9250, addr, &val);
        break;
    case AFSR_4G:
        pMPU9250->sensorData.aScaleFactor = 8192.0;
        val = 0x08;
        MPU_REG_WRITE(SPIx, pMPU9250, addr, &val);
        break;
    case AFSR_8G:
        pMPU9250->sensorData.aScaleFactor = 4096.0;
        val = 0x10;
        MPU_REG_WRITE(SPIx, pMPU9250, addr, &val);
        break;
    case AFSR_16G:
        pMPU9250->sensorData.aScaleFactor = 2048.0;
        val = 0x18;
        MPU_REG_WRITE(SPIx, pMPU9250, addr, &val);
        break;
    default:
        pMPU9250->sensorData.aScaleFactor = 8192.0;
        val = 0x08;
        MPU_REG_WRITE(SPIx, pMPU9250, addr, &val);
        break;
    }
}

/// @brief Set the gyroscope full scale range
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s
void MPU_writeGyroFullScaleRange(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t gScale)
{
    // Variable init
    uint8_t addr = GYRO_CONFIG;
    uint8_t val;

    // Set the value
    switch (gScale)
    {
    case GFSR_250DPS:
        pMPU9250->sensorData.gScaleFactor = 131.0;
        val = 0x00;
        MPU_REG_WRITE(SPIx, pMPU9250, addr, &val);
        break;
    case GFSR_500DPS:
        pMPU9250->sensorData.gScaleFactor = 65.5;
        val = 0x08;
        MPU_REG_WRITE(SPIx, pMPU9250, addr, &val);
        break;
    case GFSR_1000DPS:
        pMPU9250->sensorData.gScaleFactor = 32.8;
        val = 0x10;
        MPU_REG_WRITE(SPIx, pMPU9250, addr, &val);
        break;
    case GFSR_2000DPS:
        pMPU9250->sensorData.gScaleFactor = 16.4;
        val = 0x18;
        MPU_REG_WRITE(SPIx, pMPU9250, addr, &val);
        break;
    default:
        pMPU9250->sensorData.gScaleFactor = 65.5;
        val = 0x08;
        MPU_REG_WRITE(SPIx, pMPU9250, addr, &val);
        break;
    }
}


void AK8963_Write(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t reg, uint8_t data) {
    uint8_t writeData;

    // AK8963의 I2C 주소 설정 (쓰기 모드)
    writeData = 0x0C;  // AK8963 주소 (쓰기 모드)
    MPU_REG_WRITE(SPIx, pMPU9250, I2C_SLV0_ADDR, &writeData);

    // 접근할 AK8963 레지스터 설정
    writeData = reg;
    MPU_REG_WRITE(SPIx, pMPU9250, I2C_SLV0_REG, &writeData);

    // 레지스터에 쓰고자 하는 데이터 설정
    writeData = data;
    MPU_REG_WRITE(SPIx, pMPU9250, I2C_SLV0_DO, &writeData);

    // I2C_SLV0 활성화 (쓰기 작업)
    writeData = 0x81;  // 슬레이브 활성화 + 1 바이트 쓰기
    MPU_REG_WRITE(SPIx, pMPU9250, I2C_SLV0_CTRL, &writeData);

    // 약간의 지연 추가 (쓰기가 완료될 때까지 대기)
    HAL_Delay(1);
}


void AK8963_Read(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t reg, uint8_t *data, uint8_t RxSize) {
    uint8_t writeData;
    uint8_t readData_temp;

    // AK8963의 I2C 주소 설정 (읽기 모드)
    writeData = 0x0C | 0x80;  // AK8963 주소 (읽기 모드)
    MPU_REG_WRITE(SPIx, pMPU9250, I2C_SLV0_ADDR, &writeData);
    MPU_REG_READ(SPIx, pMPU9250, I2C_SLV0_ADDR, &readData_temp, 1);

    // 접근할 AK8963 레지스터 설정
    writeData = reg;
    MPU_REG_WRITE(SPIx, pMPU9250, I2C_SLV0_REG, &writeData);
    MPU_REG_READ(SPIx, pMPU9250, I2C_SLV0_REG, &readData_temp, 1);

    // I2C_SLV0 활성화 (읽기 작업)
    writeData = 0x80 | RxSize;  // 슬레이브 활성화 + Rxsize 바이트만큼 읽기. RxSize는 최대 15바이트임.
    MPU_REG_WRITE(SPIx, pMPU9250, I2C_SLV0_CTRL, &writeData);
    MPU_REG_READ(SPIx, pMPU9250, I2C_SLV0_CTRL, &readData_temp, 1);

    // 약간의 지연 추가 (읽기 완료 대기)
    HAL_Delay(12);

    // 읽은 데이터를 EXT_SENS_DATA_00에서 가져오기
    MPU_REG_READ(SPIx, pMPU9250, EXT_SENS_DATA_00, data, RxSize);


}

void replaceBlockAK(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t address, uint8_t block, uint8_t at, uint8_t sz){
    uint8_t data;
    MPU_REG_READ(SPIx, pMPU9250, address, &data, 1);
    data &= ~(((1<<sz)-1)<<at);
    data |= block<<at;
    MPU_REG_WRITE(SPIx, pMPU9250, address, &data);
}

void update(FusionMethod method, MPU9250_t *pMPU9250)
{
  uint32_t current_time = __HAL_TIM_GET_COUNTER(&htim15);  // 타이머 카운터 값 읽기
  float deltat = (float)(current_time) / 1000000.0f; // us -> s로 변환

  // 타이머 카운터를 0으로 리셋
  __HAL_TIM_SET_COUNTER(&htim15, 0);

  if (method == MADWICK) {
//   qFilter.update(-a.y, -a.x, a.z, g.y, g.x, -g.z, m.x, m.y, m.z, q, deltat);  // dot의 방향이 전진인 경우
//   updateRPY();
  }
  else {
    comFilter_func(pMPU9250, deltat);
  }
}


void comFilter_func(MPU9250_t *pMPU9250, float dt)
{
  float pitch=0.0f, roll=0.0f, yaw=0.0f;

  float ax=pMPU9250->AccelX, ay=pMPU9250->AccelY, az=pMPU9250->AccelZ;
  float gx=pMPU9250->GyroX, gy=pMPU9250->GyroY, gz=pMPU9250->GyroZ;
  float mx=pMPU9250->MagX, my=pMPU9250->MagY, mz=pMPU9250->MagZ;

  ax = ay; ay = ax; az = -az;
  gx = PI/180.0f * gy; gy = PI/180.0f * gx; gz = -PI/180.0f * gz;
  float ayz= sqrt(ay*ay + az*az);
  float rollAcc= atan2(-ay,-az);      //Accerlometer로 구한 roll 각도
  float pitchAcc= atan2(ax,ayz);      //Accerlometer로 구한 pitch 각도
  roll= ALPHA*(roll+gx*dt) + BETA*rollAcc;    //상보필터를 통과한 roll 각도
  pMPU9250->roll = roll;
  pitch= ALPHA*(pitch+gy*dt)+BETA*pitchAcc;   //상보필터를 통과한 pitch 각도
  pMPU9250->pitch = pitch;
  float c_th=cos(pitch), s_th=sin(pitch), c_pi=cos(roll), s_pi=sin(roll);
  mx= mx*c_th+my*s_pi*c_th+mz*c_pi*s_th;
  my= my*c_pi-mz*s_pi;
  float heading=-atan2(my, mx);
  if ((yaw-heading)>PI) heading+=(2*PI);
  else if ((yaw-heading)<-PI) yaw+=(2*PI);
  yaw= ALPHA*(yaw+gz*dt)+BETA*heading;
  yaw= (yaw> PI) ? (yaw - (2*PI)) : ((yaw < -PI) ? (yaw + 2*PI) : yaw);
  pMPU9250->yaw = yaw;
}




