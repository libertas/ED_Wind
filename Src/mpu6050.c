#include <math.h>

#include "kalman.h"
#include "mpu6050.h"
#include "time.h"

//float axd, ayd, azd;
float wxd, wyd, wzd;
I2C_HandleTypeDef *mpu6050_i2c_device;

float xsi = 0;
float ysi = 0;
float zsi = 0;
float xsum = 0;
float ysum = 0;
float zsum = 0;


kalman_t kalmanx, kalmany;

float int2float(signed int i)
{
	float result;
	if(i > 32767) {
		result =  (float)((unsigned int) i) - 65536.0;
	} else {
		result = (float) i;
	}
	return result;
}

bool mpu6050_read(uint8_t addr, uint8_t reg, uint8_t *result)
{
	uint8_t data;

	while(HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_READY && HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_BUSY_RX);
	if(HAL_I2C_Master_Transmit(mpu6050_i2c_device, addr, &reg, 1, 1) != HAL_OK)
		return false;
	while(HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_READY && HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_BUSY_TX);
	if(HAL_I2C_Master_Receive(mpu6050_i2c_device, addr, &data, 1, 1) != HAL_OK)
		return false;

	*result = data;
	return true;
}

uint8_t mpu6050_write(uint8_t addr, uint8_t reg, uint8_t data)
{
	uint8_t msg[2];
	msg[0] = reg;
	msg[1] = data;
	while(HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_READY && HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_BUSY_RX);
	while(HAL_I2C_Master_Transmit(mpu6050_i2c_device, addr, msg, 2, 1) != HAL_OK);
	return 0;
}

signed int mpu6050_get_data(uint8_t reg)
{
	uint16_t result;

	if(mpu6050_read(MPU6050SlaveAddress, reg, (uint8_t*)(&result) + 1) &&
			mpu6050_read(MPU6050SlaveAddress, reg + 1, (uint8_t*)(&result))) {
		return result;
	} else {
		return 0;
	}
}

float mpu6050_get_exact_data(uint8_t reg)
{
	float result = int2float(mpu6050_get_data(reg));
	switch(reg)
	{
//		case ACCEL_XOUT_H:
//			result -= axd;
//			break;
//
//		case ACCEL_YOUT_H:
//			result -= ayd;
//			break;
//
//		case ACCEL_ZOUT_H:
//			result -= azd;
//			break;

		case GYRO_XOUT_H:
			result -= wxd;
			break;

		case GYRO_YOUT_H:
			result -= wyd;
			break;

		case GYRO_ZOUT_H:
			result -= wzd;
			break;

		default:
			break;
	}
	return result;
}

void mpu6050_set_average_values(void)
{
	uint16_t i;

//	axd = 0;
//	ayd = 0;
//	azd = 0;
	wxd = 0;
	wyd = 0;
	wzd = 0;

	#define MPU_SUM 300

	for(i = 0; i < MPU_SUM; i++)
	{
//		axd += int2float(mpu6050_get_data(ACCEL_XOUT_H)) / MPU_SUM;
//		ayd += int2float(mpu6050_get_data(ACCEL_YOUT_H)) / MPU_SUM;
//		azd += int2float(mpu6050_get_data(ACCEL_ZOUT_H)) / MPU_SUM;
		wxd += int2float(mpu6050_get_data(GYRO_XOUT_H)) / MPU_SUM;
		wyd += int2float(mpu6050_get_data(GYRO_YOUT_H)) / MPU_SUM;
		wzd += int2float(mpu6050_get_data(GYRO_ZOUT_H)) / MPU_SUM;
	}
}

void mpu6050_get_kine_state(struct kine_state *result)
{
	static float lasttime = 0;

	float ax, ay, az;
	float wx, wy, wz;

	ax = mpu6050_get_exact_data(ACCEL_XOUT_H);
	ay = mpu6050_get_exact_data(ACCEL_YOUT_H);
	az = mpu6050_get_exact_data(ACCEL_ZOUT_H);

	wx = mpu6050_get_exact_data(GYRO_XOUT_H);
	wy = mpu6050_get_exact_data(GYRO_YOUT_H);
	wz = mpu6050_get_exact_data(GYRO_ZOUT_H);

	float thistime = seconds();
	float difftime = thistime - lasttime;
	lasttime = thistime;

//	result->x += result->wx * difftime;
//	result->y += result->wy * difftime;
	result->z += result->wz * difftime;

	result->ax = ax * ACCEL_RANGE / 32767;
	result->ay = ay * ACCEL_RANGE / 32767;
	result->az = az * ACCEL_RANGE / 32767;

	result->wx = wx * GYRO_RANGE / 32767;
	result->wy = wy * GYRO_RANGE / 32767;
	result->wz = wz * GYRO_RANGE / 32767;

	float cosx1, cosy1;
	cosx1 = sqrt(result->ax * result->ax + result->az * result->az) / 9.8f;
	cosy1 = sqrt(result->ay * result->ay + result->az * result->az) / 9.8f;
	if(cosx1 > 1) {
		result->y1 = result->y;
	} else {
		if(ax >= 0) {
			result->y1 = -acos(cosx1);
		} else {
			result->y1 = acos(cosx1);
		}
	}
	if(cosy1 > 1) {
			result->x1 = result->x;
	} else {
		if(ay >= 0) {
			result->x1 = acos(cosy1);
		} else {
			result->x1 = -acos(cosy1);
		}
	}

	kalmanx.dt = difftime;
	kalmany.dt = difftime;
	kalman_filter(&kalmanx, result->x1, result->wx);
	kalman_filter(&kalmany, result->y1, result->wy);

	result->x = kalmanx.angle;
	result->y = kalmany.angle;
	result->wx = kalmanx.angle_dot;
	result->wy = kalmany.angle_dot;


//	float k = 0.1f;
//	xsum += result->x;
//	xsi++;
//	result->x -= k * xsum / xsi;
//
//	ysum += result->y;
//	ysi++;
//	result->y -= k * ysum / ysi;
//
//	zsum += result->z;
//	zsi++;
//	result->z -= k * zsum / zsi;
}

void mpu6050_init(I2C_HandleTypeDef *device)
{
	mpu6050_i2c_device = device;

	mpu6050_write(MPU6050SlaveAddress, PWR_MGMT_1, 0x00);
	mpu6050_write(MPU6050SlaveAddress, SMPLRT_DIV, 0x00);
	mpu6050_write(MPU6050SlaveAddress, CONFIG, 0x00);
	mpu6050_write(MPU6050SlaveAddress, GYRO_CONFIG, 0x18);
	mpu6050_write(MPU6050SlaveAddress, ACCEL_CONFIG, 0x00);

	mpu6050_set_average_values();

	kalman_init(&kalmanx);
	kalman_init(&kalmany);

	kalmanx.K1 = 0.02;
	kalmanx.Q_gyro = 0.01;
	kalmanx.Q_angle = 0.001;
	kalmanx.R_angle = 0.1;

	kalmany.K1 = 0.02;
	kalmany.Q_gyro = 0.01;
	kalmany.Q_angle = 0.001;
	kalmany.R_angle = 0.1;
}
