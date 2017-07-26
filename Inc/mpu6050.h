
#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define MPU6050_USE_DMA

#define	SMPLRT_DIV		0x19
#define	CONFIG				0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B
#define	MPU6050SlaveAddress	0xD0

#define ACCEL_RANGE ((float)(2 * 9.8f))

#define GYRO_RANGE ((float)(2000.0f / 360.0f * 2 * 3.14159265f))

struct kine_state {
	float x, y, z;
	float x1, y1;
	float wx, wy, wz;
	float ax, ay, az;
};

void mpu6050_get_kine_state(struct kine_state *state_now);

void mpu6050_init(I2C_HandleTypeDef *device);
bool mpu6050_read(uint8_t addr, uint8_t reg, uint8_t *result);
signed int mpu6050_get_data(uint8_t reg);
float mpu6050_get_exact_data(uint8_t reg);

#ifdef MPU6050_USE_DMA

#define MPU6050_DMA_COUNT 14
#define MPU6050_DMA_ADDR_START ACCEL_XOUT_H

bool* mpu6050_start_read_dma(uint8_t addr);
void mpu6050_update_data();

#endif

#endif /* MPU6050_H_ */
