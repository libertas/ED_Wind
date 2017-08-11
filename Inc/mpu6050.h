
#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

//#define MPU6050_USE_DMA
//#define MPU6050_USE_MAG

#define MPU_SUM 300

#define	SMPLRT_DIV		0x19
#define	CONFIG			0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define I2C_MST_CTRL	0x24
#define BYPASS_EN		0x37
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

#ifdef MPU6050_USE_MAG

#define	AKM8963SlaveAddress	0x18

#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_DATA_OUT 0x63
#define USER_CTRL 0x6A
#define EXT_SENS_DATA 0x49

#define AKM8963_CNTL1 0x0A
#define AKM8963_CNTL2 0x0B
#define AKM8963_HXL 0x03
#define AKM8963_ASAX 0x10
#define AKM8963_ASAY 0x11
#define AKM8963_ASAZ 0x12

#define MAG_RANGE ((float)(4800.0f))

#define MAG_CALIB_FLASH_ADDR 0x10

#endif

#define ACCEL_RANGE ((float)(2 * 9.8f))

#define GYRO_RANGE ((float)(2000.0f / 360.0f * 2 * 3.14159265f))

struct kine_state {
	float x, y, z;
	float x1, y1, z1; /* computed from acc and mag data */
	float wx, wy, wz;
	float ax, ay, az;
	float mx, my, mz;
};

void mpu6050_get_kine_state(struct kine_state *state_now);

void mpu6050_init(I2C_HandleTypeDef *device);
bool mpu6050_read(uint8_t addr, uint8_t reg, uint8_t *result);
signed int mpu6050_get_data(uint8_t reg);
float mpu6050_get_exact_data(uint8_t reg);


#ifdef MPU6050_USE_DMA

#ifdef MPU6050_USE_MAG

#define MPU6050_DMA_COUNT 20

#else

#define MPU6050_DMA_COUNT 14

#endif

#define MPU6050_DMA_ADDR_START ACCEL_XOUT_H

bool* mpu6050_start_read_dma(uint8_t addr);
void mpu6050_update_data();

#endif

#endif /* MPU6050_H_ */
