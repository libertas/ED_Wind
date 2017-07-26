/*
 * kalman.h
 *
 *  Created on: Jul 23, 2017
 *      Author: xita
 */

#ifndef KALMAN_H_
#define KALMAN_H_

typedef struct {
	float K1;
	float angle, angle_dot;
	float Q_angle;// 过程噪声的协方差
	float Q_gyro;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	float R_angle;// 测量噪声的协方差 既测量偏差
	float dt;//
	char  C_0;
	float Q_bias, Angle_err;
	float PCt_0, PCt_1, E;
	float K_0, K_1, t_0, t_1;
	float Pdot[4];
	float PP[2][2];
} kalman_t;

void kalman_init(kalman_t *kt);
void kalman_filter(kalman_t *kt, float Accel, float Gyro);


#endif /* KALMAN_H_ */
