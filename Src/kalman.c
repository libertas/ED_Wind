/*
 * kalman.c
 *
 *  Created on: Jul 23, 2017
 *      Author: xita
 */

#include "kalman.h"

void kalman_init(kalman_t *kt)
{
	kt->K1 = 0.02;
	kt->Q_angle = 0.001;
	kt->Q_gyro = 0.003;
	kt->R_angle = 0.5;
	kt->dt = 0.005;
	kt->C_0 = 1;
	kt->Pdot[0] = 0;
	kt->Pdot[1] = 0;
	kt->Pdot[2] = 0;
	kt->Pdot[3] = 0;
	kt->PP[0][0] = 1;
	kt->PP[0][1] = 0;
	kt->PP[1][0] = 0;
	kt->PP[1][1] = 1;
}

void kalman_filter(kalman_t *kt, float Accel, float Gyro)
{
	kt->angle += (Gyro - kt->Q_bias) * kt->dt; //先验估计
	kt->Pdot[0] = kt->Q_angle - kt->PP[0][1] - kt->PP[1][0]; // Pk-先验估计误差协方差的微分
	kt->Pdot[1] = -kt->PP[1][1];
	kt->Pdot[2] = -kt->PP[1][1];
	kt->Pdot[3] = kt->Q_gyro;
	kt->PP[0][0] += kt->Pdot[0] * kt->dt;   // Pk-先验估计误差协方差微分的积分
	kt->PP[0][1] += kt->Pdot[1] * kt->dt;   // =先验估计误差协方差
	kt->PP[1][0] += kt->Pdot[2] * kt->dt;
	kt->PP[1][1] += kt->Pdot[3] * kt->dt;
	kt->Angle_err = Accel - kt->angle;	//zk-先验估计
	kt->PCt_0 = kt->C_0 * kt->PP[0][0];
	kt->PCt_1 = kt->C_0 * kt->PP[1][0];
	kt->E = kt->R_angle + kt->C_0 * kt->PCt_0;
	kt->K_0 = kt->PCt_0 / kt->E;
	kt->K_1 = kt->PCt_1 / kt->E;
	kt->t_0 = kt->PCt_0;
	kt->t_1 = kt->C_0 * kt->PP[0][1];
	kt->PP[0][0] -=kt-> K_0 * kt->t_0;		 //后验估计误差协方差
	kt->PP[0][1] -= kt->K_0 * kt->t_1;
	kt->PP[1][0] -= kt->K_1 * kt->t_0;
	kt->PP[1][1] -= kt->K_1 * kt->t_1;
	kt->angle	+= kt->K_0 * kt->Angle_err;	 //后验估计
	kt->Q_bias	+= kt->K_1 * kt->Angle_err;	 //后验估计
	kt->angle_dot   = Gyro - kt->Q_bias;	 //输出值(后验估计)的微分=角速度
}
