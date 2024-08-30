/*
 * djimotor.h
 *
 *  Created on: Jul 14, 2024
 *      Author: auroranebulas
 */

#ifndef DJIMOTOR_DJIMOTOR_H_
#define DJIMOTOR_DJIMOTOR_H_

#include "Bsp_can.h"

#define SLID_WINDOWS_SIZE 3				//滑动窗口大小

typedef struct
{
	float DataBuff[SLID_WINDOWS_SIZE];		//采样数据区
	uint16_t index;							//实现队列循环的队尾下标
	uint16_t size;							//当前窗口内元素个数
} slid_avg_filter_t;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Velocity_Calclate(int ID);
void AngleSumCalc(int ID);
void DJIMotorStop();
float Low_Pass_Filter(float data, float last_data);
float Slid_avg_filter(slid_avg_filter_t* const filter, const float data);

#endif /* DJIMOTOR_DJIMOTOR_H_ */
