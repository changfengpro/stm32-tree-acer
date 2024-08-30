/*
 * djimotor.c
 *
 *  Created on: Jul 14, 2024
 *      Author: auroranebulas
 */

#include "chassis_control.h"
#include "pid.h"
#include "Bsp_can.h"
#include "robot_cmd.h"
#include "chassis_control.h"
#include "djimotor.h"




extern Motor_6025_Typedef motor_array[6]; //前四位为底盘，后两位为云台
extern rc RC_ctrl;
static float position[5];
static float last_position[5];
float Velocity[5];
float Last_Velocity[5];
float Velocity_filter[5];
slid_avg_filter_t Slid_filter[5];	//初始化滤波器示例
extern float vt_lf, vt_rf, vt_lb, vt_rb;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim ->Instance == TIM2)
	{

		ChassisTask();

	}
}


void AngleSumCalc(int ID)
{
//	position[ID] = motor_array[ID].angle;
	float temp =   motor_array[ID].angle - last_position[ID];
	position[ID] +=  temp;


}

void Velocity_Calclate(int ID)
{
	AngleSumCalc(ID);

	Velocity[ID] = ((position[ID] - last_position[ID]) * (360.0 / 8192.0) ) * 166.66666667;

	last_position[ID] = motor_array[ID].angle;
}


void DJIMotorStop()
{
	set_motor_value(0x200, 0, 0, 0, 0);
}

/*
 * @brief 滑动滤波
 * @param filter 滤波器
 * @param data 最新数据
 * @return 滤波后的结果
 */
float Slid_avg_filter(slid_avg_filter_t* const filter, const float data)
{
	double temp;
	
	//采用循环队列形式将最新数据入队
	filter->DataBuff[filter->index] = data;
	filter->index = (filter->index + 1) % SLID_WINDOWS_SIZE;
	if(filter->size < SLID_WINDOWS_SIZE) filter->size += 1;

	//取平均
	for(uint16_t i = 0; i < filter->size; i++)
	{
		temp += filter->DataBuff[i];
	}
	temp /= filter->size;
} 

/*
 * @brief 低通滤波
 * @param data 本次数据
 * @param last_data 上次数据
 * @return 滤波后的结果
 */
float Low_Pass_Filter(float data, float last_data)
{
	double temp;

	temp = 0.7 * last_data + 0.3 * data;
}
