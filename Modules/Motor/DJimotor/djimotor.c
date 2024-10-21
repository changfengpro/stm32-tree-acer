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




extern moto_info_t motor_info[MOTOR_MAX_NUM]; //前四位为底盘，后两位为云台
extern rc RC_ctrl;
static float position[5];
static float last_position[5];
float Velocity[5];
float Last_Velocity[5];
float Velocity_filter[5];

extern float vt_lf, vt_rf, vt_lb, vt_rb;







void DJIMotorStop()
{
	set_motor_value(0x200, 0, 0, 0, 0);
}


