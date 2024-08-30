/*
 * robot.c
 *
 *  Created on: Aug 30, 2024
 *      Author: auroranebulas
 */

#include "robot_cmd.h"
#include "chassis_control.h"



void RobotTask()
{
    RobotCMDTask();
    ChassisTask();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim ->Instance == TIM2)
	{
        RobotTask();
	}
}