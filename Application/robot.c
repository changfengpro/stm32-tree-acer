/*
 * robot.c
 *
 *  Created on: Sep 23, 2024
 *      Author: auroranebulas
 */

#include "robot.h"

void RobotTask()
{
    DMMotorTask(motor);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim ->Instance == TIM2)
	{
        RobotTask();
	}
}