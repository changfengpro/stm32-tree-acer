/*
 * robot.h
 *
 *  Created on: Aug 30, 2024
 *      Author: auroranebulas
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "tim.h"
#include "main.h"
#include "dmmotor.h"

extern  DMMotorInstance *motor;

void RobotTask();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif