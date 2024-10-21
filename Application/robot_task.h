#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"

osThreadId_t robotTaskHandle;

void StartROBOTTASK(void const *argument);


/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    const osThreadAttr_t robotTask_attributes = {
    .name = "robotTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
    };

    robotTaskHandle = osThreadNew(StartROBOTTASK, NULL, &robotTask_attributes);
}


__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    RobotTask();
    osDelay(5);
}