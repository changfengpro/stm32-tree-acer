/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

#include "robot.h"




osThreadId_t robotTaskHandle;


void StartRobotTask(void *argument);



/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{ 
    const osThreadAttr_t robotTask_attributes = {
        .name = "robotTask",
        .stack_size = 128 * 8,
        .priority = (osPriority_t) osPriorityNormal,
    };
    
    robotTaskHandle = osThreadNew(StartRobotTask, NULL, &robotTask_attributes);


}


__attribute__((noreturn)) void StartRobotTask(void *argument)
{
    for(;;)
    {
        RobotTask();

        osDelay(5);
    }
}