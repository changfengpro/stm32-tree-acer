/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include "bsp_dwt.h"
#include "robot.h"
#include "motor_task.h"




osThreadId_t robotTaskHandle;
osThreadId_t insTaskHandle;
osThreadId_t motorTaskHandle;

void StartRobotTask(void *argument);
void StartInsTask(void *argument);
void StartMotorTask(void *argument);

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

    const osThreadAttr_t insTask_attributes = {
        .name = "insTask",
        .stack_size = 128 * 8,
        .priority = (osPriority_t) osPriorityAboveNormal,
    };

    insTaskHandle = osThreadNew(StartInsTask, NULL, &insTask_attributes);

    const osThreadAttr_t motorTask_attributes = {
        .name = "motorTask",
        .stack_size = 128 *8,
        .priority = (osPriority_t) osPriorityNormal,
    };

    motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);

}


__attribute__((noreturn)) void StartRobotTask(void *argument)
{
    for(;;)
    {
        RobotTask();

        osDelay(1);
    }
}

__attribute__((noreturn)) void StartInsTask(void *argument)
{
    static float ins_start;
    static float ins_dt;
    // INS_Init(); // 确保BMI088被正确初始化.
    for(;;)
    {
        ins_start = DWT_GetTimeline_ms();
        // INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;

        osDelay(1);

    }
}

__attribute__((noreturn)) void StartMotorTask(void *argument)
{   
    static float motor_dt;
    static float motor_start;

    for(;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;

        osDelay(1);
    }
}
