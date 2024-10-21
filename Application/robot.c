#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"

#include "robot_cmd.h"
#include "chassis_control.h"
#include "Bsp_can.h"
#include "pid.h"
#include "usart.h"

extern uint8_t buffer[36];
extern pid_struct_t pid[9];
void RobotInit()
{
    can_user_init(&hcan1);
    HAL_UARTEx_ReceiveToIdle_IT(&huart3 , buffer, sizeof(buffer));
    for(int i = 1; i < 5; i++)
    {
        pid_init(&pid[i], 70, 3, 0.05, 12000, 12000);
    }
    OSTaskInit();
}

void RobotTask()
{
    RobotCMDTask();
    ChassisTask();
}