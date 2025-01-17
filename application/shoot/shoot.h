#ifndef SHOOT_H
#define SHOOT_H

#include "dji_motor.h"

/**
 * @brief: 双发射机构shoot实例 
 * @return {*}
 */
typedef struct 
{
    DJIMotorInstance *friction_l;
    DJIMotorInstance *friction_r;
    DJIMotorInstance *loader;
} Shoot_Init_Config;




/**
 * @brief 发射初始化,会被RobotInit()调用
 * 
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();

#endif // SHOOT_H