/*
 * Corexy_Control.c
 *
 *  Created on: Aug 31, 2024
 *      Author: auroranebulas
 */
#include "Corexy_Control.h"
#include "Bsp_can.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "pid.h"


float vt_lu, vt_ru, vt_ld, vt_rd;
extern moto_info_t motor_info[MOTOR_MAX_NUM];
extern Corexy_Ctrl_Cmd_s Corexy_cmd_recv;             //corexy结构接收到的控制指令
extern pid_struct_t pid[9];
extern float pid_ref[9];





/*
 * @brief Corexy结构速度计算
 * @return None
 */
void CorexyCalculate()
{
    vt_lu =  Corexy_cmd_recv.vz;
    vt_ru = -Corexy_cmd_recv.vz;
}

/*
 * @brief Corexy结构控制任务
 * @return None
 */
void CorexyTask()
{
    CorexyCalculate();

    pid_ref[7] = ver_pid_calc(&pid[7], vt_lu, motor_info[7].rotor_speed);
    pid_ref[8] = ver_pid_calc(&pid[8], vt_ru, motor_info[8].rotor_speed);

    set_motor_value_CAN2(0x1FF, 0, 0, pid_ref[7], pid_ref[8]);

}