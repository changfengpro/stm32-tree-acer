// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "dji_motor.h"
#include "chassis.h"

// bsp


// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360


static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Robot_Status_e robot_state; // 机器人整体工作状态
Chassis_Ctrl_Cmd_s chassis_cmd_send;
extern RC_ctrl_t rc_ctrl[2]; 

static void RemoteControlSet();


void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}


void RobotCMDTask()

{
    RemoteControlSet();
}


/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    if(switch_is_up(rc_ctrl->rc.switch_right))          //右侧开关上，底盘控制
    {
        chassis_cmd_send.vx = rc_ctrl->rc.rocker_r_  / 1.5f;
        chassis_cmd_send.vy = rc_ctrl->rc.rocker_r1 / 1.5f;
        chassis_cmd_send.wz = - rc_ctrl->rc.rocker_l_ * 150.0f;
    }
}