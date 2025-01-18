<!--
 * @Description: 
 * @Author: 
 * @brief: 
 * @version: 
 * @Date: 2025-01-17 20:57:17
 * @LastEditors:  
 * @LastEditTime: 2025-01-17 21:52:45
-->
robot_cmd:   作了修改
* // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_**left**))



* else if (switch_is_mid(rc_data[TEMP].rc.switch_**left**)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }