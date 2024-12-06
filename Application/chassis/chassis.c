#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"



static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令

// first表示第一象限， second表示第二象限，以此类推
static DJIMotorInstance *First_GM6020_motor, *Second_GM6020_motor, *Third_GM6020_motor, *Fourth_GM6020_motor, \
                        *First_M3508_motor,  *Second_M3508_motor,  *Third_M3508_motor,  *Fourth_M3508_motor;
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
    Motor_Init_Config_s chassis_first_GM6020_motor_config =       //first表示第一象限， second表示第二象限，以此类推
    {
        .motor_type = GM6020,
        .can_init_config = 
        {.can_handle = &hcan1,
            .tx_id = 1
        },
        .controller_setting_init_config = 
        {.angle_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.angle_PID = {.Improve = 0,
                                                        .Kp = 10,
                                                        .Ki = 0,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 4000},
        .speed_PID = {.Improve = 0,
                        .Kp = 10,
                        .Ki = 0,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 4000,
        }

        }
    };

    Motor_Init_Config_s chassis_second_GM6020_motor_config = 
    {
        .motor_type = GM6020,
        .can_init_config = 
        {.can_handle = &hcan2,
            .tx_id = 1
        },
        .controller_setting_init_config = 
        {.angle_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.angle_PID = {.Improve = 0,
                                                        .Kp = 10,
                                                        .Ki = 0,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 4000},
        .speed_PID = {.Improve = 0,
                        .Kp = 10,
                        .Ki = 0,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 4000,
        }

        }
    };

    Motor_Init_Config_s chassis_third_GM6020_motor_config = 
    {
        .motor_type = GM6020,
        .can_init_config = 
        {.can_handle = &hcan2,
            .tx_id = 2
        },
        .controller_setting_init_config = 
        {.angle_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.angle_PID = {.Improve = 0,
                                                        .Kp = 10,
                                                        .Ki = 0,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 4000},
        .speed_PID = {.Improve = 0,
                        .Kp = 10,
                        .Ki = 0,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 4000,
        }

        }
    };

    Motor_Init_Config_s chassis_fourth_GM6020_motor_config = 
    {
        .motor_type = GM6020,
        .can_init_config = 
        {.can_handle = &hcan1,
            .tx_id = 2
        },
        .controller_setting_init_config = 
        {.angle_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.angle_PID = {.Improve = 0,
                                                        .Kp = 10,
                                                        .Ki = 0,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 4000},
        .speed_PID = {.Improve = 0,
                        .Kp = 10,
                        .Ki = 0,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 4000,
        }

        }
    };
    
/************************************************************************************************** */

    Motor_Init_Config_s chassis_first_M3508_motor_config = 
    {
        .motor_type = M3508,
        .can_init_config = 
        {
            .can_handle = &hcan1,
            .tx_id = 3
        },
        .controller_setting_init_config = 
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.speed_PID = {.Improve = 0,
                                                        .Kp = 10,
                                                        .Ki = 0,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 0},
        }
    };

     Motor_Init_Config_s chassis_second_M3508_motor_config = 
    {
        .motor_type = M3508,
        .can_init_config = 
        {
            .can_handle = &hcan2,
            .tx_id = 3
        },
        .controller_setting_init_config = 
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.speed_PID = {.Improve = 0,
                                                        .Kp = 10,
                                                        .Ki = 0,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 0},
        }
    };

    Motor_Init_Config_s chassis_third_M3508_motor_config = 
    {
        .motor_type = M3508,
        .can_init_config = 
        {
            .can_handle = &hcan2,
            .tx_id = 4
        },
        .controller_setting_init_config = 
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.speed_PID = {.Improve = 0,
                                                        .Kp = 10,
                                                        .Ki = 0,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 0},
        }
    };

    Motor_Init_Config_s chassis_fourth_M3508_motor_config = 
    {
        .motor_type = M3508,
        .can_init_config = 
        {
            .can_handle = &hcan1,
            .tx_id = 4
        },
        .controller_setting_init_config = 
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.speed_PID = {.Improve = 0,
                                                        .Kp = 10,
                                                        .Ki = 0,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 0},
        }
    };

    First_GM6020_motor = DJIMotorInit(&chassis_first_GM6020_motor_config);
    Second_GM6020_motor = DJIMotorInit(&chassis_second_GM6020_motor_config);
    Third_GM6020_motor = DJIMotorInit(&chassis_third_GM6020_motor_config);
    Fourth_GM6020_motor = DJIMotorInit(&chassis_fourth_GM6020_motor_config);
    First_M3508_motor = DJIMotorInit(&chassis_first_M3508_motor_config);
    Second_M3508_motor = DJIMotorInit(&chassis_second_M3508_motor_config);
    Third_M3508_motor= DJIMotorInit(&chassis_third_M3508_motor_config);
    Fourth_M3508_motor = DJIMotorInit(&chassis_fourth_M3508_motor_config);


}