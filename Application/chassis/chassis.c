#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"

#define WHEEL_LINE_RATION (1 / (180.0f * REDUCTION_RATIO_WHEEL)) * PI * RADIUS_WHEEL   //将舵轮电机转速转换为底盘线速度的比例
#define WHEEL_RPM_RATION  (1 / (RADIUS_WHEEL * REDUCTION_RATIO_WHEEL)) *(180.0f / PI)  //将底盘线速度转换为舵轮电机转速的比例
#define RADIAN_TO_ANGLE 180 / PI;   //将弧度制转为角度制

static void ChassisHandle_Deliver_Config();
static void Steer_Speed_Calcu(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz);
static void Steer_angle_change(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz);
static void Steer_Calculate(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz);
static void Steer_Chassis_Control(ChassisHandle_t *Chassis_hanlde);



float angle;
float speed;
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令

// first表示第一象限， second表示第二象限，以此类推
static DJIMotorInstance *First_GM6020_motor, *Second_GM6020_motor, *Third_GM6020_motor, *Fourth_GM6020_motor, \
                        *First_M3508_motor,  *Second_M3508_motor,  *Third_M3508_motor,  *Fourth_M3508_motor;
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
extern Chassis_Ctrl_Cmd_s chassis_cmd_send;
ChassisHandle_t chassis_handle;

static void ChassisHandle_Deliver_Config()
{
    chassis_handle.vx = chassis_cmd_send.vx;
    chassis_handle.vy = chassis_cmd_send.vy;
    chassis_handle.wz = chassis_cmd_send.wz;
}



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
                                                        .Kp = 25,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000},
        .speed_PID = {.Improve = 0,
                        .Kp = 10,
                        .Ki = 0.1,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 20000,
                        .IntegralLimit = 3000
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
            .outer_loop_type = ANGLE_LOOP ,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.angle_PID = {.Improve = 0,
                                                        .Kp = 25,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000},
        .speed_PID = {.Improve = 0,
                        .Kp = 10,
                        .Ki = 0.1,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 20000,
                        .IntegralLimit = 3000
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
                                                        .Kp = 25,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000},
        .speed_PID = {.Improve = 0,
                        .Kp = 10,
                        .Ki = 0,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 20000,
                        .IntegralLimit = 3000
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
                                                        .Kp = 25,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000},
        .speed_PID = {.Improve = 0,
                        .Kp = 10,
                        .Ki = 0,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 20000,
                        .IntegralLimit = 3000,

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
                                                        .Kp = 2.5,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000}
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
                                                        .Kp = 2.5,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000},
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
                                                        .Kp = 2.5,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000},
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
                                                        .Kp = 2.5,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000},
        }
    };

    First_GM6020_motor  = DJIMotorInit(&chassis_first_GM6020_motor_config);
    Second_GM6020_motor = DJIMotorInit(&chassis_second_GM6020_motor_config);
    Third_GM6020_motor  = DJIMotorInit(&chassis_third_GM6020_motor_config);
    Fourth_GM6020_motor = DJIMotorInit(&chassis_fourth_GM6020_motor_config);
    First_M3508_motor   = DJIMotorInit(&chassis_first_M3508_motor_config);
    Second_M3508_motor  = DJIMotorInit(&chassis_second_M3508_motor_config);
    Third_M3508_motor   = DJIMotorInit(&chassis_third_M3508_motor_config);
    Fourth_M3508_motor  = DJIMotorInit(&chassis_fourth_M3508_motor_config);


    ChassisHandle_Deliver_Config();
}




/* 机器人底盘控制核心任务 */
void ChassisTask()
{   
    DJIMotorEnable(First_GM6020_motor);
    DJIMotorEnable(Second_GM6020_motor);
    DJIMotorEnable(Third_GM6020_motor);
    DJIMotorEnable(Fourth_GM6020_motor);
    DJIMotorEnable(First_M3508_motor);
    DJIMotorEnable(Second_M3508_motor);
    DJIMotorEnable(Third_M3508_motor);
    DJIMotorEnable(Fourth_M3508_motor);

    ChassisHandle_Deliver_Config();
    Steer_Chassis_Control(&chassis_handle);

    DJIMotorSetRef(First_GM6020_motor, (float)(chassis_handle.motor_set_steer[0]) - 5.0f );
    DJIMotorSetRef(Second_GM6020_motor, (float)(chassis_handle.motor_set_steer[1]) - 145.0f);
    DJIMotorSetRef(Third_GM6020_motor, (float)(chassis_handle.motor_set_steer[2]) - 0.0f);
    DJIMotorSetRef(Fourth_GM6020_motor, (float)(chassis_handle.motor_set_steer[3]) - 20.0f);

    DJIMotorSetRef(First_M3508_motor, chassis_handle.motor_set_speed[0]);
    DJIMotorSetRef(Second_M3508_motor, chassis_handle.motor_set_speed[1]);
    DJIMotorSetRef(Third_M3508_motor, chassis_handle.motor_set_speed[2]);
    DJIMotorSetRef(Fourth_M3508_motor, chassis_handle.motor_set_speed[3]);
    
    DJIMotorControl();
}

static void Steer_Speed_Calcu(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz)
{
    float theta = atan(1.0 / 1.0);  //返回45度
    float steer_wz = chassis_wz * PI /180.0f;  //chassis_wz传入的是度/秒，转化为弧度制


    //根据公式计算电机转速
    chassis_handle->chassis_motor_speed[0] = sqrt( pow(chassis_vx - steer_wz * RADIUS * cos(theta), 2) + pow(chassis_vy + steer_wz * RADIUS * sin(theta), 2)) * WHEEL_RPM_RATION;
    chassis_handle->chassis_motor_speed[1] = sqrt( pow(chassis_vx - steer_wz * RADIUS * cos(theta), 2) + pow(chassis_vy - steer_wz * RADIUS * sin(theta), 2)) * WHEEL_RPM_RATION; 
    chassis_handle->chassis_motor_speed[2] = sqrt( pow(chassis_vx + steer_wz * RADIUS * cos(theta), 2) + pow(chassis_vy - steer_wz * RADIUS * sin(theta), 2)) * WHEEL_RPM_RATION;
    chassis_handle->chassis_motor_speed[3] = sqrt( pow(chassis_vx + steer_wz * RADIUS * cos(theta), 2) + pow(chassis_vy + steer_wz * RADIUS * sin(theta), 2)) * WHEEL_RPM_RATION;

    //寻找转速最大值，以及实现正反转
    for(int i = 0; i < 4; i++)
    {
        if(chassis_handle->TurnFlag[i] == 1)
            chassis_handle->chassis_motor_speed[i] = -chassis_handle->chassis_motor_speed[i];

        else
            chassis_handle->chassis_motor_speed[i] = chassis_handle->chassis_motor_speed[i];

        if(fabs(chassis_handle->chassis_motor_speed[i]) > chassis_handle->max_speed)
            chassis_handle->max_speed = fabs(chassis_handle->chassis_motor_speed[i]);
    }

    for(int i = 0; i < 4; i++)
    {
        chassis_handle->motor_set_speed[i] = chassis_handle->chassis_motor_speed[i];
    }

}

static void Steer_angle_change(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz)
{
    float theta = atan(1.0 / 1.0);  //返回45度
    float steer_wz = chassis_wz * PI /180.0f;  //chassis_wz传入的是度/秒，转化为弧度制

    if((chassis_vx == 0) && (chassis_vy == 0) && (chassis_wz == 0))
    {
        for(int i = 0; i < 4; i++)
        {
            chassis_handle->last_steer_target_angle[i] = chassis_handle->motor_set_steer[i];
        }
    }
    else
    {
        chassis_handle->motor_set_steer[0] = atan2((chassis_vx - steer_wz * RADIUS * cos(theta)), (chassis_vy + steer_wz * RADIUS * sin(theta)));
        chassis_handle->motor_set_steer[1] = atan2((chassis_vx - steer_wz * RADIUS * cos(theta)),(chassis_vy - steer_wz * RADIUS * sin(theta)));
        chassis_handle->motor_set_steer[2] = atan2((chassis_vx + steer_wz * RADIUS * cos(theta)),(chassis_vy - steer_wz * RADIUS * sin(theta)));
        chassis_handle->motor_set_steer[3] = atan2((chassis_vx + steer_wz * RADIUS * cos(theta)),(chassis_vy + steer_wz * RADIUS * sin(theta)));


        for(int i = 0; i < 4; i++)
        {
            if((chassis_handle->motor_set_steer[i]  - chassis_handle->last_steer_target_angle[i]) > (PI / 2))
            {
                chassis_handle->motor_set_steer[i] = fmodf(chassis_handle->motor_set_steer[i] - PI, 2 * PI);
                chassis_handle->TurnFlag[i] = 1;
            }
            else if((chassis_handle->motor_set_steer[i]  - chassis_handle->last_steer_target_angle[i]) < (-PI / 2))
            {
                chassis_handle->motor_set_steer[i] = fmodf(chassis_handle->motor_set_steer[i] + PI, 2 * PI);
                chassis_handle->TurnFlag[i] = 1;
            }
            else
            {
                chassis_handle->TurnFlag[i] = 0;
            }
        }

        for(int i = 0; i < 4; i++)
        {
            chassis_handle->motor_set_steer[i] = chassis_handle->motor_set_steer[i] * RADIAN_TO_ANGLE;
        }
    }

}

static void Steer_Calculate(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz)
{
    Steer_angle_change(chassis_handle, chassis_vx, chassis_vy, chassis_wz);
    Steer_Speed_Calcu(chassis_handle, chassis_vx, chassis_vy, chassis_wz);
}

static void Steer_Chassis_Control(ChassisHandle_t *Chassis_hanlde)
{
    Steer_Calculate(Chassis_hanlde, Chassis_hanlde->vx, Chassis_hanlde->vy, Chassis_hanlde->wz);
}
