#include "dmmotor.h"
#include "general_def.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "Bsp_can.h"
#include "arm_math.h"

#define DM_COEF PI*4.0f/12.5f

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)(x - offset) * ((float)((1 << bits) - 1) / span);
}


static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

//输出当前属于哪个圈（1234中的哪个）
//10.5    爆
static int16_t DMMotorSingleRoundSignDetect(float position)
{
    int8_t sign=0;
    if(position<-6.25)
    {
        sign=1;
    }
    else if(position < 0 &&position >= -6.25)
    {
        sign=2;
    }
    else if(position < 6.25 &&position >= 0)
    {
        sign=3;
    }
    else if(position >= 6.25)
    {
        sign=4;
    }

    return sign;
}

//达妙多圈检测，输出过圈量
//10.5    爆
static int8_t DMMotorMultiRoundDetect(DMMotorInstance * motor)
{
    static float position;
    static int8_t last_round_sign;
    static int8_t current_round_sign;
    int8_t diff_round;
    
    position = motor->measure.recv_position;
    last_round_sign = motor->measure.round_sign;
    current_round_sign=DMMotorSingleRoundSignDetect(position);

    //计算差值
    diff_round=current_round_sign-last_round_sign;
    //更新
    motor->measure.round_sign=current_round_sign;

    if(last_round_sign==0)//第一次收到，先设定当前所在的圈
    {
        motor->measure.round_sign = current_round_sign;
        return 0;
    }

    if(diff_round != 0)
    {
        if(diff_round>2)
        {
            diff_round-=4;
        }
        else if(diff_round<-2)
        {
            diff_round+=4;
        }
        return diff_round;
    }

    return 0;
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    measure->recv_last_position = measure->recv_position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->recv_position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);


    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];

    measure->recv_total_round+=DMMotorMultiRoundDetect(motor);
    measure->start_point_offset_count=(measure->recv_total_round>0)?(measure->recv_total_round+2)/4:(measure->recv_total_round-2)/4;

    measure->total_position = measure->recv_total_round * PI2 + (4*PI-12.5) * measure->start_point_offset_count + ((measure->recv_position + 12.5) - (measure->round_sign-1 )*6.25)*DM_COEF;

    measure->total_round=measure->recv_position/PI2;
    measure->last_position=measure->recv_position;
    measure->position=fmodf(measure->total_position,PI2);
    //measure->total_position = measure->recv_total_round * PI2 + (measure->position+12.5-(measure->round_sign-1)*6.25);
}

void DMMotorLostCallback(void *motor_ptr)
{
}

void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

float pdes,vdes,tdes,kpdes,kddes;

//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void DMMotorTask(DMMotorInstance *motor)
{
    float  pid_ref, set;
   //DM_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    //CANInstance *motor_can = motor->motor_can_instace;
    //uint16_t tmp;
    DMMotor_Send_s motor_send_mailbox;

        pid_ref = motor->pid_ref;
        
        set = pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;
       
        LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
        motor_send_mailbox.position_des = float_to_uint(pdes, DM_P_MIN, DM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(vdes, DM_V_MIN, DM_V_MAX, 12);
        motor_send_mailbox.torque_des = float_to_uint(tdes, DM_T_MIN, DM_T_MAX, 12);
        motor_send_mailbox.Kp = float_to_uint(kpdes, DM_KP_MIN, DM_KP_MAX, 12);
        motor_send_mailbox.Kd = float_to_uint(kddes, DM_KD_MIN, DM_KD_MAX, 12);

        if(motor->stop_flag == MOTOR_STOP)
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | ((motor_send_mailbox.Kp >> 8)&0xF));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        CANTransmit(motor->motor_can_instace, 1);

        
    
}

void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);

    }
}

void DMMotorControl()
{

}