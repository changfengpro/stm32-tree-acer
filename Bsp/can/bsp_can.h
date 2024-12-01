#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"
#include "stdint.h"


#define MOTOR_3508_FEEDBACK_ID_BASE 	0x200
#define MOTOR_CHASSIS_MAX_NUM     	4
#define MOTOR_GIMBAL_MAX_NUM     	2
#define MOTOR_MAX_NUM         7

#pragma  pack(1)

typedef struct
{
    uint16_t can_id;
    int16_t  set_current;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
}moto_info_t;

#pragma  pack()


void can_user_init(CAN_HandleTypeDef* hcan );
void set_motor_value(uint16_t tx_ID, int16_t v1, int16_t v2, int16_t v3, int16_t v4);


#endif
