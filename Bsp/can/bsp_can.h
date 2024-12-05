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

typedef struct _
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;              // can句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    void (*can_module_callback)(CANInstance *); // 处理接收数据的回调函数
    void *id;                                   // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} CAN_Init_Config_s;

#pragma  pack()


void can_user_init(CAN_HandleTypeDef* hcan );
void set_motor_value(uint16_t tx_ID, int16_t v1, int16_t v2, int16_t v3, int16_t v4);


#endif
