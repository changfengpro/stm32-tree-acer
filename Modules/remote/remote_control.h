#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "stdint.h"

#pragma pack(1)

typedef union
{
    struct
    {
        int ch0 : 11;
        int ch1 : 11;
        int ch2 : 11;
        int ch3 : 11;
        // int ch4 : 11;
        uint8_t s1 : 2;
        uint8_t s2 : 2;
        // uint8_t reserved : 2;   //保留位，确保对齐
    }bits;
    uint8_t buffer[36];
} rc_union;


typedef struct 
{
    rc_union data;
    int ch0;
    int ch1;
    int ch2;
    int ch3;
    int ch4;
    uint8_t s1;
    uint8_t s2;
    uint8_t last_s1;
    uint8_t last_ch4;
} rc;

#pragma pack()




// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 获取按键操作
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)   // 开关向上时的值
#define RC_SW_MID ((uint16_t)3)  // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)2) // 开关向下时的值
// 三个判断开关状态的宏
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

void sbus_to_rc(const uint8_t *sbus_buf);



#endif