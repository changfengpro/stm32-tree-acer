/*
 * remote_control.h
 *
 *  Created on: Jul 21, 2024
 *      Author: auroranebulas
 */

#ifndef REMOTE_REMOTE_CONTROL_H_
#define REMOTE_REMOTE_CONTROL_H_


#include "main.h"
#include "usart.h"

//用于遥控器数据读取，遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

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



// typedef struct
// {
//     struct
//     {
//         int16_t rocker_l_; // 左水平
//         int16_t rocker_l1; // 左竖直
//         int16_t rocker_r_; // 右水平
//         int16_t rocker_r1; // 右竖直
//         int16_t dial;      // 侧边拨轮

//         uint8_t switch_left;  // 左侧开关
//         uint8_t switch_right; // 右侧开关
//     } rc;
//     struct
//     {
//         int16_t x;
//         int16_t y;
//         uint8_t press_l;
//         uint8_t press_r;
//     } mouse;

//     Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

//     uint8_t key_count[3][16];
// } RC_ctrl_t;


// /* ----------------------- Data Struct ------------------------------------- */
// // 待测试的位域结构体,可以极大提升解析速度
// typedef union
// {
//     struct // 用于访问键盘状态
//     {
//         uint16_t w : 1;
//         uint16_t s : 1;
//         uint16_t d : 1;
//         uint16_t a : 1;
//         uint16_t shift : 1;
//         uint16_t ctrl : 1;
//         uint16_t q : 1;
//         uint16_t e : 1;
//         uint16_t r : 1;
//         uint16_t f : 1;
//         uint16_t g : 1;
//         uint16_t z : 1;
//         uint16_t x : 1;
//         uint16_t c : 1;
//         uint16_t v : 1;
//         uint16_t b : 1;
//     };
//     uint16_t keys; // 用于memcpy而不需要进行强制类型转换
// } Key_t;


#endif /* REMOTE_REMOTE_CONTROL_H_ */
