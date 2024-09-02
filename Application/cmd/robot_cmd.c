/*
 * robot_cmd.c
 *
 *  Created on: Aug 26, 2024
 *      Author: auroranebulas
 */


#include "robot_cmd.h"
#include "usart.h"
#include "memory.h"
#include  "remote_control.h"
#include "robot_def.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

#define CHASSIS_COEF 0.25f
#define COREXY_COEF 6.0f


static Chassis_Ctrl_Cmd_s chassis_cmd_send;        //发送给底盘的应用消息 
static uint32_t count_2;
Chassis_Ctrl_Cmd_s chassis_cmd_recv;              //底盘接收到的控制命令
Corexy_Ctrl_Cmd_s Corexy_cmd_recv;             //corexy结构接收到的控制指令




uint8_t buffer[36];
rc RC_ctrl;
// rc_union RC_data;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
   if(huart == &huart3)
  {
//    HAL_UART_AbortReceive(huart);
    //HAL_UARTEx_ReceiveToIdle_IT(&huart3,buffer,18);
//     HAL_UART_Receive_DMA(&huart1,buffer,18);

    RC_ctrl.ch0 = (buffer[0] | buffer[1] << 8) & 0x07FF;
    RC_ctrl.ch1 = (buffer[1] >> 3 |buffer[2] << 5) & 0x07FF;
    RC_ctrl.ch2 = (buffer[2] >> 6 | buffer[3] << 2 | buffer[4] << 10) & 0x07FF;
    RC_ctrl.ch3 = (buffer [4] >> 1 | buffer[5] << 7) & 0x07FF;
    RC_ctrl.ch4 = (buffer[16] | (buffer[17] << 8));
    RC_ctrl.s1  = (buffer[5] >> 4 & 0x000C) >> 2;
    RC_ctrl.s2  = (buffer[5] >> 4 & 0x0003);



    RC_ctrl.ch0 -= RC_CH_VALUE_OFFSET;
    RC_ctrl.ch1 -= RC_CH_VALUE_OFFSET;
    RC_ctrl.ch2 -= RC_CH_VALUE_OFFSET;
    RC_ctrl.ch3 -= RC_CH_VALUE_OFFSET;
    RC_ctrl.ch4 -= RC_CH_VALUE_OFFSET;

    HAL_UARTEx_ReceiveToIdle_IT(&huart3,buffer,36);
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart1,buffer,36);
  }
}


/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
void RemoteControlSet()
{

  if(switch_is_up(RC_ctrl.s2))          //右侧开关上，底盘控制
  {
    // chassis_cmd_recv.vx = -RC_ctrl.ch0 * 0.68f;
    // chassis_cmd_recv.vy = -RC_ctrl.ch1 * 0.68f;
    // chassis_cmd_recv.wz = -RC_ctrl.ch2 * 0.68f;
    chassis_cmd_recv.vx = -RC_ctrl.ch0 * CHASSIS_COEF;
    chassis_cmd_recv.vy = -RC_ctrl.ch1 * CHASSIS_COEF;
    chassis_cmd_recv.wz = -RC_ctrl.ch2 * CHASSIS_COEF;
  }

  if(switch_is_mid(RC_ctrl.s2))         //右侧开关中，xyz方向控制
  {
    Corexy_cmd_recv.vz = RC_ctrl.ch1 * COREXY_COEF;
    Corexy_cmd_recv.vx = RC_ctrl.ch2 * COREXY_COEF;
    Corexy_cmd_recv.vy = RC_ctrl.ch3 * COREXY_COEF;
  }

  if(switch_is_mid(RC_ctrl.s2) && switch_is_up(RC_ctrl.s1))        //左侧开关上，右侧开关中夹爪夹紧
  {
     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 200);
  }

  if(switch_is_mid(RC_ctrl.s1) && switch_is_mid(RC_ctrl.s2))       //左侧开关中，右侧开关中，夹爪松开
  {
     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 150);
  }
}


/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
void RobotCMDTask()
{
  RemoteControlSet();
}