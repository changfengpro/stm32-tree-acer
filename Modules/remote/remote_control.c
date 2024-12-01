#include "remote_control.h"


uint8_t buffer[36];
rc RC_ctrl;


/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
void sbus_to_rc(const uint8_t *sbus_buf)
{
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
}
