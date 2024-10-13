#include "bmi088.h"
#include "stdint.h"
#include "main.h"


float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;   
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
float gyro[3];
float accel[3];
uint8_t i = 0;
uint8_t buf[6] = {0};
uint8_t buffer[8] = {0};
uint8_t pTxData;
uint8_t pRxData;
uint8_t pTxData_gyro;
uint8_t pRxData_gyro;

extern SPI_HandleTypeDef hspi1;

void BMI088_Accel_Init()
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);    //PA4置零，片选加速度计
    pTxData = (0x7E & 0x7F);
    // HAL_SPI_Transmit(&hspi1,&pTxData,1,1000);
    HAL_SPI_Transmit_DMA(&hspi1,&pTxData,1);
    HAL_Delay(1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX); //等待数据发送完成
    pTxData = 0xB6;
    // HAL_SPI_Transmit(&hspi1,&pTxData,1,1000);
    HAL_SPI_Transmit_DMA(&hspi1,&pTxData,1);
    HAL_Delay(1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX); //等待数据发送完成
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);    //PA4置1，取消片选加速度计

    //加速度计复位后默认是暂停模式，这9行代码，向地址0x7D处写入0x04值，使加速度计进入正常模式
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);    //PA4置零，片选加速度计
    pTxData = (0x7D & 0x7F);
    // HAL_SPI_Transmit(&hspi1,&pTxData,1,1000);
    HAL_SPI_Transmit_DMA(&hspi1,&pTxData,1);
    HAL_Delay(1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX); //等待数据发送完成
    pTxData = 0x04;
    // HAL_SPI_Transmit(&hspi1,&pTxData,1,1000);
    HAL_SPI_Transmit_DMA(&hspi1,&pTxData,1);
    HAL_Delay(1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX); //等待数据发送完成
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);    //PA4置1，取消片选加速度计
}

void BMI088_Accel_Read()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4置0，片选加速度计
    pTxData = (0x12 | 0x80);                                 //#0 是1 表示读
    // HAL_SPI_Transmit(&hspi1,&pTxData,1,1000);
    HAL_SPI_Transmit_DMA(&hspi1,&pTxData,1);
    HAL_Delay(1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX); //等待数据发送完成
    // HAL_SPI_Receive(&hspi1,&pRxData,1,1000);
    HAL_SPI_Receive_DMA(&hspi1,&pRxData,1);
    HAL_Delay(1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
    for(int i = 0; i < 6; i++)
    {
        // HAL_SPI_Receive(&hspi1,&pRxData,1,1000);    //Bit #16-23，寄存器0x12的值，然后是寄存器0x13、0x14、0x15、0x16、0x17的值
        HAL_SPI_Receive_DMA(&hspi1,&pRxData,1);
        HAL_Delay(1);
        while(HAL_SPI_GetState == HAL_SPI_STATE_BUSY_RX);
        buf[i] = pRxData;
    }
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //取消片选加速度计
    accel[0] = (int16_t)((buf[1] << 8) | buf[0]) * BMI088_ACCEL_SEN;
    accel[1] = (int16_t)((buf[3] << 8) | buf[2]) * BMI088_ACCEL_SEN;
    accel[2] = (int16_t)((buf[5] << 8) | buf[4]) * BMI088_ACCEL_SEN;
}

void BMI088_Gyro_Init()
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);     //片选陀螺仪
    pTxData_gyro = (0x14 & 0x7F);   //陀螺仪复位
    // HAL_SPI_Transmit(&hspi1,&pTxData_gyro,1,1000);
    HAL_SPI_Transmit_DMA(&hspi1,&pTxData_gyro,1);
    HAL_Delay(1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    pTxData_gyro = 0xB6;
    // HAL_SPI_Transmit(&hspi1,&pTxData_gyro,1,1000);
    HAL_SPI_Transmit_DMA(&hspi1,&pTxData_gyro,1);
    HAL_Delay(1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_Delay(30);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);     //取消片选陀螺仪
}

void BMI088_Gyro_Read()
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);     //片选陀螺仪
    pTxData_gyro = (0x00 | 0x80);
    // HAL_SPI_Transmit(&hspi1,&pTxData_gyro,1,1000);
    HAL_SPI_Transmit_DMA(&hspi1,&pTxData_gyro,1);
    HAL_Delay(1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    for(int i = 0; i < 8; i++)
    {
        // HAL_SPI_Receive(&hspi1,&pRxData_gyro,1,1000);
        HAL_SPI_Receive_DMA(&hspi1,&pRxData_gyro,1);
        HAL_Delay(1);
        while(HAL_SPI_GetState == HAL_SPI_STATE_BUSY_RX);
        buffer[i] = pRxData_gyro;
    }
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);     //取消片选陀螺仪
    // if(buf[0] == 0x0F)
    // {
        gyro[0] = ((int16_t)((buffer[3] << 8) | buffer[2])) * BMI088_GYRO_SEN; 
        gyro[1] = ((int16_t)((buffer[5] << 8) | buffer[4])) * BMI088_GYRO_SEN;
        gyro[2] = ((int16_t)((buffer[7] << 8) | buffer[6])) * BMI088_GYRO_SEN;
    // }
    
}