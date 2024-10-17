#include "bmi088.h"
#include "stdint.h"
#include "main.h"
#include "cmsis_os2.h"

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t BMI088_read_write_byte(uint8_t txdata);


#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_ACCEL_NS_L HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define BMI088_ACCEL_NS_H HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)

#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L;                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H;                       \
    }

#define BMI088_TEMP_M 0x22



float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;   
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
float gyro[3];
float accel[3];
float Temperature;
float temp;
uint8_t i = 0;
uint8_t buf[6] = {0};
uint8_t buffer[8] = {0};
uint8_t pTxData;
uint8_t pRxData;
uint8_t pTxData_gyro;
uint8_t pRxData_gyro;
uint8_t pTxData_Temperature;
uint8_t pRxData_Temperature;
int16_t bmi088_raw_temp;
uint8_t buf_temp[8] = {0};

extern SPI_HandleTypeDef hspi1;

void BMI088_Accel_Init()
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);    //PA4置零，片选加速度计
    pTxData = (0x7E & 0x7F);
    HAL_SPI_Transmit(&hspi1,&pTxData,1,1000);
    //HAL_SPI_Transmit_DMA(&hspi1,&pTxData,1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX); //等待数据发送完成
    pTxData = 0xB6;
    HAL_SPI_Transmit(&hspi1,&pTxData,1,1000);
    // HAL_SPI_Transmit_DMA(&hspi1,&pTxData,1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX); //等待数据发送完成
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);    //PA4置1，取消片选加速度计
    //加速度计复位后默认是暂停模式，这9行代码，向地址0x7D处写入0x04值，使加速度计进入正常模式
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);    //PA4置零，片选加速度计
    pTxData = (0x7D & 0x7F);
    HAL_SPI_Transmit(&hspi1,&pTxData,1,1000);
    // HAL_SPI_Transmit_DMA(&hspi1,&pTxData,1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX); //等待数据发送完成
    pTxData = 0x04;
    // HAL_SPI_Transmit(&hspi1,&pTxData,1,1000);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX); //等待数据发送完成
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);    //PA4置1，取消片选加速度计
}

void BMI088_Accel_Read()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4置0，片选加速度计
    pTxData = (0x12 | 0x80);                                 //#0 是1 表示读
    HAL_SPI_Transmit(&hspi1,&pTxData,1,1000);
    // HAL_SPI_Transmit_DMA(&hspi1,&pTxData,1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX); //等待数据发送完成
    HAL_SPI_Receive(&hspi1,&pRxData,1,1000);
    // HAL_SPI_Receive_DMA(&hspi1,&pRxData,1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
    for(int i = 0; i < 6; i++)
    {
        HAL_SPI_Receive(&hspi1,&pRxData,1,1000);    //Bit #16-23，寄存器0x12的值，然后是寄存器0x13、0x14、0x15、0x16、0x17的值
        // HAL_SPI_Receive_DMA(&hspi1,&pRxData,1);
        while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
        buf[i] = pRxData;
    }
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //取消片选加速度计
    accel[0] = (int16_t)((buf[1] << 8) | buf[0]) * BMI088_ACCEL_SEN;
    accel[1] = (int16_t)((buf[3] << 8) | buf[2]) * BMI088_ACCEL_SEN;
    accel[2] = (int16_t)((buf[5] << 8) | buf[4]) * BMI088_ACCEL_SEN;
    
    // BMI088_accel_read_muli_reg(BMI088_TEMP_M, buffer, 2);
    // bmi088_raw_temp = (int16_t)((buffer[0] << 3) | (buffer[1] >> 5));
    //     if (bmi088_raw_temp > 1023)
    //         bmi088_raw_temp -= 2048;
    //   Temperature  = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    BMI088_Temper_read(buf_temp,2);
}

void BMI088_Gyro_Init()
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);     //片选陀螺仪
    pTxData_gyro = (0x14 & 0x7F);   //陀螺仪复位
    HAL_SPI_Transmit(&hspi1,&pTxData_gyro,1,1000);
    // HAL_SPI_Transmit_DMA(&hspi1,&pTxData_gyro,1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    pTxData_gyro = 0xB6;
    HAL_SPI_Transmit(&hspi1,&pTxData_gyro,1,1000);
    // HAL_SPI_Transmit_DMA(&hspi1,&pTxData_gyro,1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    // osDelay(30);
    HAL_Delay(30);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);     //取消片选陀螺仪
}

void BMI088_Gyro_Read()
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);     //片选陀螺仪
    pTxData_gyro = (0x00 | 0x80);
    HAL_SPI_Transmit(&hspi1,&pTxData_gyro,1,1000);
    // HAL_SPI_Transmit_DMA(&hspi1,&pTxData_gyro,1);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    for(int i = 0; i < 8; i++)
    {
        HAL_SPI_Receive(&hspi1,&pRxData_gyro,1,1000);
        // HAL_SPI_Receive_DMA(&hspi1,&pRxData_gyro,1);
        while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
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

void BMI088_Temper_read(uint8_t tempbuf[], uint8_t len)
{   
    uint8_t tempdata[2],index=0;
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);    //PA4置零，片选加速度计
    pTxData_Temperature = (0x22 | 0x80); //读温度寄存器
    HAL_SPI_Transmit(&hspi1,&pTxData_Temperature,1,1000);
    HAL_SPI_Receive(&hspi1,&pRxData,1,1000); //加速度计第一个接收值无用
    // for(int i = 0; i < 2; i++)
    // {
    //     HAL_SPI_Receive(&hspi1,&pRxData_Temperature,1,1000);
    //     buf[i] = pRxData_Temperature;
    //     // buf[i] = BMI088_read_write_byte(0x55);
    // }

    // while(len != 0)
    // {
    //     *tempbuf = BMI088_read_write_byte(0x55);
    //     tempbuf++;
    //     len--;
    // }

    while(len != 0)
    {
        tempdata[index] = BMI088_read_write_byte(0x55);  //0x55 是个没用的值，目的是为了使用recieve功能
        index++;
        len--;
    }

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);    //PA4置1，取消片选加速度计
    temp = (int16_t)((tempdata[0] << 3) | (tempdata[1] >> 5));
    if(temp > 1023)
    {
        temp -= 2048;
    }
    // Temperature = temp * 0.125f / buf[1] + 23.0f;
    Temperature = temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
