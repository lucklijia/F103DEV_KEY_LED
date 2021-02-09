/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-29     RT-Thread    first version
 */

#define LED0_PIN    GET_PIN(A, 7)
#define LED1_PIN    GET_PIN(B, 0)
#define LED2_PIN    GET_PIN(B, 1)
#define KEY0_PIN    GET_PIN(A, 3)
#define KEY1_PIN    GET_PIN(A, 4)
#define MOTO_IN1    GET_PIN(C, 13)
#define MOTO_IN2    GET_PIN(C, 14)

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

//mpu6050 头文件及定义
#include "sensor_inven_mpu6xxx.h"

#define MPU6050_I2C_BUS_NAME          "i2c1"      //i2c总线设备名
#define MPU6050_ADDR                  0x68        //MPU6050地址，既定的
struct mpu6xxx_device *i2c_bus;     //6050控制句柄
struct mpu6xxx_3axes gyro1, accel1;     //陀螺仪，加速度结构体

////串口使用
//static rt_thread_t usart1_thread = RT_NULL;
//static void usart1_thread_entry(void* parameter);
//static rt_err_t uart1_input(rt_device_t dev, rt_size_t size);
//static rt_device_t serialuart1;
//char str[] = "hello RT-Thread111!\r\n";
//static struct rt_semaphore rx_sem1;



int main(void)
{
//    //串口初始化
//    serialuart1 = rt_device_find("uart1");
//    rt_device_open(serialuart1 , RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX );
//    rt_device_write(serialuart1 , 0, str, (sizeof(str) - 1));
//    char ch;


    int count = 1;
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);

    rt_pin_mode(KEY0_PIN, PIN_MODE_INPUT);
    rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT);

    rt_pin_mode(MOTO_IN1, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTO_IN2, PIN_MODE_OUTPUT);

    //初始化MPU6050
    i2c_bus = (struct mpu6xxx_device *)mpu6xxx_init(MPU6050_I2C_BUS_NAME, MPU6050_ADDR);   //初始化MPU6050，测量单位为角速度，加速度
    mpu6xxx_set_param(i2c_bus, MPU6XXX_ACCEL_RANGE, MPU6XXX_GYRO_RANGE_250DPS);  //陀螺仪范围配置
    mpu6xxx_set_param(i2c_bus, MPU6XXX_ACCEL_RANGE, MPU6XXX_ACCEL_RANGE_2G);     //加速度计
    mpu6xxx_set_param(i2c_bus, MPU6XXX_DLPF_CONFIG, MPU6XXX_DLPF_188HZ);         //低通滤波
    mpu6xxx_set_param(i2c_bus, MPU6XXX_SAMPLE_RATE, 500);                        //采样频率

    while (count++)
    {
        if(rt_pin_read(KEY0_PIN) == 0)
        {
            rt_pin_write(MOTO_IN1, 1);  //开窗
            rt_pin_write(LED0_PIN, 1);
        }
        else
        {
            rt_pin_write(MOTO_IN1, 0);
            rt_pin_write(LED0_PIN, 0);
        }

        if(rt_pin_read(KEY1_PIN) == 0)
        {
            rt_pin_write(MOTO_IN2, 1);  //关窗
            rt_pin_write(LED1_PIN, 1);
        }
        else
        {
            rt_pin_write(MOTO_IN2, 0);
            rt_pin_write(LED1_PIN, 0);
        }

//        if(rt_device_read(serialuart1 , -1, &ch, 1) != 1)
//        {
//            LOG_D("%s", ch);
//        }

//        LOG_D("Hello RT-Thread!");
//        rt_thread_mdelay(100);

//        rt_pin_write(LED0_PIN, !(count%3-0));
//        rt_pin_write(LED1_PIN, !(count%3-1));
//        rt_pin_write(LED2_PIN, !(count%3-2));

        mpu6xxx_get_gyro(i2c_bus, &gyro1);
        mpu6xxx_get_accel(i2c_bus, &accel1);


        LOG_D("gyroX:%d",gyro1.x/10);
        LOG_D("accelX:%3d, %3d, %3d",accel1.x, accel1.y, accel1.z);
        LOG_D(" ");
        LOG_D(" ");

        rt_thread_mdelay(500);
    }

    return RT_EOK;
}
