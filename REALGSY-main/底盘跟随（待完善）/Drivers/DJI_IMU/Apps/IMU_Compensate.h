/**
 * @file IMU_Compensate.h
 * @author Miraggio (w1159904119@gmail)
 * @brief ��������Ư����
 * @version 0.1
 * @date 2021-04-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _IMU_COMPENSATE_H_
#define _IMU_COMPENSATE_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "PID.h"
#include "tim.h"
#include "typedef.h"

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500�����¶ȵ�����TIM������ֵ������PWM���Ϊ?MPU6500_TEMP_PWM_MAX?-?1
#define GYRO_CONST_MAX_TEMP 35.0f
//�����ǿ���У׼��ʱ��
#define GYRO_OFFSET_START_TIME 500 //ԭʼʱ��500
#define GYRO_OFFSET_KP 0.0003f //����������Ե���������У׼�ٶȣ�Խ��������У׼�仯Խ�죬����������

#define imuTempPidInit     \
    {                      \
        0,                 \
            0,             \
            0,             \
            0,             \
            0,             \
            1600.0f,       \
            0.2f,          \
            0.0f,          \
            0,             \
            0,             \
            0,             \
            0,             \
            4500.0f,       \
            0,             \
            4400.0f,       \
            &CLOUD_Position_PID, \
    }

#define IMU_CompensateFUNInit        \
    {                                \
        &Preserve_temp,              \
            &IMU_GetData_Compensate, \
    }

typedef struct
{
    void (*Preserve_temp)(float Real_temp);
    void (*IMU_GetData_Compensate)(void);
} IMU_CompensateFUN_t;

	
extern mpu6500_Exportdata_t mpu6500_Exportdata;
extern IMU_CompensateFUN_t IMU_CompensateFUN;

#endif /*_IMU_COMPENSATE_H_*/
