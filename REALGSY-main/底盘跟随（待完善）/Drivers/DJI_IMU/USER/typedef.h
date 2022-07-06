/**
 * @file typedef.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 外部接口
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __TYPEDEFS_H
#define __TYPEDEFS_H

#include <stdint.h>

#pragma anon_unions

/************A板陀螺仪对外数据接口**********/
typedef struct
{
	float angle_degree[3];
	float Gyro[3];
	float last_angle_degree[3];
	float total[3];
	int16_t turnCount[3];
	float temp;
} mpu6500_Exportdata_t;
/*****************************************/


#endif /* __TYPEDEFS_H */
