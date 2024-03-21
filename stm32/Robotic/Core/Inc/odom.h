/*
 * odom.h
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#ifndef INC_ODOM_H_
#define INC_ODOM_H_

#include "struct_types.h"
#include <math.h>
#include <stm32f4xx.h>

#define PPR            2184 	   //4*546 inc
#define WHEEL_DIAMETER 70
#define WHEEL_DISTANCE 150
#define INC_MM         0.10069207 // (WHEEL_DIAMETER*PI)/PPR
#define INC_RAD        0.00143845 // INC_MM/WHEEL_DISTANCE
#define ODOM_TIME      20         //ms

void Encoders_Init();
sOdom_t* Read_Encoders();
void Reset_Encoders(sOdom_t* new_odom);

#endif /* INC_ODOM_H_ */
