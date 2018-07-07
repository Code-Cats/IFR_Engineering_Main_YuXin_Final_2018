#ifndef __TRAILER_H_
#define __TRAILER_H_

#include "bsp.h"

void Trailer_Task(u8 sensor_data);	//传入函数为测距传感器数据

void Trailer_Turn_Task(s16* turn_tarv,u8 trailer_statu);	//转向任务
#endif
