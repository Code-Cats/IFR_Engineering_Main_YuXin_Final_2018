#ifndef __TRAILER_H_
#define __TRAILER_H_

#include "bsp.h"

void Trailer_Task(u8 sensor_data);	//���뺯��Ϊ��ഫ��������

void Trailer_Turn_Task(s16* turn_tarv,u8 trailer_statu);	//ת������
#endif
