#ifndef __UPHILL_AUTO_LIFT_H__
#define __UPHILL_AUTO_LIFT_H__

#include "bsp.h"

#define PITCH 0
#define ROLL 1
#define YAW 2

void AutoChassisAttitude_Lift_V2(float chassis_pitch_raw);	//�Զ�������̬	//pitch������Ϊǰ��	//ע�����lift_taskǰ��
void Chassis_Attitude_Angle_Convert(void);	//�ۺϵó�������̬

extern float Chassis_GYRO[3];

#endif
