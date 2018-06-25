#include "bullet_rotate.h"
#include "math.h"
#include "pid.h"

BULLETROTATE_DATA BulletRotate_Data=BULLETROTATE_DATA_DEFAULT;	//取弹旋转电机数据

PID_GENERAL   PID_BulletRotate_Position=PID_BULLETROTATE_POSITION_DEFAULT;
PID_GENERAL   PID_BulletRotate_Speed=PID_BULLETROTATE_SPEED_DEFAULT;

extern u32 time_1ms_count;
extern ViceControlDataTypeDef ViceControlData;

void BulletRotate_Task(void)
{
//	BulletRotate_Data.tarP=(s16)(12*(RC_Ctl.rc.ch3-1024)/600.0);
	
	BulletRotate_Data.tarV=PID_General(BulletRotate_Data.tarP,BulletRotate_Data.fdbP,&PID_BulletRotate_Position);
	
	BulletRotate_Data.output=PID_General(BulletRotate_Data.tarV,BulletRotate_Data.fdbV,&PID_BulletRotate_Speed);
	
}


