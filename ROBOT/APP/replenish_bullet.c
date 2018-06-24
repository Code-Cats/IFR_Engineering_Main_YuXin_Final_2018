#include "replenish_bullet.h"

extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;

extern u8 Trailer_statu;

extern u8 valve_fdbstate[6];	//��ŷ����跴��

u8 Replenish_Bullet_Statu=0;
void Replenish_Bullet_Task(u8 key_r_state)
{
	static u8 key_r_state_last;
	static u8 replenish_bullet_statu_last=0;
	if(Trailer_statu==0)	//���ϳ�����
	{
		if(key_r_state_last==0&&key_r_state==1)	//����R��
		{
			Replenish_Bullet_Statu=!Replenish_Bullet_Statu;
		}
	}
	
	if(Replenish_Bullet_Statu==1)
	{
		ViceControlData.valve[VALVE_ISLAND]=0;	//����ʱ�������ջ�
		if(SetCheck_GripBulletLift(0)==1)
		{
			ViceControlData.valve[VALVE_BULLET_STORAGE]=1;	//�������������
		}
//		if(RC_Ctl.rc.ch3-1024>80)
		if(KeyBoardData[KEY_CTRL].value==1)
		{
			ViceControlData.servo[SERVO_BULLET_DOWN]=1;
		}
	}
	else
	{
		if(valve_fdbstate[VALVE_BULLET_STORAGE]==0)	//����Ѿ��ջ�����
		{
			SetCheck_GripBulletLift(1);
		}
		
		ViceControlData.servo[SERVO_BULLET_DOWN]=0;		//���ִ�д����������Զ�ȡ���ļ�����ʱ���ϵδ�������
		ViceControlData.valve[VALVE_BULLET_STORAGE]=0;	//�����������ջ�
	}
	
	if(replenish_bullet_statu_last==0&&Replenish_Bullet_Statu==1)
	{
		SetCheck_FrontLift(1);
		SetCheck_BackLift(1);
	}
	else if(replenish_bullet_statu_last==1&&Replenish_Bullet_Statu==0)
	{
		SetCheck_FrontLift(0);
		SetCheck_BackLift(0);
	}
	
	replenish_bullet_statu_last=Replenish_Bullet_Statu;
	key_r_state_last=key_r_state;
}


