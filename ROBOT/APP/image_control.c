#include "main.h"
#include "image_control.h"

#define IMAGE_CUTLIST_REPLENISHBULLET	0	//����
#define IMAGE_CUTLIST_TRAILER	1 //�ϳ�
#define IMAGE_CUTLIST_CHASSIS	2	//����
#define IMAGE_CUTLIST_TAKEBULLET	3	//ץ��
const u8 Image_CutList[4][2]={\
{0,0},\
{1,0},\
{0,1},\
{1,1}\
};	//�ֱ�Ϊ������ �ϳ� ���� ץ��

extern u32 time_1ms_count;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;

#define STEER_IMAGE_INIT	1700
#define STEER_IMAGE_REVERSAL	650

#define IMAGE_START_DELAY	(1000*5)	//5s��ʼ

void Screen_Start(void)	//��Ļ�����л���AV�ŵ�
{
	if(KeyBoardData[KEY_G].value!=1)
	{
		if(time_1ms_count<IMAGE_START_DELAY)	//5s��ʼ
		{
			IMAGE_START=PWM_IO_ON;
		}
		else if(time_1ms_count>IMAGE_START_DELAY&&time_1ms_count<IMAGE_START_DELAY+1000)
		{
			IMAGE_START=PWM_IO_OFF;
		}
		else
		{
			IMAGE_START=PWM_IO_ON;
		}
	}
	else
	{
		IMAGE_START=PWM_IO_OFF;
	}

}


extern u8 Replenish_Bullet_Statu;	//����״̬λ
extern u8 Trailer_statu;
//u8 av_cut=0;
u8 Steer_Image_state=0;
u16 steer_image=STEER_IMAGE_REVERSAL;
void Image_Cut_Task(void)	//����ͷ�л������
{
	static u8 key_c_last=0;
	static u8 key_b_last=0;
	static u8 trailer_statu_last=0;
	static u8 replenish_bullet_statu_last=0;
	static WorkState_e State_Record=CHECK_STATE;	
//	t_AV_CUT=av_cut*20000;
	if(trailer_statu_last==0&&Trailer_statu==1)	//�л����ϳ�״̬
	{
		Steer_Image_state=1;
		Image_Cut_Screen(IMAGE_CUTLIST_TRAILER);
	}
	else if(trailer_statu_last==1&&Trailer_statu==0)	//�˳��ϳ�
	{
		Steer_Image_state=0;
		Image_Cut_Screen(IMAGE_CUTLIST_TRAILER);
	}
	
	if(replenish_bullet_statu_last==0&&Replenish_Bullet_Statu==1)	//���벹��
	{
		Steer_Image_state=1;
		Image_Cut_Screen(IMAGE_CUTLIST_REPLENISHBULLET);
	}
	else if(replenish_bullet_statu_last==1&&Replenish_Bullet_Statu==0)	//�˳�����
	{
		Steer_Image_state=0;
		Image_Cut_Screen(IMAGE_CUTLIST_REPLENISHBULLET);
	}
	
	if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)	//ȡ��ģʽ
	{
		Steer_Image_state=1;
		Image_Cut_Screen(IMAGE_CUTLIST_TAKEBULLET);
	}
	else if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)	//�˳�ȡ��ģʽ
	{
		Steer_Image_state=0;
		Image_Cut_Screen(IMAGE_CUTLIST_CHASSIS);	//��Ϊ�˳�ȡ��ģʽ��������µ�������ֱ���л�������
	}
	
	if(GetWorkState()==ASCEND_STATE||GetWorkState()==DESCEND_STATE||GetWorkState()==SEMI_ASCEND_STATE||GetWorkState()==SEMI_DESCEND_STATE)
	{
		Steer_Image_state=1;
		Image_Cut_Screen(IMAGE_CUTLIST_CHASSIS);
	}
	
//	if(Trailer_statu==0&&Replenish_Bullet_Statu==0&&GetWorkState()==NORMAL_STATE)	//��λ
	if(State_Record!=NORMAL_STATE&&GetWorkState()==NORMAL_STATE)	//��λ
	{
		Steer_Image_state=0;
		Image_Cut_Screen(IMAGE_CUTLIST_CHASSIS);	//�ճ��л�����
	}
	
	if(key_c_last==0&&KeyBoardData[KEY_C].value==1)
	{
		Steer_Image_state=!Steer_Image_state;
		if(GetWorkState()==TAKEBULLET_STATE)
		{
			Image_Cut_Screen(IMAGE_CUTLIST_TAKEBULLET);	//ͼ���л�ʱ����һ���л�����
		}
		else if(Replenish_Bullet_Statu==1)
		{
			Image_Cut_Screen(IMAGE_CUTLIST_REPLENISHBULLET);
		}
		else if(Trailer_statu==1)	//��ȡ��ģʽ
		{
			Image_Cut_Screen(IMAGE_CUTLIST_TRAILER);
		}
		else	//���������
		{
			Image_Cut_Screen(IMAGE_CUTLIST_CHASSIS);	//ͼ���л�ʱ����һ���л�����
		}
	}
	
	if(GetWorkState()==NORMAL_STATE&&key_b_last==0&&KeyBoardData[KEY_B].value==1)
	{
		ViceControlData.valve[VALVE_ISLAND]=!ViceControlData.valve[VALVE_ISLAND];
	}
	
	
	STEER_IMAGE=STEER_IMAGE_INIT-Steer_Image_state*(STEER_IMAGE_INIT-STEER_IMAGE_REVERSAL);
	
	key_c_last=KeyBoardData[KEY_C].value;
	key_b_last=KeyBoardData[KEY_B].value;
	trailer_statu_last=Trailer_statu;
	replenish_bullet_statu_last=Replenish_Bullet_Statu;
	
	State_Record=GetWorkState();
}


u8 t_state_re=0;
void Image_Cut_Screen(u8 state)
{
	Chassis_Control_Move_Reverse(Steer_Image_state,state);
	t_state_re=state;
	switch(state)
	{
		case IMAGE_CUTLIST_REPLENISHBULLET:
		{
			ViceControlData.image_cut[0]=Image_CutList[IMAGE_CUTLIST_REPLENISHBULLET][0];
			ViceControlData.image_cut[1]=Image_CutList[IMAGE_CUTLIST_REPLENISHBULLET][1];
			break;
		}
		case IMAGE_CUTLIST_TRAILER:
		{
			ViceControlData.image_cut[0]=Image_CutList[IMAGE_CUTLIST_TRAILER][0];
			ViceControlData.image_cut[1]=Image_CutList[IMAGE_CUTLIST_TRAILER][1];
			break;
		}
		case IMAGE_CUTLIST_CHASSIS:
		{
			ViceControlData.image_cut[0]=Image_CutList[IMAGE_CUTLIST_CHASSIS][0];
			ViceControlData.image_cut[1]=Image_CutList[IMAGE_CUTLIST_CHASSIS][1];
			break;
		}
		case IMAGE_CUTLIST_TAKEBULLET:
		{
			ViceControlData.image_cut[0]=Image_CutList[IMAGE_CUTLIST_TAKEBULLET][0];
			ViceControlData.image_cut[1]=Image_CutList[IMAGE_CUTLIST_TAKEBULLET][1];
			break;
		}
	}
	
}

extern s8 Chassis_Control_Heading;	//1����Ĭ�ϣ�-1������
void Chassis_Control_Move_Reverse(u8 image_steer,u8 image_cut_state)	//����ǰ����������
{
	if(image_steer==0)
	{
		Chassis_Control_Heading=1;	//����
	}
	else if(image_steer!=0&&(image_cut_state==IMAGE_CUTLIST_TRAILER||image_cut_state==IMAGE_CUTLIST_REPLENISHBULLET))	//�������ϳ�����
	{
		Chassis_Control_Heading=-1;	//����
	}
	else if(image_steer!=0&&(image_cut_state==IMAGE_CUTLIST_TAKEBULLET||image_cut_state==IMAGE_CUTLIST_CHASSIS))
	{
		Chassis_Control_Heading=1;	//����
	}
}
