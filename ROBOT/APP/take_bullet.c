#include "take_bullet.h"

//#define VALVE_ISLAND 0		//��ŷ�����λ����
//#define VALVE_BULLET_PROTRACT 1	//ǰ��
//#define VALVE_BULLET_CLAMP 2	//�н�
//#define VALVE_BULLET_STORAGE 3	//��ҩ�ղ���
//#define VALVE_TRAILER 5	//�ϳ�

TakeBulletState_e TakeBulletState=BULLET_ACQUIRE;	//(�Զ�)ȡ����־λ


#define STEER_UP_L_INIT 500//1210	//2500
#define STEER_UP_R_INIT 2500//1950	//500
#define STEER_DOWN_L_INIT 1600//1650	//1000С-��
#define STEER_DOWN_R_INIT 1430//1550	//2180С-��
//2018.5.4
#define STEER_UP_L_REVERSAL 1900//2500	//2500
#define STEER_UP_R_REVERSAL 1100//630	//500
#define STEER_DOWN_L_REVERSAL 1100	//1000
#define STEER_DOWN_R_REVERSAL 1930	//2180
u16 Steer_Send[4]={STEER_UP_L_INIT,STEER_UP_R_INIT,STEER_DOWN_L_INIT,STEER_DOWN_R_INIT};

u8 valve_fdbstate[6]={0};	//��¼�Ƿ�����ķ�����־
u8 servo_fdbstate[2]={0};
const u32 valve_GOODdelay[6]={300,1200,700,1000,1000,1000};	//�����룬��ʱ����
const u32 valve_POORdelay[6]={300,1200,700,1000,1000,1000};	//�����룬��ʱ����
const u32 servo_GOODdelay[2]={2000,1000};	//��ʱ����	//��һ��Ϊ2000�ǽ��ӵ����µ���ʱҲ�ӽ�ȥ�ˣ���Ϊ�����ת���ӵ��������������һ���
const u32 servo_POORdelay[2]={1000,1000};	//��ʱ����


extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;
extern u32 time_1ms_count;
//extern BULLETLIFT_MOTOR_DATA bulletlift_Motor_Data[2];	//������������

u8 auto_takebullet_statu=0;
void TakeBullet_Control_Center(void)
{
	static u8 swicth_Last_state=0;	//�Ҳ���
	static u8 auto_takebullet_statu_last=0;
	
	static u8 valve_last[6]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	static u8 servo_last[2]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	
	static u32 valve_startGOOD_time[6]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startGOOD_time[2]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 valve_startPOOR_time[6]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startPOOR_time[2]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	
	
	if(GetWorkState()==TAKEBULLET_STATE)	//5.9����//��һ��--��//ȡ��������DOWN-MID��ǰ�����-�н�һ�׸�DOWN-MID-->DOWN-DOWN;�����ת��DOWN-MID-->DOWN-UP
	{
		if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)
		{
//			if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
//			{
					if(RC_Ctl.rc.ch3-1024>80)	/////////////////////////////�޸Ĳ���ģʽʱ��Ҫ�޸�
					{
						auto_takebullet_statu=1;
						TakeBulletState=BULLET_ACQUIRE;
					}
					else if(RC_Ctl.rc.ch3-1024<-80)
					{
						auto_takebullet_statu=0;
						TakeBulletState=BULLET_ACQUIRE;
					}
					if(auto_takebullet_statu_last!=auto_takebullet_statu)
					{
						TakeBulletState=BULLET_ACQUIRE;
					}
//			}
			else if(swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)
			{
			}
			
			if(RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)	//��仰����bullet_lift.c��
			{
				
			}

		}

		auto_takebullet_statu_last=auto_takebullet_statu;
		
		if(auto_takebullet_statu==1)	//�Զ�ȡ��
		{
			switch(TakeBulletState)	//�Զ�ȡ������
			{
				case BULLET_ACQUIRE:	//ǰ�졢�н���̧����	��֮Ϊ��ù���
				{

					break;
				}
				case BULLET_POUROUT:	//������б�������ת	��֮Ϊ��������
				{

					break;
				}
				case BULLET_THROWOUT:	//������ء�����̧�𡢼н��ɿ�	��֮Ϊ�������
				{

					break;
				}
			}
		}
		else	//���ȡ��״̬����0���ͻص�����״̬
		{

		}
	}
	else	//GetWorkState()==TAKEBULLET_STATE&&RC_Ctl.rc.switch_left==RC_SWITCH_DOWN��else
	{
//		if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)
//		SetCheck_TakeBullet_TakeBack();	//����������״̬�ı���
	}



	
//	if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)	//���ǵ���λ��
//	{
//		Steer_Send[UP_L]=STEER_UP_L_REVERSAL;
//		Steer_Send[UP_R]=STEER_UP_R_REVERSAL;
//		Steer_Send[DOWN_L]=STEER_DOWN_L_INIT;
//		Steer_Send[DOWN_R]=STEER_DOWN_R_INIT;
//	}
//	else
//	{
//		Steer_Send[UP_L]=STEER_UP_L_INIT;
//		Steer_Send[UP_R]=STEER_UP_R_INIT;
//		Steer_Send[DOWN_L]=STEER_DOWN_L_REVERSAL;
//		Steer_Send[DOWN_R]=STEER_DOWN_R_REVERSAL;
//	}
	
//	if(auto_takebullet_statu==1)	//1/4�Զ�����
//	{
//		ViceControlData.valve[VALVE_BULLET_PROTRACT]=1;
//		if(valve_fdbstate[VALVE_BULLET_PROTRACT]==1)
//		{
//			ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
//		}
//	}
//	else
//	{
//		ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
//		if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)
//		{
//			ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;
//		}
//	}
	
	if(ViceControlData.servo[SERVO_BULLET_UP]==1)
	{
		Steer_Send[UP_L]=STEER_UP_L_REVERSAL;
		Steer_Send[UP_R]=STEER_UP_R_REVERSAL;
	}
	else
	{
		Steer_Send[UP_L]=STEER_UP_L_INIT;
		Steer_Send[UP_R]=STEER_UP_R_INIT;
	}
	
	
	if(ViceControlData.servo[SERVO_BULLET_DOWN]==1)
	{
		Steer_Send[DOWN_L]=STEER_DOWN_L_REVERSAL;
		Steer_Send[DOWN_R]=STEER_DOWN_R_REVERSAL;
	}
	else
	{
		Steer_Send[DOWN_L]=STEER_DOWN_L_INIT;
		Steer_Send[DOWN_R]=STEER_DOWN_R_INIT;
	}
	
	/******************************************************************
��������forΪ���������ⷽ��
�ֱ�Ϊ��
1.�������½��صĴ���ʱ���¼
2.���ݴ���ʱ������ݷ���ֵ����
3.���ݵ���
******************************************************************/

		for(int i=0;i<6;i++)	//����ʱ���
		{
			if(valve_last[i]==0&&ViceControlData.valve[i]==1)	//�������
			{
				valve_startGOOD_time[i]=time_1ms_count;
			}
			else if(valve_last[i]==1&&ViceControlData.valve[i]==0)//�ջش���
			{
				valve_startPOOR_time[i]=time_1ms_count;
			}
			
			if(i<2)
			{
				if(servo_last[i]==0&&ViceControlData.servo[i]==1)
				{
					servo_startGOOD_time[i]=time_1ms_count;
				}
				else if(servo_last[i]==1&&ViceControlData.servo[i]==0)
				{
					servo_startPOOR_time[i]=time_1ms_count;
				}
			}
		}
		
		for(int i=0;i<6;i++)	//��������λ
		{
			if(ViceControlData.valve[i]==1&&time_1ms_count-valve_startGOOD_time[i]>valve_GOODdelay[i])	//����ֵΪ��������λ��ʱ����ͳһ��Ϊ1000ms
			{
				valve_fdbstate[i]=1;
			}
			else if(ViceControlData.valve[i]==0&&time_1ms_count-valve_startPOOR_time[i]>valve_POORdelay[i])	//����ֵΪ�ջ�����λ��ʱ����ͳһ��Ϊ1000ms
			{
				valve_fdbstate[i]=0;
			}
			
			if(i<2)
			{
				if(ViceControlData.servo[i]==1&&time_1ms_count-servo_startGOOD_time[i]>servo_GOODdelay[i])	//����ֵΪ��������λ��ʱ����ͳһ��Ϊ1000ms
				{
					servo_fdbstate[i]=1;
				}
				else if(ViceControlData.servo[i]==0&&time_1ms_count-servo_startPOOR_time[i]>servo_POORdelay[i])	//����ֵΪ�ջ�����λ��ʱ����ͳһ��Ϊ1000ms
				{
					servo_fdbstate[i]=0;
				}
			}
		}
		
		for(int i=0;i<6;i++)	//������
		{
			valve_last[i]=ViceControlData.valve[i];
			if(i<2)	servo_last[i]=ViceControlData.servo[i];
		}
////////////////////////////////////////////////////////////////////////////////////////////�������־
	swicth_Last_state=RC_Ctl.rc.switch_right;	//����
	
	
	PWM3_1=Steer_Send[UP_L];
	PWM3_2=Steer_Send[UP_R];
	PWM3_3=Steer_Send[DOWN_L];
	PWM3_4=Steer_Send[DOWN_R];
}




/**********************�������汾********************
u8 SetCheck_TakeBullet_TakeBack_statu=0;	//�г�ȡ������ִ�б�־λ
void SetCheck_TakeBullet_TakeBack(void)	//�г�ȡ��������λ����
{
	if(SetCheck_TakeBullet_TakeBack_statu==1)//��״̬Ϊ���µ�1
	{
		ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
		ViceControlData.valve[VALVE_BULLET_PROTRACT]=0;
		
		if(valve_fdbstate[VALVE_BULLET_PROTRACT]==0)
		{
			SetCheck_GripBulletLift(1);
			if(SetCheck_LiftAll_To_bullet(0)==1)
				SetCheck_TakeBullet_TakeBack_statu=0;
//			return 1;
		}
	}
//	return 0;
}


s16 error_bullet_lift_fdb=0;
s16 error_bullet_lift_tar=0;
//#define LIFT_DISTANCE_FALL 30
#define LIFT_DISTANCE_GRIPBULLET	900	//�е�ҩ��ʱ���̸߶�900-810	890-807  880-803
#define BULLETLIFT_DISTANCE_GRIPBULLET	460	//�е�ҩ��ʱ�г������߶�	//����ʵ��300Ϊ���޿�����ǰ��߶�
#define BULLETLIFT_DISTANCE_DISGRIPBULLET	1500	//����ҩ��гֻ����߶�
extern LIFT_DATA lift_Data;

u8 SetCheck_GripBulletLift(u8 grip_state)	//�гֻ�����������//�Ƿ��뵯ҩ��ƽ��,gripץס����˼	//0��ʾ��ץס������Ҫ����ҩ������ҩ��߶ȣ�1��ʾץס������Ҫ�н���ҩ��ʱ�ĸ߶�
{
	bulletlift_Motor_Data[BULLETLIFT_FRONTID].tarP=BULLETLIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(BULLETLIFT_DISTANCE_DISGRIPBULLET-BULLETLIFT_DISTANCE_GRIPBULLET);
	bulletlift_Motor_Data[BULLETLIFT_BACKID].tarP=BULLETLIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(BULLETLIFT_DISTANCE_DISGRIPBULLET-BULLETLIFT_DISTANCE_GRIPBULLET);

	error_bullet_lift_tar=abs(2*(BULLETLIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(BULLETLIFT_DISTANCE_DISGRIPBULLET-BULLETLIFT_DISTANCE_GRIPBULLET)));	//�����ǽ���	
	error_bullet_lift_fdb=bulletlift_Motor_Data[BULLETLIFT_BACKID].fdbP;	//�����ǽ���
	return (abs(bulletlift_Motor_Data[BULLETLIFT_FRONTID].fdbP+bulletlift_Motor_Data[BULLETLIFT_BACKID].fdbP-2*(BULLETLIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(BULLETLIFT_DISTANCE_DISGRIPBULLET-BULLETLIFT_DISTANCE_GRIPBULLET)))<40);	//�����ǽ���ǰ����Ϊ�������ص�
}

u8	SetCheck_LiftAll_To_bullet(u8 bullet_state)	//������������	//ȡ��ʱ���������̶��߶ȣ�1Ϊ����0Ϊ��
{
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	
	return (abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*(LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET)))<30);	//�����ǽ���ǰ����Ϊ�������ص�
}
*****************************************************/
