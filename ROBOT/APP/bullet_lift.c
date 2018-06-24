#include "bullet_lift.h"
#include "math.h"
#include "pid.h"

BULLETLIFT_MOTOR_DATA bulletlift_Motor_Data[2] ={0};

PID_GENERAL   PID_BulletLift_Position[2]={PID_BULLETLIFT_POSITION_DEFAULT,PID_BULLETLIFT_POSITION_DEFAULT};
PID_GENERAL   PID_BulletLift_Speed[2]={PID_BULLETLIFT_SPEED_DEFAULT,PID_BULLETLIFT_SPEED_DEFAULT};

extern u32 time_1ms_count;

s32 bullet_lift_tarp=BULLETLIFT_DOWNDISTANCE;
void BulletLift_Task(void)	//��ʱƵ�ʣ�1ms
{ 
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==TAKEBULLET_STATE)
	{
		
		if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)	//����
		{
			if(time_1ms_count%10==0)
			{
				switch (RC_Ctl.rc.switch_right)
				{
					case RC_SWITCH_UP:
					{
						
						break;
					}
					case RC_SWITCH_MIDDLE:
					{
						bulletlift_Motor_Data[BULLETLIFT_FRONTID].tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/600.0);
						bulletlift_Motor_Data[BULLETLIFT_BACKID].tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/600.0);
						break;
					}
					case RC_SWITCH_DOWN:
					{
						break;
					}
				}
			}
			
		}
		else
		{
//			bulletlift_Motor_Data[BULLETLIFT_FRONTID].tarP=bullet_lift_tarp;	//debug��
//			bulletlift_Motor_Data[BULLETLIFT_BACKID].tarP=bullet_lift_tarp;
		}
	}
	
	if(GetWorkState()!=CALI_STATE&&GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CHECK_STATE)
	{
		for(int id=0;id<2;id++)
		{
			bulletlift_Motor_Data[id].tarP=bulletlift_Motor_Data[id].tarP>BULLETLIFT_UPDISTANCE?BULLETLIFT_UPDISTANCE:bulletlift_Motor_Data[id].tarP;
			bulletlift_Motor_Data[id].tarP=bulletlift_Motor_Data[id].tarP<BULLETLIFT_DOWNDISTANCE?BULLETLIFT_DOWNDISTANCE:bulletlift_Motor_Data[id].tarP;
		}
	}
	
	
	
	bulletlift_Motor_Data[BULLETLIFT_FRONTID].tarV=PID_General(bulletlift_Motor_Data[BULLETLIFT_FRONTID].tarP,bulletlift_Motor_Data[BULLETLIFT_FRONTID].fdbP,&PID_BulletLift_Position[BULLETLIFT_FRONTID]);
	bulletlift_Motor_Data[BULLETLIFT_BACKID].tarV=PID_General(bulletlift_Motor_Data[BULLETLIFT_BACKID].tarP,bulletlift_Motor_Data[BULLETLIFT_BACKID].fdbP,&PID_BulletLift_Position[BULLETLIFT_BACKID]);
	
	bulletlift_Motor_Data[BULLETLIFT_FRONTID].output=PID_General(bulletlift_Motor_Data[BULLETLIFT_FRONTID].tarV,bulletlift_Motor_Data[BULLETLIFT_FRONTID].fdbV,&PID_BulletLift_Speed[BULLETLIFT_FRONTID]);
	bulletlift_Motor_Data[BULLETLIFT_BACKID].output=PID_General(bulletlift_Motor_Data[BULLETLIFT_BACKID].tarV,bulletlift_Motor_Data[BULLETLIFT_BACKID].fdbV,&PID_BulletLift_Speed[BULLETLIFT_BACKID]);
}




LiftCaliState_e BulletLiftCali_state=UP_STATE;

u16 start_bullrtcaili_state=0;

u8 BulletLift_Cali(void)
{
	start_bullrtcaili_state++;
	switch(BulletLiftCali_state)
	{
		case UP_STATE:
		{
			bulletlift_Motor_Data[BULLETLIFT_FRONTID].tarP=60;
			bulletlift_Motor_Data[BULLETLIFT_BACKID].tarP=60;
			if(bulletlift_Motor_Data[BULLETLIFT_FRONTID].fdbP>40&&bulletlift_Motor_Data[BULLETLIFT_BACKID].fdbP>40)
			{
				BulletLiftCali_state=WAIT_STATE;
			}
			break;
		}
		case WAIT_STATE:
		{
			bulletlift_Motor_Data[BULLETLIFT_FRONTID].tarP=-10000;
			bulletlift_Motor_Data[BULLETLIFT_BACKID].tarP=-10000;
			if((bulletlift_Motor_Data[BULLETLIFT_FRONTID].fdbV+bulletlift_Motor_Data[BULLETLIFT_BACKID].fdbV)<-30)
			{
				BulletLiftCali_state=DOWN_STATE;
			}
			break;
		}
		case DOWN_STATE:
		{
			if(abs(bulletlift_Motor_Data[BULLETLIFT_FRONTID].fdbV+bulletlift_Motor_Data[BULLETLIFT_BACKID].fdbV)<20)
			{
				bulletlift_Motor_Data[BULLETLIFT_FRONTID].fdbP_raw_sum=0;
				bulletlift_Motor_Data[BULLETLIFT_BACKID].fdbP_raw_sum=0;
				
				bulletlift_Motor_Data[BULLETLIFT_FRONTID].tarP=0;
				bulletlift_Motor_Data[BULLETLIFT_BACKID].tarP=0;
				
				BulletLiftCali_state=OK_STATE;
			}
			break;
		}
		case OK_STATE:
		{
			return 1;
			break;
		}
		
	}
	return 0;
}

#define BULLETLIFT_CALI_OUTPUT_UPMAX 3300
#define BULLETLIFT_CALI_OUTPUT_DOWNMAX	1900
void BulletLift_Cali_Output_Limit(float cm_out,float * cali_out)
{
	if(cm_out>BULLETLIFT_CALI_OUTPUT_UPMAX)
	{
		*cali_out=BULLETLIFT_CALI_OUTPUT_UPMAX;
	}
	else if(cm_out<-BULLETLIFT_CALI_OUTPUT_DOWNMAX)
	{
		*cali_out=-BULLETLIFT_CALI_OUTPUT_DOWNMAX;
	}
	else
	{
		*cali_out=cm_out;
	}
}



/*****************************************
�������ƣ�BulletLift_Feedback_Deal
�������ܣ�ȡ����������������ݽ���+����

*****************************************/
void BulletLift_Feedback_Deal(BULLETLIFT_MOTOR_DATA *bulletlift_motor_data,CanRxMsg *msg)
{
	bulletlift_motor_data->fdbP_raw=(msg->Data[0]<<8)|msg->Data[1];//���յ�����ʵ����ֵ  ����Ƶ��1KHz
	bulletlift_motor_data->fdbV=(msg->Data[2]<<8)|msg->Data[3];
	
	bulletlift_motor_data->fdbP_diff=bulletlift_motor_data->fdbP_raw_last-bulletlift_motor_data->fdbP_raw;
	if(bulletlift_motor_data->fdbP_diff>5460)	//����6�����������㣬��е�Ƕȹ�8192����λ���������ֲ�ֵΪ6826
	{																			//ע���˺���δ�Ե�һ������ʱ�Ŀ��ܵ�Ȧ��ֱ��Ϊ1��ƫ���������������ڳ�ʼ���б궨��ʼ�Ƕ�ֵ��
		bulletlift_motor_data->fdbP_raw_sum+=8192;
	}
	else if(bulletlift_motor_data->fdbP_diff<-5460)
	{
		bulletlift_motor_data->fdbP_raw_sum-=8192;
	}
	
	bulletlift_motor_data->fdbP=(s32)((bulletlift_motor_data->fdbP_raw_sum+bulletlift_motor_data->fdbP_raw)/1000);	//��Ϊ2006���ٱȹ��� ���㾫ȷ
	
	bulletlift_motor_data->fdbP_raw_last=bulletlift_motor_data->fdbP_raw;	//���ݵ���
	
}


