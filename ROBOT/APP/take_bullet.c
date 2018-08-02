#include "take_bullet.h"

//#define VALVE_ISLAND 0		//��ŷ�����λ����
//#define VALVE_BULLET_HORIZONTAL 1	//����ƽ��
//#define VALVE_BULLET_CLAMP 2	//�н�
//#define VALVE_BULLET_STORAGE 3	//��ҩ�ղ���
//#define VALVE_TRAILER 5	//�ϳ�

TakeBulletState_e TakeBulletState=BULLET_OTHER;	//(�Զ�)ȡ����־λ
AutoAimBulletTypeDef AutoAimBulletData={0};

#define BULLETROTATE_OTHER	15	//��ȡ��λ��
#define BULLETROTATE_WAITING	542//556//750//650	//�ȴ�����λ��ʱλ��
#define BULLETROTATE_ACQUIRE	1110	//ȡ��λ��
#define BULLETROTATE_POUROUT	170	//����λ��
#define BULLETROTATE_THROWOUTEND 760	//�׳�ʱ���趨���յ�
#define BULLETROTATE_THROWOUT	290//280//310	//�׳�λ��

//#define LIFT_DISTANCE_FALL 30
#define LIFT_BULLET_POUROUT	540	//����ʱ����


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
const u32 valve_GOODdelay[6]={300,450,100,1000,1000,1000};	//�����룬��ʱ����	//500��α��ʱ
const u32 valve_POORdelay[6]={300,450,400,1000,1000,1000};	//�����룬��ʱ����	//�н�����ʱ����200��Ϊ�˱���ɳ�ȥ�ĵ�ҩ������
const u32 servo_GOODdelay[2]={2000,1000};	//��ʱ����	//��һ��Ϊ2000�ǽ��ӵ����µ���ʱҲ�ӽ�ȥ�ˣ���Ϊ�����ת���ӵ��������������һ���
const u32 servo_POORdelay[2]={1000,1000};	//��ʱ����


extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;
extern u32 time_1ms_count;
extern LIFT_DATA lift_Data;
extern SensorDataTypeDef SensorData;
//extern BULLETLIFT_MOTOR_DATA bulletlift_Motor_Data[2];	//������������
extern BULLETROTATE_DATA BulletRotate_Data;	//������
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern PID_GENERAL PID_Chassis_Speed[4];

extern s16 Chassis_Vx;
extern s16 Chassis_Vy;
extern u8 BulletRotate_Cali_Statu;	//�궨״̬

u8 TakeBullet_AutoAimState=1;	//Ĭ�Ͽ����Զ���λ������ȡ��ģʽ��ȡ�����Ա�����Ч
u8 Close_Valve_Island_Protect_State=0;	//ȡ����ŷ��ջر���λ

void TakeBullet_Control_Center(void)	//��ÿ��״̬��������
{
	static u8 swicth_Last_state=0;	//�Ҳ���
	
	static u8 valve_last[6]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	static u8 servo_last[2]={0};	//��¼��һ����ֵ	//�����빤�̳�������
	
	static u32 valve_startGOOD_time[6]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startGOOD_time[2]={0};	//��¼˳�򴥷�ʱ��	//�����빤�̳�������
	static u32 valve_startPOOR_time[6]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	static u32 servo_startPOOR_time[2]={0};	//��¼���򴥷�ʱ��	//�����빤�̳�������
	
	static WorkState_e State_Record=CHECK_STATE;
	
	static TakeBulletState_e takebulletstate_last=BULLET_OTHER;
	
	if(GetWorkState()==TAKEBULLET_STATE)	//5.9����//��һ��--��//ȡ��������DOWN-MID��ǰ�����-�н�һ�׸�DOWN-MID-->DOWN-DOWN;�����ת��DOWN-MID-->DOWN-UP
	{
		static u8 key_ctrl_last=0;
		static u8 key_shift_last=0;
		if(key_ctrl_last==0&&KeyBoardData[KEY_CTRL].value==1)	//ȡ��ģʽ����CTRL��ȡ���Զ�ȡ��
		{
			AutoAimBulletData.take_count=0;	//����ȡ��������¼	//ʹ����ȡ��ģʽ��ֹ��ȡ����
			TakeBullet_AutoAimState=!TakeBullet_AutoAimState;	//�����Զ���λģ��
		}
		key_ctrl_last=KeyBoardData[KEY_CTRL].value;
		
		if(State_Record!=TAKEBULLET_STATE)
		{
			TakeBulletState=BULLET_WAITING;
		}
		
		if(RC_Ctl.rc.ch3-1024>80&&TakeBulletState==BULLET_WAITING)	/////////////////////////////�޸Ĳ���ģʽʱ��Ҫ�޸�
		{
			TakeBulletState=BULLET_ACQUIRE1;
		}
		else if(RC_Ctl.rc.ch3-1024<-80)
		{
			TakeBulletState=BULLET_WAITING;
		}

		if(KeyBoardData[KEY_SHIFT].value==1&&key_shift_last==0)
		{
			BulletRotate_Cali_Statu=0;
		}
		key_shift_last=KeyBoardData[KEY_SHIFT].value;
	}	
	else
	{
//		if(State_Record==TAKEBULLET_STATE)
//		{
//		}
		
		if(TakeBulletState==BULLET_WAITING)
		{
			TakeBulletState=BULLET_OTHER;
		}
	}
			
	if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)
	{
		Close_Valve_Island_Protect_State=1;	//������ŷ��ջر���
		AutoAimBulletData.take_count=0;	//����ȡ��������¼
		AutoAimBulletData.aim_state=0;	//back
		AutoAimBulletData.control_state=0;	//�رն�λ
		TakeBullet_AutoAimState=0;	//Ĭ�Ϲر��Զ�ģʽ
	}
	
	if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)
	{
		Chassis_Vx=0;
		Chassis_Vy=0;
		AutoAimBulletData.control_state=0;	//�رն�λ
		AutoAimBulletData.take_count=0;	//����ȡ��������¼
		AutoAimBulletData.aim_state=0;	//back
		TakeBullet_AutoAimState=0;	//Ĭ�Ϲر��Զ�ģʽ
	}
	
	
	if(AutoAimBulletData.control_state==1)	//�Զ���λ�����������PID���
	{
		PID_Chassis_Speed[0].k_p=CHASSIS_SPEED_PID_P*1.2f;
		PID_Chassis_Speed[0].k_i=CHASSIS_SPEED_PID_I*1.2f;
		PID_Chassis_Speed[0].i_sum_max=CHASSIS_SPEED_I_MAX*1.2f;
//		PID_Chassis_Speed[0].k_d=CHASSIS_SPEED_PID_P*1.2f;
	}
	else
	{
		PID_Chassis_Speed[0].k_p=CHASSIS_SPEED_PID_P;
		PID_Chassis_Speed[0].k_i=CHASSIS_SPEED_PID_I;
		PID_Chassis_Speed[0].i_sum_max=CHASSIS_SPEED_I_MAX;
	}
	
	State_Record=GetWorkState();
	
	
	if(TakeBullet_AutoAimState==0)	//һ������
	{
		AutoAimBulletData.control_state=0;	//�رն�λ
		AutoAimBulletData.aim_state=0;
	}
	
	
	switch(TakeBulletState)	//�Զ�ȡ������
	{
		case BULLET_WAITING:	//�ȴ�ȡ����������λ��״̬
		{
			if(TakeBullet_AutoAimState==1)	//�Զ�ȡ��ģʽ����
			{
				if(AutoAimBulletData.take_count==0)
				{
					AutoAimBulletData.control_state=1;	//������λ
					if(AutoAimBullet_Task(&Chassis_Vx,&Chassis_Vy)==1)
					{
						TakeBulletState=BULLET_ACQUIRE1;	//�����п�
						AutoAimBulletData.control_state=0;	//�رն�λ
						AutoAimBulletData.aim_state=0;
					}
				}
				else if(AutoAimBulletData.take_count==2)
				{
					AutoAimBulletData.control_state=1;	//������λ
					if(AutoAimBullet_Task(&Chassis_Vx,&Chassis_Vy)==1)
					{
						TakeBulletState=BULLET_ACQUIRE1;	//�����п�
						AutoAimBulletData.control_state=0;	//�رն�λ
						AutoAimBulletData.aim_state=0;
					}
				}

			}
			
			
			ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//�����ɿ�
			{
				BulletRotate_Data.tarP=BULLETROTATE_WAITING;
				if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<30)	//����ջ�
				{
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL]=0;	//����1					//////////////////////////////////////////////////
				}
			}
			
			lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
			break;
		}
		case BULLET_ACQUIRE1:	//ǰ�졢�н���̧����	��֮Ϊ��ù���
		{
			BulletRotate_Data.tarP=BULLETROTATE_ACQUIRE;
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_ACQUIRE)<45)
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)
				{
					TakeBulletState=BULLET_POUROUT1;	//�л�������
				}
			}
			
			if(GetWorkState()!=TAKEBULLET_STATE)	//�˳�ȡ��ģʽ��״̬����
			{
				TakeBulletState=BULLET_WAITING;	//����ȴ�״̬
			}
			break;
		}
		case BULLET_POUROUT1:	//������б�������ת	��֮Ϊ��������
		{
			lift_Data.lf_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.lb_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.rf_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.rb_lift_tarP=LIFT_BULLET_POUROUT;
			
			BulletRotate_Data.tarP=BULLETROTATE_POUROUT;
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)<50)
			{
				TakeBulletState=BULLET_THROWOUT1;	//�л����ӳ�
			}
			break;
		}
		case BULLET_THROWOUT1:	//������ء�����̧�𡢼н��ɿ�	��֮Ϊ�������
		{
			BulletRotate_Data.tarP=BULLETROTATE_THROWOUTEND;//BULLETROTATE_WAITING;	//�趨�׳��յ��Ա��ýϴ��ٶ�
			
			if((BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)>40)	//��̧����һ���
			{
				ViceControlData.valve[VALVE_BULLET_HORIZONTAL]=1;	//ͬʱ������һ����λ				/////////////////////////////////////////////////////////////////
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_THROWOUT)<20)	//�����׳�λ��
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			}
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//�Ѿ��ɿ�����ʼ�½�
			{
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
				TakeBulletState=BULLET_ACQUIRE2;	//��һ��ȡ��	//�����󴥷�
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<40)	//��׼��λ�ã����Կ�ʼ��һ��
			{
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
				TakeBulletState=BULLET_ACQUIRE2;	//��һ��ȡ��
			}
			break;
		}
		case BULLET_ACQUIRE2:	//ǰ�졢�н���̧����	��֮Ϊ��ù���2
		{
			if(valve_fdbstate[VALVE_BULLET_HORIZONTAL]==1)	//����ƽ��			//////////////////////////////////////////////
			{
				BulletRotate_Data.tarP=BULLETROTATE_ACQUIRE;
				if(abs(BulletRotate_Data.fdbP-BULLETROTATE_ACQUIRE)<45)
				{
					ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
					if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)
					{
						TakeBulletState=BULLET_POUROUT2;	//�л�������
					}
				}
			}
			
			if(GetWorkState()!=TAKEBULLET_STATE)	//�˳�ȡ��ģʽ��״̬����
			{
				TakeBulletState=BULLET_WAITING;	//����ȴ�״̬
			}
			break;
		}
		case BULLET_POUROUT2:	//������б�������ת	��֮Ϊ��������2
		{
			lift_Data.lf_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.lb_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.rf_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.rb_lift_tarP=LIFT_BULLET_POUROUT;
			
			BulletRotate_Data.tarP=BULLETROTATE_POUROUT;
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)<50)
			{
				TakeBulletState=BULLET_THROWOUT2;	//�л����ӳ�
			}
			break;
		}
		case BULLET_THROWOUT2:	//������ء�����̧�𡢼н��ɿ�	��֮Ϊ�������2
		{
			
			BulletRotate_Data.tarP=BULLETROTATE_THROWOUTEND;//BULLETROTATE_WAITING;
			
			if((BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)>40)	//��̧����һ���
			{
				ViceControlData.valve[VALVE_BULLET_HORIZONTAL]=0;	//ƽ�����׻���			/////////////////////////////////////
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_THROWOUT)<20)
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			}
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//�Ѿ��ɿ�����ʼ�½�
			{
				BulletRotate_Data.tarP=BULLETROTATE_WAITING;
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
				if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<25)	//��׼��λ�ã����Կ�ʼ��һ��
				{
					TakeBulletState=BULLET_WAITING;	//����ȴ�״̬
					AutoAimBulletData.take_count+=2;	//���ȡ��������2
				}
				
				
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<20)
			{
//				AutoAimBulletData.take_count+=2;	//���ȡ��������2
//				TakeBulletState=BULLET_WAITING;	//����ȴ�״̬
			}
			break;
		}
		case BULLET_OTHER:	//������ȡ��״̬
		{
			ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//�����ɿ�
			{
				BulletRotate_Data.tarP=BULLETROTATE_OTHER;
				if(abs(BulletRotate_Data.fdbP-BULLETROTATE_OTHER)<30)	//����ջ�
				{
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL]=0;					/////////////////////////////////////////
				}
			}
			
			if(takebulletstate_last!=BULLET_OTHER)
			{
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
			}
			break;
		}
	}

	takebulletstate_last=TakeBulletState;


	Close_Valve_Island_Protect();	//�ǵ�ȡ����ŷ����ر���
	
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


#define AUTOAIM_SPEENX	7	//��ǰѹ��
#define AUTOAIM_VOIDSPEEDY	130//110	//�Զ�ȡ���ʱ��λ�ٶ�	//������Ϊ������
#define AUTOAIM_EXISTSPEEDY	65//55	//�Զ�ȡ�����ϰ�ʱ�ٶ�
u8 AutoAimBullet_Task(s16* chassis_vx,s16* chassis_vy)	//�Զ���λ����
{
	static u8 aim_control_state_last=0;
	u8 aim_OK_statu=0;
	
	//�ƶ�״̬��ʶ
	if(SensorData.Infrare[6]==0&&SensorData.Infrare[7]==1)	//[6]Ϊ���λ [7]Ϊ�Ҷ�λ 0Ϊ�У�1Ϊ��
	{	//�����ƶ�״̬
		AutoAimBulletData.relative_location=-1;
	}
	else if(SensorData.Infrare[6]==1&&SensorData.Infrare[7]==0)
	{
		AutoAimBulletData.relative_location=1;
	}
	else if(SensorData.Infrare[6]==1&&SensorData.Infrare[7]==1)
	{
		AutoAimBulletData.relative_location=0;
	}
	else if(SensorData.Infrare[6]==0&&SensorData.Infrare[7]==0)
	{
		AutoAimBulletData.relative_location=2;
	}
	
	if(aim_control_state_last==0&&AutoAimBulletData.control_state==1)
	{
		AutoAimBulletData.aim_state=0;
	}
	
	if(AutoAimBulletData.control_state==1)
	{
		//״̬��ʶ֮���ƽ���
		
		switch(AutoAimBulletData.aim_state)
		{
			case 0:	//�գ���Ҫ�ƶ���(�����ƶ�)
			{
				*chassis_vx=AUTOAIM_SPEENX;
				*chassis_vy=AUTOAIM_VOIDSPEEDY;
				if(AutoAimBulletData.relative_location==-1)
				{
					AutoAimBulletData.aim_state=1;
				}
				else if(AutoAimBulletData.relative_location==1)
				{
					AutoAimBulletData.aim_state=2;
				}
				else if(AutoAimBulletData.relative_location==2)	//���У��ٶȼ��������򲻱�
				{
					AutoAimBulletData.aim_state=4;
				}
				break;
			}
			case 1:	//�����ƶ�
			{
				*chassis_vx=AUTOAIM_SPEENX;
				*chassis_vy=-AUTOAIM_EXISTSPEEDY;
				if(AutoAimBulletData.relative_location==0)
				{
					AutoAimBulletData.aim_state=3;
				}
				else if(AutoAimBulletData.relative_location==1)	//�����ƶ�
				{
					AutoAimBulletData.aim_state=2;
				}
				break;
			}
			case 2:	//�����ƶ�
			{
				*chassis_vx=AUTOAIM_SPEENX;
				*chassis_vy=AUTOAIM_EXISTSPEEDY;
				if(AutoAimBulletData.relative_location==0)
				{
					AutoAimBulletData.aim_state=3;
				}
				else if(AutoAimBulletData.relative_location==-1)	//�����ƶ�
				{
					AutoAimBulletData.aim_state=1;
				}
				break;
			}
			case 3:	//�գ�����ȡ�飩
			{
				*chassis_vx=0;
				*chassis_vy=0;
				aim_OK_statu=1;
				AutoAimBulletData.aim_state=0;
				break;
			}
			case 4:	//���� �Ͷ��շ���һ�µ��ٶȼ���
			{
				*chassis_vx=AUTOAIM_SPEENX;
				*chassis_vy=AUTOAIM_EXISTSPEEDY;
				if(AutoAimBulletData.relative_location==-1)
				{
					AutoAimBulletData.aim_state=1;
				}
				else if(AutoAimBulletData.relative_location==1)
				{
					AutoAimBulletData.aim_state=2;
				}
				else if(AutoAimBulletData.relative_location==0)	//����ȡ��
				{
					AutoAimBulletData.aim_state=3;
				}
				break;
			}
		}
	}
	else
	{
		AutoAimBulletData.aim_state=0;	//����
	}
	
	aim_control_state_last=AutoAimBulletData.control_state;
	
	return aim_OK_statu;
}


#define LIFT_DISTANCE_BACKVALVE	260
//u8 Close_Valve_Island_Protect_State=0;	//����������
void Close_Valve_Island_Protect(void)	//ʹ��ŷ���λ�ı���
{
	static u8 close_state_last=0;
	static u8 execute_statu=0;
	
	if(close_state_last==0&&Close_Valve_Island_Protect_State==1)
	{
		execute_statu=1;
	}
	
	if(Close_Valve_Island_Protect_State==1)
	{
		switch(execute_statu)
		{
			case 0:
			{
				
				break;
			}
			case 1:
			{
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_BACKVALVE;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_BACKVALVE;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_BACKVALVE;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_BACKVALVE;
				
				if(lift_Data.lf_lift_fdbP+lift_Data.lb_lift_fdbP+lift_Data.rf_lift_fdbP+lift_Data.rb_lift_fdbP-4*LIFT_DISTANCE_BACKVALVE>-150)
				{
					execute_statu=2;
				}
				break;
			}
			case 2:
			{
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
				execute_statu=0;
				Close_Valve_Island_Protect_State=0;
				break;
			}
		}
	}
	
	close_state_last=Close_Valve_Island_Protect_State;
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
