#include "control.h"
#include "semi_automatic_landing.h"

WorkState_e workState=PREPARE_STATE;


extern LIFT_DATA lift_Data;

extern RC_Ctl_t RC_Ctl;
extern LIFT_POSITION_ENCODER lift_position_encoder[4];
extern GYRO_DATA Gyro_Data;

extern CHASSIS_DATA chassis_Data;
//extern BULLETLIFT_MOTOR_DATA bulletlift_Motor_Data[2];	������������
extern BULLETROTATE_DATA BulletRotate_Data;
extern ViceControlDataTypeDef ViceControlData;
extern SensorDataTypeDef SensorData;

extern s16 Chassis_Vx;
extern s16 Chassis_Vy;
extern s16 Chassis_Vw;
extern u8 cali_state_Entirety_PID;

s16 remote_tem=0;
s16 lift_tem=0;
s16 LIFT_tarP=0;

u8 t_lift_time_start=0;
///////////////////////////////////////remoteData
s16 Vw_tem=0;
///////////////////////////////////////

u32 time_1ms_count=0;
void Control_Task(void)	//2ms
{
	time_1ms_count++;

	Lift_Time_Gauge(&t_lift_time_start);
	
	Check_Task();
	
	IMU_Check_Useless();
	
	KeyboardRetset();
	
	if(time_1ms_count%50==0)
	{
//		Debug_Send_OSC();	//�����Ʒ����߼�
	}
	
	if(time_1ms_count%1==0)	//1000hz
	{
		for(int keyid=0;keyid<KEY_NUMS;keyid++)	//���ڶ�ʱ����
		{
			ButtonStatu_Verdict(&KeyBoardData[keyid]);	//��λ��Ϣ�߼�����
		}
	}
	
//Take_Bullet_Task();
////////////	Vw_tem=Chassis_Attitude_Correct(Chassis_GYRO[2],Gyro_Data.angvel[2]+2);	//��ʱ��û��������
////////////  Chassis_Vw+=Vw_tem;
	Work_State_Change_Gaming();	//ս���湤��״̬�ı�
	//
	Work_State_Change_BackProtect();
	//
	Work_Execute_Gaming();	//ս����ִ��
	//
	
	LED_Indicate();
	
	Chassis_Attitude_Angle_Convert();
	
	Motor_Send();
	
	if(time_1ms_count%2==0)
	{
		ViceBoard_SendDataRefresh();
		ViceBoard_SendDataRun();
	}
}


extern TakeBulletState_e TakeBulletState;	//(�Զ�)ȡ����־λ
extern AscendState_e AscendState;
extern DescendState_e DescendState;
/*************************************
RC��PC�Ի���״̬���л�
*************************************/
void Work_State_Change(void)
{
	static u8 Switch_Right_Last=0;
	static WorkState_e State_Record=CHECK_STATE;	
	
	switch (GetWorkState())	//2018.3.15
	{
		case CHECK_STATE:	//�Լ�ģʽ
		{	//���������ʼ���������Լ�ģʽ 
			
			break;
		}
		case PREPARE_STATE:	//Ԥ��ģʽ
		{	
			
			break;
		}
		case CALI_STATE:	//�궨ģʽ
		{
			
			break;
		}
		case NORMAL_STATE:	//��������ģʽ
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE)	//����
			{
				SetWorkState(STOP_STATE);
			}
			
			if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN&&Switch_Right_Last==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
			{
				SetWorkState(ASCEND_STATE);
			}
			else if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN&&Switch_Right_Last==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)
			{
				SetWorkState(DESCEND_STATE);
//				SetWorkState(TAKEBULLET_STATE);
			}
			
			break;
		}
		case ASCEND_STATE:	//�Զ��ϵ�ģʽ
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE)	//����
			{
				AscendState=FULLRISE_GO1;	//���÷�ֹ��һ���쳣
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case DESCEND_STATE:	//�Զ��µ�ģʽ
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE)	
			{
				DescendState=FULLFALL_DOWN1;	//���÷�ֹ��һ���쳣
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case TAKEBULLET_STATE:	//ȡ��ģʽ
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_MIDDLE)	//����
			{
				TakeBulletState=BULLET_OTHER;	//(�Զ�)ȡ����־����
				SetWorkState(STOP_STATE);
			}
			break;
		}
		case SEMI_ASCEND_STATE:	//���Զ����ֶ��ϵ�
		{

			break;
		}
		case SEMI_DESCEND_STATE:	//���Զ����ֶ��µ�
		{

			break;
		}
		case ERROR_STATE:	//����ģʽ
		{
			break;
		}
		case LOST_STATE:	//����ģʽ
		{
			SetWorkState(CHECK_STATE);
			time_1ms_count=0;	//�����ʼ״̬�����Լ�
			break;
		}
		case STOP_STATE:	//ֹͣ״̬
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_UP||RC_Ctl.rc.switch_left==RC_SWITCH_DOWN)	
			{
				SetWorkState(NORMAL_STATE);
			}
			break;
		}
		case PROTECT_STATE:	//���ұ���ģʽ
		{
			if(Error_Check.statu[LOST_DBUS]==0||abs(RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)>8)
			{
				SetWorkState(STOP_STATE);
			}
			break;
		}
	}
	Switch_Right_Last=RC_Ctl.rc.switch_right;
	State_Record=GetWorkState();
}

void Work_State_Change_Gaming(void)	//ս�������״̬�л�
{	//ս����
	static u8 Switch_Right_Last=0;
	static u8 Switch_Left_Last=0;
	if(GetWorkState()!=CHECK_STATE&&GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CALI_STATE&&GetWorkState()!=LOST_STATE&&GetWorkState()!=ERROR_STATE&&GetWorkState()!=PROTECT_STATE)	//�����ֳ�ʼ״̬+3�ֱ���״̬����״̬�л������ܿأ�
	{
		switch(RC_Ctl.rc.switch_left)
		{
			case RC_SWITCH_UP:
			{
				switch(RC_Ctl.rc.switch_right)
				{
					case RC_SWITCH_UP:	//UP-UP	һ��״̬
					{
						SetWorkState(NORMAL_STATE);
						break;
					}
					case RC_SWITCH_MIDDLE:
					{
						
						break;
					}
					case RC_SWITCH_DOWN:
					{
						break;
					}
				}
				break;
			}
			case RC_SWITCH_MIDDLE:
			{
				switch(RC_Ctl.rc.switch_right)
				{
					case RC_SWITCH_UP:
					{
						
						break;
					}
					case RC_SWITCH_MIDDLE:	//MID-MID	ȫ�Զ��µ�
					{
						if(Switch_Left_Last!=RC_SWITCH_MIDDLE)	//����һ��ʼδ�Ѳ����õ�����
						SetWorkState(DESCEND_STATE);
						break;
					}
					case RC_SWITCH_DOWN:	//MID-DOWN	�ֶ��µ�
					{
						SetWorkState(SEMI_DESCEND_STATE);
						break;
					}
				}
				break;
			}
			case RC_SWITCH_DOWN:
			{
				switch(RC_Ctl.rc.switch_right)
				{
					case RC_SWITCH_UP:	//DOWN-UP	ȫ�Զ��ϵ�
					{
						if(Switch_Left_Last!=RC_SWITCH_DOWN)	//����һ��ʼδ�Ѳ����õ�����
						SetWorkState(ASCEND_STATE);
						break;
					}
					case RC_SWITCH_MIDDLE:	//DOWN-MID	���Զ��ϵ�
					{
						SetWorkState(SEMI_ASCEND_STATE);
						break;
					}
					case RC_SWITCH_DOWN:	//DOWN-DOWN	ȡ��
					{
						SetWorkState(TAKEBULLET_STATE);
						break;
					}
				}
				break;
			}
		}
	}
	
	switch (GetWorkState())	//2018.5.9
	{
		case CHECK_STATE:	//�Լ�ģʽ
		{	//���������ʼ���������Լ�ģʽ 
			break;
		}
		case PREPARE_STATE:	//Ԥ��ģʽ
		{	
			
			break;
		}
		case CALI_STATE:	//�궨ģʽ
		{
			
			break;
		}
		case NORMAL_STATE:	//��������ģʽ
		{
			break;
		}
		case ASCEND_STATE:	//�Զ��ϵ�ģʽ
		{

			break;
		}
		case DESCEND_STATE:	//�Զ��µ�ģʽ
		{

			break;
		}
		case TAKEBULLET_STATE:	//ȡ��ģʽ
		{

			break;
		}
		case SEMI_ASCEND_STATE:	//���Զ����ֶ��ϵ�
		{

			break;
		}
		case SEMI_DESCEND_STATE:	//���Զ����ֶ��µ�
		{

			break;
		}
		case ERROR_STATE:	//����ģʽ
		{
			if(RC_Ctl.key.v_h!=0||RC_Ctl.key.v_l!=0||abs(RC_Ctl.mouse.x)>3)
			{
				SetWorkState(NORMAL_STATE);
			}
			break;
		}
		case LOST_STATE:	//����ģʽ
		{
			SetWorkState(CHECK_STATE);
			time_1ms_count=0;	//�����ʼ״̬�����Լ�
			break;
		}
		case STOP_STATE:	//ֹͣ״̬
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_UP)	
			{
				SetWorkState(NORMAL_STATE);
			}
			break;
		}
		case PROTECT_STATE:	//���ұ���ģʽ
		{
			static u32 time_count=0;
			time_count++;
			if(Error_Check.statu[LOST_DBUS]==0&&abs(RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)>8)
			{
				SetWorkState(NORMAL_STATE);
				time_count=0;
			}
			
			if(Error_Check.statu[LOST_DBUS]==0&&time_count>6000)	//�з���ȴ���˳���Ϊ�޷��ָ�	�����ݴ��ң�
			{
				time_count=0;
				NVIC_SystemReset();
			}
			break;
		}
	}

	Switch_Right_Last=RC_Ctl.rc.switch_right;
	Switch_Left_Last=RC_Ctl.rc.switch_left;
}

void Work_Execute_LastVersion(void)	//֮ǰ�汾��ִ��
{
	switch (GetWorkState())	//2018.3.15
	{
		case CHECK_STATE:	//�Լ�ģʽ
		{	//���������ʼ���������Լ�ģʽ //��ʱ����ոտ�������ȴ�һ��ʱ��ȫ���Լ�δ��⵽�쳣��2-3���Լ촥���������ϣ�������Ϊʱ��������Ϊ��ʱ�������㣬���������ʱ����¼
			if(time_1ms_count>300)	//����LOST״̬�ص�CHECKģʽ����ִ�м����������
			{	//����ִ�е�����˵��LOSTCHECKͨ����������ֵ���
				RC_Calibration();	//self check
				if(1)	//selfcheck��־
				{
					SetWorkState(PREPARE_STATE);	//�˲���ζ�Լ�ͨ����һ��Ӳ��ģ������
					//���ݳ�ʼ����
				}
			}
			break;
		}
		case PREPARE_STATE:	//Ԥ��ģʽ
		{	//�ȴ�����״̬�ȶ��������ó�ֵ
			SetWorkState(CALI_STATE);
			break;
		}
		case CALI_STATE:	//�궨ģʽ
		{
			if(Lift_Cali()==1)	//&&BulletLift_Cali()==1
			{
				SetWorkState(NORMAL_STATE);
			}
			Lift_Task();	//��������
//			BulletLift_Task();
			break;
		}
		case NORMAL_STATE:	//��������ģʽ
		{
			Teleconltroller_Data_protect();	//ң�������ݱ���
			TakeBullet_Control_Center();	//��ʱ����λ���ǵ�
			if(RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)
			{
				semi_auto_landing_center();
			}
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
//			BulletLift_Task();
			break;
		}
		case ASCEND_STATE:	//�Զ��ϵ�ģʽ
		{
			Teleconltroller_Data_protect();	//ң�������ݱ���
			Ascend_Control_Center();
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
//			BulletLift_Task();
			break;
		}
		case DESCEND_STATE:	//�Զ��µ�ģʽ
		{
			Teleconltroller_Data_protect();	//ң�������ݱ���
			Descend_Control_Center();
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
//			BulletLift_Task();
			break;
		}
		case TAKEBULLET_STATE:
		{
			Teleconltroller_Data_protect();	//ң�������ݱ���
			TakeBullet_Control_Center();	//ȡ����������
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
//			BulletLift_Task();
			break;
		}
		case SEMI_ASCEND_STATE:
		{
			Teleconltroller_Data_protect();	//ң�������ݱ���
			if(RC_Ctl.rc.switch_right==RC_SWITCH_MIDDLE)
			{
				semi_auto_landing_center();
			}
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
//			BulletLift_Task();
			break;
		}
		case SEMI_DESCEND_STATE:
		{
			Teleconltroller_Data_protect();	//ң�������ݱ���
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
//			BulletLift_Task();
			break;
		}
		case ERROR_STATE:	//����ģʽ
		{
			break;
		}
		case LOST_STATE:	//����ģʽ
		{
			break;
		}
		case STOP_STATE:	//ֹͣ״̬
		{
			break;
		}
		case PROTECT_STATE:	//���ұ���ģʽ
		{
			break;
		}
	}
}

extern u8 Replenish_Bullet_Statu;	//����״̬λ
extern u8 Trailer_statu;

u8 t_trailer_sensor_data_simu=0;	//���������ݷ���
//extern u8 SetCheck_TakeBullet_TakeBack_statu;	//�г�ȡ������ִ�б�־λ	//�����������ð��Զ��µ������½���ǰ������	//���statuΪ0������һ��ִ����ɺ����
void Work_Execute_Gaming(void)	//ս����switch����ִ��
{
	static WorkState_e State_Record=CHECK_STATE;
	switch (GetWorkState())	//2018.5.9
	{
		case CHECK_STATE:	//�Լ�ģʽ
		{	//���������ʼ���������Լ�ģʽ //��ʱ����ոտ�������ȴ�һ��ʱ��ȫ���Լ�δ��⵽�쳣��2-3���Լ촥���������ϣ�������Ϊʱ��������Ϊ��ʱ�������㣬���������ʱ����¼
			if(time_1ms_count>300)	//����LOST״̬�ص�CHECKģʽ����ִ�м����������
			{	//����ִ�е�����˵��LOSTCHECKͨ����������ֵ���
				RC_Calibration();	//self check
				if(1)	//selfcheck��־
				{
					SetWorkState(PREPARE_STATE);	//�˲���ζ�Լ�ͨ����һ��Ӳ��ģ������
					//���ݳ�ʼ����
				}
			}
			break;
		}
		case PREPARE_STATE:	//Ԥ��ģʽ
		{	//�ȴ�����״̬�ȶ��������ó�ֵ
			SetWorkState(CALI_STATE);
			break;
		}
		case CALI_STATE:	//�궨ģʽ
		{
			if(Lift_Cali()==1&&BulletRotate_Cali()==1)	//&&BulletLift_Cali()==1	//�������Ϊ��תȡ������ʼ�ں�λ���Զ���¼
			{
				SetWorkState(NORMAL_STATE);
			}
			Lift_Task();	//��������
			BulletRotate_Task();	//�궨
//			BulletLift_Task();
			break;
		}
		case NORMAL_STATE:	//��������ģʽ
		{
			BulletRotate_Cali();	//�쳣�궨
//			ViceControlData.valve[VALVE_ISLAND]=0;//��������
			Teleconltroller_Data_protect();	//ң�������ݱ���
			TakeBullet_Control_Center();	//�����������Ϊ���ڶ�������׵ļ��뷴�������������棬�г�ȡ����λ������Ҫ�������ڲ��Ѿ����˽���TAKEBULLET�����߼�����
			
			Replenish_Bullet_Task(KeyBoardData[KEY_R].value);	//������վ����
			Trailer_Task(SensorData.Infrare[5]);	//�ϳ�
			
			if(ViceControlData.valve[VALVE_ISLAND]==0&&Replenish_Bullet_Statu==0&&Trailer_statu==0)
			{
				//AutoChassisAttitude_Lift_V2(Chassis_GYRO[PITCH]);	//��ʱ����
			}
			
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
			BulletRotate_Task();
//			BulletLift_Task();
			break;
		}
		case ASCEND_STATE:	//�Զ��ϵ�ģʽ
		{
			if(State_Record!=ASCEND_STATE)	//2018.5.9
			{
				AscendState=Island_State_Recognize();	//�Զ���ʶ��ǰ״̬
			}
			
			Teleconltroller_Data_protect();	//ң�������ݱ���
			Ascend_Control_Center();
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
			BulletRotate_Task();
//			BulletLift_Task();
			break;
		}
		case DESCEND_STATE:	//�Զ��µ�ģʽ
		{
			TakeBullet_Control_Center();	//�����������Ϊ���ڶ�������׵ļ��뷴�������������棬�г�ȡ����λ������Ҫ�������ڲ��Ѿ����˽���TAKEBULLET�����߼�����
			if(State_Record!=DESCEND_STATE)
			{
				DescendState=OutIsland_State_Recognize();
			}
			
			Teleconltroller_Data_protect();	//ң�������ݱ���
			Descend_Control_Center();
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
			BulletRotate_Task();
//			BulletLift_Task();
			break;
		}
		case TAKEBULLET_STATE:
		{
			BulletRotate_Cali();	//�쳣�궨
			Teleconltroller_Data_protect();	//ң�������ݱ���
			TakeBullet_Control_Center();	//ȡ����������
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
			BulletRotate_Task();
//			BulletLift_Task();
			break;
		}
		case SEMI_ASCEND_STATE:
		{
			Teleconltroller_Data_protect();	//ң�������ݱ���

			semi_auto_landing_center();

			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
			BulletRotate_Task();
//			BulletLift_Task();
			break;
		}
		case SEMI_DESCEND_STATE:
		{
			Teleconltroller_Data_protect();	//ң�������ݱ���
			
			TakeBullet_Control_Center();	//�����������Ϊ���ڶ�������׵ļ��뷴�������������棬�г�ȡ����λ������Ҫ�������ڲ��Ѿ����˽���TAKEBULLET�����߼�����
///////			if(SetCheck_TakeBullet_TakeBack_statu==0)	//ֻ�е�ȡ��״̬��ȫ�˳�ʱ��statu�Żᱻ��0
			{
				semi_auto_outlanding_center();
			}
			
			Remote_Task();	//ִ���ƶ�
			Lift_Task();	//��������
			BulletRotate_Task();
//			BulletLift_Task();
			break;
		}
		case ERROR_STATE:	//����ģʽ
		{
			break;
		}
		case LOST_STATE:	//����ģʽ
		{
			break;
		}
		case STOP_STATE:	//ֹͣ״̬
		{
			break;
		}
		case PROTECT_STATE:	//���ұ���ģʽ
		{
			break;
		}
	}
	Image_Cut_Task();
	State_Record=GetWorkState();
}


extern u8 full_fall_statu;
extern u8 full_rise_statu;

extern u8 descend_valve_prepare_state;	//�Զ��µ���ŷ���λ����
extern u32 descend_valve_prepare_state_count;
//extern u8 SetCheck_TakeBullet_TakeBack_statu;	//�г�ȡ������ִ�б�־λ	//����ǰ��extern
void Work_State_Change_BackProtect(void)	//����ĳһ״̬�˳�ʱ��ȷ����״̬��һ���������ƶ���λ
{
	static WorkState_e State_Record=CHECK_STATE;
	
	if(State_Record!=DESCEND_STATE&&GetWorkState()==DESCEND_STATE)
	{
		descend_valve_prepare_state=0;
		descend_valve_prepare_state_count=0;
		full_fall_statu=0;
		full_rise_statu=0;
	}
	
	if(State_Record!=ASCEND_STATE&&GetWorkState()==ASCEND_STATE)
	{
		full_fall_statu=0;
		full_rise_statu=0;
	}
	
	if(State_Record!=NORMAL_STATE&&GetWorkState()==NORMAL_STATE)
	{
		ViceControlData.valve[VALVE_ISLAND]=0;	//ȡ���ջص���
	}
	
	if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)	//�˳�ȡ��ģʽ
	{
//////		SetCheck_TakeBullet_TakeBack_statu=1;	//ˢ�´�
	}
	
	if(State_Record!=TAKEBULLET_STATE&&GetWorkState()==TAKEBULLET_STATE)
	{
		Replenish_Bullet_Statu=0;	//�����Դ�ʩ	//��������һ��Ϊ0
		Trailer_statu=0;	//�����Դ�ʩ//��������һ��Ϊ0
		
//		SetCheck_GripBulletLift(1);
		ViceControlData.valve[VALVE_ISLAND]=0;	//ȡ���ջص���
	}
//	SetCheck_TakeBullet_TakeBack();	//ִ�д�
	State_Record=GetWorkState();
}

extern s16 t_error_i_record;
void LED_Indicate(void)
{
	if(time_1ms_count%BLINK_CYCLE==0)
	{
		switch (GetWorkState())	//2018.3.15
		{
			case CHECK_STATE:	//�Լ�ģʽ
			{	//���������ʼ���������Լ�ģʽ 
				LED_Blink_Set(10,10);
				break;
			}
			case PREPARE_STATE:	//Ԥ��ģʽ
			{	
				LED_Blink_Set(9,9);
				break;
			}
			case CALI_STATE:	//�궨ģʽ	�쿪����
			{
				LED_Blink_Set(9,9);
				break;
			}
			case NORMAL_STATE:	//��������ģʽ	�������
			{
				LED_Blink_Set(1,0);
				break;
			}
			case ASCEND_STATE:	//�Զ��ϵ�ģʽ	����
			{
				LED_Blink_Set(1,0);
				break;
			}
			case DESCEND_STATE:	//�Զ��µ�ģʽ	����
			{
				LED_Blink_Set(1,0);
				break;
			}
			case TAKEBULLET_STATE:
			{
				LED_Blink_Set(1,0);
				break;
			}
			case SEMI_ASCEND_STATE:
			{
				LED_Blink_Set(1,0);
				break;
			}
			case SEMI_DESCEND_STATE:
			{
				LED_Blink_Set(1,0);
				break;
			}
			case ERROR_STATE:	//����ģʽ
			{
				if(t_error_i_record==LOST_BULLETROTATE1)	//ȡ����ת
				{
					LED_Blink_Set(3,10);
				}
				else if(t_error_i_record==LOST_CM1||t_error_i_record==LOST_CM2||t_error_i_record==LOST_CM3||t_error_i_record==LOST_CM4)	//���̵��
				{
					LED_Blink_Set(2,10);
				}
				else if(t_error_i_record==LOST_LIFT1||t_error_i_record==LOST_LIFT2||t_error_i_record==LOST_LIFT3||t_error_i_record==LOST_LIFT4)	//����
				{
					LED_Blink_Set(1,10);
				}
				else
				{
					LED_Blink_Set(6,10);
				}
				
				break;
			}
			case LOST_STATE:	//����ģʽ
			{
				LED_Blink_Set(1,1);
				break;
			}
			case STOP_STATE:	//ֹͣ״̬	����
			{
				if(time_1ms_count%BLINK_INTERVAL==0)
				{
					LED_Blink_Set(0,10);
				}
				else if((time_1ms_count+BLINK_INTERVAL/2)%BLINK_INTERVAL==0)
				{
					LED_Blink_Set(10,0);
				}
				
				break;
			}
			case PROTECT_STATE:	//���ұ���ģʽ	˫��
			{
				LED_Blink_Set(1,1);
				break;
			}
		}
LED_Blink_Run();
	}
}

/////////////////////////////////ԭ����lift task�ĵط�
s16 t_send_cna1_id6=0;

float lift_calisend[4]={0};

void Motor_Send(void)
{
	switch (GetWorkState())	//2018.3.15
	{	
		case CHECK_STATE:	//�Լ�ģʽ
		{	//���������ʼ���������Լ�ģʽ //��ʱ����ոտ�������ȴ�һ��ʱ��ȫ���Լ�δ��⵽�쳣��2-3���Լ촥���������ϣ�������Ϊʱ��������Ϊ��ʱ�������㣬���������ʱ����¼
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletRotate_SendMsg(0,0);
			break;
		}
		case PREPARE_STATE:	//Ԥ��ģʽ
		{	//�ȴ�����״̬�ȶ��������ó�ֵ
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletRotate_SendMsg(0,0);
			break;
		}
		case CALI_STATE:	//�궨ģʽ
		{
			Lift_Cali_Output_Limit(lift_Data.lf_lift_output,&lift_calisend[LF]);
			Lift_Cali_Output_Limit(lift_Data.rf_lift_output,&lift_calisend[RF]);
			Lift_Cali_Output_Limit(lift_Data.lb_lift_output,&lift_calisend[LB]);
			Lift_Cali_Output_Limit(lift_Data.rb_lift_output,&lift_calisend[RB]);
//		Entirety_PID(&lift_Data,cali_send);  	//����PID����
//			Lift_Cali_GYRO_Compensate(cali_send);	//�����ǲ���.��������3.14

			CAN2_Chassis_SendMsg(0,0,0,0);
//		CAN1_Lift_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg((s16)lift_calisend[LF],(s16)lift_calisend[RF],(s16)lift_calisend[LB],(s16)lift_calisend[RB]);

			CAN2_BulletRotate_SendMsg((s16)BulletRotate_Data.output,0);
			break;
		}
		case NORMAL_STATE:	//��������ģʽ
		{
//		CAN_Chassis_SendMsg((s16)remote_tem,(s16)remote_tem,(s16)remote_tem,(s16)remote_tem);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
//			CAN_Chassis_SendMsg(0,0,0,0);
//    CAN_Lift_SendMsg((s16)lift_tem,(s16)lift_tem,(s16)lift_tem,(s16)lift_tem);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_BulletRotate_SendMsg((s16)BulletRotate_Data.output,0);
			break;
		}
		case ERROR_STATE:	//����ģʽ
		{	
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletRotate_SendMsg(0,0);
//			CAN1_Yun_SendMsg(t_send_cna1_id6,t_send_cna1_id6);
			break;
		}
		case STOP_STATE:	//ֹͣ״̬
		{
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletRotate_SendMsg(0,0);
			break;
		}
		case PROTECT_STATE:	//���ұ���ģʽ
		{
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletRotate_SendMsg(0,0);
			break;
		}
		case ASCEND_STATE:	//�Զ��ϵ�ģʽ
		{
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
//			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_BulletRotate_SendMsg((s16)BulletRotate_Data.output,0);
			break;
		}
		case DESCEND_STATE:	//�Զ��µ�ģʽ
		{
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			CAN2_BulletRotate_SendMsg((s16)BulletRotate_Data.output,0);
			break;
		}
		case TAKEBULLET_STATE:
		{
			CAN2_BulletRotate_SendMsg((s16)BulletRotate_Data.output,0);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			break;
		}
		case SEMI_ASCEND_STATE:
		{
			CAN2_BulletRotate_SendMsg((s16)BulletRotate_Data.output,0);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			break;
		}
		case SEMI_DESCEND_STATE:
		{
			CAN2_BulletRotate_SendMsg((s16)BulletRotate_Data.output,0);
			CAN2_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
			CAN1_Lift_SendMsg((s16)lift_Data.lf_lift_output,(s16)lift_Data.rf_lift_output,(s16)lift_Data.lb_lift_output,(s16)lift_Data.rb_lift_output);
			break;
		}
		default:
		{
			CAN2_Chassis_SendMsg(0,0,0,0);
			CAN1_Lift_SendMsg(0,0,0,0);
			CAN2_BulletRotate_SendMsg(0,0);
			break;
		}
	}
	
}


void Lift_Cali_Output_Limit(float cm_out,float * cali_out)
{
	if(cm_out>LIFT_CALI_OUTPUT_MAX+3300)
	{
		*cali_out=LIFT_CALI_OUTPUT_MAX+3300;
	}
	else if(cm_out<-LIFT_CALI_OUTPUT_MAX)
	{
		*cali_out=-LIFT_CALI_OUTPUT_MAX;
	}
	else
	{
		*cali_out=cm_out;
	}
}



s32 t_entirety_lf=0;
s32 t_entirety_rf=0;
s32 t_entirety_lb=0;
s32 t_entirety_rb=0;
s32 t_out_lf=0;
s32 t_out_rf=0;
s32 t_out_lb=0;
s32 t_out_rb=0;
#define ENTIRETY_LIFT_P 8	//����PID����
void Entirety_PID(const LIFT_DATA * pliftdata,float cali_send[4])	//����PID����		//2018.2.26DEBUG��
{
	s32 lift_average=(s32)(pliftdata->lf_lift_fdbP+pliftdata->rf_lift_fdbP+pliftdata->lb_lift_fdbP+pliftdata->rb_lift_fdbP)/4;
	if(cali_state_Entirety_PID==1)
	{
		t_entirety_lf=(lift_average-pliftdata->lf_lift_fdbP)*ENTIRETY_LIFT_P;
		t_entirety_rf=(lift_average-pliftdata->rf_lift_fdbP)*ENTIRETY_LIFT_P;
		t_entirety_lb=(lift_average-pliftdata->lb_lift_fdbP)*ENTIRETY_LIFT_P;
		t_entirety_rb=(lift_average-pliftdata->rb_lift_fdbP)*ENTIRETY_LIFT_P;
		
		cali_send[LF]+=t_entirety_lf;
		cali_send[RF]+=t_entirety_rf;
		cali_send[LB]+=t_entirety_lb;
		cali_send[RB]+=t_entirety_rb;
		
		t_out_lf=cali_send[LF];
		t_out_rf=cali_send[RF];
		t_out_lb=cali_send[LB];
		t_out_rb=cali_send[RB];
	}
}

#define LIFT_GYRO_CALI_K 65
s16 t_cali_gyro_lf=0;
s16 t_cali_gyro_rf=0;
s16 t_cali_gyro_lb=0;
s16 t_cali_gyro_rb=0;
void Lift_Cali_GYRO_Compensate(float cali_send[4])	//���������ǵĵ��̱궨�������3.13���������
{	//��chassis_pitch>0ʱǰ�ߺ��  ��Ҫ��ǰ����LF RF���Ӻ�����LB RB
	//��roll>0ʱ����ҵ�	��Ҫ��������LF LB,��������RF RB
	if(cali_state_Entirety_PID==1)
	{
		cali_send[LF]+=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		cali_send[RF]+=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
		cali_send[LB]+=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		cali_send[RB]+=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
		t_cali_gyro_lf=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		t_cali_gyro_rf=LIFT_GYRO_CALI_K*(-Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
		t_cali_gyro_lb=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]-Chassis_GYRO[ROLL]);
		t_cali_gyro_rb=LIFT_GYRO_CALI_K*(Chassis_GYRO[PITCH]+Chassis_GYRO[ROLL]);
	}
	  
}




u16 lift_time_gauge_count=0;
void Lift_Time_Gauge(u8 *trigger)	//����ʱ���Բ���
{
	if(*trigger==1)
	{
		lift_time_gauge_count++;
		if(SetCheck_FrontLift(1)==1||SetCheck_BackLift(1)==1)
		{
			*trigger=0;
			SetCheck_FrontLift(0);
			SetCheck_BackLift(0);
		}
	}
}


void KeyboardRetset(void)	//���ս���������⣬�ͽ��и�λ����
{
	if(KeyBoardData[KEY_CTRL].value==1&&KeyBoardData[KEY_SHIFT].value==1&&KeyBoardData[KEY_Z].value==0&&KeyBoardData[KEY_X].value==0&&KeyBoardData[KEY_C].value==1&&KeyBoardData[KEY_V].value==1)	//������Ƿ�ֹ��ʼ��ʱȫ��Ϊ0
	{
		time_1ms_count=0;
//		RC_Ctl={1024,1024,1024,1024,3,3};
		NVIC_SystemReset();
	}
}

void Data_Init(void)	//�ں˸�λ����������
{
	RC_Ctl.rc.ch0=1024;
	RC_Ctl.rc.ch1=1024;
	RC_Ctl.rc.ch2=1024;
	RC_Ctl.rc.ch3=1024;
	RC_Ctl.rc.switch_left=3;
	RC_Ctl.rc.switch_right=3;
	time_1ms_count=0;
}

void RC_Calibration(void)	//�ϵ���ң��������ֵ����Ĭ�ϲ����Ƚϣ��ж��Ƿ�������������λ
{													//ע���������ң�������ճ�ʼ����
	if(abs(RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)>8)
	{
		NVIC_SystemReset();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************************��������״̬***********************************************/
RC_Ctl_t RC_DATA_ERROR={0};	//��¼����֡����
void Teleconltroller_Data_protect(void)	//ң���������Ա��� 
{
	u8 protect_state=0xC0;	//��λ��ʾ��ǰң���������Ƿ�����	//���2λΪ����λ����Ϊ1	//364-1024-1684
	protect_state|=(abs(RC_Ctl.rc.ch0-1024)<=662);
	protect_state|=(abs(RC_Ctl.rc.ch1-1024)<=662)<<1;
	protect_state|=(abs(RC_Ctl.rc.ch2-1024)<=662)<<2;
	protect_state|=(abs(RC_Ctl.rc.ch3-1024)<=662)<<3;
	protect_state|=(RC_Ctl.rc.switch_left==1||RC_Ctl.rc.switch_left==2||RC_Ctl.rc.switch_left==3)<<4;
	protect_state|=(RC_Ctl.rc.switch_right==1||RC_Ctl.rc.switch_right==2||RC_Ctl.rc.switch_right==3)<<5;
	
	if(protect_state!=0xFF)	{SetWorkState(PROTECT_STATE); RC_DATA_ERROR=RC_Ctl;}
}




////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

/***********************--����״̬--**********************/
void SetWorkState(WorkState_e state)
{
    workState = state;
}


WorkState_e GetWorkState()
{
	return workState;
}


