#include "chassis.h"

CHASSIS_DATA chassis_Data={0};

PID_GENERAL PID_Chassis_Speed[4]={PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT,PID_CHASSIS_SPEED_DEFAULT};
PID_GENERAL PID_Chassis_Follow=PID_CHASSIS_FOLLOW_DEFAULT;

FIRST_ORDER_FILTER Yaw_Follow_Filter=YAW_FOLLOW_FILTER_DEFAULT;

s16 Chassis_Vx=0;
s16 Chassis_Vy=0;
s16 Chassis_Vw=0;

extern RC_Ctl_t RC_Ctl;
extern GYRO_DATA Gyro_Data;
extern u32 time_1ms_count;

extern AutoAimBulletTypeDef AutoAimBulletData;

extern u8 Trailer_Turn180_fbdStatu;	//180��ת��λ��־λ������������
extern u8 Trailer_statu;

#define K_SPEED 11
s32 t_Vw_PID=0;

u8 Chassis_Control_RCorPC=RC_CONTROL;
s8 Chassis_Control_Heading=1;	//����ǰ������


s16 test_chassis_vx=100;
s16 test_chassis_vx_=0;
void Remote_Task(void)
{
	if(GetWorkState()==NORMAL_STATE)	//����PID��λ
	{
		for(int i=0;i<4;i++)
		{
			PID_Chassis_Speed[i].k_i=CHASSIS_SPEED_PID_I;
			PID_Chassis_Speed[i].i_sum_max=CHASSIS_SPEED_I_MAX;
		}
	}
	
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==TAKEBULLET_STATE||GetWorkState()==SEMI_ASCEND_STATE||GetWorkState()==SEMI_DESCEND_STATE)	//ģʽ�л�
	{
		if(RC_Ctl.key.v_h!=0||RC_Ctl.key.v_l!=0||abs(RC_Ctl.mouse.x)>3)
		{
			Chassis_Control_RCorPC=PC_CONTROL;
		}
		else if(abs(RC_Ctl.rc.ch0-1024)>3||abs(RC_Ctl.rc.ch1-1024)>3||abs(RC_Ctl.rc.ch2-1024)>3)
		{
			Chassis_Control_RCorPC=RC_CONTROL;
		}
	}
	
	
	if((Trailer_statu==1&&Trailer_Turn180_fbdStatu==0)||AutoAimBulletData.control_state==1)	//�����ģʽ���ܿ�
	{
		if(AutoAimBulletData.control_state!=1)
		{
			Chassis_Vx=0;
			Chassis_Vy=0;
		}
	}
	else if(1)
	{
		if(Chassis_Control_RCorPC==RC_CONTROL)
		{
			RC_Control_Chassis();
		}
		else if(Chassis_Control_RCorPC==PC_CONTROL)
		{
			PC_Control_Chassis(&Chassis_Vx,&Chassis_Vy,&Chassis_Vw);
		}
	}

//	if(GetWorkState()==NORMAL_STATE||GetWorkState()==TAKEBULLET_STATE||GetWorkState()==SEMI_ASCEND_STATE||GetWorkState()==SEMI_DESCEND_STATE)
//	{
//		Chassis_Vx=RC_Ctl.rc.ch1-1024;
//	}
//	
//	if(GetWorkState()==NORMAL_STATE||GetWorkState()==TAKEBULLET_STATE||GetWorkState()==SEMI_ASCEND_STATE||GetWorkState()==SEMI_DESCEND_STATE)	//�������������ң���������������(�Զ�)�ǵ�ģʽ�½��ɳ����Զ�����
//	{
//		Chassis_Vw=RC_Ctl.rc.ch2-1024;
//	}
//	Chassis_Vy=RC_Ctl.rc.ch0-1024;
	test_chassis_vx_=Chassis_Vx;
	
	if(Chassis_Control_Heading==-1)	//����������Ϣ	//��������ִ�з��������Ӱ��RC_Control�еĸ�ֵ���ʷ��ڶ���
	{
//		s16 chassis_vx=Chassis_Vx;
//		s16 chassis_vy=Chassis_Vy;
//		Chassis_Vx=-Chassis_Vx;
//		Chassis_Vy=-Chassis_Vy;
//		test_chassis_vx_=-test_chassis_vx;
//		test_chassis_vx_=-test_chassis_vx_;
	}
	else
	{
//		test_chassis_vx_=test_chassis_vx;
	}

	chassis_Data.lf_wheel_tarV=(Chassis_Vx+Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.rf_wheel_tarV=(-Chassis_Vx+Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.lb_wheel_tarV=(Chassis_Vx-Chassis_Vy+Chassis_Vw)*K_SPEED;
	chassis_Data.rb_wheel_tarV=(-Chassis_Vx-Chassis_Vy+Chassis_Vw)*K_SPEED;
	

	Overall_Motion_Ratio_Protect(&chassis_Data);	//�����ٶȱ���
///////////////////////////////////////////////////////////////// 
//	chassis_Data.lf_wheel_tarV=remote_tem;
//	chassis_Data.rf_wheel_tarV=remote_tem;
//	chassis_Data.lb_wheel_tarV=remote_tem;
//	chassis_Data.rb_wheel_tarV=remote_tem;
	
	chassis_Data.lf_wheel_output=PID_General(chassis_Data.lf_wheel_tarV,chassis_Data.lf_wheel_fdbV,&PID_Chassis_Speed[LF]);
	chassis_Data.rf_wheel_output=PID_General(chassis_Data.rf_wheel_tarV,chassis_Data.rf_wheel_fdbV,&PID_Chassis_Speed[RF]);
	chassis_Data.lb_wheel_output=PID_General(chassis_Data.lb_wheel_tarV,chassis_Data.lb_wheel_fdbV,&PID_Chassis_Speed[LB]);
	chassis_Data.rb_wheel_output=PID_General(chassis_Data.rb_wheel_tarV,chassis_Data.rb_wheel_fdbV,&PID_Chassis_Speed[RB]);
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==ASCEND_STATE||GetWorkState()==TAKEBULLET_STATE)
	{
		Extended_Integral_PID(&chassis_Data);
	}
	
//	CAN_Chassis_SendMsg(chassis_Data.lf_wheel_output,chassis_Data.rf_wheel_output,chassis_Data.lb_wheel_output,chassis_Data.rb_wheel_output);
}



void RC_Control_Chassis(void)
{
	static s16 Chassis_Vx_last=0;
	static s16 Chassis_Vy_last=0;
	
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==TAKEBULLET_STATE||GetWorkState()==SEMI_ASCEND_STATE||GetWorkState()==SEMI_DESCEND_STATE)	//��ʱ����ȡ���ܿ�
	{
		
		if(time_1ms_count%1==0)
		{
			if(RC_Ctl.rc.ch1-1024-Chassis_Vx_last>1&&RC_Ctl.rc.ch1-1024>10)	//ֻ��ǰ������ʱ��Ч��
			{
				Chassis_Vx+=1;
			}
			else if(RC_Ctl.rc.ch1-1024-Chassis_Vx_last<-1&&RC_Ctl.rc.ch1-1024<-10)	//ֻ�ں��˼���ʱ��Ч
			{
				Chassis_Vx-=1;
			}
			else
			{
				Chassis_Vx=RC_Ctl.rc.ch1-1024;
			}
		}
		Chassis_Vw=RC_Ctl.rc.ch2-1024;	//Ų��ȡ����������
//		Chassis_Vx=RC_Ctl.rc.ch1-1024;	//����Ϊб�º���
		Chassis_Vx_last=Chassis_Vx;
	}
	
	if(GetWorkState()==ASCEND_STATE)
	{
		Chassis_Vw=RC_Ctl.rc.ch2-1024;	//�ϵ�ʱҲ���Բ���
	}
	
	
	if(time_1ms_count%1==0)
	{
		if(RC_Ctl.rc.ch0-1024-Chassis_Vy_last>1&&RC_Ctl.rc.ch0-1024>10)
		{
			Chassis_Vy+=1;
		}
		else if(RC_Ctl.rc.ch0-1024-Chassis_Vy_last<-1&&RC_Ctl.rc.ch0-1024<-10)	//ɲ����������
		{
			Chassis_Vy-=1;
		}
		else
		{
			Chassis_Vy=RC_Ctl.rc.ch0-1024;
		}
	}
	Chassis_Vy_last=Chassis_Vy;

//	Chassis_Vy=RC_Ctl.rc.ch0-1024;
}


#define CHASSIS_V_PC	660
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
void PC_Control_Chassis(s16 * chassis_vx,s16 * chassis_vy,s16 * chassis_vw)	//1000Hz	//��Ӣ�۲�ͬ����Vw�Ŀ���
{
	static s16 chassis_vx_record=0;
	static s16 chassis_vy_record=0;
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==TAKEBULLET_STATE||GetWorkState()==SEMI_ASCEND_STATE||GetWorkState()==SEMI_DESCEND_STATE)
	{
		if(time_1ms_count%2==0)
		{
			if(KeyBoardData[KEY_W].value!=0)
			{
				if(chassis_vx_record<CHASSIS_V_PC&&chassis_vx_record>=0)
				{
					chassis_vx_record+=2;
				}
				else if(chassis_vx_record<0)
				{
					chassis_vx_record=0;
				}
			}
			else if(KeyBoardData[KEY_S].value!=0)
			{
				if(chassis_vx_record>-CHASSIS_V_PC&&chassis_vx_record<=0)
				{
					chassis_vx_record-=2;
				}
				else if(chassis_vx_record>0)
				{
					chassis_vx_record=0;
				}
			}
			else
			{
				chassis_vx_record=0;
			}
			///////////////////////////////////////
			if(KeyBoardData[KEY_D].value!=0)
			{
				if(chassis_vy_record<CHASSIS_V_PC&&chassis_vy_record>=0)
				{
					chassis_vy_record+=2;
				}
				else if(*chassis_vy<0)
				{
					chassis_vy_record=0;
				}
			}
			else if(KeyBoardData[KEY_A].value!=0)
			{
				if(chassis_vy_record>-CHASSIS_V_PC&&chassis_vy_record<=0)
				{
					chassis_vy_record-=2;
				}
				else if(chassis_vy_record>0)
				{
					chassis_vy_record=0;
				}
			}
			else
			{
				chassis_vy_record=0;
			}
			*chassis_vx=chassis_vx_record;
			*chassis_vy=chassis_vy_record;
		}

		*chassis_vw=RC_Ctl.mouse.x*8;
		if(KeyBoardData[KEY_Q].value!=0)
		{
			*chassis_vw=-300;
		}
		
		if(KeyBoardData[KEY_E].value!=0)
		{
			*chassis_vw=300;
		}
		
	}
	else
	{
	}
		
}



#define CHASSIS_SPEEDMAX 8500.0f
void Overall_Motion_Ratio_Protect(CHASSIS_DATA* chassis_data)	//�������ٱ�������,һ��Ҫ�����������ٽ��������
{
	s32 chassis_tarV_max=0;
	float chassis_protect_k=1;
	
	chassis_tarV_max=chassis_data->lf_wheel_tarV>chassis_data->rf_wheel_tarV?chassis_data->lf_wheel_tarV:chassis_data->rf_wheel_tarV;
	chassis_tarV_max=chassis_tarV_max>chassis_data->lb_wheel_tarV?chassis_tarV_max:chassis_data->lb_wheel_tarV;
	chassis_tarV_max=chassis_tarV_max>chassis_data->rb_wheel_tarV?chassis_tarV_max:chassis_data->rb_wheel_tarV;
	
	if(chassis_tarV_max>CHASSIS_SPEEDMAX)	//���������ϵ��Ϊ�˼��ټ���������С�������IF��
	{
		chassis_protect_k=CHASSIS_SPEEDMAX/chassis_tarV_max;
		
		chassis_data->lf_wheel_tarV*=chassis_protect_k;
		chassis_data->rf_wheel_tarV*=chassis_protect_k;
		chassis_data->lb_wheel_tarV*=chassis_protect_k;
		chassis_data->rb_wheel_tarV*=chassis_protect_k;
	}
	
}


#define CHASSIS_INTEGRAL_PID_KP 3
#define CHASSIS_INTEGRAL_PID_KI 0.01f
#define CHASSIS_INTEGRAL_PID_I_SUM_LIM 1000
void Extended_Integral_PID(CHASSIS_DATA* chassis_data)	//��չ������PID�����������⶯������	2018.4.19
{
	float tarv_sum=abs(chassis_data->lf_wheel_tarV)+abs(chassis_data->rf_wheel_tarV)+abs(chassis_data->lb_wheel_tarV)+abs(chassis_data->rb_wheel_tarV);
	float fdbv_sum=abs(chassis_data->lf_wheel_fdbV)+abs(chassis_data->rf_wheel_fdbV)+abs(chassis_data->lb_wheel_fdbV)+abs(chassis_data->rb_wheel_fdbV);
	float expect[4]={0};
	float error[4]={0};
	static float inte[4];
	s32 output_compensation[4];
	
	if(abs(tarv_sum)<0.1f)	//�൱�ڱ�����Ϊ0
	{
		expect[LF]=0;
		expect[RF]=0;
		expect[LB]=0;
		expect[RB]=0;
	}
	else
	{
		expect[LF]=fdbv_sum*chassis_data->lf_wheel_tarV/tarv_sum;
		expect[RF]=fdbv_sum*chassis_data->rf_wheel_tarV/tarv_sum;
		expect[LB]=fdbv_sum*chassis_data->lb_wheel_tarV/tarv_sum;
		expect[RB]=fdbv_sum*chassis_data->rb_wheel_tarV/tarv_sum;
	}
	
	error[LF]=expect[LF]-chassis_data->lf_wheel_fdbV;
	error[RF]=expect[RF]-chassis_data->rf_wheel_fdbV;
	error[LB]=expect[LB]-chassis_data->lb_wheel_fdbV;
	error[RB]=expect[RB]-chassis_data->rb_wheel_fdbV;
	
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	inte[LF]+=error[LF]*CHASSIS_INTEGRAL_PID_KI;
	
	for(int id=0;id<4;id++)
	{
		inte[id]=inte[id]>CHASSIS_INTEGRAL_PID_I_SUM_LIM?CHASSIS_INTEGRAL_PID_I_SUM_LIM:inte[id];
		inte[id]=inte[id]<-CHASSIS_INTEGRAL_PID_I_SUM_LIM?-CHASSIS_INTEGRAL_PID_I_SUM_LIM:inte[id];
	}
	
	output_compensation[LF]=(s32)(error[LF]*CHASSIS_INTEGRAL_PID_KP+inte[LF]);
	output_compensation[RF]=(s32)(error[RF]*CHASSIS_INTEGRAL_PID_KP+inte[RF]);
	output_compensation[LB]=(s32)(error[LB]*CHASSIS_INTEGRAL_PID_KP+inte[LB]);
	output_compensation[RB]=(s32)(error[RB]*CHASSIS_INTEGRAL_PID_KP+inte[RB]);
	
	for(int id=0;id<4;id++)
	{
		output_compensation[id]=output_compensation[id]>4000?4000:output_compensation[id];
		output_compensation[id]=output_compensation[id]<-4000?-4000:output_compensation[id];
	}
	
	chassis_data->lf_wheel_output+=output_compensation[LF];
	chassis_data->rf_wheel_output+=output_compensation[RF];
	chassis_data->lb_wheel_output+=output_compensation[LB];
	chassis_data->rb_wheel_output+=output_compensation[RB];
}




