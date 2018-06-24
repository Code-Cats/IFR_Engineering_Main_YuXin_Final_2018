#include "uphill_auto_lift.h"

extern GYRO_DATA Gyro_Data;
//#define PITCH 0	//�Ƶ�����ȥ��
//#define ROLL 1
//#define YAW 2
float Chassis_GYRO[3]={0};	//pitch roll yaw		AutoChassisAttitude_Lift_V2(Chassis_GYRO[PITCH])
/*************************************************
���ܣ��������ںϸ���������̬������ģ�����
���ݵ�λ���� Gyro_Data.angle
��̨����������������pitch:�� 	roll:����ҵ�	 yaw����ʱ��
���λ��������pitch����  yaw:��ʱ��
�����ֵ��YAW_INIT PITCH_INIT
�ںϺ�chassis����:pitch:��		roll����ҵ�		yaw:��ʱ��
**************************************************/
void Chassis_Attitude_Angle_Convert(void)	//�ۺϵó�������̬
{
	float deviation_pitch=0;//PITCH_GYRO_INIT-yunMotorData.pitch_fdbP;	//���ڵ�����˵����̨��ֵ���ǵ�������̨����ϵ�ϵ�λ��
	float deviation_yaw=0;//YAW_INIT-yunMotorData.yaw_fdbP;
	//��yaw��������ƣ���׼���㣨-180����+180��
	Chassis_GYRO[PITCH]=-Gyro_Data.angle[PITCH]-deviation_pitch*360.0f/8192;	//��Ϊ��̨���λ�÷������������������������෴pitch��-2
	Chassis_GYRO[ROLL]=Gyro_Data.angle[ROLL]-3;	//roll	-3Ϊ��ֹʱ����
	Chassis_GYRO[YAW]=Gyro_Data.angle[YAW]+deviation_yaw*360.0f/8192;	//��Ϊ��̨���λ�÷�������������������������ͬ
 
	//����-180_+180
	Chassis_GYRO[YAW]=Chassis_GYRO[YAW]>180?Chassis_GYRO[YAW]-360:Chassis_GYRO[YAW];
	Chassis_GYRO[YAW]=Chassis_GYRO[YAW]<-180?Chassis_GYRO[YAW]+360:Chassis_GYRO[YAW];
}




extern LIFT_DATA lift_Data;
extern u32 time_1ms_count;

extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];

#define AUTOCHASSIS_LIFT 12	//���൱��KP

	//��ʵ���ö��ڳ������ϵ��˵����ǰ����ʱ���ȵ���˵����ǰ����ʱ��̬���Ϊǰ���󸽣����������
	//														��ǰ����ʱǰ�ȵ���˵����ǰ���٣����ٶȸ���ʱ���Ϊ��
	//��ʵ����IMU_Uranus��оƬ��ǰ����ʱ��ǰ���ٶ�������Ϊǰ��Ϊacc��0λ	//����200���ٶ�4����������㣬����ϵ��Ϊ4/200
#define ATTITUDE_ACC_COMPENSATE (5.0f/200)	//���ٶȶ�����̬����ϵ��

#define TILT 1	//��б״̬
#define STAEDY_REAL 0	//ƽ��״̬
#define STAEDY_ADJUST 2	//������ƽ��״̬
u8 Adjust_Statu=STAEDY_REAL;

void AutoChassisAttitude_Lift_V2(float chassis_pitch_raw)	//�Զ�������̬	//pitch������Ϊǰ��	//ע�����lift_taskǰ��
{
	static float chassis_pitch=0;
	static float ka=0.05f;
	
//	chassis_pitch_raw-=Gyro_Data.acc[0]*ATTITUDE_ACC_COMPENSATE;	//��IMU_Uranus�����㷨ȱ�ݽ��в���
	
	chassis_pitch=chassis_pitch*(1-ka)+chassis_pitch_raw*ka;
	
	if(GetWorkState()==NORMAL_STATE&&IMU_Check_Useless_State==0)	//����������ʧЧ����//����CTRL����   &&KeyBoardData[KEY_CTRL].value==1		//�Ȳ������Ա���ԣ����ڱ������
	{
		switch(Adjust_Statu)
		{
			case STAEDY_REAL:
			{
				static u16 tilt_change_count=0;	//����ֵ���Ч�����ã���ʹ���������
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
				if(chassis_pitch>(8)&&tilt_change_count<0xFFFE)	//������ֵ	Ϊ7ʱ�����ⴥ������			+Gyro_Data.acc[0]*4.0f/200��̨��ˮƽӰ��ϴ�
				{
					tilt_change_count++;
				}
				else if((chassis_pitch<(-8)&&tilt_change_count<0xFFFE))		//+Gyro_Data.acc[0]*4.0f/200
				{
					tilt_change_count++;
				}
				else
				{
					tilt_change_count=0;
				}
				
				if(tilt_change_count>100)
				{
					Adjust_Statu=TILT;
					tilt_change_count=0;
				}
				break;
			}
			case TILT:
			{
				static u16 staedy_adjust_count=0;
				if(chassis_pitch>0)	//ǰ��������ͨ����ǰ���������ǰ�޷�����������Ϻ�
				{
					if(lift_Data.lf_lift_tarP<=LIFT_DISTANCE_FALL)	//����ǰ������
					{
						lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
						lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
					}
					else
					{
						lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
						lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;	//����ǰ���Ϊ��׼
					}
					
				
				}
				else	//ǰ��������ͨ���º��������º��޷������������ǰ
				{
					if(lift_Data.lb_lift_tarP<=LIFT_DISTANCE_FALL)	//�º�
					{
						lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
						lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;	//����ǰ���Ϊ��׼
					}
					else
					{
						lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
						lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
					}
					
					
				}
				
//				lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
//				lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;	//����ǰ���Ϊ��׼
//				lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
//				lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
				
////////				if(chassis_pitch>0)	//ǰ��������ͨ����ǰ���������ǰ�޷�����������Ϻ�	���ڴ˴�дǿ���½���ǰ�º�
////////				{
////////					if(abs(lift_Data.lf_lift_tarP-FALL)>)
////////				}
////////				else	//ǰ��������ͨ���º��������º��޷������������ǰ
////////				{
////////					
////////				}
				
//				if(abs(lift_Data.lf_lift_tarP-FALL)>10&&abs(lift_Data.lb_lift_tarP-FALL)>10)	//δ��������ߵ�	��Ч
//				{
//					if(time_1ms_count%20==0)
//					{
//						lift_Data.lf_lift_tarP-=5;
//						lift_Data.rf_lift_tarP-=5;
//						lift_Data.lb_lift_tarP-=5;
//						lift_Data.rb_lift_tarP-=5;
//					}
//				}
				
				if(abs(chassis_pitch)<2.2f&&staedy_adjust_count<0xFFFE)
				{
					staedy_adjust_count++;
				}
				else
				{
					staedy_adjust_count=0;
				}
				
				if(staedy_adjust_count>300)	//�ȶ���	//������
				{
					Adjust_Statu=STAEDY_ADJUST;
					staedy_adjust_count=0;
				}
				break;
			}
			case STAEDY_ADJUST:
			{
				static u16 staedy_real_count=0;
				if(abs(chassis_pitch)>4)
				{
					Adjust_Statu=TILT;
				}
				else
				{
					if(abs(lift_Data.lf_lift_tarP-lift_Data.lb_lift_tarP)<300&&staedy_real_count<0xFFFE)
					{
						staedy_real_count++;
					}
				}
				
				if(staedy_real_count>150)	//0.3s
				{
					Adjust_Statu=STAEDY_REAL;
					staedy_real_count=0;
				}
				break;
			}
		}	
	}
	
	
	lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lf_lift_tarP;	//�����г�
	lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lf_lift_tarP;
	
	lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rf_lift_tarP;	//�����г�
	lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rf_lift_tarP;
	
	lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lb_lift_tarP;	//�����г�
	lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lb_lift_tarP;
	
	lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rb_lift_tarP;	//�����г�
	lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rb_lift_tarP;
}


/***********************************************************
����V2�汾�����ֻ���ʱ��������������˼��
1.������ʱ�����ڼ�⵽��ƽ�����һ��ֵʱ��ʱ�������ƽ�棨������ֵ���³�����ӳ����⣩
2.�ڷ����׶α��⣬��pitch>0,����ǰ������TITL�����׶�
	���������Ĳ���������ͨ���½������ȫͨ���½����
	
***********************************************************/

