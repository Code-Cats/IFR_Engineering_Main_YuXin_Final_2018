#include "lift.h"
#include "main.h"

LIFT_DATA lift_Data={0};
PID_GENERAL PID_Lift_Position[4]={PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT,PID_LIFT_POSITION_DEFAULT};
PID_GENERAL PID_Lift_Speed[4]={PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT,PID_LIFT_SPEED_DEFAULT};

extern u32 time_1ms_count;
extern RC_Ctl_t RC_Ctl;
extern LIFT_POSITION_ENCODER lift_position_encoder[4];
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern u8 cali_state_Entirety_PID;

s16 lift_k_tem=0;
#define LIFT_K 0.8
void Lift_Task(void)
{
	///////////////////////////////////////////////////////////////////
//	lift_Data.lf_lift_tarP=LIFT_tarP;
//	lift_Data.rf_lift_tarP=LIFT_tarP;
//	lift_Data.lb_lift_tarP=LIFT_tarP;
//	lift_Data.rb_lift_tarP=LIFT_tarP;
	if(GetWorkState()==NORMAL_STATE||GetWorkState()==SEMI_ASCEND_STATE||GetWorkState()==SEMI_DESCEND_STATE)	//�����⼸��ģʽ�ֶ��ɿ�����
	{
		
		if(RC_Ctl.rc.switch_left==RC_SWITCH_UP)	//���ϣ������ɾ������Ϊ��λģʽ�������޷��ڰ��Զ����µ�ʱ���������������ݶ�
		{
			if(time_1ms_count%10==0)
			{
				switch (RC_Ctl.rc.switch_right)
				{
					case 1:
					{
						lift_Data.lf_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rf_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.lb_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rb_lift_tarP+=(s16)(12*(RC_Ctl.rc.ch3-1024)/660.0);
						break;
					}
					case 2:
					{
						lift_Data.lf_lift_tarP+=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rf_lift_tarP+=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
							
						lift_Data.lb_lift_tarP-=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
						lift_Data.rb_lift_tarP-=(s16)(7*(RC_Ctl.rc.ch3-1024)/660.0);
						break;
					}
					case 3:
					{
						
					}
				}
			}
			
		}
		
		if(KeyBoardData[KEY_Z].value==1)	//�µ���ǰ��
		{
			SetCheck_FrontLift(1);
		}
		
		if(KeyBoardData[KEY_X].value==1)	//�µ������
		{
			SetCheck_BackLift(1);
		}
	}
	
	
	if(GetWorkState()!=CALI_STATE&&GetWorkState()!=PREPARE_STATE&&GetWorkState()!=CHECK_STATE)	//�궨״̬�²������г�
	{
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lf_lift_tarP;	//�����г�
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lf_lift_tarP;
		
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rf_lift_tarP;	//�����г�
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rf_lift_tarP;
		
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lb_lift_tarP;	//�����г�
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lb_lift_tarP;
		
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rb_lift_tarP;	//�����г�
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rb_lift_tarP;
	}
		//�෴�� �Խǻ���	//�ֻ������ˣ��������ĵط�Ų������
	lift_Data.lf_lift_tarV=(int32_t)PID_General(lift_Data.lf_lift_tarP,lift_Data.lf_lift_fdbP,&PID_Lift_Position[LF]);	//λ�û�PID����
	lift_Data.rf_lift_tarV=(int32_t)PID_General(lift_Data.rf_lift_tarP,lift_Data.rf_lift_fdbP,&PID_Lift_Position[RF]);
	lift_Data.lb_lift_tarV=(int32_t)PID_General((lift_Data.lb_lift_tarP),lift_Data.lb_lift_fdbP,&PID_Lift_Position[LB]);
	lift_Data.rb_lift_tarV=(int32_t)PID_General((lift_Data.rb_lift_tarP),lift_Data.rb_lift_fdbP,&PID_Lift_Position[RB]);
	
	lift_Data.lf_lift_output=PID_General(lift_Data.lf_lift_tarV,lift_Data.lf_lift_fdbV,&PID_Lift_Speed[LF]);	//�ٶȻ�PID����
	lift_Data.rf_lift_output=PID_General(lift_Data.rf_lift_tarV,lift_Data.rf_lift_fdbV,&PID_Lift_Speed[RF]);
	lift_Data.lb_lift_output=PID_General(lift_Data.lb_lift_tarV,lift_Data.lb_lift_fdbV,&PID_Lift_Speed[LB]);
	lift_Data.rb_lift_output=PID_General(lift_Data.rb_lift_tarV,lift_Data.rb_lift_fdbV,&PID_Lift_Speed[RB]);
	
//	CAN_Lift_SendMsg(lift_Data.lf_lift_output,lift_Data.rf_lift_output,lift_Data.lb_lift_output,lift_Data.rb_lift_output);
	
}


/*****************************************�������궨����**********************************************/
LiftCaliState_e liftcaliState=UP_STATE;
u8 Lift_Cali(void)
{
	switch (liftcaliState)
	{
		case UP_STATE:
		{
			PID_Lift_Speed[LF].k_i=2*LIFT_SPEED_PID_I;
			PID_Lift_Speed[RF].k_i=2*LIFT_SPEED_PID_I;
			PID_Lift_Speed[LB].k_i=2*LIFT_SPEED_PID_I;
			PID_Lift_Speed[RB].k_i=2*LIFT_SPEED_PID_I;
			
			lift_Data.lf_lift_tarP=70;
			lift_Data.rf_lift_tarP=70;
			lift_Data.lb_lift_tarP=70;
			lift_Data.rb_lift_tarP=70;

			if(lift_Data.lf_lift_fdbP>30&&lift_Data.rf_lift_fdbP>30&&lift_Data.lb_lift_fdbP>30&&lift_Data.rb_lift_fdbP>30)
			{
				liftcaliState=WAIT_STATE;
				cali_state_Entirety_PID=1;
			}
			break;
		}
		case WAIT_STATE:
		{
			PID_Lift_Speed[LF].k_i=0;
			PID_Lift_Speed[RF].k_i=0;
			PID_Lift_Speed[LB].k_i=0;
			PID_Lift_Speed[RB].k_i=0;
				
			lift_Data.lf_lift_tarP=-10000;
			lift_Data.rf_lift_tarP=-10000;
			lift_Data.lb_lift_tarP=-10000;
			lift_Data.rb_lift_tarP=-10000;
			if((lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<-25)	//���ѹ��ٶ����
			{
				liftcaliState=DOWN_STATE;
			}
			break;
		}
		case DOWN_STATE:
		{
			if(abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<10)	//��һ����һ��λ��ֵ֮��//��Ϊ��
			{
				lift_position_encoder[LF].turns=0;
				lift_position_encoder[RF].turns=0;
				lift_position_encoder[LB].turns=0;
				lift_position_encoder[RB].turns=0;
				
				lift_Data.lf_lift_tarP=0;
				lift_Data.rf_lift_tarP=0;
				lift_Data.lb_lift_tarP=0;
				lift_Data.rb_lift_tarP=0;
				
				PID_Lift_Speed[LF].k_i=LIFT_SPEED_PID_I;
				PID_Lift_Speed[RF].k_i=LIFT_SPEED_PID_I;
				PID_Lift_Speed[LB].k_i=LIFT_SPEED_PID_I;
				PID_Lift_Speed[RB].k_i=LIFT_SPEED_PID_I;
				
				cali_state_Entirety_PID=0;	//����
				liftcaliState=OK_STATE;
			}
			break;
		}
		case OK_STATE:
		{
			return 1;
		}
	}
	return 0;
}



u8 cali_state_Entirety_PID=0;	//����PID�ڱ궨�����еĴ���ʱ����־λ
u8 Calibration_state=0;
s16 t_fdbV_sum=0;
u16 t_cali_count=0;
/*****************************************�ڶ����궨����**********************************************/
void Lift_Calibration(void)	//��������ϵ�궨
{
//	u32 Record_Last=0;	//�궨��������һ�νǶȼ�¼ֵ
	u8 Calibration_Time_Count=0;	//��ʱ
	
	
	SetWorkState(CALI_STATE);
	PID_Lift_Speed[LF].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[RF].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[LB].k_i=2*LIFT_SPEED_PID_I;
	PID_Lift_Speed[RB].k_i=2*LIFT_SPEED_PID_I;
	
	lift_Data.lf_lift_tarP=100;
	lift_Data.rf_lift_tarP=100;
	lift_Data.lb_lift_tarP=100;
	lift_Data.rb_lift_tarP=100;
	while(lift_Data.lf_lift_fdbP<40||lift_Data.rf_lift_fdbP<40||lift_Data.lb_lift_fdbP<40||lift_Data.rb_lift_fdbP<40)	//�ȴ�������һ���߶�
	{;}

	
	PID_Lift_Speed[LF].k_i=0;
	PID_Lift_Speed[RF].k_i=0;
	PID_Lift_Speed[LB].k_i=0;
	PID_Lift_Speed[RB].k_i=LIFT_SPEED_PID_I;
		
	lift_Data.lf_lift_tarP=-10000;
	lift_Data.rf_lift_tarP=-10000;
	lift_Data.lb_lift_tarP=-10000;
	lift_Data.rb_lift_tarP=-10000;
		
	cali_state_Entirety_PID=1;
	
	Calibration_Time_Count=time_1ms_count;	//	��¼��ǰϵͳʱ��
		
	while(Calibration_state!=1||lift_position_encoder[LF].turns!=0||lift_position_encoder[RF].turns!=0||lift_position_encoder[LB].turns!=0||lift_position_encoder[RB].turns!=0)
	{
		if((time_1ms_count-Calibration_Time_Count)>1000)	//����ʱ�����������������ԭ��
		{
			
			if(time_1ms_count%500==0)
			{
				t_cali_count++;
				t_fdbV_sum=abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV);
				if(abs(lift_Data.lf_lift_fdbV+lift_Data.rf_lift_fdbV+lift_Data.lb_lift_fdbV+lift_Data.rb_lift_fdbV)<15)	//��һ����һ��λ��ֵ֮��//��Ϊ��
				{
					
					Calibration_state=1;
					lift_position_encoder[LF].turns=0;
					lift_position_encoder[RF].turns=0;
					lift_position_encoder[LB].turns=0;
					lift_position_encoder[RB].turns=0;
					
					lift_Data.lf_lift_tarP=0;
					lift_Data.rf_lift_tarP=0;
					lift_Data.lb_lift_tarP=0;
					lift_Data.rb_lift_tarP=0;
					
					PID_Lift_Speed[LF].k_i=LIFT_SPEED_PID_I;
					PID_Lift_Speed[RF].k_i=LIFT_SPEED_PID_I;
					PID_Lift_Speed[LB].k_i=LIFT_SPEED_PID_I;
					PID_Lift_Speed[RB].k_i=LIFT_SPEED_PID_I;
					
					cali_state_Entirety_PID=0;	//����
					SetWorkState(NORMAL_STATE);	//3.16��ʱ��
				}
				//Record_Last= lift_position_encoder[LF].calc+lift_position_encoder[RF].calc+lift_position_encoder[LB].calc+lift_position_encoder[RB].calc;
			}

		}
		Calibration_Time_Count=0;
	}
}

