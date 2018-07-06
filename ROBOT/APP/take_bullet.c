#include "take_bullet.h"

//#define VALVE_ISLAND 0		//电磁阀控制位定义
//#define VALVE_BULLET_HORIZONTAL 1	//左右平移
//#define VALVE_BULLET_CLAMP 2	//夹紧
//#define VALVE_BULLET_STORAGE 3	//弹药舱补弹
//#define VALVE_TRAILER 5	//拖车

TakeBulletState_e TakeBulletState=BULLET_OTHER;	//(自动)取弹标志位

#define BULLETROTATE_OTHER	0	//非取弹位置
#define BULLETROTATE_WAITING	750//650	//等待（对位）时位置
#define BULLETROTATE_ACQUIRE	1100	//取弹位置
#define BULLETROTATE_POUROUT	160	//倒弹位置
#define BULLETROTATE_THROWOUT	280//310	//抛出位置

//#define LIFT_DISTANCE_FALL 30
#define LIFT_BULLET_POUROUT	540	//倒弹时升起


#define STEER_UP_L_INIT 500//1210	//2500
#define STEER_UP_R_INIT 2500//1950	//500
#define STEER_DOWN_L_INIT 1600//1650	//1000小-内
#define STEER_DOWN_R_INIT 1430//1550	//2180小-外
//2018.5.4
#define STEER_UP_L_REVERSAL 1900//2500	//2500
#define STEER_UP_R_REVERSAL 1100//630	//500
#define STEER_DOWN_L_REVERSAL 1100	//1000
#define STEER_DOWN_R_REVERSAL 1930	//2180
u16 Steer_Send[4]={STEER_UP_L_INIT,STEER_UP_R_INIT,STEER_DOWN_L_INIT,STEER_DOWN_R_INIT};

u8 valve_fdbstate[6]={0};	//记录是否伸出的反馈标志
u8 servo_fdbstate[2]={0};
const u32 valve_GOODdelay[6]={300,400,120,1000,1000,1000};	//待加入，延时参数	//500是伪延时
const u32 valve_POORdelay[6]={300,400,120,1000,1000,1000};	//待加入，延时参数
const u32 servo_GOODdelay[2]={2000,1000};	//延时参数	//第一段为2000是将子弹落下的延时也加进去了，因为舵机翻转和子弹下落必须是连在一体的
const u32 servo_POORdelay[2]={1000,1000};	//延时参数


extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;
extern u32 time_1ms_count;
extern LIFT_DATA lift_Data;
//extern BULLETLIFT_MOTOR_DATA bulletlift_Motor_Data[2];	//分区赛后弃用
extern BULLETROTATE_DATA BulletRotate_Data;	//国赛版


void TakeBullet_Control_Center(void)	//在每个状态都有运行
{
	static u8 swicth_Last_state=0;	//右拨杆
	
	static u8 valve_last[6]={0};	//记录上一次数值	//保持与工程车兼容性
	static u8 servo_last[2]={0};	//记录上一次数值	//保持与工程车兼容性
	
	static u32 valve_startGOOD_time[6]={0};	//记录顺向触发时间	//保持与工程车兼容性
	static u32 servo_startGOOD_time[2]={0};	//记录顺向触发时间	//保持与工程车兼容性
	static u32 valve_startPOOR_time[6]={0};	//记录逆向触发时间	//保持与工程车兼容性
	static u32 servo_startPOOR_time[2]={0};	//记录逆向触发时间	//保持与工程车兼容性
	
	static WorkState_e State_Record=CHECK_STATE;
	
	static TakeBulletState_e takebulletstate_last=BULLET_OTHER;
	
	if(GetWorkState()==TAKEBULLET_STATE)	//5.9更新//上一版--》//取弹升降给DOWN-MID，前伸出发-夹紧一套给DOWN-MID-->DOWN-DOWN;舵机旋转给DOWN-MID-->DOWN-UP
	{
		if(State_Record!=TAKEBULLET_STATE)
		{
			TakeBulletState=BULLET_WAITING;
		}
		
		if(RC_Ctl.rc.ch3-1024>80&&TakeBulletState==BULLET_WAITING)	/////////////////////////////修改操作模式时需要修改
		{
			TakeBulletState=BULLET_ACQUIRE1;
		}
		else if(RC_Ctl.rc.ch3-1024<-80)
		{
			TakeBulletState=BULLET_WAITING;
		}			
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
			
	State_Record=GetWorkState();
	
	
	switch(TakeBulletState)	//自动取弹过程
	{
		case BULLET_WAITING:	//等待取弹动作（对位）状态
		{
			ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//气缸松开
			{
				BulletRotate_Data.tarP=BULLETROTATE_WAITING;
				if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<30)	//电机收回
				{
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL]=0;
				}
			}
			
			lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
			break;
		}
		case BULLET_ACQUIRE1:	//前伸、夹紧、抬起动作	称之为获得过程
		{
			BulletRotate_Data.tarP=BULLETROTATE_ACQUIRE;
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_ACQUIRE)<45)
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
				if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)
				{
					TakeBulletState=BULLET_POUROUT1;	//切换到倒弹
				}
			}
			break;
		}
		case BULLET_POUROUT1:	//车身倾斜、舵机旋转	称之为倒弹过程
		{
			lift_Data.lf_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.lb_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.rf_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.rb_lift_tarP=LIFT_BULLET_POUROUT;
			
			BulletRotate_Data.tarP=BULLETROTATE_POUROUT;
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)<50)
			{
				TakeBulletState=BULLET_THROWOUT1;	//切换到扔出
			}
			break;
		}
		case BULLET_THROWOUT1:	//舵机旋回、车身抬起、夹紧松开	称之为抛落过程
		{
			BulletRotate_Data.tarP=BULLETROTATE_WAITING;
			
			if((BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)>40)	//在抬起来一点后
			{
				ViceControlData.valve[VALVE_BULLET_HORIZONTAL]=1;	//同时进行下一个对位
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_THROWOUT)<20)
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			}
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//已经松开，开始下降
			{
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<40)	//到准备位置，可以开始下一次
			{
				TakeBulletState=BULLET_ACQUIRE2;	//下一次取弹
			}
			break;
		}
		case BULLET_ACQUIRE2:	//前伸、夹紧、抬起动作	称之为获得过程2
		{
			if(valve_fdbstate[VALVE_BULLET_HORIZONTAL]==1)	//气缸平移
			{
				BulletRotate_Data.tarP=BULLETROTATE_ACQUIRE;
				if(abs(BulletRotate_Data.fdbP-BULLETROTATE_ACQUIRE)<45)
				{
					ViceControlData.valve[VALVE_BULLET_CLAMP]=1;
					if(valve_fdbstate[VALVE_BULLET_CLAMP]==1)
					{
						TakeBulletState=BULLET_POUROUT2;	//切换到倒弹
					}
				}
			}
			break;
		}
		case BULLET_POUROUT2:	//车身倾斜、舵机旋转	称之为倒弹过程2
		{
			lift_Data.lf_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.lb_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.rf_lift_tarP=LIFT_BULLET_POUROUT;
			lift_Data.rb_lift_tarP=LIFT_BULLET_POUROUT;
			
			BulletRotate_Data.tarP=BULLETROTATE_POUROUT;
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)<50)
			{
				TakeBulletState=BULLET_THROWOUT2;	//切换到扔出
			}
			break;
		}
		case BULLET_THROWOUT2:	//舵机旋回、车身抬起、夹紧松开	称之为抛落过程2
		{
			BulletRotate_Data.tarP=BULLETROTATE_WAITING;
			
			if((BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)>40)	//在抬起来一点后
			{
				ViceControlData.valve[VALVE_BULLET_HORIZONTAL]=0;	//平移气缸回中
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_THROWOUT)<20)
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			}
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//已经松开，开始下降
			{
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<40)
			{
				TakeBulletState=BULLET_WAITING;	//进入等待状态
			}
			break;
		}
		case BULLET_OTHER:	//其他非取弹状态
		{
			ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//气缸松开
			{
				BulletRotate_Data.tarP=BULLETROTATE_OTHER;
				if(abs(BulletRotate_Data.fdbP-BULLETROTATE_OTHER)<30)	//电机收回
				{
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL]=0;
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


	
//	if(RC_Ctl.rc.switch_left==RC_SWITCH_DOWN&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)	//给登岛腾位置
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
	
//	if(auto_takebullet_statu==1)	//1/4自动程序
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
以下三个for为反馈假设检测方案
分别为：
1.上升，下降沿的触发时间记录
2.根据触发时间的推演反馈值计算
3.数据迭代
******************************************************************/

		for(int i=0;i<6;i++)	//触发时间块
		{
			if(valve_last[i]==0&&ViceControlData.valve[i]==1)	//伸出触发
			{
				valve_startGOOD_time[i]=time_1ms_count;
			}
			else if(valve_last[i]==1&&ViceControlData.valve[i]==0)//收回触发
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
		
		for(int i=0;i<6;i++)	//反馈计算位
		{
			if(ViceControlData.valve[i]==1&&time_1ms_count-valve_startGOOD_time[i]>valve_GOODdelay[i])	//本数值为启动至到位延时，暂统一定为1000ms
			{
				valve_fdbstate[i]=1;
			}
			else if(ViceControlData.valve[i]==0&&time_1ms_count-valve_startPOOR_time[i]>valve_POORdelay[i])	//本数值为收回至到位延时，暂统一定为1000ms
			{
				valve_fdbstate[i]=0;
			}
			
			if(i<2)
			{
				if(ViceControlData.servo[i]==1&&time_1ms_count-servo_startGOOD_time[i]>servo_GOODdelay[i])	//本数值为启动至到位延时，暂统一定为1000ms
				{
					servo_fdbstate[i]=1;
				}
				else if(ViceControlData.servo[i]==0&&time_1ms_count-servo_startPOOR_time[i]>servo_POORdelay[i])	//本数值为收回至到位延时，暂统一定为1000ms
				{
					servo_fdbstate[i]=0;
				}
			}
		}
		
		for(int i=0;i<6;i++)	//迭代块
		{
			valve_last[i]=ViceControlData.valve[i];
			if(i<2)	servo_last[i]=ViceControlData.servo[i];
		}
////////////////////////////////////////////////////////////////////////////////////////////块结束标志
	swicth_Last_state=RC_Ctl.rc.switch_right;	//迭代
	
	
	PWM3_1=Steer_Send[UP_L];
	PWM3_2=Steer_Send[UP_R];
	PWM3_3=Steer_Send[DOWN_L];
	PWM3_4=Steer_Send[DOWN_R];
}




/**********************分区赛版本********************
u8 SetCheck_TakeBullet_TakeBack_statu=0;	//切出取弹保护执行标志位
void SetCheck_TakeBullet_TakeBack(void)	//切出取弹机构回位保护
{
	if(SetCheck_TakeBullet_TakeBack_statu==1)//当状态为更新到1
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
#define LIFT_DISTANCE_GRIPBULLET	900	//夹弹药箱时底盘高度900-810	890-807  880-803
#define BULLETLIFT_DISTANCE_GRIPBULLET	460	//夹弹药箱时夹持升降高度	//经过实验300为极限可容许前伸高度
#define BULLETLIFT_DISTANCE_DISGRIPBULLET	1500	//拔起弹药箱夹持机构高度
extern LIFT_DATA lift_Data;

u8 SetCheck_GripBulletLift(u8 grip_state)	//夹持机构升降函数//是否与弹药箱平齐,grip抓住的意思	//0表示不抓住，即需要丢弹药箱或拔起弹药箱高度，1表示抓住，即需要夹紧弹药箱时的高度
{
	bulletlift_Motor_Data[BULLETLIFT_FRONTID].tarP=BULLETLIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(BULLETLIFT_DISTANCE_DISGRIPBULLET-BULLETLIFT_DISTANCE_GRIPBULLET);
	bulletlift_Motor_Data[BULLETLIFT_BACKID].tarP=BULLETLIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(BULLETLIFT_DISTANCE_DISGRIPBULLET-BULLETLIFT_DISTANCE_GRIPBULLET);

	error_bullet_lift_tar=abs(2*(BULLETLIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(BULLETLIFT_DISTANCE_DISGRIPBULLET-BULLETLIFT_DISTANCE_GRIPBULLET)));	//这里是仅以	
	error_bullet_lift_fdb=bulletlift_Motor_Data[BULLETLIFT_BACKID].fdbP;	//这里是仅以
	return (abs(bulletlift_Motor_Data[BULLETLIFT_FRONTID].fdbP+bulletlift_Motor_Data[BULLETLIFT_BACKID].fdbP-2*(BULLETLIFT_DISTANCE_DISGRIPBULLET-(grip_state!=0)*(BULLETLIFT_DISTANCE_DISGRIPBULLET-BULLETLIFT_DISTANCE_GRIPBULLET)))<40);	//这里是仅以前两腿为反馈传回的
}

u8	SetCheck_LiftAll_To_bullet(u8 bullet_state)	//底盘升降函数	//取弹时底盘升至固定高度，1为升，0为降
{
	lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET);
	
	return (abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*(LIFT_DISTANCE_FALL-(bullet_state!=0)*(LIFT_DISTANCE_FALL-LIFT_DISTANCE_GRIPBULLET)))<30);	//这里是仅以前两腿为反馈传回的
}
*****************************************************/
