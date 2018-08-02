#include "take_bullet.h"

//#define VALVE_ISLAND 0		//电磁阀控制位定义
//#define VALVE_BULLET_HORIZONTAL 1	//左右平移
//#define VALVE_BULLET_CLAMP 2	//夹紧
//#define VALVE_BULLET_STORAGE 3	//弹药舱补弹
//#define VALVE_TRAILER 5	//拖车

TakeBulletState_e TakeBulletState=BULLET_OTHER;	//(自动)取弹标志位
AutoAimBulletTypeDef AutoAimBulletData={0};

#define BULLETROTATE_OTHER	15	//非取弹位置
#define BULLETROTATE_WAITING	542//556//750//650	//等待（对位）时位置
#define BULLETROTATE_ACQUIRE	1110	//取弹位置
#define BULLETROTATE_POUROUT	170	//倒弹位置
#define BULLETROTATE_THROWOUTEND 760	//抛出时的设定的终点
#define BULLETROTATE_THROWOUT	290//280//310	//抛出位置

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
const u32 valve_GOODdelay[6]={300,450,100,1000,1000,1000};	//待加入，延时参数	//500是伪延时
const u32 valve_POORdelay[6]={300,450,400,1000,1000,1000};	//待加入，延时参数	//夹紧回延时给到200是为了避免飞出去的弹药箱误检测
const u32 servo_GOODdelay[2]={2000,1000};	//延时参数	//第一段为2000是将子弹落下的延时也加进去了，因为舵机翻转和子弹下落必须是连在一体的
const u32 servo_POORdelay[2]={1000,1000};	//延时参数


extern RC_Ctl_t RC_Ctl;
extern ViceControlDataTypeDef ViceControlData;
extern u32 time_1ms_count;
extern LIFT_DATA lift_Data;
extern SensorDataTypeDef SensorData;
//extern BULLETLIFT_MOTOR_DATA bulletlift_Motor_Data[2];	//分区赛后弃用
extern BULLETROTATE_DATA BulletRotate_Data;	//国赛版
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern PID_GENERAL PID_Chassis_Speed[4];

extern s16 Chassis_Vx;
extern s16 Chassis_Vy;
extern u8 BulletRotate_Cali_Statu;	//标定状态

u8 TakeBullet_AutoAimState=1;	//默认开启自动对位，单词取弹模式可取消，对本次有效
u8 Close_Valve_Island_Protect_State=0;	//取弹电磁阀收回保护位

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
		static u8 key_ctrl_last=0;
		static u8 key_shift_last=0;
		if(key_ctrl_last==0&&KeyBoardData[KEY_CTRL].value==1)	//取弹模式按了CTRL就取消自动取块
		{
			AutoAimBulletData.take_count=0;	//清零取块数量记录	//使单次取弹模式不止能取两块
			TakeBullet_AutoAimState=!TakeBullet_AutoAimState;	//屏蔽自动对位模块
		}
		key_ctrl_last=KeyBoardData[KEY_CTRL].value;
		
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
		Close_Valve_Island_Protect_State=1;	//启动电磁阀收回保护
		AutoAimBulletData.take_count=0;	//清零取块数量记录
		AutoAimBulletData.aim_state=0;	//back
		AutoAimBulletData.control_state=0;	//关闭对位
		TakeBullet_AutoAimState=0;	//默认关闭自动模式
	}
	
	if(State_Record==TAKEBULLET_STATE&&GetWorkState()!=TAKEBULLET_STATE)
	{
		Chassis_Vx=0;
		Chassis_Vy=0;
		AutoAimBulletData.control_state=0;	//关闭对位
		AutoAimBulletData.take_count=0;	//清零取块数量记录
		AutoAimBulletData.aim_state=0;	//back
		TakeBullet_AutoAimState=0;	//默认关闭自动模式
	}
	
	
	if(AutoAimBulletData.control_state==1)	//自动对位开启，则底盘PID变高
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
	
	
	if(TakeBullet_AutoAimState==0)	//一个保护
	{
		AutoAimBulletData.control_state=0;	//关闭对位
		AutoAimBulletData.aim_state=0;
	}
	
	
	switch(TakeBulletState)	//自动取弹过程
	{
		case BULLET_WAITING:	//等待取弹动作（对位）状态
		{
			if(TakeBullet_AutoAimState==1)	//自动取弹模式开启
			{
				if(AutoAimBulletData.take_count==0)
				{
					AutoAimBulletData.control_state=1;	//启动对位
					if(AutoAimBullet_Task(&Chassis_Vx,&Chassis_Vy)==1)
					{
						TakeBulletState=BULLET_ACQUIRE1;	//启动夹块
						AutoAimBulletData.control_state=0;	//关闭对位
						AutoAimBulletData.aim_state=0;
					}
				}
				else if(AutoAimBulletData.take_count==2)
				{
					AutoAimBulletData.control_state=1;	//启动对位
					if(AutoAimBullet_Task(&Chassis_Vx,&Chassis_Vy)==1)
					{
						TakeBulletState=BULLET_ACQUIRE1;	//启动夹块
						AutoAimBulletData.control_state=0;	//关闭对位
						AutoAimBulletData.aim_state=0;
					}
				}

			}
			
			
			ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//气缸松开
			{
				BulletRotate_Data.tarP=BULLETROTATE_WAITING;
				if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<30)	//电机收回
				{
					ViceControlData.valve[VALVE_BULLET_HORIZONTAL]=0;	//改了1					//////////////////////////////////////////////////
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
			
			if(GetWorkState()!=TAKEBULLET_STATE)	//退出取弹模式的状态保护
			{
				TakeBulletState=BULLET_WAITING;	//进入等待状态
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
			BulletRotate_Data.tarP=BULLETROTATE_THROWOUTEND;//BULLETROTATE_WAITING;	//设定抛出终点以便获得较大速度
			
			if((BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)>40)	//在抬起来一点后
			{
				ViceControlData.valve[VALVE_BULLET_HORIZONTAL]=1;	//同时进行下一个对位				/////////////////////////////////////////////////////////////////
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_THROWOUT)<20)	//距离抛出位置
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			}
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//已经松开，开始下降
			{
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
				TakeBulletState=BULLET_ACQUIRE2;	//下一次取弹	//这个会后触发
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<40)	//到准备位置，可以开始下一次
			{
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
				TakeBulletState=BULLET_ACQUIRE2;	//下一次取弹
			}
			break;
		}
		case BULLET_ACQUIRE2:	//前伸、夹紧、抬起动作	称之为获得过程2
		{
			if(valve_fdbstate[VALVE_BULLET_HORIZONTAL]==1)	//气缸平移			//////////////////////////////////////////////
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
			
			if(GetWorkState()!=TAKEBULLET_STATE)	//退出取弹模式的状态保护
			{
				TakeBulletState=BULLET_WAITING;	//进入等待状态
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
			
			BulletRotate_Data.tarP=BULLETROTATE_THROWOUTEND;//BULLETROTATE_WAITING;
			
			if((BulletRotate_Data.fdbP-BULLETROTATE_POUROUT)>40)	//在抬起来一点后
			{
				ViceControlData.valve[VALVE_BULLET_HORIZONTAL]=0;	//平移气缸回中			/////////////////////////////////////
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_THROWOUT)<20)
			{
				ViceControlData.valve[VALVE_BULLET_CLAMP]=0;
			}
			
			if(valve_fdbstate[VALVE_BULLET_CLAMP]==0)	//已经松开，开始下降
			{
				BulletRotate_Data.tarP=BULLETROTATE_WAITING;
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
				if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<25)	//到准备位置，可以开始下一次
				{
					TakeBulletState=BULLET_WAITING;	//进入等待状态
					AutoAimBulletData.take_count+=2;	//标记取弹数量加2
				}
				
				
			}
			
			if(abs(BulletRotate_Data.fdbP-BULLETROTATE_WAITING)<20)
			{
//				AutoAimBulletData.take_count+=2;	//标记取弹数量加2
//				TakeBulletState=BULLET_WAITING;	//进入等待状态
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


	Close_Valve_Island_Protect();	//登岛取弹电磁阀返回保护
	
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


#define AUTOAIM_SPEENX	7	//向前压力
#define AUTOAIM_VOIDSPEEDY	130//110	//自动取块空时对位速度	//以向右为正方向
#define AUTOAIM_EXISTSPEEDY	65//55	//自动取块有障碍时速度
u8 AutoAimBullet_Task(s16* chassis_vx,s16* chassis_vy)	//自动对位任务
{
	static u8 aim_control_state_last=0;
	u8 aim_OK_statu=0;
	
	//移动状态辨识
	if(SensorData.Infrare[6]==0&&SensorData.Infrare[7]==1)	//[6]为左对位 [7]为右对位 0为有，1为无
	{	//向左移动状态
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
		//状态辨识之后移进来
		
		switch(AutoAimBulletData.aim_state)
		{
			case 0:	//空（需要移动）(向右移动)
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
				else if(AutoAimBulletData.relative_location==2)	//都有，速度减慢，方向不变
				{
					AutoAimBulletData.aim_state=4;
				}
				break;
			}
			case 1:	//向左移动
			{
				*chassis_vx=AUTOAIM_SPEENX;
				*chassis_vy=-AUTOAIM_EXISTSPEEDY;
				if(AutoAimBulletData.relative_location==0)
				{
					AutoAimBulletData.aim_state=3;
				}
				else if(AutoAimBulletData.relative_location==1)	//向右移动
				{
					AutoAimBulletData.aim_state=2;
				}
				break;
			}
			case 2:	//向右移动
			{
				*chassis_vx=AUTOAIM_SPEENX;
				*chassis_vy=AUTOAIM_EXISTSPEEDY;
				if(AutoAimBulletData.relative_location==0)
				{
					AutoAimBulletData.aim_state=3;
				}
				else if(AutoAimBulletData.relative_location==-1)	//向左移动
				{
					AutoAimBulletData.aim_state=1;
				}
				break;
			}
			case 3:	//空（可以取块）
			{
				*chassis_vx=0;
				*chassis_vy=0;
				aim_OK_statu=1;
				AutoAimBulletData.aim_state=0;
				break;
			}
			case 4:	//都有 和都空方向一致但速度减慢
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
				else if(AutoAimBulletData.relative_location==0)	//可以取块
				{
					AutoAimBulletData.aim_state=3;
				}
				break;
			}
		}
	}
	else
	{
		AutoAimBulletData.aim_state=0;	//清零
	}
	
	aim_control_state_last=AutoAimBulletData.control_state;
	
	return aim_OK_statu;
}


#define LIFT_DISTANCE_BACKVALVE	260
//u8 Close_Valve_Island_Protect_State=0;	//放在了上面
void Close_Valve_Island_Protect(void)	//使电磁阀归位的保护
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
