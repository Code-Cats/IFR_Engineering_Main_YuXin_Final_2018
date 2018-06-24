#ifndef __BULLET_LIFT_
#define __BULLET_LIFT_

#include "stm32f4xx.h"

#define BULLETLIFT_FRONTID 0
#define BULLETLIFT_BACKID 1

#define BULLETLIFT_UPDISTANCE 1500
#define BULLETLIFT_DOWNDISTANCE 10


#define BULLETLIFT_POSITION_PID_KP            110//120
#define BULLETLIFT_POSITION_PID_KI            0.01f
#define BULLETLIFT_POSITION_PID_KD            0.01	//0
#define BULLETLIFT_POSITION_PID_MER					10000
#define BULLETLIFT_POSITION_PID_I_MAX				0.0f
#define BULLETLIFT_POSITION_PID_MAXIN        60000
#define BULLETLIFT_POSITION_MAXOUT           12000	//12000

#define BULLETLIFT_SPEED_PID_KP           2//2	//0.32
#define BULLETLIFT_SPEED_PID_KI           0.01f	//0.002
#define BULLETLIFT_SPEED_PID_KD           0.0f 
#define BULLETLIFT_SPEED_PID_MER					7000//7000	//4000
#define BULLETLIFT_SPEED_PID_I_MAX 			3000.0f/BULLETLIFT_SPEED_PID_KI//3000.0f/SHOOT_SPEED_PID_KI
#define BULLETLIFT_SPEED_PID_MAXIN       12000//12000
#define BULLETLIFT_SPEED_MAXOUT      	  8000



//���̵��λ�û�PID����
#define PID_BULLETLIFT_POSITION_DEFAULT \
{\
	BULLETLIFT_POSITION_PID_KP,\
	BULLETLIFT_POSITION_PID_KI,\
  BULLETLIFT_POSITION_PID_KD,\
	BULLETLIFT_POSITION_PID_MER,\
	-BULLETLIFT_POSITION_PID_MAXIN,\
	BULLETLIFT_POSITION_PID_MAXIN,\
	-BULLETLIFT_POSITION_MAXOUT,\
	BULLETLIFT_POSITION_MAXOUT,\
	BULLETLIFT_POSITION_PID_I_MAX,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	0,\
}\


//��������ٶȻ�PID����
#define PID_BULLETLIFT_SPEED_DEFAULT \
{\
	BULLETLIFT_SPEED_PID_KP,\
	BULLETLIFT_SPEED_PID_KI,\
  BULLETLIFT_SPEED_PID_KD,\
	BULLETLIFT_SPEED_PID_MER,\
	-BULLETLIFT_SPEED_PID_MAXIN,\
	BULLETLIFT_SPEED_PID_MAXIN,\
	-BULLETLIFT_SPEED_MAXOUT,\
	BULLETLIFT_SPEED_MAXOUT,\
	BULLETLIFT_SPEED_PID_I_MAX,\
	{0.0,0.0},\
	0.0,\
	0.0,\
	0.0,\
	0,\
}\



typedef struct
{
	/*fdbp:��ǰ��е�Ƕ�
  fdbv:��ǰת��
  Tarp:Ŀ���е�Ƕ�
  Tarv:Ŀ��ת��*/
	s32 fdbP;	//����������
	
	s16 fdbP_raw;	//λ�õ�ԭʼ����
	s32 fdbP_raw_sum;
	s16 fdbP_raw_last;
	s16 fdbP_diff;
	
	s16 fdbV;
	
	s32 tarP;
	s16 tarV;

  float output;
}BULLETLIFT_MOTOR_DATA;

u8 BulletLift_Cali(void);
void BulletLift_Cali_Output_Limit(float cm_out,float * cali_out);

void BulletLift_Feedback_Deal(BULLETLIFT_MOTOR_DATA *bulletlift_motor_data,CanRxMsg *msg);

void BulletLift_Task(void);	//��ʱƵ�ʣ�1ms

#endif
