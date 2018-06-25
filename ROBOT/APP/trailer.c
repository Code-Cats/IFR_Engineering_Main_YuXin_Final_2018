#include "trailer.h"
/*
�ϳ�����
*/

#define LIFT_DISTANCE_TRAILER 400

extern LIFT_DATA lift_Data;
extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];
extern ViceControlDataTypeDef ViceControlData;
extern SensorDataTypeDef SensorData;
extern u8 valve_fdbstate[6];	//��¼�Ƿ�����ķ�����־
extern u8 servo_fdbstate[2];

/*
#define VALVE_ISLAND 0		//��ŷ�����λ����
#define VALVE_BULLET_HORIZONTAL 1	//����ƽ��
#define VALVE_BULLET_CLAMP 2	//�н�
#define VALVE_BULLET_STORAGE 3	//��ҩ��
#define VALVE_TRAILER 5	//�ϳ�
*/

//����Ľ�����Ϊ0��Զ����Ϊ1
extern u8 Replenish_Bullet_Statu;	//������վ����״̬λ

u8 Trailer_statu=0;	//�ϳ���־λ
u8 Trailer_liftend_statu=0;	//�ϳ�����������־λ ��Ϊ��Ҫ��ʱ ������˲�����
void Trailer_Task(u8 sensor_data)	//���뺯��Ϊ��ഫ��������
{
	static u8 sensor_data_last=0;
	static u8 key_f_last=0;
	static u8 trailer_statu_last=0;
	
	if(Replenish_Bullet_Statu!=1)	//�벹������
	{
		if(key_f_last==0&&KeyBoardData[KEY_F].value==1)	//������ͼ���Զ�ת�����ϳ�
		{
			Trailer_statu=!Trailer_statu;	//��ת
		}
	}
	
	
	if(Trailer_statu==1)
	{
		ViceControlData.valve[VALVE_TRAILER]=1;
		if(sensor_data_last==1&&sensor_data==0)
		{
			//�����������
			lift_Data.lf_lift_tarP=LIFT_DISTANCE_TRAILER;
			lift_Data.rf_lift_tarP=LIFT_DISTANCE_TRAILER;
			lift_Data.lb_lift_tarP=LIFT_DISTANCE_TRAILER;
			lift_Data.rb_lift_tarP=LIFT_DISTANCE_TRAILER;
		}
	}
	else if(trailer_statu_last==1&&Trailer_statu==0)
	{
//		ViceControlData.valve[VALVE_TRAILER]=0;
		Trailer_liftend_statu=1;	//ˢ��
//		lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
//		lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
//		lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
//		lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
	}
	
	if(Trailer_liftend_statu==1)
	{
		SetCheck_FrontLift(0);
		SetCheck_BackLift(0);
		if(SetCheck_FrontLift(0)==1&&SetCheck_BackLift(0)==1)
		{
			ViceControlData.valve[VALVE_TRAILER]=0;
			Trailer_liftend_statu=0;
		}
	}
	
	key_f_last=KeyBoardData[KEY_F].value;	//�Լ���¼last����ֱ�ӵ���keydata�е�last���Ա�����Ϊˢ��Ƶ�ʲ�ͬ���±�ֵ���ⴰ�ڵĶ�ʧ
	sensor_data_last=sensor_data;
	trailer_statu_last=Trailer_statu;
}


