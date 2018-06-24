#include "semi_automatic_landing.h"
#include "auto_lift.h"

/***************************************
���ã�u8 SetCheck_FrontLift(u8 rise_state);	//ǰ����������/���²����	//0��ʾFALL��1��ʾISLAND
u8 SetCheck_BackLift(u8 rise_state);

		λ��ʾ��ͼ
		 �ϵ�����
			3			2	//�����
			1			0	//����ǰ
****************************************/
extern SensorDataTypeDef SensorData;
extern u32 time_1ms_count;

void semi_auto_landing_center(void)	//������λ���Զ��ǵ�
{
	if(SensorData.Limit[2]==1&&SensorData.Limit[3]==1)	//�������ȵ�����
	{
		SetCheck_BackLift(0);
	}
	if(SensorData.Limit[0]==1&&SensorData.Limit[1]==1)	//����ǰ�ȵ�����
	{
		SetCheck_FrontLift(0);
	}
}


void semi_auto_outlanding_center(void)	//���ú�����Զ��µ�
{
	if(SensorData.Infrare[0]==1&&SensorData.Infrare[1]==1)	//����ǰ�����
	{
		SetCheck_FrontLift(1);
	}
	if(SensorData.Infrare[2]==1&&SensorData.Infrare[3]==1)	//����ǰ�ȵ�����
	{
		SetCheck_BackLift(1);
	}
}
