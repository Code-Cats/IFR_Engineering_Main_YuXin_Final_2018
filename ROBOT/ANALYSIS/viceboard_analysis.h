#ifndef __VICEBOARD_ANALYSIS_H
#define __VICEBOARD_ANALYSIS_H

#include "bsp.h"


void ViceBoard_SendDataRun(void);
void ViceBoard_SendDataRefresh(void);//����Ƶ�ʷ��ڵ��ò�
void Data_Receive(u8 data);	//�����崫���������ݽ�����������ͨ�ã�
void SensorData_Deal(u8 *pData);

typedef struct
{
	u8 statu;
	u8 data[5];
	u8 count;
}ViceBoardSendTypeDef;


typedef struct
{
	u8 valve[6];
	u8 servo[2];
	u8 image_cut[2];	//�����ǵ�1λ��2λ����Ч
}ViceControlDataTypeDef;	//���Ƹ���


#define MAINBOARD_SENDDATA_DEFAULT \
{\
	0,\
	{0x5A,0,0,0,0xA5},\
	0,\
}\

typedef struct
{
	u8 headOK_state;
	u8 valid_state;	//����֡��Ч��־λ
	u8 databuffer[5];
	u8 count;
}ReceiveDataTypeDef;

typedef struct
{
	u8 Infrare[6];		//[4]Ϊ���µ����ٱ�����[5]Ϊ�ϳ����
	u8 Limit[4];
}SensorDataTypeDef;

extern ViceBoardSendTypeDef SendData;
extern ReceiveDataTypeDef ReceiveData;


#endif

