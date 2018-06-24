#ifndef __TAKE_BULLET_H_
#define __TAKE_BULLET_H_
#include "bsp.h"

#define VALVE_ISLAND 0		//��ŷ�����λ����
#define VALVE_BULLET_PROTRACT 1	//ǰ��
#define VALVE_BULLET_CLAMP 2	//�н�
#define VALVE_BULLET_STORAGE 3	//��ҩ�ղ���
#define VALVE_TRAILER 5	//�ϳ�

#define	SERVO_BULLET_UP 0
#define SERVO_BULLET_DOWN 1

#define UP_L 0
#define UP_R 1
#define DOWN_L 2
#define DOWN_R 3


typedef enum
{
    BULLET_ACQUIRE,  		//ǰ�졢�н���̧����	��֮Ϊ��ù���
    BULLET_POUROUT,			//������б�������ת	��֮Ϊ��������
		BULLET_THROWOUT,			//������ء�����̧�𡢼н��ɿ�	��֮Ϊ�������
}TakeBulletState_e;


void TakeBullet_Control_Center(void);
u8 SetCheck_GripBulletLift(u8 grip_state);	//�гֻ�����������//�Ƿ��뵯ҩ��ƽ��,gripץס����˼	//0��ʾ��ץס������Ҫ����ҩ������ҩ��߶ȣ�1��ʾץס������Ҫ�н���ҩ��ʱ�ĸ߶�
u8	SetCheck_LiftAll_To_bullet(u8 bullet_state);	//������������	//ȡ��ʱ���������̶��߶ȣ�1Ϊ����0Ϊ��
void SetCheck_TakeBullet_TakeBack(void);	//�г�ȡ��״̬ʱ��λ

#endif

