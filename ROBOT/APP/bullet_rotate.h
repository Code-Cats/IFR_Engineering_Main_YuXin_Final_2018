#ifndef __BULLET_ROTATE_H_
#define __BULLET_ROTATE_H_

#include "stm32f4xx.h"

typedef struct
{
	/*fdbp:��ǰ��е�Ƕ�
  fdbv:��ǰת��
  Tarp:Ŀ���е�Ƕ�
  Tarv:Ŀ��ת��*/
	s32 fdbP;	//����������
	
	s32 fdbV;
	
	s32 tarP;
	s32 tarV;

  float output;
}BULLETROTATE_DATA;


#endif
