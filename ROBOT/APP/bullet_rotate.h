#ifndef __BULLET_ROTATE_H_
#define __BULLET_ROTATE_H_

#include "stm32f4xx.h"

typedef struct
{
	/*fdbp:当前机械角度
  fdbv:当前转速
  Tarp:目标机械角度
  Tarv:目标转速*/
	s32 fdbP;	//处理后的数据
	
	s32 fdbV;
	
	s32 tarP;
	s32 tarV;

  float output;
}BULLETROTATE_DATA;


#endif
