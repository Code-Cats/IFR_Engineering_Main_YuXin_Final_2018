#include "main.h"
u16 tim5_t_send[4];

int main(void)
{
   delay_ms(2000);
	 SetWorkState(CHECK_STATE);	//������Ĭ���Լ�״̬	//����״̬�л���������������
   delay_ms(500);
	Data_Init();
	 BSP_Init();	//���������ʼ��	//��ʱ��ʱ����������ʼ��ʱ
	Data_Init();	//����������
	 SetWorkState(CHECK_STATE);	//�����Լ�״̬
	 delay_ms(100);

	while(1)
	 {
//		 STEER_IMAGE=tim5_t_send[0];
//		 STEER_RFID=tim5_t_send[1];
		 Screen_Start();
		 get_raw_acc(Gyro_Data.acc);
     get_raw_gyo(Gyro_Data.angvel);
     get_eular(Gyro_Data.angle);
	 }
}





//////////////////////////////////////////////////////////////////
//??????,??printf??,??????use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//??????????                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//??_sys_exit()??????????    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//???fputc?? 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//????,??????   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
