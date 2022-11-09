#ifndef AIO_H_
#define AIO_H_

#include "../../main.h"

//-----------------------���������� �����/������----------------------
void DIO_Init(void);
void DI_Scan(void);
u8_t DI_Get(void);
void DO_Set(u8_t out, bool_t state);  //��������� ���������� ������� � �������� ���������

//-----------------------���������� �����/������----------------------
void AIO_Init(void);
u8_t AI_Get(void);
void AO_Set(u8_t out, u8_t value);  //��������� ���������� ������� � �������� ���������

//-----------------------PWM ������----------------------
void PWM_Init(void);
void PWM_Set(u8_t out, u8_t value);  //��������� PWM ������� � �������� ���������
void PWM_Loop(void);                 //���������� �������� �������, ������� ������ 100 ��

#endif /* AIO_H_ */