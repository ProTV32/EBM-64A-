#ifndef AIO_H_
#define AIO_H_

#include "../../main.h"

//-----------------------Дискретные входа/выхода----------------------
void DIO_Init(void);
void DI_Scan(void);
u8_t DI_Get(void);
void DO_Set(u8_t out, bool_t state);  //Установка дискретных выходов в заданное состояние

//-----------------------Аналоговые входа/выхода----------------------
void AIO_Init(void);
u8_t AI_Get(void);
void AO_Set(u8_t out, u8_t value);  //Установка аналоговых выходов в заданное состояние

//-----------------------PWM выхода----------------------
void PWM_Init(void);
void PWM_Set(u8_t out, u8_t value);  //Установка PWM выходов в заданное состояние
void PWM_Loop(void);                 //Обновление значений выходов, частота вызова 100 Гц

#endif /* AIO_H_ */