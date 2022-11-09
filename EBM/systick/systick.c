#include "systick.h"
#include <util/atomic.h>
#include <string.h>

//Определения для TIMER0 интервал 1(10)мс
#if     F_CPU == 11059200UL
	#define TIMER0_TIC 213 //Значение для счётчика TCNT0 переполнение каждые 1 мс 256-43=213
	#define TIMER0_DIV 256 //Предделитель

#elif   F_CPU == 14745600UL
	#define TIMER0_TIC 141 //Значение для счётчика TCNT0 переполнение каждые 1 мс 256-115=141
	#define TIMER0_DIV 128 //Предделитель
	
#elif   F_CPU == 8000000UL
	#define TIMER0_TIC 194 //Значение для счётчика TCNT0 переполнение каждые 1 мс 256-63=194
	#define TIMER0_DIV 128 //Предделитель
	
#endif


static volatile bool_t timer_TIC              = False;	//Флаг срабатывания системного таймера 1мс
static volatile u32_t GlobalTime              = 0;       
			 volatile u8_t   tempSREG;

//Инициализация
void SysTick_Init(void)
{
	//Настройка секции таймера Т0 по переполнению
	TCNT0 = TIMER0_TIC;			 //
	TIFR  = (1<<TOV0);		 //Сброс флага переполнения
	TIMSK = (1<<TOIE0);		 //Прерывание по переполнению
	
	#if TIMER0_DIV == 1024
	TCCR0 = (1<<CS02)|(1<<CS01)|(1<<CS00);//Предделитель на 1024
	
	#elif TIMER0_DIV == 256
	TCCR0 = (1<<CS02)|(1<<CS01)|(0<<CS00);//Предделитель на 256
	
	#elif TIMER0_DIV == 128
	TCCR0 = (1<<CS02)|(0<<CS01)|(1<<CS00);//Предделитель на 128
	
	#elif TIMER0_DIV == 64
	TCCR0 = (0<<CS02)|(1<<CS01)|(1<<CS00);//Предделитель на 64
	
	#elif TIMER0_DIV == 8
	TCCR0 = (0<<CS02)|(1<<CS01)|(0<<CS00);//Предделитель на 8
	#endif
}

//Получение текущего значения счётчика глобального времени
u32_t SysTick_GetTime(void)
{
	return GlobalTime;
}

void Delay_Set(delay_t *delay, u32_t period, bool_t cycle)
{
	delay->period     = period;
	delay->bit.cycle  = cycle;
	delay->bit.status = False;
	delay->bit.step   = False;
	delay->bit.worked = False;
}

bool_t Delay_Scan(delay_t *delay)
{
	switch(delay->bit.step)
		{
			case False: //Первое вхождение
				if (delay->bit.cycle || !delay->bit.worked) //Выполнение циклически или разово
					{
						delay->bit.step   = True;
						delay->bit.status = False;
						delay->timer      = GlobalTime + delay->period;
					}
				break;
		
			case True:
				if((GlobalTime - delay->timer) < (1UL<<31))
					{
						delay->timer      = 0;
						delay->bit.step   = False;
						delay->bit.status = True;
						delay->bit.worked = True;
					}
			break;
	}
	
	return delay->bit.status;
}



//Обработчик прерывания системного таймера по переполнению
ISR(TIMER0_OVF_vect)
{
	tempSREG  = SREG;
	TCNT0     = TIMER0_TIC;
	timer_TIC = True;
	GlobalTime++;
	SREG      = tempSREG;
}





