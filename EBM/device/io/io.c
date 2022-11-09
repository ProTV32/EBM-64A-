#include "io.h"
#include "../mcp4921e/mcp4921e.h"
#include "../../util/pin.h"

//-----------------------Дискретные входа/выхода----------------------
enum {Idle, Run}StateDi; //Состояние КА
#define LOCK_TIME  3    //время обработки, анти дребезга в милисекундах х10 (1-10)

#define DI1 B,4,L
#define DI2 B,5,L
#define DI3 E,2,L
#define DI4 E,3,L
#define DI5 E,4,L

#define DO1 G,1,H
#define DO2 G,0,H
#define DO3 C,3,H
#define DO4 C,2,H
#define DO5 C,1,H

u8_t DiFlags;
u8_t JitterCnt;	//Счетчик (защита от дребезга)
u8_t MaskOld;		//Последнее сохранённое состояние

//-----------------------PWM выходы----------------------
#define PWM1 C,0,L //G3?
#define PWM2 B,6,L
#define PWM3 B,7,L
#define PWM4 D,4,L
#define PWM5 D,5,L 

static const u16_t PWMt = 1000;		//Период сигнала 10 сек / 10 мс

typedef struct 
{
	u16_t PWMset;       //хранящий значения заданий, % * 10 (20% = 20 * 10 = 200  длительность открытого состояния)
	u16_t PWMcnt;       //хранящий счётчики
}PWM_t; 
	
static PWM_t PWMout[5] = {0}; //Массив структур данных каналов PWM


void DIO_Init(void) //Инициализация дискретных входов/выходов
{
	StateDi		= Idle;
	DiFlags = 0;
	MaskOld = 0;
	DRIVER(DI1,IN);		DRIVER(DI2,IN);		DRIVER(DI3,IN);		DRIVER(DI4,IN);		DRIVER(DI5,IN);
	DRIVER(DO1,OUT);	DRIVER(DO2,OUT);	DRIVER(DO3,OUT);	DRIVER(DO4,OUT);	DRIVER(DO5,OUT);
	OFF(DO1);					OFF(DO2);					OFF(DO3);					OFF(DO4);					OFF(DO5);
}

void DI_Scan(void)  //Функция сканирования дискретных входов, интервал вызова 10мс
{
	u8_t MaskNew = (ACTIVE(DI5) << 4) | (ACTIVE(DI4) << 3) | (ACTIVE(DI3) << 2) | (ACTIVE(DI2) << 1) | (ACTIVE(DI1) << 0);
	
	if (StateDi == Idle) //Изменение состояния входов
		{
			if (MaskNew ^ MaskOld)
				{
					MaskOld = MaskNew;
					StateDi = Run;
					JitterCnt = 0;
				}
		}
	
	else
		{
			if (MaskNew == MaskOld)
				{
					if(JitterCnt < LOCK_TIME)
						{
							JitterCnt++;
							return;															//защелка еще не дощитала - возврат
						}
			
					DiFlags = MaskOld;
				}
		
			StateDi = Idle;
		}
}

u8_t DI_Get(void)   //Отправка флагов текущего состояния
{
	return DiFlags;
}

void DO_Set(u8_t out, bool_t state)  //Установка дискретных выходов в заданное состояние
{
	switch(out)
		{
			case 1:
				if (state)
					ON(DO1);
				else
					OFF(DO1);
				break;
			
			case 2:
				if (state)
					ON(DO2);
				else
					OFF(DO2);
				break;
			
			case 3:
				if (state)
					ON(DO3);
				else
					OFF(DO3);
				break;
			
			case 4:
				if (state)
					ON(DO4);
				else
					OFF(DO4);
				break;
			
			case 5:
				if (state)
					ON(DO5);
				else
					OFF(DO5);
				break;
		}
}


void AIO_Init(void)
{
	DAC_Init();
}

u8_t AI_Get(void)
{
	return 10;
}

void AO_Set(u8_t out, u8_t value)  //Установка аналоговых выходов в заданное состояние
{
	DAC_Write(out, value);
}


//-----------------------PWM выходы----------------------
void PWM_Init(void)
{
	DRIVER(PWM1,OUT);	DRIVER(PWM2,OUT);	DRIVER(PWM3,OUT);	DRIVER(PWM4,OUT);	DRIVER(PWM5,OUT);
	OFF(PWM1);				OFF(PWM2);				OFF(PWM3);				ON(PWM4);				  ON(PWM5);
}

void PWM_Set(u8_t out, u8_t value)  //Установка PWM выходов в заданное состояние
{
	u16_t temp;
	
	if ((value > 10) && (value < 90))
		temp = value * 10;
	
	else if (value > 90)
		temp = 1000;
	
	else
		temp = 0;
	
	if(PWMout[out-1].PWMset != temp)
		{
			PWMout[out-1].PWMset = temp;
			PWMout[out-1].PWMcnt = 0;
		}
}

void PWM_Loop(void) //Обработка сигналов PWM, вызывать в цикле через 10 мс вместе с опросом дискретных входов
{
  //---------------------------Chanal #1------------------------
	
	if(PWMout[0].PWMset > 0) //Если есть задание - работаем
		{
			if ((PWMout[0].PWMcnt >= PWMout[0].PWMset) && ACTIVE(PWM1))
				OFF(PWM1);
			
			else if((PWMout[0].PWMcnt < PWMout[0].PWMset) && ~ACTIVE(PWM1))
				ON(PWM1);
				
			PWMout[0].PWMcnt++;
			
			if (PWMout[0].PWMcnt >= PWMt) //Период пройден - сброс счётчика
				PWMout[0].PWMcnt = 0;
		}
	
	else if (ACTIVE(PWM1))
		OFF(PWM1);
	
	//---------------------------Chanal #2------------------------
	
	if(PWMout[1].PWMset > 0) //Если есть задание - работаем
		{
			if ((PWMout[1].PWMcnt >= PWMout[1].PWMset) && ACTIVE(PWM2))
				OFF(PWM2);
		
			else if((PWMout[1].PWMcnt < PWMout[1].PWMset) && ~ACTIVE(PWM2))
				ON(PWM2);
		
			PWMout[1].PWMcnt++;
		
			if (PWMout[1].PWMcnt >= PWMt) //Период пройден - сброс счётчика
				PWMout[1].PWMcnt = 0;
		}
	
	else if (ACTIVE(PWM2))
		OFF(PWM2);
	
	//---------------------------Chanal #3------------------------
	
	if(PWMout[2].PWMset > 0) //Если есть задание - работаем
		{
			if ((PWMout[2].PWMcnt >= PWMout[2].PWMset) && ACTIVE(PWM3))
				OFF(PWM3);
		
			else if((PWMout[2].PWMcnt < PWMout[2].PWMset) && ~ACTIVE(PWM3))
				ON(PWM3);
		
			PWMout[2].PWMcnt++;
		
			if (PWMout[2].PWMcnt >= PWMt) //Период пройден - сброс счётчика
				PWMout[2].PWMcnt = 0;
		}
	
	else if (ACTIVE(PWM3))
		OFF(PWM3);
		
	//---------------------------Chanal #4------------------------
	
	if(PWMout[3].PWMset > 0) //Если есть задание - работаем
		{
			if ((PWMout[3].PWMcnt >= PWMout[3].PWMset) && ACTIVE(PWM4))
				OFF(PWM4);
		
			else if((PWMout[3].PWMcnt < PWMout[3].PWMset) && ~ACTIVE(PWM4))
				ON(PWM4);
		
			PWMout[3].PWMcnt++;
		
			if (PWMout[3].PWMcnt >= PWMt) //Период пройден - сброс счётчика
				PWMout[3].PWMcnt = 0;
		}
	
	else if (ACTIVE(PWM4))
		OFF(PWM4);
	
	//---------------------------Chanal #5------------------------
	
	if(PWMout[4].PWMset > 0) //Если есть задание - работаем
		{
			if ((PWMout[4].PWMcnt >= PWMout[4].PWMset) && ACTIVE(PWM5))
				OFF(PWM5);
		
			else if((PWMout[4].PWMcnt < PWMout[4].PWMset) && ~ACTIVE(PWM5))
				ON(PWM5);
		
			PWMout[4].PWMcnt++;
		
			if (PWMout[4].PWMcnt >= PWMt) //Период пройден - сброс счётчика
			PWMout[4].PWMcnt = 0;
		}
	
	else if (ACTIVE(PWM5))
		OFF(PWM5);	
}