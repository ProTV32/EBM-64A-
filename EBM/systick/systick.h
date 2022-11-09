#ifndef SYSTICK_H_
#define SYSTICK_H_

#include "../main.h"

//--------------------------------------------------Delay SysTick------------------------------------------------------------//
typedef struct
{
	u32_t timer;
	u32_t period;
	struct
	{
		bool_t  cycle :1;
		bool_t  step  :1;
		bool_t  status:1;
		bool_t  worked:1;
	}bit;
}delay_t;

void   SysTick_Init(void);
u32_t  SysTick_GetTime(void);
void   Delay_Set(delay_t *delay, u32_t period, bool_t cycle);
bool_t Delay_Scan(delay_t *delay);
																										

#endif 



