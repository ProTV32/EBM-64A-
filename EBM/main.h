#ifndef MAIN_H_
	#define MAIN_H_

#include <avr/io.h>

//#define  F_CPU 14745600UL
#define  F_CPU 11059200UL
//#define  F_CPU 8000000UL

#ifndef _DISABLE_IRQ_
	#define _DISABLE_IRQ_ cli();
#endif

#ifndef _ENABLE_IRQ_
	#define _ENABLE_IRQ_ sei();
#endif

#ifndef NULL
	#define NULL 0
#endif

#ifndef FALSE
	#define FALSE 0
#endif

#ifndef TRUE
	#define TRUE 1
#endif



//--------------------------------------------------Data Type Section------------------------------------------------------------//
typedef unsigned char  u8_t;
typedef unsigned int  u16_t;
typedef unsigned long u32_t;
typedef signed   char  s8_t;
typedef signed   int  s16_t;
typedef signed   long s32_t;
typedef enum {False = 0, True} bool_t;

typedef enum{StopFan, RunFan, AlarmFan}FanState_t;

typedef struct
{
	u8_t      num;
	u16_t			oldSpd;
	volatile u16_t		*newSpd;
	volatile u16_t    *newState;
	FanState_t state;
}Fan_t;

#endif 