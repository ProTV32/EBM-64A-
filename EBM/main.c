#include "main.h"
#include "systick/systick.h"
#include "device/io/io.h"
#include <avr/interrupt.h>
#include "driver/mbrtu/serialslave.h"
#include "driver/uart/uart.h"
#include <avr/eeprom.h>


//-------------------------------------Декларация функций---------------------------------------
static void Cycle_10ms(void);  //Сканирование дискретных входов (100Гц)
static void Cycle_1000ms(void);
static void CheckFan(Fan_t *Fan, bool_t input);

//------------------------------------Переменные--------------------------------------------------
volatile u16_t IRd1[21] = {8, 0x0017, 0x0532, 3, 0xB006, 0x000B, 0, 0, 0, 0x00A1, 0, 22, 0, 0, 0, 0x0300, 1, 0, 0, 0x0300, 0x0300};
volatile u16_t IRd2[21] = {8, 0x0017, 0x0532, 3, 0xB006, 0x000B, 0, 0, 0, 0x00A1, 0, 23, 0, 0, 0, 0x0300, 1, 0, 0, 0x0300, 0x0300};
volatile u16_t IRd3[21] = {8, 0x0017, 0x0532, 3, 0xB006, 0x000B, 0, 0, 0, 0x00A1, 0, 24, 0, 0, 0, 0x0300, 1, 0, 0, 0x0300, 0x0300};
volatile u16_t IRd4[21] = {8, 0x0017, 0x0532, 3, 0xB006, 0x000B, 0, 0, 0, 0x00A1, 0, 25, 0, 0, 0, 0x0300, 1, 0, 0, 0x0300, 0x0300};

volatile u16_t HRd1[51] = {0, 0, 0, 0, 0, 0, 12, 1, 1, 1, 2, 0x0884, 5, 5, 0, 1, 1, 3, 0x00FF, 2, 0, 0x8000, 0x0080, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,0xFFFF, 0xFFFF, 0xFFFF, 0xAFC8, 0x0EA6, 0x5849, 0x3047, 0x141F, 0x3352, 0x3447, 0x3030, 0x5941, 0x3738, 0x3130, 0}; 
volatile u16_t HRd2[51] = {0, 0, 0, 0, 0, 0, 14, 1, 1, 1, 2, 0x0884, 5, 5, 0, 1, 1, 3, 0x00FF, 2, 0, 0x8000, 0x0080, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,0xFFFF, 0xFFFF, 0xFFFF, 0xAFC8, 0x0EA6, 0x5849, 0x3047, 0x141F, 0x3352, 0x3447, 0x3030, 0x5941, 0x3738, 0x3130, 0};
volatile u16_t HRd3[51] = {0, 0, 0, 0, 0, 0, 11, 1, 1, 1, 2, 0x0884, 5, 5, 0, 1, 1, 3, 0x00FF, 2, 0, 0x8000, 0x0080, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,0xFFFF, 0xFFFF, 0xFFFF, 0xAFC8, 0x0EA6, 0x5849, 0x3047, 0x141F, 0x3352, 0x3447, 0x3030, 0x5941, 0x3738, 0x3130, 0};
volatile u16_t HRd4[51] = {0, 0, 0, 0, 0, 0, 13, 1, 1, 1, 2, 0x0884, 5, 5, 0, 1, 1, 3, 0x00FF, 2, 0, 0x8000, 0x0080, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,0xFFFF, 0xFFFF, 0xFFFF, 0xAFC8, 0x0EA6, 0x5849, 0x3047, 0x141F, 0x3352, 0x3447, 0x3030, 0x5941, 0x3738, 0x3130, 0};

u8_t	CRd1[1]	 = {0}; u8_t	CRd2[1]	 = {0}; u8_t	CRd3[1]	 = {0}; u8_t	CRd4[1]	 = {0};

u8_t  DRd1[1]	 = {0}; u8_t  DRd2[1]	 = {0}; u8_t  DRd3[1]	 = {0}; u8_t  DRd4[1]	 = {0};


MB_Data_t DataReg = 
{
	{(void*) IRd1, sizeof(IRd1)/sizeof(IRd1[0])}, {(void*) IRd2, sizeof(IRd2)/sizeof(IRd2[0])}, {(void*) IRd3, sizeof(IRd3)/sizeof(IRd3[0])}, {(void*) IRd4, sizeof(IRd4)/sizeof(IRd4[0])},
	{(void*) HRd1, sizeof(HRd1)/sizeof(HRd1[0])}, {(void*) HRd2, sizeof(HRd2)/sizeof(HRd2[0])}, {(void*) HRd3, sizeof(HRd3)/sizeof(HRd3[0])}, {(void*) HRd4, sizeof(HRd4)/sizeof(HRd4[0])},
	{(void*) CRd1, sizeof(CRd1)/sizeof(CRd1[0])}, {(void*) CRd2, sizeof(CRd2)/sizeof(CRd2[0])}, {(void*) CRd3, sizeof(CRd3)/sizeof(CRd3[0])}, {(void*) CRd4, sizeof(CRd4)/sizeof(CRd4[0])},
	{(void*) DRd1, sizeof(DRd1)/sizeof(DRd1[0])}, {(void*) DRd2, sizeof(DRd2)/sizeof(DRd2[0])}, {(void*) DRd3, sizeof(DRd3)/sizeof(DRd3[0])},	{(void*) DRd4, sizeof(DRd4)/sizeof(DRd4[0])}
};


static delay_t delay_10   = {0};
static delay_t delay_1000 = {0};

Fan_t Fan[4] = {{1, 0, &HRd1[1], &IRd1[7], StopFan}, {2, 0, &HRd2[1], &IRd2[7], StopFan}, {3, 0, &HRd3[1], &IRd3[7], StopFan}, {4, 0, &HRd4[1], &IRd4[7], StopFan}};
UART_t uart; //Структура настроек ModBus Slave в ОЗУ для использования в программе


//------------------------------------Настройки ModBus  в EEPROM--------------------------------------------------
EEMEM UART_t uartEE = {1, BaudMid, InfBitEith, ParityNone, StopBitOne, 0, 0, 0};



int main(void)
{
	//------------------------Заполнение структур значениями из EEPROM
	eeprom_read_block((void *) &uart, &uartEE, sizeof(UART_t));
	
	
			
	//------------------------Первичная инициализация модулей
	SysTick_Init();
	DIO_Init();
	AIO_Init();
	MBslave_Init1(&uart, &DataReg); //Передача настроек, и указателя на массив данных
		
	Delay_Set(&delay_10,   10,   True); //Установка таймеров
	Delay_Set(&delay_1000, 1000, True);
	
	_ENABLE_IRQ_
	
	while (1)
	{
		Cycle_10ms();
		Cycle_1000ms();
		MBslave_Scan1();
	}
}



static void Cycle_10ms(void) //Опрос дискретных входов, обработка ШИМ выходов
{
	if (Delay_Scan(&delay_10))
		DI_Scan();
}

static void Cycle_1000ms(void)
{
	if (Delay_Scan(&delay_1000))
		{
			u8_t di = DI_Get();
			
			for (u8_t i = 0; i < 4; i++)
				CheckFan(&Fan[i], di & (1<<i));
		}
}

static void CheckFan(Fan_t *Fan, bool_t input)
{
	u16_t tempSpd = *(Fan->newSpd);
		
	switch(Fan->state)
		{
			case StopFan:
				if (input)
					{
						if(tempSpd != Fan->oldSpd)
							{
								Fan->state = RunFan;
								DO_Set(Fan->num, True);
							}
					}
							
				else
					{
						Fan->state    = AlarmFan;
					 *(Fan->newState) = 0x0010;
					}
					
				break;
			
			case RunFan:
				if (input)
					{
						if(tempSpd!= Fan->oldSpd)
							{
								u8_t temp = (tempSpd * 100UL) >> 16; //Дебильное преобразование у EBM
								AO_Set(Fan->num, temp);
								Fan->oldSpd = tempSpd;
								
								if (tempSpd == 0)
									{
										Fan->state = StopFan;
										DO_Set(Fan->num, False);
									}
							}
					}
			
				else
					{
						Fan->state = AlarmFan;
						AO_Set(Fan->num, 0);
						DO_Set(Fan->num, False);
					  *(Fan->newState) = 0x0010; 
					}
					
				break;
				
			case AlarmFan:
				if (input)
					{
						Fan->state = StopFan;
						Fan->oldSpd = 0;
						*(Fan->newState) = 0x0000;
					}
								
				break;
		}
}


