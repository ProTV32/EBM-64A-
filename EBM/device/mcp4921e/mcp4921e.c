#include "mcp4921e.h"
#include "../../driver/spi/spi.h"
#include "../../util/pin.h"

#define VREF      25000 //Опорное напряжение питания для цельночисленных операций
#define DEVISION  250		//Коефициент перевода % в напряжение
#define COMAND    (0x5000)//Командное слово согласно даташита (старшие 4 разряда), приведено к u16_t и сдвинуто в старший байт
//AB   - Выбор выхода: 0 - А линия, 1 - В линия
//BUFF - Задаёт диапазон напряжений: 0 - GND...Vdd
//GA	 - Усиление 0 - 2х, 1 - 1х
//SHDN - Програмное отключение: 0 - активно, 1 - не активно

#define AO1_SS		A,0,L
#define AO1_LDAC  A,4,L

#define AO2_SS		A,1,L
#define AO2_LDAC  A,5,L

#define AO3_SS		A,2,L
#define AO3_LDAC  A,6,L

#define AO4_SS		A,3,L
#define AO4_LDAC  A,7,L

#define AO5_SS		C,7,L
#define AO5_LDAC  C,5,L

#define AO6_SS		C,6,L
#define AO6_LDAC  C,4,L

volatile u8_t AOx;

typedef enum{TX_END, TX_START, TX_WORK}TXstatus_t;

//----------------------------------------------------------------------Раздел переменных--------------------------------------------------------------
SPI_t SPI		= {0};											//Структура инициализации SPI
volatile u16_t Frame = 0;												//Фрейм
volatile TXstatus_t TXstatus = TX_END;

//----------------------------------------------------------------------Модульные функции--------------------------------------------------------------
static void SPIcallback(u8_t *data, bool_t errorTX)
{
	if (TXstatus == TX_START)
		{
			*data = (u8_t)(Frame & 0xFF);
			TXstatus = TX_WORK;
		}
	
	else if(TXstatus == TX_WORK)
		{	
			SPI_Interrupt(False);
			switch(AOx)
				{
					case  1:
						OFF(AO1_SS);
						ON(AO1_LDAC);
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						OFF(AO1_LDAC);
						break;
					case  2:
						OFF(AO2_SS);
						ON(AO2_LDAC);
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						OFF(AO2_LDAC);
						break;
					case  3:
						OFF(AO3_SS);
						ON(AO3_LDAC);
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						OFF(AO3_LDAC);
						break;
					case  4:
						OFF(AO4_SS);
						ON(AO4_LDAC);
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						OFF(AO4_LDAC);
						break;
					case  5:
						OFF(AO5_SS);
						ON(AO5_LDAC);
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						OFF(AO5_LDAC);
						break;
					case  6:
						OFF(AO6_SS);
						ON(AO6_LDAC);
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						OFF(AO6_LDAC);
						break;
				}
			
			SPI_Clean();
			
			TXstatus = TX_END;
			AOx			 = 0;
		}
}

//----------------------------------------------------------------------Внешние функции--------------------------------------------------------------
void DAC_Init(void)
{
	SPI.type				= Master;
	SPI.mode				= Mode0;
	SPI.clk					= High; //На низкой или средней скорости, ведёт себя не коореткно - пропуски байт, не понятная работа прерывания
	SPI.order       = MSB;
	SPI.SPIcallback = SPIcallback;
	
	SPI_Init(&SPI);
	
	DRIVER(AO1_SS,OUT); DRIVER(AO1_LDAC,OUT); DRIVER(AO2_SS,OUT); DRIVER(AO2_LDAC,OUT);
	DRIVER(AO3_SS,OUT); DRIVER(AO3_LDAC,OUT); DRIVER(AO4_SS,OUT); DRIVER(AO4_LDAC,OUT);
	DRIVER(AO5_SS,OUT); DRIVER(AO5_LDAC,OUT); DRIVER(AO6_SS,OUT); DRIVER(AO6_LDAC,OUT);
	
	OFF(AO1_SS); OFF(AO1_LDAC); OFF(AO2_SS); OFF(AO2_LDAC);
	OFF(AO3_SS); OFF(AO3_LDAC); OFF(AO4_SS); OFF(AO4_LDAC);
	OFF(AO5_SS); OFF(AO5_LDAC); OFF(AO6_SS); OFF(AO6_LDAC);
}

void DAC_Write(u8_t out, u8_t value) //Входное значение подаётся в процентах 1% = 0,01 * 2,5В = 0,0249В = 25мВ цена деления. (35% = 35 * 0,025В = 871мВ)
{
	u16_t data;
	
	if ((value > 0) && (value < 100))
		{
			data = (value * DEVISION);	//Искомое значение напряжения
			data = (u16_t) (((float) data / VREF) * 4095);	//Искомое значение DAC
		}
	
	else
		value == 0? (data = 0) : (data = 4095);
			
		
	Frame			= data | COMAND;
	TXstatus	= TX_START;
	AOx				= out;
	
	switch(AOx)
		{
			case  1:
				ON(AO1_SS);
				break;
			case  2:
				ON(AO2_SS);
				break;
			case  3:
				ON(AO3_SS);
				break;
			case  4:
				ON(AO4_SS);
				break;
			case  5:
				ON(AO5_SS);
				break;
			case  6:
				ON(AO6_SS);
				break;
		}
	
	SPI_Start((u8_t) (Frame >> 8));
	
	while(TXstatus != TX_END);
}
