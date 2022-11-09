#include "spi.h"
#include <avr/interrupt.h>

/*
SPI mode 0: CPOL = 0, CPHA=0.
Тактовый сигнал начинается с уровня логического нуля. 
Защелкивание данных выполняется по нарастающему фронту. 
Смена данных происходит по падающему фронту. 
Моменты защелкивание данных показаны на рисунках стрелочками

SPI mode 1: CPOL = 0, CPHA=1. 
Тактовый сигнал начинается с уровня логического нуля. 
Смена данных происходит по нарастающему фронту. 
Защелкивание данных выполняется по падающему фронту.

SPI mode 2: CPOL = 1, CPHA=0. 
Тактовый сигнал начинается с уровня логической единицы. 
Защелкивание данных выполняется по падающему фронту. 
Смена данных выполняется по нарастающему фронту тактового сигнала.

SPI mode 3: CPOL = 1, CPHA=1. 
Тактовый сигнал начинается с уровня логической единицы. 
Смена данных выполняется по падающему фронту тактового сигнала. 
Защелкивание данных выполняется по нарастающему фронту.
*/

#define ERROR_MASK (1<<WCOL)							//Маска ошибки передачи (перезапись)

static volatile u8_t SREG_temp;						//Переменная хранения регистра состояний
static SPI_t *SPIptr;											//Указатель на внешнюю структуру UART
static volatile u8_t IOReg;								//Вспомогательный байт для отчистки флагов

inline void SPI_Enable(bool_t enable)			//Разрешение/Запрет работы модуля
{
	if (enable)
		SPCR |= (1<<SPE);
	
	else
		SPCR &= ~(1<<SPE);
}

inline void SPI_Interrupt(bool_t enable)	//Разрешение/Запрет прерывания
{
	if (enable)
		SPCR |= (1<<SPIE);
	
	else
		SPCR &= ~(1<<SPIE);
}

inline void SPI_Clean(void)								//Отчистка флага прерывания SPIF в SPSR, считыванием регистров
{
	IOReg = SPSR;
	IOReg = SPDR;
}

void SPI_Init(SPI_t *SPI)									//Инициализация переферии SPI
{
	SPIptr = SPI;
	DDRB	|= (1<<2) | (1<<1) | (1<<0);								// Настраиваем порты сигналов SCK и MOSI на выход.
	PORTB |= (1<<2) | (1<<1) | (1<<0);
	
	SPI_Enable(True);
	
	switch (SPIptr->type)
		{
			case Master:
				SPCR |= (1<<MSTR);
				break;
		
			case Slave:
				SPCR &= ~(1<<MSTR);
				break;
		}
		
	switch(SPIptr->mode)
		{
			case Mode0:
				SPCR &= ~((1<<CPOL) | (1<<CPHA));
				break;
			
			case Mode1:
				SPCR &= ~(1<<CPOL);
				SPCR |= (1<<CPHA);
				break;
				
			case Mode2:
				SPCR &= ~(1<<CPHA);
				SPCR |= (1<<CPOL);
				break;
				
			case Mode3:
				SPCR |= (1<<CPOL) | (1<<CPHA);
				break;
		}
	
	switch(SPIptr->clk)
		{
			case Low: // FCPU/128
				SPCR |= (1<<SPR1) | (1<<SPR0);
				SPSR &= ~(1<<SPI2X);
				break;
				
			case Mid: // FCPU/16
				SPCR |= (1<<SPR0);
				SPCR &= ~(1<<SPR1);
				SPSR &= ~(1<<SPI2X);
				break;
				
			case High: // FCPU/2
				SPCR &= ~((1<<SPR1) | (1<<SPR0));
				SPSR = (1<<SPI2X);                
				break;
		}
	
	switch(SPIptr->order)
		{
			case LSB:
				SPCR |= (1<<DORD);	
				break;
			
			case MSB:
				SPCR &= ~(1<<DORD);
				break;	
		}
		
	SPI_Clean();
}

void SPI_Start(u8_t data)                 //Начало процесса передачи
{
	SPI_Clean();
	SPI_Interrupt(True);
	SPDR  = data;														//Запись данных в регистр, вызовет прерывание после окончания передачи
}

ISR(SPI_STC_vect)
{
	SREG_temp			= SREG;								//Сохраняем контекст регистра SREG
	bool_t error	= SPSR & ERROR_MASK;
	u8_t temp			= SPDR;								//Читаем байт из регистра, и передаём его в функцию, 
	SPIptr->SPIcallback(&temp, error);  //которая после прочтения, присвоит переменной по этому адресу новое значения для передачи.
	SPDR					= temp;								//Сдвиговый регистр, выталкиванием байта на передачу - принимаем байт.
	SREG					= SREG_temp;					//Востанавливаем контекст регистра SREG
}