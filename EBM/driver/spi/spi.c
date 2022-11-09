#include "spi.h"
#include <avr/interrupt.h>

/*
SPI mode 0: CPOL = 0, CPHA=0.
�������� ������ ���������� � ������ ����������� ����. 
������������ ������ ����������� �� ������������ ������. 
����� ������ ���������� �� ��������� ������. 
������� ������������ ������ �������� �� �������� �����������

SPI mode 1: CPOL = 0, CPHA=1. 
�������� ������ ���������� � ������ ����������� ����. 
����� ������ ���������� �� ������������ ������. 
������������ ������ ����������� �� ��������� ������.

SPI mode 2: CPOL = 1, CPHA=0. 
�������� ������ ���������� � ������ ���������� �������. 
������������ ������ ����������� �� ��������� ������. 
����� ������ ����������� �� ������������ ������ ��������� �������.

SPI mode 3: CPOL = 1, CPHA=1. 
�������� ������ ���������� � ������ ���������� �������. 
����� ������ ����������� �� ��������� ������ ��������� �������. 
������������ ������ ����������� �� ������������ ������.
*/

#define ERROR_MASK (1<<WCOL)							//����� ������ �������� (����������)

static volatile u8_t SREG_temp;						//���������� �������� �������� ���������
static SPI_t *SPIptr;											//��������� �� ������� ��������� UART
static volatile u8_t IOReg;								//��������������� ���� ��� �������� ������

inline void SPI_Enable(bool_t enable)			//����������/������ ������ ������
{
	if (enable)
		SPCR |= (1<<SPE);
	
	else
		SPCR &= ~(1<<SPE);
}

inline void SPI_Interrupt(bool_t enable)	//����������/������ ����������
{
	if (enable)
		SPCR |= (1<<SPIE);
	
	else
		SPCR &= ~(1<<SPIE);
}

inline void SPI_Clean(void)								//�������� ����� ���������� SPIF � SPSR, ����������� ���������
{
	IOReg = SPSR;
	IOReg = SPDR;
}

void SPI_Init(SPI_t *SPI)									//������������� ��������� SPI
{
	SPIptr = SPI;
	DDRB	|= (1<<2) | (1<<1) | (1<<0);								// ����������� ����� �������� SCK � MOSI �� �����.
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

void SPI_Start(u8_t data)                 //������ �������� ��������
{
	SPI_Clean();
	SPI_Interrupt(True);
	SPDR  = data;														//������ ������ � �������, ������� ���������� ����� ��������� ��������
}

ISR(SPI_STC_vect)
{
	SREG_temp			= SREG;								//��������� �������� �������� SREG
	bool_t error	= SPSR & ERROR_MASK;
	u8_t temp			= SPDR;								//������ ���� �� ��������, � ������� ��� � �������, 
	SPIptr->SPIcallback(&temp, error);  //������� ����� ���������, �������� ���������� �� ����� ������ ����� �������� ��� ��������.
	SPDR					= temp;								//��������� �������, ������������� ����� �� �������� - ��������� ����.
	SREG					= SREG_temp;					//�������������� �������� �������� SREG
}