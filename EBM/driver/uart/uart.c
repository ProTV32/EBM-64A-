#include "uart.h"
#include <avr/interrupt.h>

#define ERROR_MASK ((1<<UPE0) | (1<<FE0) | (1<<DOR0)) //Маска ошибок приёма

static volatile u8_t SREG_temp;												//Переменная хранения регистра состояний
static UART_t *UART1ptr = NULL;												//Указатель на внешнюю структуру UART
static UART_t *UART0ptr = NULL;												//Указатель на внешнюю структуру UART

void UART_RX(UART_t *UART, bool_t enable)							//Управление работой приёмника
{
	if (UART->port == 1)
		{
			if (enable)
				UCSR1B |= (1<<RXEN1);
			else
				UCSR1B &= ~(1<<RXEN1);
		}
	
	else
		{
			if (enable)
				UCSR0B |= (1<<RXEN0);
			else
				UCSR0B &= ~(1<<RXEN0);
		}
}

void UART_TX(UART_t *UART, bool_t enable)							//Управление работой передатчика
{
	if (UART->port == 1)
		{
			if (enable)
				UCSR1B |= (1<<TXEN1);
			else
				UCSR1B &= ~(1<<TXEN1);
		}
	
	else	
		{
			if (enable)
				UCSR0B |= (1<<TXEN0);
			else
				UCSR0B &= ~(1<<TXEN0);
		}
}

void UART_RXCint(UART_t *UART, bool_t enable)					//Управление прерыванием по окончанию приёма байта
{
	if (UART->port == 1)
		{
			if (enable)
				UCSR1B |= (1<<RXCIE1);
			else
				UCSR1B &= ~(1<<RXCIE1);
		}
		
	else
		{
			if (enable)
			UCSR0B |= (1<<RXCIE0);
			else
			UCSR0B &= ~(1<<RXCIE0);
		}
} 

void UART_TXCint(UART_t *UART, bool_t enable)					//Управление прерыванием по окончанию передачи байта
{
	if (UART->port == 1)
		{
			if (enable)
				UCSR1B |= (1<<TXCIE1);
			else
				UCSR1B &= ~(1<<TXCIE1);
		}
	
	else
		{
			if (enable)
				UCSR0B |= (1<<TXCIE0);
			else
				UCSR0B &= ~(1<<TXCIE0);
		}
}

void UART_UDREint(UART_t *UART, bool_t enable)				//Управление прерыванием по опустошению буфера передатчика
{
	if (UART->port == 1)
		{
			if (enable)
				UCSR1B |= (1<<UDRIE1);
			
			else
				UCSR1B &= ~(1<<UDRIE1);
		}
		
	else
		{
			if (enable)
				UCSR0B |= (1<<UDRIE0);
			
			else
				UCSR0B &= ~(1<<UDRIE0);
		}
}

static void UART_SetBaud(void)	//Расчёт и установка значения регистра скорости передачи
{
	if (UART0ptr)
		{
			u16_t ubrr = F_CPU / 16 / UART0ptr->baudRate - 1;
			
			UBRR0H = (u8_t) (ubrr>>8);
			UBRR0L = (u8_t) ubrr;
		}
	
	if (UART1ptr)
		{
			u16_t ubrr = F_CPU / 16 / UART1ptr->baudRate - 1;
			
			UBRR1H = (u8_t) (ubrr>>8);
			UBRR1L = (u8_t) ubrr;
		}
}

static void UART_SetBit(void)		//Установка значения регистров количества информационных бит
{
	if (UART0ptr)
		{
			switch(UART0ptr->bit)
				{
					case InfBitFifht:
					UCSR0C &= ~((1<<UCSZ01) | (1<<UCSZ00));
					UCSR0B &= ~(1<<UCSZ02);
					break;
					
					case InfBitSix:
					UCSR0B &= ~(1<<UCSZ02);
					UCSR0C &= ~(1<<UCSZ01);
					UCSR0C |= (1<<UCSZ00);
					break;
					
					case InfBitSeven:
					UCSR0B &= ~(1<<UCSZ02);
					UCSR0C &= ~(1<<UCSZ00);
					UCSR0C |= (1<<UCSZ01);
					break;
					
					case InfBitEith:
					UCSR0B &= ~(1<<UCSZ02);
					UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
					break;
					
					default:
					UCSR0B &= ~(1<<UCSZ02);
					UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
				}
		}
	
	if (UART1ptr)
	{
		switch(UART1ptr->bit)
		{
			case InfBitFifht:
			UCSR1C &= ~((1<<UCSZ11) | (1<<UCSZ10));
			UCSR1B &= ~(1<<UCSZ12);
			break;
			
			case InfBitSix:
			UCSR1B &= ~(1<<UCSZ12);
			UCSR1C &= ~(1<<UCSZ11);
			UCSR1C |= (1<<UCSZ10);
			break;
			
			case InfBitSeven:
			UCSR1B &= ~(1<<UCSZ12);
			UCSR1C &= ~(1<<UCSZ10);
			UCSR1C |= (1<<UCSZ11);
			break;
			
			case InfBitEith:
			UCSR1B &= ~(1<<UCSZ12);
			UCSR1C |= (1<<UCSZ11) | (1<<UCSZ10);
			break;
			
			default:
			UCSR1B &= ~(1<<UCSZ12);
			UCSR1C |= (1<<UCSZ11) | (1<<UCSZ10);
		}
	}
}

static void UART_SetParity(void)//Установка значения регистров чётности
{
	if (UART0ptr)
		{
			switch(UART0ptr->parity)
				{
					case ParityNone:
					UCSR0C &= ~((1<<UPM01) | (1<<UPM00));
					break;
					
					case ParityEven:
					UCSR0C |= (1<<UPM01);
					UCSR0C &= ~(1<<UPM00);
					break;
					
					case ParityOdd:
					UCSR0C |= (1<<UPM01) | (1<<UPM00);
					break;
					
					default:
					UCSR0C &= ~((1<<UPM01) | (1<<UPM01));
				}
		}
	
	if (UART1ptr)
	{
		switch(UART1ptr->parity)
		{
			case ParityNone:
			UCSR1C &= ~((1<<UPM11) | (1<<UPM10));
			break;
			
			case ParityEven:
			UCSR1C |= (1<<UPM11);
			UCSR1C &= ~(1<<UPM10);
			break;
			
			case ParityOdd:
			UCSR1C |= (1<<UPM11) | (1<<UPM10);
			break;
			
			default:
			UCSR1C &= ~((1<<UPM11) | (1<<UPM11));
		}
	}
	
}

static void UART_SetStopBit(void)//Установка значения регистров количества стоповых бит
{
	if (UART0ptr)
		{
			if (UART0ptr->stopBit == StopBitOne)
				UCSR0C &= ~(1<<USBS0);
			
			else
				UCSR0C |= (1<<USBS0);
		}
		
	if (UART1ptr)
		{
			if (UART1ptr->stopBit == StopBitOne)
				UCSR1C &= ~(1<<USBS1);
			
			else
				UCSR1C |= (1<<USBS1);
		}
}

void UART_Init(UART_t *UART)						 //Инициализация порта UART
{
	if (UART->port == 0)
		UART0ptr = UART;
	
	else
		UART1ptr = UART;		
			
	UART_SetBaud();
	UART_SetBit();
	UART_SetParity();
	UART_SetStopBit();
	UART_RX(UART, True);
	UART_TX(UART, True);
}


ISR(USART1_RX_vect)
{
	SREG_temp			= SREG; //Сохранение контекста
	bool_t error	= UCSR1A & ERROR_MASK; //Вычисление ошибки приёма
	u8_t data			= UDR1; //Чтение данных
	
	UART1ptr->RXcallback(data, error); //Передача в обработку
	SREG = SREG_temp; //Восстановление контекста
}

ISR(USART1_UDRE_vect)
{
	SREG_temp = SREG;	//Сохраняем контекст регистра SREG
	u8_t data;
	
	UART1ptr->UDREcallback(&data);		//	
	UDR1 = data;			//Записываем в регистр
			
	SREG = SREG_temp;	//Востанавливаем контекст регистра SREG
}

ISR(USART1_TX_vect)
{
	SREG_temp = SREG;	//Сохраняем контекст регистра SREG
	UART1ptr->TXcallback(); 
	SREG = SREG_temp;	//Востанавливаем контекст регистра SREG
}


ISR(USART0_RX_vect)
{
	SREG_temp			= SREG; //Сохранение контекста
	bool_t error	= UCSR0A & ERROR_MASK; //Вычисление ошибки приёма
	u8_t data			= UDR0; //Чтение данных
	
	UART0ptr->RXcallback(data, error); //Передача в обработку
	SREG = SREG_temp; //Восстановление контекста
}

ISR(USART0_UDRE_vect)
{
	SREG_temp = SREG;	//Сохраняем контекст регистра SREG
	u8_t data;
	
	UART0ptr->UDREcallback(&data);		//
	UDR0 = data;			//Записываем в регистр
	
	SREG = SREG_temp;	//Востанавливаем контекст регистра SREG
}

ISR(USART0_TX_vect)
{
	SREG_temp = SREG;	//Сохраняем контекст регистра SREG
	UART0ptr->TXcallback();
	SREG = SREG_temp;	//Востанавливаем контекст регистра SREG
}
