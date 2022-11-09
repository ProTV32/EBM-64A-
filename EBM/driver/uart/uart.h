#ifndef UART_H_
#define UART_H_

#include "../../main.h"

//--------------------------------------------------UART------------------------------------------------------------//
typedef enum {ParityNone = 0, ParityEven, ParityOdd}UART_Parity_t;
typedef enum {BaudLow = 9600, BaudMid = 19200, BaudHi = 115200}UART_Baudrate_t;
typedef enum {InfBitFifht = 5, InfBitSix = 6, InfBitSeven = 7, InfBitEith = 8, InfBitNine}UART_InfBit_t;
typedef enum {StopBitOne = 1, StopBitTwo}UART_StopBit_t;

typedef struct
{
	u8_t port;
	UART_Baudrate_t baudRate;		//Скорость передачи данных
	UART_InfBit_t   bit;				//Количество информационных бит
	UART_Parity_t   parity;			//Бит чётности
	UART_StopBit_t  stopBit;		//Количество стоп бит
	void (*RXcallback)(u8_t data, bool_t error); //Указатель на функцию обратного вызова: прерывание при приёме байта
	void (*TXcallback)(void);                    //Указатель на функцию обратного вызова: прерывание при окончании передачи
	void (*UDREcallback)(u8_t *data);          //Указатель на функцию обратного вызова: прерывание при опустошении передающего буфера Принимает указатель на байт для можификации, возвращает событие последнего символа
}UART_t;

void UART_RX(UART_t *UART, bool_t enable);			//Управление работой приёмника
void UART_TX(UART_t *UART, bool_t enable);			//Управление работой передатчика
void UART_RXCint(UART_t *UART, bool_t enable);	//Управление прерыванием по окончанию приёма байта
void UART_TXCint(UART_t *UART,bool_t enable);	//Управление прерыванием по окончанию передачи байта
void UART_UDREint(UART_t *UART, bool_t enable); //Управление прерыванием по опустошению буфера передатчика
void UART_Init(UART_t *UART);			//Инициализация порта UART

#endif 