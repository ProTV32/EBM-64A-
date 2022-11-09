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
	UART_Baudrate_t baudRate;		//�������� �������� ������
	UART_InfBit_t   bit;				//���������� �������������� ���
	UART_Parity_t   parity;			//��� ��������
	UART_StopBit_t  stopBit;		//���������� ���� ���
	void (*RXcallback)(u8_t data, bool_t error); //��������� �� ������� ��������� ������: ���������� ��� ����� �����
	void (*TXcallback)(void);                    //��������� �� ������� ��������� ������: ���������� ��� ��������� ��������
	void (*UDREcallback)(u8_t *data);          //��������� �� ������� ��������� ������: ���������� ��� ����������� ����������� ������ ��������� ��������� �� ���� ��� �����������, ���������� ������� ���������� �������
}UART_t;

void UART_RX(UART_t *UART, bool_t enable);			//���������� ������� ��������
void UART_TX(UART_t *UART, bool_t enable);			//���������� ������� �����������
void UART_RXCint(UART_t *UART, bool_t enable);	//���������� ����������� �� ��������� ����� �����
void UART_TXCint(UART_t *UART,bool_t enable);	//���������� ����������� �� ��������� �������� �����
void UART_UDREint(UART_t *UART, bool_t enable); //���������� ����������� �� ����������� ������ �����������
void UART_Init(UART_t *UART);			//������������� ����� UART

#endif 