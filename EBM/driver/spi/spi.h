#ifndef SPI_H_
#define SPI_H_


#include "../../main.h"

//--------------------------------------------------SPI------------------------------------------------------------//
typedef enum {Master, Slave}SPI_Type_t;
typedef enum {Mode0, Mode1, Mode2, Mode3}SPI_Mode_t;
typedef enum {Low, Mid, High}SPI_Clock_t;
typedef enum {LSB, MSB}SPI_Order_t;

typedef struct
{
	SPI_Type_t  type;		//��� (Master/Slave)
	SPI_Mode_t  mode;		//�����
	SPI_Clock_t clk;    //�������
	SPI_Order_t order;  //���������� ����
	void (*SPIcallback)(u8_t *data, bool_t errorTX); //��������� �� ������� ��������� ������: ���������� ��� ��������� �������� �����, ������� �� ��������� �������� ������, ����� ������������
}SPI_t;

void SPI_Enable(bool_t enable);			// ����������/������ ������ ������
void SPI_Interrupt(bool_t enable);  // ����������/������ ����������
void SPI_Clean(void);								//������ ���������
void SPI_Init(SPI_t *SPI);					// ������������� ����
void SPI_Start(u8_t data);          // ������ �������� ��������


#endif /* SPI_H_ */