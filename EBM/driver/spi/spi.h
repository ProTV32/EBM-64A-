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
	SPI_Type_t  type;		//Тип (Master/Slave)
	SPI_Mode_t  mode;		//Режим
	SPI_Clock_t clk;    //Частота
	SPI_Order_t order;  //Очерёдность байт
	void (*SPIcallback)(u8_t *data, bool_t errorTX); //Указатель на функцию обратного вызова: прерывание при окончании передачи байта, вначале по указателю читаются данные, после записываются
}SPI_t;

void SPI_Enable(bool_t enable);			// Разрешение/Запрет работы модуля
void SPI_Interrupt(bool_t enable);  // Разрешение/Запрет прерывания
void SPI_Clean(void);								//Чистка регистров
void SPI_Init(SPI_t *SPI);					// Инициализация шины
void SPI_Start(u8_t data);          // Начало процесса передачи


#endif /* SPI_H_ */