#include <avr/interrupt.h>
#include "serialslave.h"
#include "../uart/uart.h"
#include "../../util/crc.h"
#include "../../util/pin.h"

#define  DIR0 E,5,H       //Порт управления драйвером на UART0
#define  DIR1 E,6,H       //Порт управления драйвером на UART1
#define  BUFFER_SIZE	64  //Размерность буфера
#define  TIMER_DIV    8	  //Коэффициент делителя частоты таймера 1

//----------------------------------------------------------------------Раздел типов данных--------------------------------------------------------------
typedef enum
{
	NO_ERROR,							//Нет ошибок
	ILLEGAL_FUNCTION,     //Не верная функция
	ILLEGAL_DATA_ADDR,    //Не верный адрес регистра
	ILLEGAL_DATA_VAL,     //Не верное значение в поле данных
	SLAVE_DEVICE_FAIL,
	ACKNOWLEDGE,
	SLAVE_DEVICE_BUSY,
	NEGATIVE_ACKNOWLEDGE,
	MEMORY_PARITY_ERROR
}MB_Error_t; //Тип данных "Ошибка ModBus"

typedef enum 
{
	READ_COIL = 1, 
	READ_DI, 
	READ_HR, 
	READ_IR, 
	WRITE_COIL, 
	WRITE_HR, 
	WRITE_COILS = 0x0F, 
	WRITE_HRS		= 0x10
}MB_Func_t; //Тип данных "Функция ModBus"

typedef struct
{
	MB_Func_t fcode;  //Номер функции
	u8_t reqLen;			//Длина фрейма без CRC
}FUNCTION_CODES_t;

typedef enum {Idle = 0, RX, Parse, TX} MB_State_t; //Тип данных "Состояние конечного автомата ModBus"

//----------------------------------------------------------------------Раздел переменных--------------------------------------------------------------
FUNCTION_CODES_t readCoil						= {READ_COIL,		5};
FUNCTION_CODES_t readDiscreteInput	= {READ_DI,			5};
FUNCTION_CODES_t readRegister				= {READ_HR,			5};
FUNCTION_CODES_t readInputRegister	= {READ_IR,			5};
FUNCTION_CODES_t writeCoil					= {WRITE_COIL,	5};
FUNCTION_CODES_t writeRegister			= {WRITE_HR,		5};
FUNCTION_CODES_t writeCoils					= {WRITE_COILS, 0};
FUNCTION_CODES_t writeRegisters			= {WRITE_HRS,		0}; //
	
FUNCTION_CODES_t *Regester[] = {&readCoil, &readDiscreteInput, &readRegister, &readInputRegister, &writeCoil, &writeRegister, &writeCoils, &writeRegisters};

const  u8_t MIN_REQUEST_LENGTH	= 7;			//Минимальное количество байт в фрейме для начала обработки
static volatile u8_t SREG_temp;								//Переменная хранения регистра состояний

//---------------------------------------------------------------------Составляющие типы данных для устройства---------------------------------------------------------------

typedef volatile struct												            //Битовые поля служебных данных 
{
	bool_t		  t25		:1; //Флаг превышения интервала 2,5 символа
	bool_t		  t45		:1; //Флаг окончания интервала 4,5 символа
	MB_State_t	state	:2; //Состояние конечного автомата
}MB_Flag_t;

typedef struct											                        //Буфер данных
{
	         u8_t   buff[BUFFER_SIZE];	//Буфер хранения/обработки фрейма
	volatile u8_t   idx;								//Индекс записи/чтения буффера хранения/обработки фрейма
	volatile u8_t   lenght;							//Длинна фрейма
}MB_Buff_t;

typedef struct																	            //Временный пакет Aplication Data Unit 
{
	u8_t			 funcCode;	//Номер функции
	s8_t		  requestLen;  //Пересмотреть размерность
	u16_t	 startAddress;	//Стартовый адрес регистра
	u16_t			 quantity;	//Количество регистров
	u16_t		writeValues;	//Записываемое значение
	u8_t				byteNum;	//Количество байт далее (множественное чтение/запись)
	MB_Error_t		error;	//Код ошибки
	u16_t           crc;	//Контрольная сумма
}ADUtemp_t;

//---------------------------------------------------------------------Структура типа данных для устройства---------------------------------------------------------------
typedef struct
{
	u8_t            ID;		  //Адрес слейва
	MB_Data_t *DATAptr;			//Модульный указатель на внешнюю структуру регистров
	MB_Reg_t	 *REGptr;			//Модульный указатель на внешнюю структуру регистра
	UART_t    *UARTptr;     //Модульный указатель на внешнюю структуру UART 
	MB_Flag_t     Flag;     //Структурная переменная служебных флагов
	MB_Buff_t     Buff;     //Структурная переменная временного хранилища на приём и отправку
	ADUtemp_t  ADUtemp;     //Структурная переменная временного хранения данных при парсинге
}Device_t;


static Device_t Device0, Device1;


//----------------------------------------------------------------------Декларации модульных функций--------------------------------------------------------------
static void RXcallback0		(u8_t data, bool_t errorRX);
static void TXcallback0		(void);
static void UDREcallback0	(u8_t *data);

static void RXcallback1		(u8_t data, bool_t errorRX);
static void TXcallback1		(void);
static void UDREcallback1	(u8_t *data);

static void		ResetState						(Device_t *Device);
static s8_t		GetRequestLength			(Device_t *Device);
static void		Swap									(Device_t *Device, u16_t *destination);
static void		CreateADU							(Device_t *Device);
static bool_t CreateReadCoilPDU			(Device_t *Device);
static bool_t CreateReadRegisterPDU	(Device_t *Device);
static bool_t CreateWriteCoilPDU		(Device_t *Device, bool_t single);
static bool_t CreateWriteRegisterPDU(Device_t *Device, bool_t single);
static bool_t CreatePDU							(Device_t *Device);
static bool_t CheckPDU							(Device_t *Device);
static bool_t ParsePDU							(Device_t *Device);
static bool_t ProcessBuffer					(Device_t *Device);


//----------------------------------------------------------------------Обработчики прерываний (калбеки)--------------------------------------------------------------
static void RXcallback0(u8_t data, bool_t errorRX)		//Функция обратного вызова: прерывание при приёме байта
{
	if (Device0.Flag.state == Idle) //-----------------------------------------------------------Режим ожидания-------------------------------------------------------------------//
		{
			if (!errorRX && Device0.Flag.t45 && ((data == 10) || (data == 11) || (data == 12) || (data == 13))) //Проверка условий, для начала записи
				{
					Device0.ID                            = data; //Сохранение текущего слейва с которым работаем
					Device0.Buff.buff[Device0.Buff.idx++] = data;	//Сохраняем байт в буфер
					Device0.Flag.state		                = RX;		//Переход в состояние приёма фрейма
				}
		}
	
	else if (Device0.Flag.state == RX) //-----------------------------------------------------------Режим приёма фрейма--------------------------------------------------------------//
		{
			if (!errorRX && !Device0.Flag.t25 && (Device0.Buff.idx < BUFFER_SIZE))	//Не нарушен интервал 2,5 символа, нет ошибки приёма, приёмный буфер не переполнен
				Device0.Buff.buff[Device0.Buff.idx++] = data;												  //Сохраняем байт в буфер
		
			else //во всех других ситуациях, выжидаем паузу 4,5 символа, и возвращаемя в режим ожидания
				{
					Device0.Flag.state	= Idle;	//Переход в режим ожидания
					Device0.Buff.idx		= 0;		//Сброс индекса буфера
				}
		}
	
	Device0.Flag.t45 = False;           //Сброс флага тишины между фреймами
	TCNT3   = 0;												//Обнуление счётчика
	ETIFR	 |= (1<<OCF3A) | (1<<OCF3B);	//Сброс флагов прерываний если они были установлены
	TCCR3B  = (1<<WGM32) | (1<<CS31);		//Выбор режима работы таймера по совпадению, установка делителя/запуск таймера
	
}

static void TXcallback0(void)												//Функция обратного вызова: прерывание при окончании передачи
{
	UART_TXCint(Device0.UARTptr, False);				//Запрещаем прерывание по окончанию передачи
	OFF(DIR0);					                        //Переводим драйвер RS485 в режим приёма
	ResetState(&Device0);
}

static void UDREcallback0(u8_t *data)								//Функция обратного вызова: прерывание при опустошениии передающего буфера
{
	*data = Device0.Buff.buff[Device0.Buff.idx++];
	
	if (Device0.Buff.idx == Device0.Buff.lenght)
	{
		UART_UDREint(Device0.UARTptr, False);
		UART_TXCint(Device0.UARTptr, True);
	}
}

ISR(TIMER3_COMPA_vect) //Превышение интервала 4,5
{
	SREG_temp	= SREG; //Сохранение контекста
	
	TCCR3B			     = 0;														//Остановка счётчика
	Device0.Flag.t25 = False;												//Сброс флага ошибки приёма фрейма
	Device0.Flag.t45 = True;												//Установка флага окончания фрейма
	
	if (Device0.Flag.state == RX)										//Если находились в состоянии приёма
		{
			if (Device0.Buff.idx > MIN_REQUEST_LENGTH)	//Колличество принятых байт не меньше минимальной посылки ModBus
				{
					UART_RXCint(Device0.UARTptr, False);		//Запрет прерываний на приём во время обработки
					Device0.Flag.state  = Parse;						//Разрешение обработки фрейма
					Device0.Buff.lenght	= Device0.Buff.idx;	//Сохранение длинны фрейма
				}
		
			else
				Device0.Flag.state	= Idle;								//Возврат в режим ожидания
		
			Device0.Buff.idx = 0;												//Сброс индекса
		}
	
	else
		{
			UART_RXCint(Device0.UARTptr, True);	//Разрешение прерываний при приёме байта
			Device0.Flag.state	  = Idle;				//Переход в режим ожидания
			Device0.ADUtemp.error	= NO_ERROR;		//Сброс ошибки
			Device0.Buff.idx			= 0;					//Сброс индекса указателя буфера
			Device0.Buff.lenght		= 0;					//Сброс счётчика байт в буфере
		}
	
	
	SREG = SREG_temp; //Восстановление контекста
}

ISR(TIMER3_COMPB_vect) //Превышение интервала 2,5
{
	Device0.Flag.t25 = True;	//Установка флага ошибки при приёме фрейма
}


static void RXcallback1(u8_t data, bool_t errorRX)		//Функция обратного вызова: прерывание при приёме байта
{
	if (Device1.Flag.state == Idle) //-----------------------------------------------------------Режим ожидания-------------------------------------------------------------------//
		{
			if (!errorRX && Device1.Flag.t45 && ((data == 11) || (data == 12) || (data == 13) || (data == 14))) //Проверка условий, для начала записи
				{
					Device1.ID                            = data; //Сохранение текущего слейва с которым работаем
					Device1.Buff.buff[Device1.Buff.idx++] = data;	//Сохраняем байт в буфер
					Device1.Flag.state		                = RX;		//Переход в состояние приёма фрейма
				}
		}
	
	else if (Device1.Flag.state == RX) //-----------------------------------------------------------Режим приёма фрейма--------------------------------------------------------------//
		{
			if (!errorRX && !Device1.Flag.t25 && (Device1.Buff.idx < BUFFER_SIZE))	//Не нарушен интервал 2,5 символа, нет ошибки приёма, приёмный буфер не переполнен
				Device1.Buff.buff[Device1.Buff.idx++] = data;												  //Сохраняем байт в буфер
							
			else //во всех других ситуациях, выжидаем паузу 4,5 символа, и возвращаемя в режим ожидания
				{
					Device1.Flag.state	= Idle;	//Переход в режим ожидания
					Device1.Buff.idx		= 0;		//Сброс индекса буфера
				}
			}
			
	Device1.Flag.t45 = False;           //Сброс флага тишины между фреймами
	TCNT1   = 0;												//Обнуление счётчика
	TIFR	 |= (1<<OCF1A) | (1<<OCF1B);	//Сброс флагов прерываний если они были установлены
	TCCR1B  = (1<<WGM12) | (1<<CS11);		//Выбор режима работы таймера по совпадению, установка делителя/запуск таймера
	
} 		
  
static void TXcallback1(void)												//Функция обратного вызова: прерывание при окончании передачи
{
	UART_TXCint(Device1.UARTptr, False);				//Запрещаем прерывание по окончанию передачи
	OFF(DIR1);					                        //Переводим драйвер RS485 в режим приёма
	ResetState(&Device1);
}

static void UDREcallback1(u8_t *data)								//Функция обратного вызова: прерывание при опустошениии передающего буфера
{
	*data = Device1.Buff.buff[Device1.Buff.idx++];
		
	if (Device1.Buff.idx == Device1.Buff.lenght)
		{
			UART_UDREint(Device1.UARTptr, False);
			UART_TXCint(Device1.UARTptr, True);
		}
}

ISR(TIMER1_COMPA_vect) //Превышение интервала 4,5
{
	SREG_temp	= SREG; //Сохранение контекста
	
	TCCR1B			     = 0;														//Остановка счётчика
	Device1.Flag.t25 = False;												//Сброс флага ошибки приёма фрейма
	Device1.Flag.t45 = True;												//Установка флага окончания фрейма
	
	if (Device1.Flag.state == RX)										//Если находились в состоянии приёма
		{
			if (Device1.Buff.idx > MIN_REQUEST_LENGTH)	//Колличество принятых байт не меньше минимальной посылки ModBus
				{
					UART_RXCint(Device1.UARTptr, False);		//Запрет прерываний на приём во время обработки
					Device1.Flag.state  = Parse;						//Разрешение обработки фрейма
					Device1.Buff.lenght	= Device1.Buff.idx;	//Сохранение длинны фрейма
				}
		
			else
				Device1.Flag.state	= Idle;								//Возврат в режим ожидания
					
			Device1.Buff.idx = 0;												//Сброс индекса
		}
		
	else
		{
			UART_RXCint(Device1.UARTptr, True);	//Разрешение прерываний при приёме байта
			Device1.Flag.state	  = Idle;				//Переход в режим ожидания
			Device1.ADUtemp.error	= NO_ERROR;		//Сброс ошибки
			Device1.Buff.idx			= 0;					//Сброс индекса указателя буфера
			Device1.Buff.lenght		= 0;					//Сброс счётчика байт в буфере
		}
		
	
	SREG = SREG_temp; //Восстановление контекста
}

ISR(TIMER1_COMPB_vect) //Превышение интервала 2,5
{
	Device1.Flag.t25 = True;	//Установка флага ошибки при приёме фрейма
}



//----------------------------------------------------------------------Модульные функции--------------------------------------------------------------
static void ResetState(Device_t *Device)												//Сброс параметров
{
		if (Device->UARTptr->port == 1)
			{
				TCNT1   = 0;
				TIFR	 |= (1<<OCF1A) | (1<<OCF1B);
				TCCR1B  = (1<<WGM12) | (1<<CS11);
			}
		
		else
			{
				TCNT3   = 0;												//Обнуление счётчика
				ETIFR	 |= (1<<OCF3A) | (1<<OCF3B);	//Сброс флагов прерываний если они были установлены
				TCCR3B  = (1<<WGM32) | (1<<CS31);		//Выбор режима работы таймера по совпадению, установка делителя/запуск таймера
			}
		
}

static s8_t GetRequestLength(Device_t *Device)									//Возврат длины фрейма с полезной нагрузкой, в зависимости от номера функции (поиск возможных) +
{
	for (u8_t i = 0; i < 8; i++)//Исправить на 8
		{
			if (Device->ADUtemp.funcCode == (Regester[i]->fcode))
				return (Regester[i]->reqLen);
		}
		
	return -1;
}

static void Swap(Device_t *Device, u16_t *destination)					//Заполнение данными +
{
	*destination	= ((u16_t) Device->Buff.buff[Device->Buff.idx] << 8) | Device->Buff.buff[Device->Buff.idx + 1];
	Device->Buff.idx += 2;
}

static void CreateADU(Device_t *Device)													//Формирование полного фрейма +
{
	//Особенности CRC: передаётся, младшим байтом вперёд (требуется переворот)
	Device->ADUtemp.crc = crc16_2(Device->Buff.buff, Device->Buff.idx);
	Device->Buff.buff[Device->Buff.idx++] = (u8_t) (Device->ADUtemp.crc & 0x00FF);
	Device->Buff.buff[Device->Buff.idx++] = (u8_t) (Device->ADUtemp.crc >> 8);
}

static void CreateErrorPDU(Device_t *Device)										//Формирование PDU, для ответа с ошибкой +
{
	Device->Buff.idx = 1;
	Device->Buff.buff[Device->Buff.idx++]	|= 0x80;
	Device->Buff.buff[Device->Buff.idx++]	= Device->ADUtemp.error;
}

static bool_t CreateReadCoilPDU(Device_t *Device)								//Формирование PDU, для ответа с функцией ReadCoil 
{
	//Расположение бит в массиве u8_t
	//Элемент		  0       1         2       3        4        5
	//				00000000 00000000 00000000 00000000 00000000 00000000
	//Регистр 7......0 15.....8 23....16 31....24 39....32 47....40
	
	//Особенности дискретных регистров:
	//Младший бит первого байта данных в ответе, содержит значение регистра, адрес которого указывался в запросе.
	//Остальные значения DO следуют по нарастающей к старшему значению байта. Т.е. справа на лево.
	//Если запрашивалось меньше восьми значений DO, то оставшиеся биты в ответе будут заполнены нулями (в направлении от младшего к старшему байту).
	//Поле Byte Count Количество байт далее указывает количество полных байтов данных в ответе.
	
	
	u8_t bit		= 0;	//Искомый номер элемента массива к которому обращаемся
	u8_t index	= 0;	//Искомый номер бита в элементе массива к которому обращаемся
	u8_t stepr	= 0;	//Шаг итерации в цикле (чтения)
	u8_t stepw	= 0;	//Шаг итерации в цикле (записи)
	u8_t byteNum = (Device->ADUtemp.quantity + 8 - 1) >> 3; //Количество байт для записи, один байт содержит 8 дискретных регистров (округление вверх)
	bool_t status = False;
	
	Device->Buff.idx = 2;
		
	Device->Buff.buff[Device->Buff.idx++]	= byteNum;		
	Device->Buff.buff[Device->Buff.idx + (byteNum - 1)] = 0x00; //Заполнение нулями последнего байта
	bit = Device->ADUtemp.startAddress;
	
	while(bit >= 8)
		{
			bit -= 8;				//Определение начального номера бита в элементе массива к которому обращаемся
			index++;				//Определение начального номера элемента массива к которому обращаемся
		}
	
	stepr = bit;				//определение начального шага для чтения
		
	for (u16_t i = 0; i < Device->ADUtemp.quantity; i++)	//Количество итераций в цикле = количеству запрашиваемых регистров
		{
			if (*(((u8_t*) Device->REGptr->buff) + index) & (1 << stepr)) //Накладыванием смещаемой маски на текущий читаемый элемент
				Device->Buff.buff[Device->Buff.idx] |= (1 << stepw);									//Копируем значение в записываемый буфер на каждом шаге
			
			else
				Device->Buff.buff[Device->Buff.idx] &= ~(1 << stepw); 
			
			stepr++;
			stepw++;
						
			if (stepr > 7) //Переход в следующий элемент массива из которого читаем
				{
					index++;
					stepr = 0;
				}
			
			if (stepw > 7) //Переход в следующий элемент массива в который пишем
				{
					Device->Buff.idx++;
					stepw	= 0;
				}
		}
	
	if (stepw	!= 0)
		Device->Buff.idx++;
		
	return status = True;
}

static bool_t CreateReadRegisterPDU(Device_t *Device)						//Формирование PDU, для ответа с функцией ReadRegister
{
	u16_t ofsetReg  = 0;
	u8_t byteNum = 2 * Device->ADUtemp.quantity;
	Device->Buff.idx = 2;
	Device->Buff.buff[Device->Buff.idx++]	= byteNum;
	
	
	if (Device->ADUtemp.funcCode == READ_IR)
		{
			if(Device->ADUtemp.startAddress > 53281)
				ofsetReg = (Device->ADUtemp.startAddress - 53283) + 19;
			
			else if (Device->ADUtemp.startAddress > 53277)
				ofsetReg = (Device->ADUtemp.startAddress - 53281) + 18;
			
			else if(Device->ADUtemp.startAddress > 53268)	
				ofsetReg = (Device->ADUtemp.startAddress - 53271) + 11;
			
			else if(Device->ADUtemp.startAddress > 53253)
				ofsetReg = (Device->ADUtemp.startAddress - 53264) + 6;
				
			else
				ofsetReg = Device->ADUtemp.startAddress - 53248;
		}
	
	else
		{
			if(Device->ADUtemp.startAddress > 53631)
				ofsetReg = (Device->ADUtemp.startAddress - 53664) + 39;
			
			else if (Device->ADUtemp.startAddress > 53598)
				ofsetReg = (Device->ADUtemp.startAddress - 53616) + 23;
			
			else if(Device->ADUtemp.startAddress > 53589)
				ofsetReg = (Device->ADUtemp.startAddress - 53595) + 19;
			
			else if(Device->ADUtemp.startAddress > 53578)
				ofsetReg = (Device->ADUtemp.startAddress - 53589) + 18;
			
			else if(Device->ADUtemp.startAddress > 53573)
				ofsetReg = (Device->ADUtemp.startAddress - 53575) + 15;
			
			else if(Device->ADUtemp.startAddress > 53536)
				ofsetReg = (Device->ADUtemp.startAddress - 53573) + 14;
			
			else if(Device->ADUtemp.startAddress > 53529)
				ofsetReg = (Device->ADUtemp.startAddress - 53535) + 12;
			
			else if(Device->ADUtemp.startAddress > 53510)
				ofsetReg = (Device->ADUtemp.startAddress - 5329) + 11;
			
			else if(Device->ADUtemp.startAddress > 53507)
				ofsetReg = (Device->ADUtemp.startAddress - 53510) + 10;
			
			else if(Device->ADUtemp.startAddress > 53257)
				ofsetReg = (Device->ADUtemp.startAddress - 53504) + 6;
			
			else if(Device->ADUtemp.startAddress > 53252)
				ofsetReg = (Device->ADUtemp.startAddress - 53257) + 5;
			
			else
				ofsetReg = Device->ADUtemp.startAddress - 53248;
		}
		
	u16_t *ptr = ((u16_t *) Device->REGptr->buff) + ofsetReg; //Смещение указателя на стартовый адрес
		
	for (u8_t i = 0; i < Device->ADUtemp.quantity; i++)
		{
			Device->Buff.buff[Device->Buff.idx++]	= (u8_t) (*ptr >> 8);		//Перенести байт данных
			Device->Buff.buff[Device->Buff.idx++]	= (u8_t) (*ptr & 0xFF);	//Перенести байт данных
			ptr++;																			//Переход на следующую ячейку буфера
		}
		
	return True;
}

static bool_t CreateWriteCoilPDU(Device_t *Device, bool_t single)			//Формирование PDU, для ответа с функцией WriteCoil
{
	//Расположение бит в массиве u8_t
	//Элемент		  0       1         2       3        4        5
	//				00000000 00000000 00000000 00000000 00000000 00000000
	//Регистр 7......0 15.....8 23....16 31....24 39....32 47....40
	
	//Особенности записи одного дискретного регистра:
	//в 4м элементе фрейма (Hi u16_t) должно находится 0xFF - высокий уровень, или 0х00 - низкий уровень
	//в 5м элементе фрейма (Lo u16_t) должно находится всегда - 0х00, иначе не обрабатывается
	
	u8_t bit		= 0;	//Искомый номер бита в элементе массива к которому обращаемся
	u8_t index	= 0;	//Искомый номер элемента массива к которому обращаемся
	u8_t stepr	= 0;	//Шаг итерации в цикле (чтения)
	u8_t stepw	= 0;	//Шаг итерации в цикле (записи)
	u8_t byteNum = (Device->ADUtemp.quantity + 8 - 1) >> 3;	//Количество байт для записи, один байт содержит 8 дискретных регистров (округление вверх)
	bool_t status = False;
		
	bit = Device->ADUtemp.startAddress;
	
	while(bit >= 8)
		{
			bit -= 8;				//Определение начального номера бита в элементе массива к которому обращаемся
			index++;				//Определение начального номера элемента массива к которому обращаемся
		}
	
	if (single) //Одиночная запись
		{
			Device->Buff.idx = 4; //Данные находятся начиная с 4го элемента фрейма
			
			if ((Device->Buff.buff[Device->Buff.idx] == 0xFF) && (Device->Buff.buff[Device->Buff.idx+1] == 0x00))				//Проверка корректности запроса на установку высокого уровня
				{
					*(((u8_t*) Device->REGptr->buff) + index) |= (1 << bit);																								//Запись высокого значения
					Device->Buff.buff[Device->Buff.idx++]	&= ~(~(*(((u8_t*) Device->REGptr->buff) + index)) & (1 << bit));  //Повторное чтение - контроль записи (наложение маски на 0xFF)
				}
				
			else if ((Device->Buff.buff[Device->Buff.idx] == 0x00) && (Device->Buff.buff[Device->Buff.idx+1] == 0x00))	//Проверка корректности запроса на установку низкого уровня
				{
					*(((u8_t*) Device->REGptr->buff) + index)  &= ~(1 << bit);																							//Запись низкого значения
					Device->Buff.buff[Device->Buff.idx++] |= *(((u8_t*) Device->REGptr->buff) + index) & (1 << bit);				//Повторное чтение - контроль записи (наложение маски на 0x00)
				}
			
			else
				{
					Device->ADUtemp.error = ILLEGAL_DATA_VAL; //---------------------- Ответ "Ошибка - не корректное значение" ----------------------
					return status;
				}
			
			Device->Buff.idx++;																										
		}
		
	else //Множественная запись
		{
			u16_t cnt;						//Счётчик
			stepw			= bit;			//определение начального шага для записи
			Device->Buff.idx = 7;	//Данные находятся начиная с 7го элемента фрейма
			
			if (Device->ADUtemp.byteNum != byteNum) //Проверка соответствия количества расчитанных байт передаваемым в запросе
				{
					Device->ADUtemp.error = ILLEGAL_DATA_VAL; //---------------------- Ответ "Ошибка - не корректное значение" ----------------------
					return status;
				}
			
			for (cnt = 0; cnt < Device->ADUtemp.quantity; cnt++)							//Количество итераций в цикле = количеству запрашиваемых регистров
				{
					if (Device->Buff.buff[Device->Buff.idx] & (1 << stepr))				//Накладыванием смещаемой маски на текущий читаемый элемент
						*(((u8_t*) Device->REGptr->buff) + index)  |= (1 << stepw);	//Копируем значение в записываемый буфер на каждом шаге
		
					else
						*(((u8_t*) Device->REGptr->buff) + index) &= ~(1 << stepw);
		
					stepr++;
					stepw++;
		
					if (stepw > 7) //Переход в следующий элемент массива в который пишем
						{
							index++;
							stepw = 0;
						}
		
					if (stepr > 7) //Переход в следующий элемент массива из которого читаем
						{
							Device->Buff.idx++;
							stepr	= 0;
						}
				}
			Device->Buff.idx = 4;     //Количество записанных регистров, передаётся в u16_t начиная с 4го
			Device->Buff.buff[Device->Buff.idx++] = (u8_t) (cnt >> 8);
			Device->Buff.buff[Device->Buff.idx++] = (u8_t) (cnt & 0xFF);	
		}
	return status = True;
}

static bool_t CreateWriteRegisterPDU(Device_t *Device, bool_t single)	//Формирование PDU, для ответа с функцией WriteRegister
{
	bool_t status = False;
	u16_t *ptr = 0; 
	u16_t ofsetReg  = 0;
	
	if(Device->ADUtemp.startAddress > 53631)
		ofsetReg = (Device->ADUtemp.startAddress - 53664) + 39;
	
	else if (Device->ADUtemp.startAddress > 53598)
		ofsetReg = (Device->ADUtemp.startAddress - 53616) + 23;
	
	else if(Device->ADUtemp.startAddress > 53589)
		ofsetReg = (Device->ADUtemp.startAddress - 53595) + 19;
	
	else if(Device->ADUtemp.startAddress > 53578)
		ofsetReg = (Device->ADUtemp.startAddress - 53589) + 18;
	
	else if(Device->ADUtemp.startAddress > 53573)
		ofsetReg = (Device->ADUtemp.startAddress - 53575) + 15;
	
	else if(Device->ADUtemp.startAddress > 53536)
		ofsetReg = (Device->ADUtemp.startAddress - 53573) + 14;
	
	else if(Device->ADUtemp.startAddress > 53529)
		ofsetReg = (Device->ADUtemp.startAddress - 53535) + 12;
	
	else if(Device->ADUtemp.startAddress > 53510)
		ofsetReg = (Device->ADUtemp.startAddress - 5329) + 11;
	
	else if(Device->ADUtemp.startAddress > 53507)
		ofsetReg = (Device->ADUtemp.startAddress - 53510) + 10;
	
	else if(Device->ADUtemp.startAddress > 53257)
		ofsetReg = (Device->ADUtemp.startAddress - 53504) + 6;
	
	else if(Device->ADUtemp.startAddress > 53252)
		ofsetReg = (Device->ADUtemp.startAddress - 53257) + 5;
	
	else
		ofsetReg = Device->ADUtemp.startAddress - 53248;
	
	ptr = ((u16_t *) Device->REGptr->buff) + ofsetReg; //Смещение указателя на стартовый адрес
	
	if (single)
		{
			Device->Buff.idx = 4;																						//Данные находятся начиная с 4го элемента фрейма
			*ptr = ((u16_t) Device->Buff.buff[Device->Buff.idx]) << 8;			//Перенести байт данных из фрейма в буфер (запись)
			*ptr |= Device->Buff.buff[Device->Buff.idx + 1];								//Перенести байт данных из фрейма в буфер (запись)
			Device->Buff.buff[Device->Buff.idx++]	= (u8_t) (*ptr >> 8);			//Перенести байт данных из буфера в фрейм (чтение/проверка)
			Device->Buff.buff[Device->Buff.idx++]	= (u8_t) (*ptr & 0xFF);		//Перенести байт данных из буфера в фрейм (чтение/проверка)
		}
	
	else
		{
			u8_t byteNum = Device->ADUtemp.quantity << 1;
			u16_t cnt;
			Device->Buff.idx = 7; //Данные находятся начиная с 7го элемента фрейма
			
			if (Device->ADUtemp.byteNum != byteNum) //Проверка соответствия количества расчитанных байт передаваемым в запросе
				{
					Device->ADUtemp.error = ILLEGAL_DATA_VAL; //---------------------- Ответ "Ошибка - не корректное значение" ----------------------
					return status;
				}
			
			for (cnt = 0; cnt < Device->ADUtemp.quantity; cnt++)
				{
					*ptr = ((u16_t) Device->Buff.buff[Device->Buff.idx++]) << 8;
					*ptr |= Device->Buff.buff[Device->Buff.idx++];
					ptr++;																			//Переход на следующую ячейку буфера
				}
	
			Device->Buff.idx = 4;     //Количество записанных регистров, передаётся в u16_t начиная с 4го
			Device->Buff.buff[Device->Buff.idx++] = (u8_t) (cnt >> 8);
			Device->Buff.buff[Device->Buff.idx++] = (u8_t) (cnt & 0xFF);
		}
		
		return status = True;
}

static bool_t CreatePDU(Device_t *Device)												//Формирование тела фрейма при нормальном ответе +
{
	bool_t status = False;
	
	switch (Device->ADUtemp.funcCode)
		{
			case READ_COIL:
			case READ_DI:
				status = CreateReadCoilPDU(Device);
				break;
		
			case READ_HR:
			case READ_IR:
				status = CreateReadRegisterPDU(Device);
				break;
		
			case WRITE_COIL:
				status = CreateWriteCoilPDU(Device, True);
				break;
				
			case WRITE_COILS:
				status = CreateWriteCoilPDU(Device, False);
				break;
				
			case WRITE_HR:
				status = CreateWriteRegisterPDU(Device, True);
				break;
		
			case WRITE_HRS:
				status = CreateWriteRegisterPDU(Device, False);
				break;
		}
	return status;
}

static bool_t CheckPDU(Device_t *Device)												//Проверка корретности запрашиваемых данных
{
	u16_t max_register;    //Доступное количество регистров в массиве
	bool_t typreg = False; //Флаг дискретных регистров
	bool_t status = False;
	
	switch (Device->ADUtemp.funcCode) //
		{
			case READ_IR:
				if (Device->ID == 12)
					Device->REGptr = (&Device->DATAptr->IR1);
				else if (Device->ID == 14)
					Device->REGptr = (&Device->DATAptr->IR2);
				else if (Device->ID == 11)
					Device->REGptr = (&Device->DATAptr->IR3);
				else if (Device->ID == 13)
					Device->REGptr = (&Device->DATAptr->IR4);
				break;
			
			case READ_HR:
			case WRITE_HR:
			case WRITE_HRS:
				if (Device->ID == 12)
					Device->REGptr = (&Device->DATAptr->HR1);
				else if (Device->ID == 14)
					Device->REGptr = (&Device->DATAptr->HR2);
				else if (Device->ID == 11)
					Device->REGptr = (&Device->DATAptr->HR3);
				else if (Device->ID == 13)
					Device->REGptr = (&Device->DATAptr->HR4);
				break;
			case READ_DI:
				if (Device->ID == 12)
					Device->REGptr = (&Device->DATAptr->DR1);
				else if (Device->ID == 14)
					Device->REGptr = (&Device->DATAptr->DR2);
				else if (Device->ID == 11)
					Device->REGptr = (&Device->DATAptr->DR3);
				else if (Device->ID == 13)
					Device->REGptr = (&Device->DATAptr->DR4);
				typreg = True;
				break;
			
			case READ_COIL:
			case WRITE_COIL:
			case WRITE_COILS:
				if (Device->ID == 12)
					Device->REGptr = (&Device->DATAptr->CR1);
				else if (Device->ID == 14)
					Device->REGptr = (&Device->DATAptr->CR2);
				else if (Device->ID == 11)
					Device->REGptr = (&Device->DATAptr->CR3);
				else if (Device->ID == 13)
					Device->REGptr = (&Device->DATAptr->CR4);
				typreg = True;
				break;
		}
	
	if (!typreg)	//16bit
		{
			max_register = Device->REGptr->length;			//Определение общего возможного количества регистров = количеству элементов массива
		
			if (Device->ADUtemp.quantity > (BUFFER_SIZE - 5) / 2) //Вычет пяти байт это запас для контрольной суммы(2), адреса(1), номера функции(1), и количества инфомационных байт в фрейме(1)
				{
					Device->ADUtemp.error = SLAVE_DEVICE_FAIL; //Возможно переполнение буфера при формировании ответа
					return status;
				}
		}
	
	else					//1bit
		{
			max_register = Device->REGptr->length << 3; //Определение общего возможного количества регистров (1 элемент массива u8_t хранит 8 регистров)
		
			if(((Device->ADUtemp.quantity - 1) >> 3) + 1 > (BUFFER_SIZE - 5)) //Вычет пяти байт это запас для контрольной суммы(2), адреса(1), номера функции(1), и количества инфомационных байт в фрейме(1)
				{
					Device->ADUtemp.error = SLAVE_DEVICE_FAIL; //Если ёмкости буфера для формирования ответа не достаточно - ошибка
					return status;
				}
		}
	
	/* Обойдена защита по причине не соответсвии адресации
	if ((Device->ADUtemp.startingAddress >= 0) && (Device->ADUtemp.startingAddress < max_register)) //Если начальный регистр существует - обрабатываем
		{
			if ((Device->ADUtemp.quantity < 1) || ((Device->ADUtemp.quantity + Device->ADUtemp.startingAddress) > max_register))	//Если количество запрашиваемых регистров не соответствует существующим - ошибка
				{
					Device->ADUtemp.error = ILLEGAL_DATA_VAL; //---------------------- Ответ "Ошибка - не корректное количество регистров" ----------------------
					return status;
				}
		}
	
	else //---------------------- Ответ "Ошибка - не существующий регистр" ----------------------
		{
			Device->ADUtemp.error = ILLEGAL_DATA_ADDR;
			return status;
		}
	*/
	return status = True;
}

static bool_t ParsePDU(Device_t *Device)												//Обработка фрейма PDU
{
	if (Device->ADUtemp.requestLen == -1) // Не поддерживаемая функция
		{
			Device->ADUtemp.error = ILLEGAL_FUNCTION;
			return False;
		}
	
	else if (Device->ADUtemp.requestLen == 0) //Множественное чтение/запись регистров
		{
			Swap(Device, &Device->ADUtemp.quantity);												//Определение количества регистров для чтения/записи
			Device->ADUtemp.byteNum				 = Device->Buff.buff[Device->Buff.idx++];	//Определение количества байт далее, которые содержат информацию
			Device->ADUtemp.requestLen = Device->ADUtemp.byteNum + 6;  //Расчётное количество байт в PDU
		}
	
	switch (Device->ADUtemp.funcCode)
		{
			case READ_IR:
			case READ_HR:
			case READ_DI:
			case READ_COIL:
				Swap(Device, &Device->ADUtemp.quantity); //Определение количества регистров для чтения/записи
				break;
		
			case WRITE_COIL:
			case WRITE_HR:
				Swap(Device, &Device->ADUtemp.writeValues); //
				Device->ADUtemp.quantity		= 1;
				break;
		}
		
	return CheckPDU(Device);
}	

static bool_t ProcessBuffer(Device_t *Device)										//Обработка данных
{
	Device->Buff.idx = 1;																				//Установка индекса записи в позицию 1 (0й элемент с ID не трогаем)
                                                  //Заполнение временной структуры данными для обработки	
	Device->ADUtemp.funcCode	= Device->Buff.buff[Device->Buff.idx++];		//Номер функции
	Swap(Device, &Device->ADUtemp.startAddress);															//Определение начального адреса
	Device->ADUtemp.requestLen	= GetRequestLength(Device);								//Вычисление длины PDU фрейма, от номера функции
	Device->ADUtemp.crc					= ((u16_t) Device->Buff.buff[Device->Buff.lenght - 1] << 8) | ((u16_t) Device->Buff.buff[Device->Buff.lenght - 2]); //Сохранение контрольной суммы
	
	if (Device->Buff.lenght < Device->ADUtemp.requestLen + 3)							//Проверка длины ADU, совпадения с (CRC+Addr)+PDU
		return False;
	
	if (crc16_2(Device->Buff.buff, Device->Buff.lenght - 2) != Device->ADUtemp.crc) //Проверка контрольной суммы
		return False;
		
	if (!ParsePDU(Device) || !CreatePDU(Device)) //При обнаружении ошибки во время обработки - формирование ответа с кодом ошибки
		CreateErrorPDU(Device);
			
	CreateADU(Device);
		
	return True;
}


//----------------------------------------------------------------------Внешние функции--------------------------------------------------------------		
void MBslave_Init0(UART_t *uart, MB_Data_t *Data) //Инициализация
{
	DRIVER(DIR0,OUT);
	OFF(DIR0);
	Device0.UARTptr = uart;
	Device0.UARTptr->TXcallback   = TXcallback0;
	Device0.UARTptr->RXcallback   = RXcallback0;
	Device0.UARTptr->UDREcallback = UDREcallback0;
	UART_Init(Device0.UARTptr);																					  //Базовая конфигурация UART
	Device0.DATAptr = Data;
	TCCR3A  = 0;																								  //Инициализация регистров таймера
	OCR3A   = 45*(float)F_CPU/Device0.UARTptr->baudRate/TIMER_DIV + 0.5;	//Загрузка значения в регистр для определения временного интервала 4,5 символа
	OCR3B   = 25*(float)F_CPU/Device0.UARTptr->baudRate/TIMER_DIV + 0.5;	//Загрузка значения в регистр для определения временного интервала 2,5 символа
	ETIMSK  |= (1<<OCIE3A)|(1<<OCIE3B);													  //Разрешение прерывание по совпадению А и B таймера 1
	ResetState(&Device0);																					//Иницилизация служебный переменных
}

void MBslave_Init1(UART_t *uart, MB_Data_t *Data) //Инициализация 
{
		DRIVER(DIR1,OUT);
		OFF(DIR1);
		Device1.UARTptr = uart;
		Device1.UARTptr->TXcallback   = TXcallback1;
		Device1.UARTptr->RXcallback   = RXcallback1;
		Device1.UARTptr->UDREcallback = UDREcallback1;
		UART_Init(Device1.UARTptr);																					  //Базовая конфигурация UART
		Device1.DATAptr = Data;
		TCCR1A  = 0;																													//Инициализация регистров таймера
		OCR1A   = 45*(float)F_CPU/Device1.UARTptr->baudRate/TIMER_DIV + 0.5;	//Загрузка значения в регистр для определения временного интервала 4,5 символа
		OCR1B   = 25*(float)F_CPU/Device1.UARTptr->baudRate/TIMER_DIV + 0.5;	//Загрузка значения в регистр для определения временного интервала 2,5 символа
		TIMSK  |= (1<<OCIE1A)|(1<<OCIE1B);																		//Разрешение прерывание по совпадению А и B таймера 1
		Device1.Flag.state	= Idle;																					//Переход в режим ожидания
		ResetState(&Device1);																									//Иницилизация служебный переменных
		
}

void MBslave_Scan0(void)																				//Сканирование запросов в цикле
{
	if (Device0.Flag.state == Parse)	//Обработка фрейма
	{
		if (ProcessBuffer(&Device0))
		{
			Device0.Flag.state = TX;  //Фрейм сформирован, переход в режим передачи
			ON(DIR0);	           //Переводим его в режим передачи
			Device0.Buff.lenght	= Device0.Buff.idx;
			Device0.Buff.idx	  = 0;
			UART_UDREint(Device0.UARTptr, True);
		}
		
		else
		ResetState(&Device0);
	}
}

void MBslave_Scan1(void)																				//Сканирование запросов в цикле
{
	if (Device1.Flag.state == Parse)	//Обработка фрейма
		{
			if (ProcessBuffer(&Device1)) 
				{
					Device1.Flag.state = TX;  //Фрейм сформирован, переход в режим передачи
					ON(DIR1);	                   //Переводим его в режим передачи
					Device1.Buff.lenght	= Device1.Buff.idx;
					Device1.Buff.idx	= 0;
					UART_UDREint(Device1.UARTptr, True);
				}
					
			else
				ResetState(&Device1);
		}
}

                                                        
																															 
//Порядок обработки фреймов - запрос/ответ
//ProcessBuffer - ParsePDU - CheckPDU - CreatePDU - CreateADU - SendADU

//ProcessBuffer. Инициализация: код функции, начальный адрес, проверочная длина PDU, CRC
//ParsePDU.      Проверка корректности кода функции, 
//CheckPDU.      Проверка корректности: начального адреса, количества регистров от начального адреса, возможности передачи запрашиваемых данных от ёмкости передающего буфера
//CreatePDU.     Формирование PDU ответа, согласно запросу
//CreateADU.     Формирование полного фрейма ADU PDU + CRC
