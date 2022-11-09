#ifndef UTIL_H_
#define UTIL_H_

#include "../main.h"

/********************************************************************************************************************************/
//Функции ковертирования чисел в символы
char *utoa_32(u32_t value, char *buffer);
void utoa_16(u16_t value, char *buffer);
void itoa_16(s16_t value, char *buffer);
char *futoa_32(u32_t value, char *buffer);
u16_t atou_16(char *data);

/********************************************************************************************************************************/
//Операции с байтами
// Варианты создания INT из двух байт, а - младший байт, в - старший байт
/*#define MAKE_U16(low,high) ((u16)   (((u8)(low)) | (((u16)((u8)(high))) <<8)))
#define MAKE_U16(low,high) ((u16) low | (high<<8))

u16_t MakeU16(u8_t low, u8_t high)
{
	return ((u16_t)high << 8) | low;
}

// Разбивка INT на байты. Принимает на вход значение, и адреса на ячейки байт для расположения результата
#define BREAK_U16(value,a,b) {(*a = (u8) (value & 0x00ff)); (*b = (u8) (value>>8));}
*/


#endif /* UTIL_H_ */