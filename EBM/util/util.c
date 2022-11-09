#include "util.h"

const u32_t  pow10Table32[]=
{
	1000000000ul,
	100000000ul,
	10000000ul,
	1000000ul,
	100000ul,
	10000ul,
	1000ul,
	100ul,
	10ul,
	1ul
};

//В массиве из 10 элементов, сохраняется значение в символьном типе, функция возвращает указатель на элемент из этого массива с которого начинается значение 
// после преобразования числа 10 (0000000010) вернётся указатель на 8й элемент
char *utoa_32(u32_t value, char *buffer) 
{
	u8_t i = 0, cnt;
	char *ptr = buffer;
	u32_t pow10;
		
	if(value == 0)
		{
			buffer[0] = '0';
			buffer[1] = '\0';
						
			return buffer;
		}
	/*
	do
		{
			pow10 = pow10Table32[i++];
			cnt = 0;
			
			while(value >= pow10)
				{
					cnt++;
					value -= pow10;
				}
		
			*ptr++ = cnt + '0';
		}
		
	while(i < 10);
	
	*ptr = 0;
	
	// удаляем ведущие нули
	while(buffer[0] == '0') 
		++buffer;
			
	return buffer;
	*/
	
	/*while(value < pow10Table32[i]) i++;
	while(i < 10)
		pow10 = pow10Table32[i++];
		
	*ptr = 0;
	return buffer; //return ptr; // указатель на конец строки полезен для конкатенации
*/
	return buffer;//
}

void utoa_16(u16_t value, char *buffer) //Работает в диапазоне 0...+50569
{
	u8_t p = 4, d[5], q;
	char *ptr = buffer;

	if(value == 0)
		{
			buffer[0] = '0';
			buffer[1] = '\0';
			return;
		}

	d[1] = (value>>4)  & 0xF;
	d[2] = (value>>8)  & 0xF;
	d[3] = (value>>12) & 0xF;

	d[0] = 6 * (d[3] + d[2] + d[1]) + (value & 0xF);
	q    = (d[0] * 0xCD) >> 11;
	d[0] = d[0] - 10 * q;

	d[1] = q + 9 * d[3] + 5 * d[2] + d[1];
	q    = (d[1] * 0xCD) >> 11;
	d[1] = d[1] - 10 * q;

	d[2] = q + 2 * d[2];
	q    = (d[2] * 0x1A) >> 8;
	d[2] = d[2] - 10 * q;

	d[3] = q + 4 * d[3];
	d[4] = (d[3] * 0x1A) >> 8;
	d[3] = d[3] - 10 * d[4];
	
	while(d[p] == 0) //Поиск и отсечение первого нуля
		p--;
	
	for (s8_t i = p; i >= 0; i--)
		*ptr++ = (d[i] + '0');
	
	*ptr = '\0'; //Конец строки в последнюю ячейку
}
//http://homepage.cs.uiowa.edu/~jones/bcd/decimal.html

//http://we.easyelectronics.ru/Soft/preobrazuem-v-stroku-chast-1-celye-chisla.html
void itoa_16(s16_t value, char *buffer) //Работает в диапазоне -32089 ... +32089
{
	u8_t p = 4, d[5], q;
	char *ptr = buffer;

	if (value < 0)
		{
			*ptr++ = '-';
			value = -value;
		}
	
	else if(value == 0)
		{
			buffer[0] = '0';
			buffer[1] = '\0';
			return;
		}
	
	d[1] = (value>>4)  & 0xF;
	d[2] = (value>>8)  & 0xF;
	d[3] = (value>>12) & 0xF;

	d[0] = 6 * (d[3] + d[2] + d[1]) + (value & 0xF);
	q    = (d[0] * 0xCD) >> 11;
	d[0] = d[0] - 10 * q;

	d[1] = q + 9 * d[3] + 5 * d[2] + d[1];
	q    = (d[1] * 0xCD) >> 11;
	d[1] = d[1] - 10 * q;

	d[2] = q + 2 * d[2];
	q    = (d[2] * 0x1A) >> 8;
	d[2] = d[2] - 10 * q;

	d[3] = q + 4 * d[3];
	d[4] = (d[3] * 0x1A) >> 8;
	d[3] = d[3] - 10 * d[4];
  
	while(d[p] == 0) //Поиск и отсечение первого нуля
		p--;
		
	for (s8_t i = p; i >= 0; i--)
		*ptr++ = (d[i] + '0');
	
	*ptr = '\0'; //Конец строки в последнюю ячейку
}



//--------------------------------------------------------------

struct divmod10_t
	{
		u32_t quot;
		u8_t rem;
	};
	
inline static struct divmod10_t divmodu10(u32_t n)
{
	struct divmod10_t res;
	u32_t qq;
	// умножаем на 0.8
	res.quot = n >> 1;
	res.quot += res.quot >> 1;
	res.quot += res.quot >> 4;
	res.quot += res.quot >> 8;
	res.quot += res.quot >> 16;
	qq = res.quot;
	
	// делим на 8
	res.quot >>= 3;
	
	// вычисляем остаток
	res.rem = (u8_t)(n - ((res.quot << 1) + (qq & ~7ul)));
	
	// корректируем остаток и частное
	if(res.rem > 9)
		{
			res.rem -= 10;
			res.quot++;
		}
	
	return res;
}

static char *utoa_fast_div(u32_t value, char *buffer)
{
	struct divmod10_t res;
	
	buffer += 11;
	*--buffer = 0;
	do
		{
			res = divmodu10(value);
			*--buffer = res.rem + '0';
			value = res.quot;
		}
	while (value != 0);
	
	return buffer;
}

char *futoa_32(u32_t value, char *buffer)
{
	const s16_t fractBits = 16;
	u32_t intPart;
	char *ptr, *str_begin;
	const u32_t fractPartMask = (1ul << fractBits) - 1;
	
	if(value == 0)
		{
			buffer[0] = '0';
			buffer[1] = '\0';
			return buffer;
		}
	
	intPart = value >> fractBits;
	value &= fractPartMask;
	ptr = buffer + 5;
	// преобразуем целую часть
	str_begin = utoa_fast_div(intPart, buffer);
	
	/*По моему в функции char *fix_tostr1(uint32_t value, char *buffer) вкралась ошибка, вместо char *str_begin = utoa_fast_div(intPart, ptr); 
	должно было быть char *str_begin = utoa_fast_div(intPart, buffer);. У меня только так заработало. */
	
	// если есть дробная часть
	if(value != 0)
		{
			*ptr = '.';
			
			for(uint_fast8_t i=0; i < 6; i++)
				{
					value &= fractPartMask;
					//value *= 10;
					value <<= 1;
					value += value << 2;
					*++ptr = (uint8_t)(value >> fractBits) + '0';
				}
				// удаляем завершаюшие нули
			while(ptr[0] == '0') --ptr;
			ptr[1] = 0;
		}
		
	return str_begin;
}


u16_t atou_16(char *data)
{
	u16_t value = 0;
	
	while ('0' == *data || *data == ' ' || *data == '"')
		data++;
	
	while ('0' <= *data && *data <= '9')
		{
			value *= 10;
			value += *data - '0';
			data++;
		}
	
	return value;
}





