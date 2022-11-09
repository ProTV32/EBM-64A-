#ifndef CRC_H_
#define CRC_H_

#include "../main.h"


/********************************************************************************************************************************/
//Функции расчёта контрольной суммы
u8_t crc8_1(u8_t *data, u16_t number_of_bytes_in_data);
u8_t crc8_2(u8_t data, u8_t crc);
u8_t crc8_3(u8_t b, u8_t crc);
u8_t crc8_4(u8_t crc, u8_t b);
u8_t crc8_5(u8_t *data, u8_t length);
u16_t crc16_1(u8_t *dta, u8_t length);
u16_t crc16_2 (u8_t *data, u8_t length);


#endif /* CRC_H_ */