#ifndef SERIALSLAVE_H_
#define SERIALSLAVE_H_

#include "../../main.h"
#include "../uart/uart.h"

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 1 page 12)
 * Quantity of Coils to read (2 bytes): 1 to 2000 (0x7D0)
 * (chapter 6 section 11 page 29)
 * Quantity of Coils to write (2 bytes): 1 to 1968 (0x7B0)
 */
#define MODBUS_MAX_READ_BITS              2000
#define MODBUS_MAX_WRITE_BITS             1968

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 3 page 15)
 * Quantity of Registers to read (2 bytes): 1 to 125 (0x7D)
 * (chapter 6 section 12 page 31)
 * Quantity of Registers to write (2 bytes) 1 to 123 (0x7B)
 * (chapter 6 section 17 page 38)
 * Quantity of Registers to write in R/W registers (2 bytes) 1 to 121 (0x79)
 */
#define MODBUS_MAX_READ_REGISTERS          125
#define MODBUS_MAX_WRITE_REGISTERS         123
#define MODBUS_MAX_WR_WRITE_REGISTERS      121
#define MODBUS_MAX_WR_READ_REGISTERS       125

/* The size of the MODBUS PDU is limited by the size constraint inherited from
 * the first MODBUS implementation on Serial Line network (max. RS485 ADU = 256
 * bytes). Therefore, MODBUS PDU for serial line communication = 256 - Server
 * address (1 byte) - CRC (2 bytes) = 253 bytes.
 */
#define MODBUS_MAX_PDU_LENGTH              253

/* Consequently:
 * - RTU MODBUS ADU = 253 bytes + Server address (1 byte) + CRC (2 bytes) = 256
 *   bytes.
 * - TCP MODBUS ADU = 253 bytes + MBAP (7 bytes) = 260 bytes.
 * so the maximum of both backend in 260 bytes. This size can used to allocate
 * an array of bytes to store responses and it will be compatible with the two
 * backends.
 */
#define MODBUS_MAX_ADU_LENGTH              260

/**UTILS FUNCTIONS**/

#define MODBUS_GET_HIGH_BYTE(data) (((data) >> 8) & 0xFF)
#define MODBUS_GET_LOW_BYTE(data) ((data) & 0xFF)
#define MODBUS_GET_INT64_FROM_INT16(tab_int16, index) \
    (((int64_t)tab_int16[(index)    ] << 48) | \
     ((int64_t)tab_int16[(index) + 1] << 32) | \
     ((int64_t)tab_int16[(index) + 2] << 16) | \
      (int64_t)tab_int16[(index) + 3])
#define MODBUS_GET_INT32_FROM_INT16(tab_int16, index) \
    (((int32_t)tab_int16[(index)    ] << 16) | \
      (int32_t)tab_int16[(index) + 1])
#define MODBUS_GET_INT16_FROM_INT8(tab_int8, index) \
    (((int16_t)tab_int8[(index)    ] << 8) | \
      (int16_t)tab_int8[(index) + 1])
#define MODBUS_SET_INT16_TO_INT8(tab_int8, index, value) \
    do { \
        ((int8_t*)(tab_int8))[(index)    ] = (int8_t)((value) >> 8);  \
        ((int8_t*)(tab_int8))[(index) + 1] = (int8_t)(value); \
    } while (0)
#define MODBUS_SET_INT32_TO_INT16(tab_int16, index, value) \
    do { \
        ((int16_t*)(tab_int16))[(index)    ] = (int16_t)((value) >> 16); \
        ((int16_t*)(tab_int16))[(index) + 1] = (int16_t)(value); \
    } while (0)
#define MODBUS_SET_INT64_TO_INT16(tab_int16, index, value) \
    do { \
        ((int16_t*)(tab_int16))[(index)    ] = (int16_t)((value) >> 48); \
        ((int16_t*)(tab_int16))[(index) + 1] = (int16_t)((value) >> 32); \
        ((int16_t*)(tab_int16))[(index) + 2] = (int16_t)((value) >> 16); \
        ((int16_t*)(tab_int16))[(index) + 3] = (int16_t)(value); \
    } while (0)


typedef struct
{
	void *buff;
	u16_t length;
}MB_Reg_t;

typedef struct 
{
	MB_Reg_t IR1, IR2, IR3, IR4, HR1, HR2, HR3, HR4, CR1, CR2, CR3, CR4, DR1, DR2, DR3, DR4;
} MB_Data_t;

//--------------------------------------------------ModBus Slave EEPROM------------------------------------------------------------//


void MBslave_Init0(UART_t *uart, MB_Data_t *Data);
void MBslave_Init1(UART_t *uart, MB_Data_t *Data);
void MBslave_Scan0(void);
void MBslave_Scan1(void);


#endif /* SERIALSLAVE_H_ */