#ifndef MCP4921E_H_
#define MCP4921E_H_

#include "../../main.h"

void DAC_Init(void);
void DAC_Write(u8_t out, u8_t value);

#endif /* MCP4921E_H_ */