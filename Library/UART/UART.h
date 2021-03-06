#pragma once
#include "stm32f4xx_hal.h"

class UART{
public:
    UART();
    virtual ~UART();
    virtual void _putc(uint8_t c);
    virtual uint8_t _getc();
    virtual void _puts(uint8_t *c);
    virtual void _gets(uint8_t *c);
protected:
    UART_HandleTypeDef huart;
};