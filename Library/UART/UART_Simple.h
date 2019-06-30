#pragma once
#include "UART.h"

class UART_Simple:public UART{
public:
    UART_Simple(USART_TypeDef *uartx);
    virtual ~UART_Simple();
    virtual void _putc(uint8_t c);
    virtual uint8_t _getc();
    virtual void _puts(uint8_t *c);
    virtual void _gets(uint8_t *c);
};