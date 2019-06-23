#pragma once
#include "../Library/GPIO/GPIO.h"

class Input:public GPIO{
private:
    GPIO_TypeDef *_port;
public:
    Input(GPIO_TypeDef *port,uint16_t pin);
    virtual ~Input();
    virtual void set(PIN pin,bool state);
    virtual void on(PIN pin);
    virtual void off(PIN pin);
    virtual void toggle(PIN pin);
    virtual bool read(PIN pin);
};