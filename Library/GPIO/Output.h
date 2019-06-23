#pragma once
#include "../Library/GPIO/GPIO.h"

class Output:public GPIO{
private:
    GPIO_TypeDef *_port;
public:
    Output(GPIO_TypeDef *port,uint16_t pin);
    virtual ~Output();
    virtual void set(PIN pin,bool state);
    virtual void on(PIN pin);
    virtual void off(PIN pin);
    virtual void toggle(PIN pin);
    virtual bool read(PIN pin);
};