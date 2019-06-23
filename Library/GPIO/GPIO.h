#pragma once
#include "stm32f4xx_hal.h"

class GPIO{
protected:
    bool state;
public:
    enum class PIN{
        PIN_1 = GPIO_PIN_1,
        PIN_2 = GPIO_PIN_2,
        PIN_3 = GPIO_PIN_3,
        PIN_4 = GPIO_PIN_4,
        PIN_5 = GPIO_PIN_5,
        PIN_6 = GPIO_PIN_6,
        PIN_7 = GPIO_PIN_7,
        PIN_8 = GPIO_PIN_8,
        PIN_9 = GPIO_PIN_9,
        PIN_10 = GPIO_PIN_10,
        PIN_11 = GPIO_PIN_11,
        PIN_12 = GPIO_PIN_12,
        PIN_13 = GPIO_PIN_13,
        PIN_14 = GPIO_PIN_14,
        PIN_15 = GPIO_PIN_15,
    };
    GPIO();
    virtual ~GPIO();
    virtual void set(PIN pin,bool state);
    virtual void on(PIN pin);
    virtual void off(PIN pin);
    virtual void toggle(PIN pin);
    virtual bool read(PIN pin);
};