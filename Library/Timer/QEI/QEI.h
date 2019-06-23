#pragma once
#include "stm32f4xx_hal.h"

class QEI{
public:
    QEI();
    virtual ~QEI();
    virtual void start();
    virtual void stop();
    virtual int read();
    virtual void set(int set_value);
    void reset();
};