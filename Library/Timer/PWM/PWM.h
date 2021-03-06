#pragma once
#include "stm32f4xx_hal.h"

class PWM{
public:
    PWM();
    virtual ~PWM();
    virtual void start(uint32_t channel);
    virtual void start_n(uint32_t channel);
    virtual void stop(uint32_t channel);
    virtual void stop_n(uint32_t channel);
    virtual void set_channel(uint32_t channel);
    virtual void output(uint32_t channel,float duty);
    virtual void output_n(uint32_t channel,float duty);
    virtual void gpio_init();
};