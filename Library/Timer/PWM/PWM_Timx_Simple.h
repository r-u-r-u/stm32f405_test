#pragma once
#include "../Library/Timer/PWM/PWM.h"

class PWM_Timx_Simple:public PWM{
public:
    PWM_Timx_Simple(TIM_TypeDef *timx,uint32_t prescaler,uint32_t period);
    virtual ~PWM_Timx_Simple();
    
    virtual void start(uint32_t channel);
    virtual void stop(uint32_t channel);
    virtual void set_channel(uint32_t channel);
    virtual void output(uint32_t channel,float duty);
    virtual void gpio_init();
private:
    TIM_HandleTypeDef htim;
    int32_t _period;
};