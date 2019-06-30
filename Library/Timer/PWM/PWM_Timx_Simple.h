#pragma once
#include "../Library/Timer/PWM/PWM.h"

__weak void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
class PWM_Timx_Simple:public PWM{
public:
    PWM_Timx_Simple(TIM_TypeDef *timx,uint32_t prescaler,uint32_t period);
    ~PWM_Timx_Simple();
    
    virtual void start(uint32_t channel);
    virtual void start_n(uint32_t channel);
    virtual void stop(uint32_t channel);
    virtual void stop_n(uint32_t channel);
    virtual void set_channel(uint32_t channel);
    virtual void output(uint32_t channel,float duty);
    virtual void output_n(uint32_t channel,float duty);
    virtual void gpio_init();
private:
    TIM_HandleTypeDef htim;
    int32_t _period;
};