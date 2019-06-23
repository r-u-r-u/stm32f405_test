#pragma once
#include "../Library/Timer/QEI/QEI.h"

class QEI_Timx:public QEI{
public:
    QEI_Timx(TIM_TypeDef *timx,uint32_t prescaler,uint32_t period);
    virtual ~QEI_Timx();
    
    virtual void start();
    virtual void stop();
    virtual int read();
    virtual void set(int set_value);
private:
    TIM_HandleTypeDef htim;
};