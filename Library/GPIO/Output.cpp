#include "../Library/GPIO/Output.h"

Output::Output(GPIO_TypeDef *port,uint16_t pin){
    _port = port;
    GPIO_InitTypeDef GPIO_InitStruct;
    if(_port == GPIOA){
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }else if(_port == GPIOB){
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }else if(_port == GPIOC){
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }else{

    }
    HAL_GPIO_WritePin(_port,pin,GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(_port, &GPIO_InitStruct);
}
Output::~Output(){
}
void Output::on(PIN pin){
    HAL_GPIO_WritePin(_port,static_cast<uint16_t>(pin),GPIO_PIN_SET);
    state[pin_num(pin)] = true;
}
void Output::off(PIN pin){
    HAL_GPIO_WritePin(_port,static_cast<uint16_t>(pin),GPIO_PIN_RESET);
    state[pin_num(pin)] = false;
}
void Output::set(PIN pin,bool _state){
    if(_state){
        on(pin);
    }else{
        off(pin);
    }
}
void Output::toggle(PIN pin){
    if(state[pin_num(pin)]){
        off(pin);
    }else{
        on(pin);
    }
}
bool Output::read(PIN pin){
    return false;
}