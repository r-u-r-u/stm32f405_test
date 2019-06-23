#include "../Library/GPIO/Input.h"

Input::Input(GPIO_TypeDef *port,uint16_t pin){
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
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(_port, &GPIO_InitStruct);
}
Input::~Input(){
}
void Input::set(PIN pin,bool state){
}
void Input::on(PIN pin){
}
void Input::off(PIN pin){
}
void Input::toggle(PIN pin){
}
bool Input::read(PIN pin){
    return (bool)HAL_GPIO_ReadPin(_port,static_cast<uint16_t>(pin));
}
