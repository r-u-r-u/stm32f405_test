#include "../GPIO/GPIO.h"

GPIO::GPIO():state(){
}
GPIO::~GPIO(){
}
void GPIO::set(PIN pin,bool state){}
void GPIO::on(PIN pin){}
void GPIO::off(PIN pin){}
void GPIO::toggle(PIN pin){}
bool GPIO::read(PIN pin){}
uint8_t GPIO::pin_num(PIN pin){
    for(uint8_t i = 0;i<15;i++){
        if(static_cast<uint16_t>(pin) == (2<<i) ){
            return i;
        }
    }
}