#pragma once
#include "stm32f4xx_hal.h"

class I2C{
protected:
    I2C_HandleTypeDef hi2c;
public:
    I2C();
    ~I2C();
    void read(uint8_t ID,uint8_t address,uint8_t *data);
    void write(uint8_t ID,uint8_t address,uint8_t *data);
};