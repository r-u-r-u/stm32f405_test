#pragma once
#include"../I2C/I2C.h"

class Gyro{
public:
    virtual ~Gyro(){}
    virtual float getAngleVelocity()=0;
};