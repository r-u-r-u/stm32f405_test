#pragma once
#include"../I2C/I2C.h"
#include"GYRO.h"

class MPU6050:public Gyro{
public:
    MPU6050(I2C *i2c);
    virtual ~MPU6050();
    virtual float getAngleVelocity();
    bool WhoAmI();
protected:
    I2C *_i2c;
    float angle;
    float angle_velocity;
};