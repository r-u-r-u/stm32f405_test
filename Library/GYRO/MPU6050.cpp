#include"MPU6050.h"

MPU6050::MPU6050(I2C *i2c):_i2c(i2c){
}
MPU6050::~MPU6050()
{
}
float MPU6050::getAngleVelocity(){
	union Gyro_Data{
		uint8_t data_splited[2];
		int16_t data_composed;
	} dat;
	_i2c->read(0xD0,0x47,&dat.data_splited[1]);
	_i2c->read(0xD0,0x48,&dat.data_splited[0]);
	return (float)(dat.data_composed*1.0/16400*3.1415926/180);
}
bool MPU6050::WhoAmI(){
    bool ret=false;
    uint8_t read=0,data=0;
    _i2c->read(0xD0,0x75,&read);
    if(read == 0x68){
        data = 0x00;
        _i2c->write(0xD0,0x6B,&data);
        data = 0x00;
        _i2c->write(0xD0,0x1a,&data);
        data = 0x10;
        _i2c->write(0xD0,0x1b,&data);
        _i2c->read(0xD0,0x1b,&read);
        
        if(read == data) ret = true;
    }
    return ret;
}
