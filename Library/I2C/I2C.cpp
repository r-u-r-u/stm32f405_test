
#include"../Library/I2C/I2C.h"

I2C::I2C():hi2c(){}
I2C::~I2C(){}
void I2C::read(uint8_t ID,uint8_t address,uint8_t *data){
    HAL_I2C_Mem_Read(&hi2c ,ID , address,I2C_MEMADD_SIZE_8BIT,data,1,10);
}
void I2C::write(uint8_t ID,uint8_t address,uint8_t *data){
    HAL_I2C_Mem_Write(&hi2c ,ID , address,I2C_MEMADD_SIZE_8BIT,data,1,10);
}