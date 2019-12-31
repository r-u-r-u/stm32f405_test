
#include"../Library/I2C/I2C1.h"

I2C_1::I2C_1(){
  hi2c.Instance = I2C1;
  hi2c.Init.ClockSpeed = 100000;
  hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c.Init.OwnAddress1 = 0;
  hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c.Init.OwnAddress2 = 0;
  hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}
I2C_1::~I2C_1(){}