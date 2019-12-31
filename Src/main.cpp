
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "../Library/Timer/QEI/QEI_Timx.h"
#include "../Library/Timer/PWM/PWM_Timx_Simple.h"
#include "../Library/GPIO/Output.h"
#include "../Library/GPIO/Input.h"
#include "../Library/UART/UART_Simple.h"
#include "../Library/I2C/I2C2.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

__weak void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
I2C_HandleTypeDef hi2c2;
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  GPIO *InA,*InB;
  GPIO *OutB;
  InA = new Input(GPIOA,GPIO_PIN_11);
  InB = new Input(GPIOB,GPIO_PIN_12);
  OutB = new Output(GPIOB,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
  // QEI *qei_tim2;
  // qei_tim2=new QEI_Timx(TIM2,0,65535);
  // qei_tim2->start();
  QEI *qei_tim3;
  // qei_tim3=new QEI_Timx(TIM3,0,65535);
  qei_tim3->start();
	uint8_t read=0;
	uint8_t address=0x75;//WHO_AM_I
	uint8_t config=0x12;
	uint8_t set[]={0x1A,0x12};
	uint8_t dat=0x00;

  // PWM *pwm_tim5;
  // pwm_tim5 = new PWM_Timx_Simple(TIM5,0,800);
  // pwm_tim5->set_channel(TIM_CHANNEL_3);
  // pwm_tim5->set_channel(TIM_CHANNEL_4);
  PWM *pwm_tim8;
  pwm_tim8 = new PWM_Timx_Simple(TIM8,0,800);
  pwm_tim8->set_channel(TIM_CHANNEL_3);
  pwm_tim8->set_channel(TIM_CHANNEL_4);
  // PWM *pwm_buz;
  // pwm_buz = new PWM_Timx_Simple(TIM1,0,4000);
  // pwm_buz->set_channel(TIM_CHANNEL_2);

  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */
  
    /**I2C2 GPIO Configuration    
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA 
    */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  
  HAL_I2C_Mem_Read(&hi2c2,0xD0/*device ID*/,0x75/*WHO_AM_I*/,I2C_MEMADD_SIZE_8BIT/*Memory Address Size*/,&read/*Data Buffer*/,1,10);
	if( read == 0x68 ){
//			HAL_GPIO_WritePin(GPIO_PIN_1,GPIO_PIN_1,GPIO_PIN_SET);
//if 'read' have correct data
		//wakeup sensor
		HAL_I2C_Mem_Write(&hi2c2,0xD0,0x6B,I2C_MEMADD_SIZE_8BIT,&dat,1,10);
		HAL_I2C_Mem_Read(&hi2c2 ,0xD0,0x6B,I2C_MEMADD_SIZE_8BIT,&read,1,10);
		//configure sensor
		HAL_I2C_Mem_Write(&hi2c2,0xD0,0x1a,I2C_MEMADD_SIZE_8BIT,&config,1,10);
		HAL_I2C_Mem_Read(&hi2c2,0xD0,0x1a,I2C_MEMADD_SIZE_8BIT,&read,1,10);
		if(read==0x12){
//if 'read' have correct data
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		}else{/*read == 0x12*/
//if 'read' have wrong data
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		}
	}else{/*read == 0x68*/
//if 'read' have wrong data
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	}
  // I2C *i2c_gyro;
  // i2c_gyro = new I2C_2();
  // uint8_t data=0,read=0;
  // i2c_gyro->read(0xD0,0x75,&read);
  // if(read == 0x68){
  //   data = 0x00;
  //   i2c_gyro->write(0xD0,0x6B,&data);
  //   i2c_gyro->read(0xD0,0x6B,&read);
  //   data = 0x12;
  //   i2c_gyro->write(0xD0,0x1a,&data);
  //   i2c_gyro->read(0xD0,0x1a,&read);
  //   if(read == 0x12){
  //     OutB->on(GPIO::PIN::PIN_5);
  //   }else{
  //     OutB->off(GPIO::PIN::PIN_5);
  //   }
  // }else{
  //   OutB->off(GPIO::PIN::PIN_5);
  // }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    OutB->set(GPIO::PIN::PIN_15,InA->read(GPIO::PIN::PIN_11));
    OutB->set(GPIO::PIN::PIN_14,InB->read(GPIO::PIN::PIN_12));
    // OutB->set(GPIO::PIN::PIN_1,(qei_tim2->read()&0x01)&&0x01);
    // OutB->set(GPIO::PIN::PIN_2,(qei_tim2->read()&0x10)&&0x10);
    // OutB->set(GPIO::PIN::PIN_3,(qei_tim3->read()&0x01)&&0x01);
    // OutB->set(GPIO::PIN::PIN_4,(qei_tim3->read()&0x10)&&0x10);
    if(!InA->read(GPIO::PIN::PIN_11)){
      // pwm_buz->output_n(TIM_CHANNEL_2,0.5);
      // pwm_tim5->output(TIM_CHANNEL_3,0.0);
      pwm_tim8->output(TIM_CHANNEL_3,0.0);
      // pwm_tim5->output(TIM_CHANNEL_4,0.5);
      pwm_tim8->output(TIM_CHANNEL_4,0.5);
    }else if(!InB->read(GPIO::PIN::PIN_12)){
      // pwm_buz->output_n(TIM_CHANNEL_2,0.5);
      // pwm_tim5->output(TIM_CHANNEL_3,0.5);
      pwm_tim8->output(TIM_CHANNEL_3,0.5);
      // pwm_tim5->output(TIM_CHANNEL_4,0.0);
      pwm_tim8->output(TIM_CHANNEL_4,0.0);
    }else{
      // pwm_buz->stop_n(TIM_CHANNEL_2);
      // pwm_tim5->output(TIM_CHANNEL_3,0.0);
      pwm_tim8->output(TIM_CHANNEL_3,0.0);
      // pwm_tim5->output(TIM_CHANNEL_4,0.0);
      pwm_tim8->output(TIM_CHANNEL_4,0.0);
      // pwm_tim5->stop(TIM_CHANNEL_3);
      pwm_tim8->stop(TIM_CHANNEL_3);
      // pwm_tim5->stop(TIM_CHANNEL_4);
      // pwm_tim8->stop(TIM_CHANNEL_4);
    }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
