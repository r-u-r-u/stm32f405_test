#include "../Library/Timer/QEI/QEI_Timx.h"

QEI_Timx::QEI_Timx(TIM_TypeDef *timx,uint32_t prescaler,uint32_t period){

    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    
    htim.Instance = timx;
    htim.Init.Prescaler = prescaler;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = period;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;

    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}
QEI_Timx::~QEI_Timx(){
    
}

void QEI_Timx::start(){
    HAL_TIM_Encoder_Start(&htim,TIM_CHANNEL_ALL);
}

void QEI_Timx::stop(){
    HAL_TIM_Encoder_Stop(&htim,TIM_CHANNEL_ALL);
}

int QEI_Timx::read(){
    return htim.Instance->CNT;
}

void QEI_Timx::set(int set_value){
    htim.Instance->CNT = set_value;
}