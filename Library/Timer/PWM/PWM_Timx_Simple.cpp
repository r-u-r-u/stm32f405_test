#include "../Library/Timer/PWM/PWM_Timx_Simple.h"

PWM_Timx_Simple::PWM_Timx_Simple(TIM_TypeDef *timx,uint32_t prescaler,uint32_t period){

    TIM_MasterConfigTypeDef sMasterConfig;

    htim.Instance = timx;
    htim.Init.Prescaler = prescaler;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = period;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_TIM_MspPostInit(&htim);
}
PWM_Timx_Simple::~PWM_Timx_Simple(){

}
void PWM_Timx_Simple::start(uint32_t channel){
    HAL_TIM_PWM_Start(&htim, channel);
}
void PWM_Timx_Simple::stop(uint32_t channel){
    HAL_TIM_PWM_Stop(&htim, channel);
}
void PWM_Timx_Simple::set_channel(uint32_t channel){
    
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, channel) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_TIM_MspPostInit(&htim);
}
void PWM_Timx_Simple::output(uint32_t channel,float duty){
        
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint32_t)400*duty;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, channel) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}
