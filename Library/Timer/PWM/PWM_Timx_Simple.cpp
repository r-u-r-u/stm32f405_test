#include "../Library/Timer/PWM/PWM_Timx_Simple.h"

PWM_Timx_Simple::PWM_Timx_Simple(TIM_TypeDef *timx,uint32_t prescaler,uint32_t period):htim(){
    _period = period;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim.Instance = timx;
    htim.Init.Prescaler = prescaler;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = period-1;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&htim) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    if(timx == TIM8){
        TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
        sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
        sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
        sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
        sBreakDeadTimeConfig.DeadTime = 0;
        sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
        sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
        sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
        if (HAL_TIMEx_ConfigBreakDeadTime(&htim, &sBreakDeadTimeConfig) != HAL_OK)
        {
        _Error_Handler(__FILE__, __LINE__);
        }
    }
    HAL_TIM_MspPostInit(&htim);
}
PWM_Timx_Simple::~PWM_Timx_Simple(){

}
void PWM_Timx_Simple::start(uint32_t channel){
    HAL_TIM_PWM_Start(&htim, channel);
}
void PWM_Timx_Simple::start_n(uint32_t channel){
    HAL_TIMEx_PWMN_Start(&htim, channel);
}
void PWM_Timx_Simple::stop(uint32_t channel){
    HAL_TIM_PWM_Stop(&htim, channel);
}
void PWM_Timx_Simple::stop_n(uint32_t channel){
    HAL_TIMEx_PWMN_Stop(&htim, channel);
}
void PWM_Timx_Simple::set_channel(uint32_t channel){
    
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, channel) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}
void PWM_Timx_Simple::gpio_init(){
}
void PWM_Timx_Simple::output(uint32_t channel,float duty){
        
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint32_t)(_period*duty);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, channel) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    start(channel);
}
void PWM_Timx_Simple::output_n(uint32_t channel,float duty){
        
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint32_t)(_period*duty);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, channel) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    start_n(channel);
}
