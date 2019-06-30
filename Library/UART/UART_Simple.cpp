#include "../Library/UART/UART_Simple.h"

UART_Simple::UART_Simple(USART_TypeDef *uartx){
  huart.Instance = uartx;
  huart.Init.BaudRate = 115200;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits = UART_STOPBITS_1;
  huart.Init.Parity = UART_PARITY_NONE;
  huart.Init.Mode = UART_MODE_TX_RX;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

UART_Simple::~UART_Simple(){
}

void UART_Simple::_putc(uint8_t c){
  HAL_UART_Transmit(&huart,&c,1,1);
}

uint8_t UART_Simple::_getc(){
  return 0;
}
void UART_Simple::_puts(uint8_t *c){
  HAL_UART_Transmit(&huart,c,sizeof(c) / sizeof(c[0]),10);
}
void UART_Simple::_gets(uint8_t *c){

}