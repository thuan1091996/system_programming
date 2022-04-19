#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "utils.h"

#define START_STOP_BUTTON_PRIO 6 // Priority should be "lower" than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (which =5)
#define RESET_BUTTON_PRIO 7

// ============================================================================
void vUSART2_Init(void)
{
  USART_ClockInitTypeDef USART_ClockInitStruct;
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable needed clocks for uart.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // Make sure you use 'GPIO_PinSource2' and NOT 'GPIO_Pin_2'.  Using the
  // latter will not work!
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  // Setup Tx / Rx pins.
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // Tx Pin
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // Rx Pin
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Make sure syncro clock is turned off.
  USART_ClockStructInit(&USART_ClockInitStruct);
  USART_ClockInit(USART2, &USART_ClockInitStruct);

  // Use defaults (except baud rate).
  USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = 38400;
  USART_Init(USART2, &USART_InitStructure);
  USART_Cmd(USART2, ENABLE);
}

// ============================================================================
void vUSART2_RX_IRQ_Init(void)
{
  // Setup receive irq.
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  // Enable USART2 interrupt
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_SetPriority(USART2_IRQn, 11);
}

/* ADC1 Initialization to reading sensor value (RV2) which connected to RV3 - ADC1-IN12
 */
void vADC1_Init(void)
{
  //Enable clocks for ADC and ADC pin
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
  // Init GPIO for ADC1 Channel 12 (PC2) pin as analog input
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = 0x04;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // ADC Init
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;

  // ADC Common Init
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  // ADC Init
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  // ADC Channel Init
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_480Cycles);

  // ADC Enable 
  ADC_Cmd(ADC1, ENABLE);
}

uint16_t sADC1_GetData(void) //s for short
{

}

/* ADC2 Initialization to reading user setting threshold which connected to RV2 - ADC2-IN11
 */
void vADC2_Init(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
}

// ============================================================================

// ============================================================================
void vButton_Init(void)
{
  // Enable GPIOE clock
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  // PE8, PE15 as inputs
  GPIO_InitStructure.GPIO_Pin = 0x8100;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Configure EXTI Line8, 15 */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource8);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource15);

  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  /* Enable and set EXTI Line8 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = START_STOP_BUTTON_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable and set EXTI Line8 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RESET_BUTTON_PRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}