#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "utils.h"


#define SWITCHES_PRIORITY          7	//Priority should be "lower" than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (which =5)
#define TIM2_HW_PRIORITY			     7

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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;			// Tx Pin
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;			// Rx Pin	
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
    //NVIC_SetPriority(USART2_IRQn, 11);
}

void vLEDs_Init(void)
{
	// Init PD8 -> PD15 outputs
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 
	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = 0xFF00; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOD, &GPIO_InitStructure); 

	//Turn off LEDs
	GPIO_ResetBits(GPIOD, 0xFF00); 
 
} 

void vSWs_Init(void)
{
    // Init PE8 -> PE11 inputs
    EXTI_InitTypeDef   EXTI_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStructure.GPIO_Pin = 0xF00;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* Configure EXTI Line8 -> 11 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource8);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource9);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource10);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource11);
    
    EXTI_InitStructure.EXTI_Line = 0xF00;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    /* Enable and set EXTI Line8 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SWITCHES_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable and set EXTI Line8 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SWITCHES_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void vTIM2_Init(void) 
{ 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
	/* TIM2 clock enable */ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	/* Compute the prescaler value */ 

	/* Time base configuration */ 
	TIM_TimeBaseStructure.TIM_Period = 0xffffffff; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 

	/* Prescaler configuration */ 
	TIM_PrescalerConfig(TIM2, ( (16000/2) - 1), TIM_PSCReloadMode_Immediate); //Timer clock = 16Mhz -> 1 tick = 1 ms 
	TIM_SetCompare2(TIM2, 10000); //Interrupt every 1s //TODO: Fixme
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

	/* Enable the TIM2 Trigger and commutation interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM2_HW_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_Init(&NVIC_InitStructure);   
	
	/* TIM2 counter enable */
  	TIM_Cmd(TIM2, ENABLE);
} 
