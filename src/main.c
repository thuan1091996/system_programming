/******************************************************************************
* Includes
*******************************************************************************/
#include <stdio.h>
#include <string.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

// STM32F4 includes
#include "stm32f4xx.h"
#include "utils.h"
#include "lcd.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define BUTT_QUEUE_LEN 				4 /* in bytes */
#define BUTT_QUEUE_SIZE 			100
#define DB_TIMING 					(500 / portTICK_PERIOD_MS) /* Button debouncing 500 ms */
#define START_STOP_BUTTON 			0x100
#define RESET_BUTTON 				0x8000
#define DEBUG_LOG 					0
#define STOP_WATCH_XPOS 			4
#define STOP_WATCH_YPOS 			1

#define TESTING						(1)


/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
xTaskHandle hTask1;
xTaskHandle hTask2;
xTaskHandle hPrintTask;

#if TESTING
volatile uint32_t auto_reload = 1000;
volatile uint32_t compare_value = 500;
volatile uint32_t g_test = 1;
#endif /* End of TESTING */

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/
// portTASK_FUNCTION_PROTO(vTask1, pvParameters);
// portTASK_FUNCTION_PROTO(vTask2, pvParameters);
// portTASK_FUNCTION_PROTO(vPrintTask, pvParameters);
/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/
void bsp_init(void);
void led_init(void);
void button_init(void);
void thermo_init(void);

void tim4_init(void);

/******************************************************************************
* Function Definitions
*******************************************************************************/





typedef enum timer_state
{
	TIMER_STATE_STOP = 0,
	TIMER_STATE_START
} timer_state_t;

// *** Declare a queue HERE
static QueueHandle_t button_queue = NULL;
static SemaphoreHandle_t lcd_mutex = NULL;
// ============================================================================
int main(void)
{
	vUSART2_Init();			// Initialise serial port
	bsp_init();				// Initialize the LCD, LEDs and buttons

	#if 0
	//Run time display
	uint8_t run_time_pos = sizeof("Run time:");
	lcd_move(0, 0);
	lcd_print("Run time:");
	lcd_move(run_time_pos, 0);
	lcd_print("000000");

	//Stopwatch display
	lcd_move(STOP_WATCH_XPOS, STOP_WATCH_YPOS);
	lcd_print("000000000");

	vTIM2_Init();

	// Button queue to handle button pressed events to communicate with timer task
	button_queue = xQueueCreate(BUTT_QUEUE_LEN, sizeof(uint32_t));
	assert_param(button_queue != NULL);

	// LCD mutex to guarantee only 1 task using the LCD at a time
	lcd_mutex = xSemaphoreCreateMutex();
	assert_param(lcd_mutex != NULL);

	// Welcome message
	printf("\r\n ====================Stop watch demo====================\r\n");

	printf("\r\n =======================================================\r\n");

	// Tasks get started here.
	// Arguments to xTaskCreate are:
	// 1- The function to execute as a task (make sure it never exits!)
	// 2- The task name (keep it short)
	// 3- The stack size for the new task (configMINIMAL_STACK_SIZE == 130)
	// 4- Parameter for the task (we won't be using this, so set it to NULL)
	// 5- The task priority; higher numbers are higher priority, and the idle task has priority tskIDLE_PRIORITY
	// 6- A pointer to an xTaskHandle variable where the TCB will be stored
	// xTaskCreate(vTask1, "TIMER", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &hTask1);
	// xTaskCreate(vTask2, "SYSTEMTIME", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &hTask2);
	// xTaskCreate(vPrintTask, "PRINT", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &hPrintTask);

	vTaskStartScheduler(); // This should never return.

	// Will only get here if there was insufficient memory to create
	// the idle task.
	while (1)
	{}
	#else
	while(1)
	{
		if(g_test == 1)
		{
			TIM_SetCompare4(TIM4, compare_value);
			g_test = 0;
		}
	}
	#endif /* End of 0 */


}

/**
 * @brief  This function handles External line 5-9 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		static TickType_t last_tick = 0;
		TickType_t cur_tick = xTaskGetTickCountFromISR();
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		uint32_t io_pin = START_STOP_BUTTON;
		if (cur_tick >= last_tick + DB_TIMING)
		{
			xQueueSendFromISR(button_queue, &io_pin, &xHigherPriorityTaskWoken);
			last_tick = cur_tick;
		}
		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
}

// ============================================================================

void bsp_init(void)
{
	// Initialize LCD
	lcd_init();
	led_init();
	button_init();
	thermo_init();
}

void led_init(void)
{
	// Init PD8, PD9 as output
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 

	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_8 | GPIO_Pin_9); 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOD, &GPIO_InitStructure); 

	GPIO_WriteBit(GPIOD, (GPIO_Pin_8| GPIO_Pin_9), Bit_RESET);	//Turn off PD8, PD9
}

void thermo_init(void)
{

    /* GPIOD clock enable */
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );

	/* GPIOD Configuration:  PD15 as TIM4 CH4  */
	GPIO_InitTypeDef GPIO_InitStructure; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init( GPIOB, &GPIO_InitStructure ); 

    /* Connect TIM4 pins to AF2 */  
    GPIO_PinAFConfig( GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

	// Initialize the timer4
	tim4_init();

}

void tim4_init(void)
{
	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* Time base configuration */
	#if TESTING
	TIM_TimeBaseStructure.TIM_Period = auto_reload; // TODO: Auto reload value
	#else
	TIM_TimeBaseStructure.TIM_Period = 4294967295; // TODO: Auto reload value
	#endif /* End of TESTING */
	
	

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM4, ( (16000/2) - 1), TIM_PSCReloadMode_Immediate); //Timer clock = 16Mhz -> 1 tick = 1ms

	TIM_OCInitTypeDef  TIM_OCInitStructure;
	 /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // output is active low when the counter value is less than the OC value
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	#if TESTING
    TIM_OCInitStructure.TIM_Pulse = compare_value; // TODO: Compare value
	#else
    TIM_OCInitStructure.TIM_Pulse = 0; // TODO: Compare value
	#endif /* End of TESTING */
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_Low;	
	TIM_OC1Init( TIM4, &TIM_OCInitStructure );

    TIM_OC1PreloadConfig( TIM4, TIM_OCPreload_Enable );
    TIM_ARRPreloadConfig( TIM4, ENABLE );

    /* TIM4 enable counter */
    TIM_Cmd( TIM4, ENABLE );
}

void button_init(void)
{
	// Initialize PE8, PE9 as input 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_8 | GPIO_Pin_9);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Initialize EXTI
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	EXTI_InitStructure.EXTI_Line = (EXTI_Line8 | EXTI_Line9);
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource8);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource9);

	// Clear pending bits
	EXTI_ClearITPendingBit(EXTI_Line8);
	EXTI_ClearITPendingBit(EXTI_Line9);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}
