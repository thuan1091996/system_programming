#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "utils.h"
#include "lcd.h"

// Defines
#define DEBUG_LOG 					1
// Macros
#define TASK_PRIO_UPDATE_TIME 		tskIDLE_PRIORITY + 1//highest
#define TASK_PRIO_UPDATE_DISPLAY 	tskIDLE_PRIORITY 
#define TASK_PRIO_ALARM 			tskIDLE_PRIORITY

// Custom typedef
typedef struct time
{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
}time_t;

// Variables
time_t cur_time;
time_t alarm_time;
bool isAlarmRunning = false;

//
SemaphoreHandle_t xTimeUpdateSemaphore;


//Function prototypes 
void vTimeDisplayLCD(time_t time);
void vTaskUpdateTime(void *argument);
void vTaskUpdateDisplay(void *argument);
void vTaskAlarm(void *argument);
void vTIM2_Init(void) ;

int main(void) 
{
	// System initializations
	vUSART2_Init();
	vSWs_Init();
	//lcd_init();
	vTIM2_Init();

	// System services
    xTimeUpdateSemaphore = xSemaphoreCreateBinary();
	if(xTimeUpdateSemaphore == NULL) { while(1); }

	// System tasks
	if(xTaskCreate( vTaskUpdateTime, "Task_TimeUpdate", configMINIMAL_STACK_SIZE, (void*const)"Task_TimeUpdate", TASK_PRIO_UPDATE_TIME, NULL) != pdPASS){ while(1); }
	if(xTaskCreate( vTaskUpdateDisplay, "Task_DisplayUpdate", configMINIMAL_STACK_SIZE, (void*const)"Task_DisplayUpdate", TASK_PRIO_UPDATE_DISPLAY, NULL) != pdPASS){ while(1); }
	if(xTaskCreate( vTaskAlarm, "Task_Alarm", configMINIMAL_STACK_SIZE, (void*const)"Task_Alarm", TASK_PRIO_ALARM, NULL) != pdPASS){ while(1); }


	vTaskStartScheduler(); // This should never return.

	// Will only get here if there was insufficient memory to create
	// the idle task.
	while(1);  
}


// ============================================================================


/**
  * @brief  This function handles External line 5-9 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	// if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	// {
	// 	static TickType_t last_tick=0; 
	// 	TickType_t cur_tick= xTaskGetTickCountFromISR(); 
	// 	BaseType_t xHigherPriorityTaskWoken = pdTRUE; 
	// 	uint32_t io_pin = START_STOP_BUTTON;
	// 	if(cur_tick >= last_tick + DB_TIMING) 
	// 	{ 
	// 		xQueueSendFromISR(button_queue, &io_pin, &xHigherPriorityTaskWoken); 
	// 		last_tick = cur_tick; 
	// 	} 
    // /* Clear the EXTI line 0 pending bit */
    // EXTI_ClearITPendingBit(EXTI_Line8);
  	// }
}

// ============================================================================


/**
  * @brief  This function handles External line 10-15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	// if(EXTI_GetITStatus(EXTI_Line15) != RESET)
	// {
	// 		static TickType_t last_tick=0; 
	// 			TickType_t cur_tick= xTaskGetTickCountFromISR(); 
	// 			BaseType_t xHigherPriorityTaskWoken = pdTRUE; 
	// 			uint32_t io_pin = RESET_BUTTON;
	// 			if(cur_tick >= last_tick + DB_TIMING) 
	// 			{ 
	// 				xQueueSendFromISR(button_queue, &io_pin, &xHigherPriorityTaskWoken); 
	// 				last_tick = cur_tick; 
	// 			} 
	// 	/* Clear the EXTI line 0 pending bit */
	// 	EXTI_ClearITPendingBit(EXTI_Line15);
	// }
}

void vTimeDisplayLCD(time_t time)
{
	while(1)
	{

	

		
	}

}

/* Update current time based on tick counts*/
void vTaskUpdateTime(void *argument)
{
	while(1)
	{
		cur_time.sec++;
		if (cur_time.sec == 60) // One minute
		{
			cur_time.sec = 0; 
			cur_time.min++; 
			if (cur_time.min == 60) // One hour
			{
				cur_time.min = 0;
				cur_time.hour = (cur_time.hour + 1) % 24; 
			}
		}
		#if DEBUG_LOG
		printf("Current time: %02d: %02d: %02d \r\n", cur_time.hour, cur_time.min, cur_time.sec);
		#endif /* DEBUG_LOG */
		xSemaphoreTake( xTimeUpdateSemaphore, portMAX_DELAY );

	}
}

/* Display alarm and clock */
void vTaskUpdateDisplay(void *argument)
{
	// Take Mutex lock
	// Display alarm and clock
	// Release Mutex lock
	while(1)
	{
		vTaskDelay(1000 / portTICK_RATE_MS); 
		
	}
}

void vTaskAlarm(void *argument)
{
	while(1)
	{
		vTaskDelay(1000 / portTICK_RATE_MS); 
	}
}

void vApplicationTickHook( void )
{
	static uint32_t counter_tick=0;
	if(counter_tick == 1000)
	{
		cur_time.sec++;
		counter_tick=0;
	
	}
}

void vTIM2_Init(void) 
{ 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
	/* TIM2 clock enable */ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	/* Compute the prescaler value */ 

	/* Time base configuration */ 
	TIM_TimeBaseStructure.TIM_Period = 4294967295; 
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_Init(&NVIC_InitStructure);   
	
	  /* TIM2 counter enable */
  	TIM_Cmd(TIM2, ENABLE);
} 

void TIM2_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
	  {
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
			TIM_SetCounter(TIM2, 0);
			
			BaseType_t xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE;
			/* 'Give' the semaphore to unblock the task. */
			xSemaphoreGiveFromISR( xTimeUpdateSemaphore, &xHigherPriorityTaskWoken );
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	  }
	
}