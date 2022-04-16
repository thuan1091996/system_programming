#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "utils.h"
#include "lcd.h"

#define	BUTT_QUEUE_LEN			4	/* in bytes */ 
#define BUTT_QUEUE_SIZE			100 
#define DB_TIMING						(1000 / portTICK_PERIOD_MS) /* Button debouncing 1000 ms */ 
#define START_STOP_BUTTON		0x100
#define RESET_BUTTON				0x8000						 
#define DEBUG_LOG						1



xTaskHandle hTask1;
xTaskHandle hTask2;
xTaskHandle hPrintTask;

portTASK_FUNCTION_PROTO(vTask1, pvParameters);
portTASK_FUNCTION_PROTO(vTask2, pvParameters);
portTASK_FUNCTION_PROTO(vPrintTask, pvParameters);

typedef enum timer_state
{
	TIMER_STATE_STOP =0,
	TIMER_STATE_START
}timer_state_t;

// *** Declare a queue HERE
static QueueHandle_t button_queue = NULL; 
static SemaphoreHandle_t lcd_mutex = NULL;
// ============================================================================
int main(void) {

	// Initialise all of the STM32F4DISCOVERY hardware (including the serial port)
	vUSART2_Init();
	lcd_init();
	vTIM2_Init();
	vButton_Init();
	// *** Initialise the queue HERE
	// Button queue to handle button pressed events
	button_queue = xQueueCreate(BUTT_QUEUE_LEN, sizeof(uint32_t)); 
	assert_param(button_queue != NULL); 
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
	xTaskCreate(vTask1, "TIMER", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, &hTask1);
	xTaskCreate(vTask2, "TASK2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, &hTask2);
	xTaskCreate(vPrintTask, "PRINT", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, &hPrintTask);
	
	

	vTaskStartScheduler(); // This should never return.

	// Will only get here if there was insufficient memory to create
	// the idle task.
	while(1);  
}

// Timer task
// ---------------------------------------------------------------------------- 
portTASK_FUNCTION(vTask1, pvParameters) {
	timer_state_t cur_state = TIMER_STATE_STOP;
	uint32_t data_recv=0;
	uint32_t timer_value=0;
	char timer_value_buffer[16]={0};
	printf("Timer is initialized \r\n");
	while(1) {
		if(cur_state == TIMER_STATE_STOP)
		{
				
				xQueueReceive(button_queue, &data_recv, portMAX_DELAY);
				if(data_recv == START_STOP_BUTTON)
				{
						//start timer						

						#if DEBUG_LOG
						printf("Timer was started \r\n");
						#endif /*End of DEBUG_LOG */
						TIM_Cmd(TIM2, ENABLE);
						cur_state = TIMER_STATE_START;
				}
				else if(data_recv == RESET_BUTTON)
				{
						#if DEBUG_LOG
						printf("Timer was reset \r\n");
						#endif /*End of DEBUG_LOG */
						TIM_SetCounter(TIM2, 0);
				}
		}
		else //state start
		{
				if (xQueueReceive(button_queue, &data_recv, 0) == pdTRUE)
				{
						if(data_recv == START_STOP_BUTTON)
						{
							#if DEBUG_LOG
							printf("Timer was stopped \r\n");
							#endif /*End of DEBUG_LOG */
							TIM_Cmd(TIM2, DISABLE);
							cur_state = TIMER_STATE_STOP;
						}
				}
				else
				{
						//No button event, wait until pressed stop
						//Delay 100ms, then read timer and send to display task
						vTaskDelay(100/portTICK_RATE_MS);
						timer_value = TIM_GetCounter(TIM2);
						if(xSemaphoreTake(lcd_mutex, (1000/portTICK_PERIOD_MS)) == pdPASS)
						{
								memset((void*)timer_value_buffer, 0, 16);
								sprintf(timer_value_buffer, "%09d", timer_value);
								lcd_move(2, 1); 
								lcd_print(timer_value_buffer);
								xSemaphoreGive(lcd_mutex);							
						}
						#if DEBUG_LOG
						printf("Timer value %d \r\n", timer_value);
						printf("System tick %d \r\n \r\n", xTaskGetTickCount());
						#endif /*End of DEBUG_LOG */
				}
		}
	}
}

// This task should run every 700ms and send a message to the print task.
// ---------------------------------------------------------------------------- 
portTASK_FUNCTION(vTask2, pvParameters) {
	TickType_t total_run_time = 0;
	lcd_move(0,0);
	lcd_print("Run time:");
	char run_time_buffer[16]={0};
	while(1) {
		total_run_time = (xTaskGetTickCount()/portTICK_RATE_MS)/1000; //run time in secs
		memset((void*)run_time_buffer, 0, 16);
		sprintf(run_time_buffer, "%06d", total_run_time);
		if(xSemaphoreTake(lcd_mutex, (1000/portTICK_PERIOD_MS)) == pdPASS)
		{
			lcd_move(sizeof("Run time:"), 0); 
			lcd_print(run_time_buffer);
			xSemaphoreGive(lcd_mutex);
		}
		#if DEBUG_LOG
		printf("Task 2 Run time: %d (s)\r\n", total_run_time);
		#endif /*End of DEBUG_LOG */
		vTaskDelay(500/portTICK_RATE_MS);
	}
}

// This task should run whenever a message is waiting in the queue.
// ---------------------------------------------------------------------------- 
portTASK_FUNCTION(vPrintTask, pvParameters) {
	while(1) {
			//printf("Print task is running ..\r\n");
			vTaskDelay(5000/portTICK_RATE_MS);

			
		// *** Receive a message from the queue and print it HERE
		
	}
}


// ============================================================================


/**
  * @brief  This function handles External line 5-9 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
		if(EXTI_GetITStatus(EXTI_Line8) != RESET)
		{
			static TickType_t last_tick=0; 
			TickType_t cur_tick= xTaskGetTickCountFromISR(); 
			BaseType_t xHigherPriorityTaskWoken = pdTRUE; 
			uint32_t io_pin = START_STOP_BUTTON;
			if(cur_tick >= last_tick + DB_TIMING) 
			{ 
				xQueueSendFromISR(button_queue, &io_pin, &xHigherPriorityTaskWoken); 
				last_tick = cur_tick; 
			} 
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line8);
  }
}

// ============================================================================


/**
  * @brief  This function handles External line 10-15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line15) != RESET)
  {
    	static TickType_t last_tick=0; 
			TickType_t cur_tick= xTaskGetTickCountFromISR(); 
			BaseType_t xHigherPriorityTaskWoken = pdTRUE; 
			uint32_t io_pin = RESET_BUTTON;
			if(cur_tick >= last_tick + DB_TIMING) 
			{ 
				xQueueSendFromISR(button_queue, &io_pin, &xHigherPriorityTaskWoken); 
				last_tick = cur_tick; 
			} 
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line15);
  }
}