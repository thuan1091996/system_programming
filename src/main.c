#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "utils.h"
#include "lcd.h"

xTaskHandle hTask1;
xTaskHandle hTask2;
xTaskHandle hPrintTask;

portTASK_FUNCTION_PROTO(vTask1, pvParameters);
portTASK_FUNCTION_PROTO(vTask2, pvParameters);
portTASK_FUNCTION_PROTO(vPrintTask, pvParameters);

// *** Declare a queue HERE
static QueueHandle_t button_queue = NULL; 
static SemaphoreHandle_t lcd_mutex = NULL;
	uint16_t data_adc=0;

// ============================================================================
int main(void) {

	// Initialise all of the STM32F4DISCOVERY hardware (including the serial port)
	vUSART2_Init();
	vADC1_Init();
	  /* Start ADC Software Conversion */ 
	// *** Initialise the queue HERE
	#if 0
	// Button queue to handle button pressed events
	button_queue = xQueueCreate(BUTT_QUEUE_LEN, sizeof(uint32_t)); 
	assert_param(button_queue != NULL); 
	lcd_mutex = xSemaphoreCreateMutex();
	assert_param(lcd_mutex != NULL); 
	#endif
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

	while(1) 
	{
		ADC_SoftwareStartConv(ADC1);
		data_adc = ADC_GetConversionValue(ADC1);
	}	
}

// This task should run every 700ms and send a message to the print task.
// ---------------------------------------------------------------------------- 
portTASK_FUNCTION(vTask2, pvParameters) {

	while(1) 
	{
		vTaskDelay(5000/portTICK_RATE_MS);
		#if 0
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
		#endif
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
	#if 0
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
	#endif
}

// ============================================================================