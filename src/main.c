#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "utils.h"
#include "lcd.h"

typedef enum {
	SENSOR_DATA=0,		//Send from sensor reading task
	SENSOR_SETTING		//Send from sensor setting task
}data_type_t;

typedef struct data_msg
{
	data_type_t type;
	uint16_t	data_value;
}data_msg_t;


xTaskHandle hSensorReadingTask;
xTaskHandle hSensorSettingTask;
xTaskHandle hControlTask;

portTASK_FUNCTION_PROTO(vSensorReading, pvParameters);
portTASK_FUNCTION_PROTO(vSensorSetting, pvParameters);
portTASK_FUNCTION_PROTO(vControl, pvParameters);

// *** Declare a queue HERE
static SemaphoreHandle_t xbinary_sem = NULL;



static QueueHandle_t button_queue = NULL; 
static SemaphoreHandle_t lcd_mutex = NULL;
uint16_t data_adc1=0, data_adc2=0;

// ============================================================================
int main(void) {

	// Initialise all of the STM32F4DISCOVERY hardware (including the serial port)
	vUSART2_Init();
	vLED_Init();
	vSW1_Init();
	vADC1_Init();
	vADC2_Init();


	xbinary_sem = xSemaphoreCreateBinary();
	if(xbinary_sem == NULL)
	{
		printf("Cant create binary semaphore \r\n ");
		while(1)	//Hang in here
		{

		};
	}

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
	xTaskCreate(vSensorSetting, "SENSOR_SETTING", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, &hSensorSettingTask);
	xTaskCreate(vSensorReading, "SENSOR_READING", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, &hSensorReadingTask);
	xTaskCreate(vControl, "CONTROL", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, &hControlTask);
	
	

	vTaskStartScheduler(); // This should never return.

	// Will only get here if there was insufficient memory to create
	// the idle task.
	while(1);  
}

// Sensor reading task, read sensor value (RV3) each 100ms and send data to control task
// ---------------------------------------------------------------------------- 
portTASK_FUNCTION(vSensorReading, pvParameters) {
	uint16_t sensor_data=0;
	data_msg_t sensor_data_msg;
	sensor_data_msg.type = SENSOR_DATA;
	sensor_data_msg.data_value = 0;
	while(1) 
	{
		sensor_data = sADC1_GetData();
		sensor_data_msg.data_value = sensor_data;
		//Sending to control task

		printf("Sensor reading %d \r\n", sensor_data);
		vTaskDelay(1000/portTICK_RATE_MS);
	}	
}

// Sensor setting task, unblock by SW1 then reading RV2 and sending data to control task
// ---------------------------------------------------------------------------- 
portTASK_FUNCTION(vSensorSetting, pvParameters) {
	uint16_t sensor_threashold=0;
	data_msg_t sensor_setting_msg;
	sensor_setting_msg.type = SENSOR_SETTING;
	sensor_setting_msg.data_value = 0;
	while(1) 
	{
		//Waiting for SW1 pressed
		xSemaphoreTake(xbinary_sem, portMAX_DELAY);
		sensor_threashold = sADC2_GetData();
		sensor_setting_msg.data_value = sensor_threashold;
		printf("\r\n ====================User settings====================\r\n"); 
 		printf("Sensor threshold setting %d \r\n", sensor_threashold);
		printf("\r\n =======================================================\r\n	"); 
		


		//Send data to control task
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
portTASK_FUNCTION(vControl, pvParameters) {
	while(1) {
			//printf("Print task is running ..\r\n");
			vTaskDelay(5000/portTICK_RATE_MS);

			
		// *** Receive a message from the queue and print it HERE
		
	}
}
// ============================================================================


// EXTI9_5_IRQHandler interrupt handler
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
		xSemaphoreGiveFromISR(xbinary_sem, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		EXTI_ClearITPendingBit(EXTI_Line8);
  }
}

// ============================================================================