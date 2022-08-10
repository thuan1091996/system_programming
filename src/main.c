#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f4xx.h"
#include "utils.h"
#include "lcd.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

// Defines
#define DEBUG_LOG 							0
#define DEBUG_EN								0

#define ALARM_LEDS_PIN					0xFF00						
#define SWITCH_UP_PIN						8
#define SWITCH_DOWN_PIN					9
#define SWITCH_SET_PIN					10
#define SWITCH_STOP_PIN					11
#define QUEUE_MAX_LEN						20
#define ALARM_SEC_START_ADDR		12
#define ALARM_MIN_START_ADDR		9
#define ALARM_HOUR_START_ADDR		6
#define TIMER_SEC_START_ADDR		12
#define TIMER_MIN_START_ADDR		7
#define TIMER_HOUR_START_ADDR		2
// Macros
#define TASK_PRIO_UPDATE_TIME 		tskIDLE_PRIORITY + 2//highest
#define TASK_PRIO_UPDATE_DISPLAY 	tskIDLE_PRIORITY 
#define TASK_PRIO_ALARM 					tskIDLE_PRIORITY + 1
#define TASK_PRIO_CONFIG 					tskIDLE_PRIORITY 

#define DEC(x,min_value,max_value)		(x = (x == min_value)? (max_value-1) : (x-1) )
#define INC(x,min_value,max_value)		(x = (x >= (max_value-1))? min_value : (x+1) )

// Custom typedef
typedef struct timer
{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
}time_t;

typedef enum mode
{
	CONFIG_MODE_DEFAULT=0,
	CONFIG_ALARM_SEC,
	CONFIG_ALARM_MIN,
	CONFIG_ALARM_HOUR,
	CONFIG_TIMER_SEC,
	CONFIG_TIMER_MIN,
	CONFIG_TIMER_HOUR,
	CONFIG_MAX_VALUE
}mode_t;

// Variables
time_t cur_time;
time_t alarm_time;
bool is_alarm_active = false;

char lcd_buf0[16]={0};
char lcd_buf1[16]={0};


SemaphoreHandle_t xTimeUpdateSemaphore = NULL;
SemaphoreHandle_t xLCDMutex = NULL; 
QueueHandle_t xSwitches_Queue = NULL;  

//Function prototypes 
void vTimeDisplayLCD();
void vAlarm_Off();
void vAlarm_On();

void vTaskUpdateTime(void *argument);
void vTaskUpdateDisplay(void *argument);
void vTaskAlarm(void *argument);
void vTaskConfig(void *argument);


int main(void) 
{
	// System initializations
	vUSART2_Init();
	lcd_init();

	vSWs_Init();
	vLEDs_Init();
	vTIM2_Init();

	// System services
  	xTimeUpdateSemaphore = xSemaphoreCreateBinary();
	if(xTimeUpdateSemaphore == NULL) { while(1); }
	
	xLCDMutex = xSemaphoreCreateMutex(); 
	if(xLCDMutex == NULL) { while(1); }

	xSwitches_Queue = xQueueCreate(QUEUE_MAX_LEN, sizeof(uint32_t));
	if(xSwitches_Queue == NULL) { while(1); }

	// System tasks
	if(xTaskCreate( vTaskUpdateTime, "Task_TimeUpdate", configMINIMAL_STACK_SIZE, (void*const)"Task_TimeUpdate", TASK_PRIO_UPDATE_TIME, NULL) != pdPASS){ while(1); }
	if(xTaskCreate( vTaskUpdateDisplay, "Task_DisplayUpdate", configMINIMAL_STACK_SIZE, (void*const)"Task_DisplayUpdate", TASK_PRIO_UPDATE_DISPLAY, NULL) != pdPASS){ while(1); }
	if(xTaskCreate( vTaskAlarm, "Task_Alarm", configMINIMAL_STACK_SIZE, (void*const)"Task_Alarm", TASK_PRIO_ALARM, NULL) != pdPASS){ while(1); }
	if(xTaskCreate( vTaskConfig, "Task_Config", configMINIMAL_STACK_SIZE, (void*const)"Task_Config", TASK_PRIO_CONFIG, NULL) != pdPASS){ while(1); }



	vTaskStartScheduler(); // This should never return.

	// Will only get here if there was insufficient memory to create
	// the idle task.
	while(1);  
}


void vAlarm_On()
{
	GPIO_SetBits(GPIOD, 0xFF00);
	vTaskDelay(1000 / portTICK_RATE_MS); 
}

void vAlarm_Off()
{
	GPIO_ResetBits(GPIOD, 0xFF00);
}


void vTaskAlarm(void *argument)
{
	bool alarm_running=false;
	while(1)
	{
		if( (alarm_running) && (is_alarm_active == false) )
		{
			//if stop pressed
			{
				#if DEBUG_LOG
				printf(" \r\n TURN OFF ACTIVE ALARM !!\r\n");
				#endif /* End of DEBUG_LOG */
				vAlarm_Off();
				alarm_running = false;
			}
		}
		else 
		{
			if ( (is_alarm_active) && (memcmp(&cur_time, &alarm_time, sizeof(time_t)) == 0) ) 
			{
				#if DEBUG_LOG
				printf(" \r\n TIME UP !!\r\n");
				#endif /* End of DEBUG_LOG */
				vAlarm_On();
				alarm_running = true;
			}
		}
		#if DEBUG_EN
		vTaskDelay(5000 / portTICK_RATE_MS); 
		#else
		vTaskDelay(100 / portTICK_RATE_MS); 
		#endif /* DEBUG_EN */
	}
}

void vTimeDisplayLCD()
{
	sprintf(lcd_buf0, "ALARM:%02d:%02d:%02d ", alarm_time.hour, alarm_time.min, alarm_time.sec);
	sprintf(lcd_buf1, "%02d : %02d : %02d", cur_time.hour, cur_time.min, cur_time.sec);
	#if DEBUG_LOG
	printf("\r\n");
	printf("%s", lcd_buf0);
	printf("\r\n");
	printf("%s", lcd_buf1);
	printf("\r\n");
	#endif // DEBUG_LOG
	lcd_move(0,0);
	lcd_print(lcd_buf0); 
	lcd_move(2,1); //Center align
	lcd_print(lcd_buf1); 
}

/* Display alarm and clock */
void vTaskUpdateDisplay(void *argument)
{
	while(1)
	{
		// Take Mutex lock
		if(xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdPASS) 
		{ 
			// Display alarm and clock
			vTimeDisplayLCD();
			// Release Mutex lock
			xSemaphoreGive(xLCDMutex); 
		}

		#if DEBUG_EN
		vTaskDelay(10000 / portTICK_RATE_MS); 
		#else
		vTaskDelay(250 / portTICK_RATE_MS); 
		#endif /* DEBUG_EN */
	}
}

/* Update current time based on tick counts*/
volatile uint32_t simulated_test_sw = SWITCH_DOWN_PIN;
volatile bool g_simulating_pressed_sw = true;
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
		if(g_simulating_pressed_sw)
		xQueueSendToBack(xSwitches_Queue, (const void *const) &simulated_test_sw, portMAX_DELAY);
		#if DEBUG_EN

		#endif /* End of DEBUG_EN */
		#if DEBUG_LOG
		// printf("Current time: %02d: %02d: %02d \r\n", cur_time.hour, cur_time.min, cur_time.sec);
		#endif /* DEBUG_LOG */
		xSemaphoreTake( xTimeUpdateSemaphore, portMAX_DELAY );
	}
}
uint8_t blink_location=0;
static mode_t cur_mode = CONFIG_MODE_DEFAULT+1;
void vTaskConfig(void *argument)
{
	uint32_t queue_msg_data = 0;

	while(1)
	{
		/* Handle button pressed */
		if(xQueueReceive(xSwitches_Queue, &queue_msg_data, 0) == pdTRUE)
		{
			#if DEBUG_LOG
			printf("pressed %d \r\n",queue_msg_data);
			#endif /* DEBUG_LOG */
			if(queue_msg_data == SWITCH_STOP_PIN)
			{
				is_alarm_active = (is_alarm_active + 1) % 2;
				if(xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdPASS) 
				{
					if(is_alarm_active)
					{
						#if DEBUG_LOG
						printf(" \r\n TURN ON ALARM !!\r\n");
						#endif /* End of DEBUG_LOG */

						#if DEBUG_EN
						lcd_buf0[14] = 'A';
						#else
						lcd_buf0[15] = 'A';
						#endif /* End of DEBUG_EN */
					}
					else 
					{
						#if DEBUG_LOG
						printf(" \r\n TURN OFF ALARM !!\r\n");
						#endif /* End of DEBUG_LOG */
						#if DEBUG_EN
						lcd_buf0[14] = 0;
						#else
						lcd_buf0[15] = 0;
						#endif /* End of DEBUG_EN */
					}
					lcd_move(0,0);
					lcd_print(lcd_buf0); 

					// Release Mutex lock
					xSemaphoreGive(xLCDMutex); 
				}
			}
			else if(queue_msg_data == SWITCH_SET_PIN)
			{
				INC(cur_mode, 0, CONFIG_MAX_VALUE);
			}

			switch (cur_mode) 
			{
				case CONFIG_ALARM_SEC:
				{
					blink_location = ALARM_SEC_START_ADDR;
					if(queue_msg_data == SWITCH_UP_PIN)
						INC(alarm_time.sec, 0 ,60);
					else if (queue_msg_data == SWITCH_DOWN_PIN)
						DEC(alarm_time.sec,  0 , 60);

				}
				break;

				case CONFIG_ALARM_MIN:
				{
					blink_location = ALARM_MIN_START_ADDR;
					if(queue_msg_data == SWITCH_UP_PIN)
						INC(alarm_time.min,  0 , 60);
					else if (queue_msg_data == SWITCH_DOWN_PIN)
						DEC(alarm_time.min,  0 , 60);

				}
				break;

				case CONFIG_ALARM_HOUR:
				{
					blink_location = ALARM_HOUR_START_ADDR;
					if(queue_msg_data == SWITCH_UP_PIN)
						INC(alarm_time.hour, 0 , 24);
					else if (queue_msg_data == SWITCH_DOWN_PIN)
						DEC(alarm_time.hour, 0, 24);

				}
				break;

				case CONFIG_TIMER_SEC:
				{
					blink_location = TIMER_SEC_START_ADDR;
					if(queue_msg_data == SWITCH_UP_PIN)
						INC(cur_time.sec,  0 , 60);
					else if (queue_msg_data == SWITCH_DOWN_PIN)
						DEC(cur_time.sec,  0 , 60);

				}
				break;

				case CONFIG_TIMER_MIN:
				{
					blink_location = TIMER_MIN_START_ADDR;
					if(queue_msg_data == SWITCH_UP_PIN)
						INC(cur_time.min,  0 , 60);
					else if (queue_msg_data == SWITCH_DOWN_PIN)
						DEC(cur_time.min,  0 , 60);

				}
				break;

				case CONFIG_TIMER_HOUR:
				{
					blink_location = TIMER_HOUR_START_ADDR;
					if(queue_msg_data == SWITCH_UP_PIN)
						INC(cur_time.hour,0,24);
					else if (queue_msg_data == SWITCH_DOWN_PIN)
						DEC(cur_time.hour,0,24);

				}
				break;

				default:
				break;
			}

			}
			// Display alarm status
		
		/* Blinking configure parameter on LCD */

		if ( (cur_mode == CONFIG_ALARM_SEC) || (cur_mode == CONFIG_ALARM_MIN) || (cur_mode == CONFIG_ALARM_HOUR) )
		{
			if(xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdPASS) 
			{
				lcd_move(blink_location, 0);
				lcd_print("  ");
				#if DEBUG_LOG
				lcd_buf0[blink_location] = ' '; lcd_buf0[blink_location+1] = ' ';
				#endif /* End of DEBUG_LOG */
				// Release Mutex lock
				xSemaphoreGive(xLCDMutex); 
			}
		}
		else if ( (cur_mode == CONFIG_TIMER_SEC) || (cur_mode == CONFIG_TIMER_MIN) || (cur_mode == CONFIG_TIMER_HOUR) )
		{
			if(xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdPASS) 
			{
				#if DEBUG_LOG
				lcd_buf1[blink_location] = ' '; lcd_buf1[blink_location+1] = ' ';
				#endif /* End of DEBUG_LOG */
				lcd_move(blink_location, 1);
				lcd_print("  ");
				// Release Mutex lock
				xSemaphoreGive(xLCDMutex); 
			}
		}
		vTaskDelay(500 / portTICK_RATE_MS); 

		
		#if DEBUG_LOG
		if(xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdPASS) 
		{ 
			printf("\r\n");
			printf("%s", lcd_buf0);
			printf("\r\n");
			printf("%s", lcd_buf1);
			printf("\r\n");
			xSemaphoreGive(xLCDMutex); 
		}
		#endif // DEBUG_LOG
}

	
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

void EXTI9_5_IRQHandler(void)
{
	if ((EXTI_GetITStatus(1 << SWITCH_UP_PIN) != RESET) || (EXTI_GetITStatus(1 << SWITCH_DOWN_PIN) != RESET))
	{
		static TickType_t last_tick = 0; 
		TickType_t cur_tick= xTaskGetTickCountFromISR(); 
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		uint32_t pressed_sw = 0;

		if(EXTI_GetITStatus(1 << SWITCH_UP_PIN) != RESET)
		{
			pressed_sw = SWITCH_UP_PIN;
			EXTI_ClearITPendingBit(1 << SWITCH_UP_PIN);
		}
		else if (EXTI_GetITStatus(1 << SWITCH_DOWN_PIN) != RESET)
		{
			pressed_sw = SWITCH_DOWN_PIN;
			EXTI_ClearITPendingBit(1 << SWITCH_DOWN_PIN);
		}
		if(cur_tick >= (last_tick + 5000))
		{ 
			xQueueSendFromISR(xSwitches_Queue, &pressed_sw, &xHigherPriorityTaskWoken); 
			last_tick = cur_tick; 
		}
  	}
}


void EXTI15_10_IRQHandler(void)
{
	if((EXTI_GetITStatus(1 << SWITCH_SET_PIN) != RESET) || (EXTI_GetITStatus(1 << SWITCH_STOP_PIN) != RESET))
	{
		
		static TickType_t last_tick = 0; 
		TickType_t cur_tick= xTaskGetTickCountFromISR(); 
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		uint32_t pressed_sw = 0;
		if(EXTI_GetITStatus(1 << SWITCH_SET_PIN) != RESET)
		{
			pressed_sw = SWITCH_SET_PIN;
			EXTI_ClearITPendingBit(1 << SWITCH_SET_PIN);
		}
		else if (EXTI_GetITStatus(1 << SWITCH_STOP_PIN) != RESET)
		{
			pressed_sw = SWITCH_STOP_PIN;
			EXTI_ClearITPendingBit(1 << SWITCH_STOP_PIN);
		}
		if(cur_tick >= (last_tick + 5000)) 
		{ 
			xQueueSendFromISR(xSwitches_Queue, &pressed_sw, &xHigherPriorityTaskWoken); 
			last_tick = cur_tick; 
		} 

  	}
}