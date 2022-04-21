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


#define LOG_DEBUG								1
#define CONTROL_QUEUE_LEN				50
#define THRESHOLD_DEFAULT				1500 //in mV

typedef enum {
	SENSOR_DATA=0,		//Send from sensor reading task
	SENSOR_SETTING		//Send from sensor setting task
}data_type_t;

typedef struct data_msg
{
	data_type_t type;
	uint16_t	data_value;
}data_msg_t;


void vLED_Init(void);
void vSW1_Init(void);
void vADC1_Init(void);
void vADC2_Init(void);
uint16_t sADC1_GetData(void);
uint16_t sADC2_GetData(void); 

xTaskHandle hSensorReadingTask;
xTaskHandle hSensorSettingTask;
xTaskHandle hControlTask;

portTASK_FUNCTION_PROTO(vSensorReading, pvParameters);
portTASK_FUNCTION_PROTO(vSensorSetting, pvParameters);
portTASK_FUNCTION_PROTO(vControl, pvParameters);

// *** Declare a queue HERE
static SemaphoreHandle_t xbinary_sem = NULL;
static QueueHandle_t control_queue = NULL; 

// ============================================================================
int main(void) {
	#if LOG_DEBUG
	vUSART2_Init();
	#endif //LOG_DEBUG
	vLED_Init();
	vSW1_Init();
	vADC1_Init();
	vADC2_Init();
	lcd_init();

	xbinary_sem = xSemaphoreCreateBinary();
	if(xbinary_sem == NULL)
	{
		printf("Cant create binary semaphore \r\n ");
		while(1)	//Hang in here
		{

		}
	}
	
	control_queue = xQueueCreate(CONTROL_QUEUE_LEN, sizeof(data_msg_t)); 
	if(control_queue == NULL)
	{
		printf("Cant create control queue \r\n ");
		while(1)	//Hang in here
		{

		}
	}

	xTaskCreate(vSensorSetting, "SENSOR_SETTING", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, &hSensorSettingTask);
	xTaskCreate(vSensorReading, "SENSOR_READING", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, &hSensorReadingTask);
	xTaskCreate(vControl, "CONTROL", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, &hControlTask);
	
	vTaskStartScheduler(); // This should never return.

	// Will only get here if there was insufficient memory to create the idle task.
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
		xQueueSend(control_queue, &sensor_data_msg, portMAX_DELAY);
		vTaskDelay(5000/portTICK_RATE_MS);
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
		//Send data to control task
		xQueueSend(control_queue, &sensor_setting_msg, portMAX_DELAY);
	}
}

// Control task which receive data from sensor reading task and sensor setting task
// ---------------------------------------------------------------------------- 
portTASK_FUNCTION(vControl, pvParameters) {
	bool is_ledon=false;
	data_msg_t data_recv = {0};
	uint16_t sensor_reading=0;
	uint16_t threshold_setting=THRESHOLD_DEFAULT;
	char lcd_buffer[16]={0};
	
	#if LOG_DEBUG
	printf("\r\n ====================User settings====================\r\n"); 
	printf("Default threshold %d \r\n", threshold_setting);
	printf("\r\n =======================================================\r\n"); 	
	#endif //LOG_DEBUG
	lcd_move(0,0);
	lcd_print("Threshold:");
	
	lcd_move(0,1);
	lcd_print("Sensor data:");

	while(1) 
	{	
		//Receive data
		if(xQueueReceive(control_queue, &data_recv, portMAX_DELAY) == pdTRUE)
		{
			if(data_recv.type == SENSOR_SETTING)
			{
				threshold_setting = data_recv.data_value;
				//Display to LCD
				sprintf(lcd_buffer, "%04d", threshold_setting);
				lcd_move(sizeof("Threshold:"), 0);
				#if LOG_DEBUG
				printf("\r\n ====================User settings====================\r\n"); 
				printf("Sensor threshold setting %d \r\n", threshold_setting);
				printf("\r\n =======================================================\r\n	"); 
				#endif //LOG_DEBUG

			}
			else if(data_recv.type == SENSOR_DATA)
			{
				sensor_reading = data_recv.data_value;
				//Display to LCD
				sprintf(lcd_buffer, "%04d", sensor_reading);
				lcd_move(sizeof("Sensor data:"), 1);
				#if LOG_DEBUG
				printf("Sensor reading %d \r\n", sensor_reading);
				#endif //LOG_DEBUG
			}
			else
			{

			}
		}
		
		if(is_ledon == false)
		{
			if (sensor_reading >= threshold_setting) 
			{
				//Turn on LED
				GPIO_SetBits(GPIOD, GPIO_Pin_8);
				is_ledon = true;
				#if LOG_DEBUG
				printf("\r\n =======================================================\r\n"); 
				printf("LED ON \r\n");
				printf("\r\n =======================================================\r\n"); 
	
				#endif //LOG_DEBUG
			}
		}
		else //led is on already
		{
			if (sensor_reading < threshold_setting) 
			{
				//Turn off LED
				GPIO_ResetBits(GPIOD, GPIO_Pin_8);
				is_ledon = false;
				#if LOG_DEBUG
				printf("\r\n =======================================================\r\n"); 
				printf("LED OFF \r\n");
				printf("\r\n =======================================================\r\n"); 
				#endif //LOG_DEBUG
			}
		}
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
void vLED_Init(void)
{
  // Init LED pin (PD8)
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  //Turn off LED
  GPIO_ResetBits(GPIOD, GPIO_Pin_8);

}

// ============================================================================
void vSW1_Init(void)
{
  // Enable GPIOE clock
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
  // Enable SYSCFG clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
  // PE8 input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  // Configure EXTI Line8
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource8);

  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  // Enable and set EXTI Line8 Interrupt to the lowest priority
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

// ============================================================================
/* ADC1 Initialization to reading sensor value (RV2) which connected to RV3 - ADC1-IN12
 */
void vADC1_Init(void)
{
  //Enable clocks for ADC and ADC pin
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
  // Init GPIO for ADC1 Channel 12 (PC2) pin as analog input
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;;
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
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  // ADC Channel Init
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_480Cycles);

  // ADC Enable 
  ADC_Cmd(ADC1, ENABLE);
	
	// Start the conversion
	ADC_SoftwareStartConv(ADC1);
}

// ============================================================================
uint16_t sADC1_GetData(void) //s for short
{
  	uint16_t raw_adc = ADC_GetConversionValue(ADC1);
		double cal_temp = raw_adc * 3000.0 / 4095;	// casting to double for accurately calculation
		return (uint16_t)cal_temp;
}

// ============================================================================
/* ADC2 Initialization to reading user setting threshold which connected to RV2 - ADC2-IN11
 */
void vADC2_Init(void)
{
  //Enable clocks for ADC and ADC pin
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    
  // Init GPIO for ADC2 Channel 11 (PC1) pin as analog input
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
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
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC2, &ADC_InitStructure);

  // ADC Channel Init
  ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_480Cycles);

  // ADC Enable 
  ADC_Cmd(ADC2, ENABLE);
	
	// Start the conversion
	ADC_SoftwareStartConv(ADC2);
}

// ============================================================================
uint16_t sADC2_GetData(void) //s for short
{
  	uint16_t raw_adc = ADC_GetConversionValue(ADC2);
		double cal_temp = raw_adc * 3000.0 / 4095;	// convert to mV casting to double for accurately calculation
		return (uint16_t)cal_temp;
}

// ============================================================================