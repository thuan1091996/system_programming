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
#define TESTING (1)
#define TIM4_RELOAD_VALUE (10000) // TIM4 count up 0 -> TIM4_RELOAD_VALUE
#define QUEUE_LENGTH (20)

#define HARDWARE_INT_MIN_PRO (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)

#define START_STOP_BUTTON_PRIO 6 // Priority should be "lower" than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (which =5)
#define RESET_BUTTON_PRIO 7

#define LED_MSG_TYPE 1
#define THERMO_MSG_TYPE 2
/******************************************************************************
 * Module Typedefs
 *******************************************************************************/
typedef struct
{
	uint8_t msg_type;
	union
	{
		uint8_t thermo_percent;
		struct
		{
			uint16_t led_pin;
			uint8_t led_status;
		} led_data;
	} msg_data;
} lcd_task_data_t;

/******************************************************************************
 * Module Variable Definitions
 *******************************************************************************/
xTaskHandle hTask1;
xTaskHandle hTask2;
xTaskHandle hTask3;

#if TESTING
volatile uint32_t auto_reload = 10000;
volatile uint32_t compare_value = 9000;
volatile uint32_t g_test = 1;
#endif /* End of TESTING */

static QueueHandle_t xled_control_queue = NULL;
static QueueHandle_t xlcd_display_queue = NULL;
/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/
portTASK_FUNCTION_PROTO(vTask1, pvParameters);
portTASK_FUNCTION_PROTO(vTask2, pvParameters);
portTASK_FUNCTION_PROTO(vTask3, pvParameters);

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void bsp_init(void);
void led_init(void);
void button_init(void);
void tim4_init(void);
void thermo_init(void);
void thermo_set_duty(uint8_t duty_cycle);
void pot_init(void);
uint16_t pot_readdata(void);

/******************************************************************************
 * Function Definitions
 *******************************************************************************/

int main(void)
{
	vUSART2_Init(); // Initialise serial port

	printf("Hardware initialisation...\r\n");
	bsp_init(); // Initialize the LCD, LEDs, POT and buttons

	printf("FreeRTOS services initialisation...\r\n");
	xled_control_queue = xQueueCreate(QUEUE_LENGTH, sizeof(uint16_t));
	if (xled_control_queue == NULL)
	{
		printf("LED queue creation failed!\r\n");
	}

	xlcd_display_queue = xQueueCreate(QUEUE_LENGTH, sizeof(lcd_task_data_t));
	if (xlcd_display_queue == NULL)
	{
		printf("LCD queue creation failed!\r\n");
	}

	printf("FreeRTOS tasks initialisation...\r\n");

	xTaskCreate(vTask1, "LED_CONTROL", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &hTask1);
	if (hTask1 == NULL)
	{
		printf("Task 1 creation failed!\r\n");
	}

	xTaskCreate(vTask2, "LCD_DISPLAY", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &hTask2);
	if (hTask2 == NULL)
	{
		printf("Task 2 creation failed!\r\n");
	}

	xTaskCreate(vTask3, "THERMO_TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &hTask3);
	if (hTask3 == NULL)
	{
		printf("Task 3 creation failed!\r\n");
	}

	vTaskStartScheduler(); // This should never return.
	while (1)
	{
	}
}

portTASK_FUNCTION(vTask1, pvParameters)
{
	uint16_t sw_pressed;
	uint8_t led1_status = 0;
	uint8_t led2_status = 0;
	lcd_task_data_t led_msg_2lcd =
		{
			.msg_type = LED_MSG_TYPE,
		};
	GPIO_WriteBit(GPIOD, (GPIO_Pin_8 | GPIO_Pin_9), Bit_RESET); // Turn off PD8, PD9
	while (1)
	{
		if (xQueueReceive(xled_control_queue, &sw_pressed, portMAX_DELAY) == pdTRUE)
		{
			printf("Task LED_CONTROL: Button %d pressed\r\n", sw_pressed);
			if (sw_pressed == GPIO_Pin_8)
			{
				if (led1_status == 1)
				{
					GPIO_WriteBit(GPIOD, GPIO_Pin_8, Bit_RESET); // Turn off PD8
					led1_status = 0;
				}
				else
				{
					GPIO_WriteBit(GPIOD, GPIO_Pin_8, Bit_SET); // Turn on PD8
					led1_status = 1;
				}
				led_msg_2lcd.msg_data.led_data.led_pin = GPIO_Pin_8;
				led_msg_2lcd.msg_data.led_data.led_status = led1_status;
			}
			else if (sw_pressed == GPIO_Pin_9)
			{
				if (led2_status == 1)
				{
					GPIO_WriteBit(GPIOD, GPIO_Pin_9, Bit_RESET); // Turn off PD9
					led2_status = 0;
				}
				else
				{
					GPIO_WriteBit(GPIOD, GPIO_Pin_9, Bit_SET); // Turn on PD9
					led2_status = 1;
				}
				led_msg_2lcd.msg_data.led_data.led_pin = GPIO_Pin_9;
				led_msg_2lcd.msg_data.led_data.led_status = led2_status;
			}
			else
			{
				printf("Task LED_CONTROL: Unknown button pressed\r\n");
			}
			// Send update to LCD task
			if (xQueueSend(xlcd_display_queue, &led_msg_2lcd, portMAX_DELAY) != pdTRUE)
			{
				printf("Task LED_CONTROL: Failed to send message to LCD task\r\n");
			}
		}
	}
}

portTASK_FUNCTION(vTask2, pvParameters)
{
	lcd_task_data_t lcd_msg_recv;
	char lcd_buf0[16] = {0};
	// Display initial message
	lcd_move(0, 0);
	lcd_print("LED1: O");
	lcd_move(8, 0);
	lcd_print("LED2: O");
	lcd_move(0, 0);
	lcd_print("THERMAL: 000 %");

	while (1)
	{
		if (xQueueReceive(xled_control_queue, &lcd_msg_recv, portMAX_DELAY) == pdTRUE)
		{
			if (lcd_msg_recv.msg_type == LED_MSG_TYPE)
			{
				if (lcd_msg_recv.msg_data.led_data.led_pin == GPIO_Pin_8)
				{
					if (lcd_msg_recv.msg_data.led_data.led_status == 1)
					{
						lcd_move(6, 0);
						lcd_print("I");
					}
					else
					{
						lcd_move(6, 0);
						lcd_print("O");
					}
				}
				else if (lcd_msg_recv.msg_data.led_data.led_pin == GPIO_Pin_9)
				{
					if (lcd_msg_recv.msg_data.led_data.led_status == 1)
					{
						lcd_move(14, 0);
						lcd_print("I");
					}
					else
					{
						lcd_move(14, 0);
						lcd_print("O");
					}
				}
				else
				{
					printf("Task LCD_DISPLAY: Unknown LED pin\r\n");
				}
			}
			else if (lcd_msg_recv.msg_type == THERMO_MSG_TYPE)
			{
				memset(lcd_buf0, 0, sizeof(lcd_buf0));
				sprintf(lcd_buf0, "%03d", lcd_msg_recv.msg_data.thermo_percent);
				lcd_move(10, 1);
				lcd_print(lcd_buf0);
			}
			else
			{
				printf("Task LCD_DISPLAY: Unknown message type\r\n");
			}
		}
	}
}
portTASK_FUNCTION(vTask3, pvParameters)
{
	uint16_t pot_voltage = 0;
	uint8_t input_percentage = 0;
	lcd_task_data_t thermo_msg_2lcd =
		{
			.msg_type = THERMO_MSG_TYPE,
		};
	while (1)
	{
		// Read POT value
		pot_voltage = pot_readdata();
		// Convert POT value to percentage
		input_percentage = (double)((pot_voltage * 100.0) / 3300); /* max 3300 milivolt */
		// Send duty cycle to TIM4 channel 4
		thermo_set_duty(input_percentage);
		// Send update to LCD task
		thermo_msg_2lcd.msg_data.thermo_percent = input_percentage;
		printf("Sending thermo percent: %d\r\n", input_percentage);
		if (xQueueSend(xlcd_display_queue, &thermo_msg_2lcd, portMAX_DELAY) != pdTRUE)
		{
			printf("Task THERMO_TASK: Failed to send message to LCD task\r\n");
		}
		vTaskDelay(10000);
	}
}

/**
 * @brief  Set the PWM duty cycle of TIM4 channel 4
 * @param  duty_cycle: duty cycle value (0- 100%)
 */
void thermo_set_duty(uint8_t duty_cycle)
{
	double timer_setting_value = duty_cycle * TIM4_RELOAD_VALUE * 1.0 / 100.0;
	TIM_SetCompare4(TIM4, timer_setting_value);
}

/**
 * @brief  This function handles External line 5-9 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI9_5_IRQHandler(void)
{
	static TickType_t last_tick = 0;
	TickType_t cur_tick = xTaskGetTickCountFromISR();
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint16_t pressed_button = 0;

	if (cur_tick >= last_tick + 5000) // 5s debouncing button pressed
	{
		if (EXTI_GetITStatus(EXTI_Line8) != RESET)
		{
			pressed_button = GPIO_Pin_8;
		}
		else if (EXTI_GetITStatus(EXTI_Line9) != RESET)
		{
			pressed_button = GPIO_Pin_9;
		}
		xQueueSendFromISR(xled_control_queue, &pressed_button, &xHigherPriorityTaskWoken);
		last_tick = cur_tick;
	}
	EXTI_ClearITPendingBit(EXTI_Line8);
	EXTI_ClearITPendingBit(EXTI_Line9);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ============================================================================

void bsp_init(void)
{
	lcd_init();	   // Initialize LCD
	led_init();	   // Init LEDs (PD8, PD9)
	thermo_init(); // Init thermometer (PD15)
	pot_init();	   // Init potentiometer (PC2)
	button_init(); // Init buttons (PE8, PE9)
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

	GPIO_WriteBit(GPIOD, (GPIO_Pin_8 | GPIO_Pin_9), Bit_RESET); // Turn off PD8, PD9
}

void thermo_init(void)
{

	/* GPIOD clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* GPIOD Configuration:  PD15 as PWM output (TIM4 CH4)  */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect TIM4 pins to AF2 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

	// Initialize the timer4
	tim4_init();
}

void tim4_init(void)
{
	// TIM4 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = TIM4_RELOAD_VALUE;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// Prescaler configuration
	TIM_PrescalerConfig(TIM4, ((160 / 2) - 1), TIM_PSCReloadMode_Immediate); // Timer clock = 16Mhz -> 1 tick = 1ms

	// PWM1 Mode configuration: Channel4
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // output is active low when the counter value is less than the OC value
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	/* TIM4 enable counter */
	TIM_Cmd(TIM4, ENABLE);
}

/**
 * @brief ADC1 Initialization RV3 - ADC1-IN12
 */
void pot_init(void)
{
	// Enable clocks for ADC and ADC pin
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// Init GPIO for ADC1 Channel 12 (PC2) pin as analog input
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	;
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
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_15Cycles;
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

/**
 * @brief  Read potentiometer voltage value
 * @param  None
 * @retval pot voltage in milivolts
 */
uint16_t pot_readdata(void)
{
	uint16_t adc_value = ADC_GetConversionValue(ADC1);
	double mil_volt = adc_value * 3300.0 / 4095.0;
	return mil_volt;
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
	EXTI_InitTypeDef EXTI_InitStructure;

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource8);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource9);

	EXTI_InitStructure.EXTI_Line = (EXTI_Line8 | EXTI_Line9);
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Clear pending bits
	EXTI_ClearITPendingBit(EXTI_Line8);
	EXTI_ClearITPendingBit(EXTI_Line9);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HARDWARE_INT_MIN_PRO + 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
