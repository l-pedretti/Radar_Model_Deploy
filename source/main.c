/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the <CE Title> Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#include <malloc.h>
#include <inttypes.h>
#include <stdio.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "resource_map.h"
#include "xensiv_bgt60trxx_mtb.h"

#define XENSIV_BGT60TRXX_CONF_IMPL
#include <radar_settings.h>
#include "ifx_sensor_dsp.h"

#include "model.h"


/*******************************************************************************
* Macros
********************************************************************************/

#define XENSIV_BGT60TRXX_SPI_FREQUENCY      (25000000UL)

#define NUM_SAMPLES_PER_FRAME               (XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP *\
                                             XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *\
                                             XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS)

#define NUM_CHIRPS_PER_FRAME                XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME
#define NUM_SAMPLES_PER_CHIRP               XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP
#define NUM_RANGE_BINS 						(NUM_SAMPLES_PER_CHIRP / 2)
#define NUM_DOPPLER_BINS					NUM_CHIRPS_PER_FRAME
/* RTOS tasks */
#define MAIN_TASK_NAME                      "main_task"
#define MAIN_TASK_STACK_SIZE                (configMINIMAL_STACK_SIZE * 2)
#define MAIN_TASK_PRIORITY                  (configMAX_PRIORITIES - 1)

#define PREPROCESSING_TASK_NAME                "preprocessing_task"
#define PREPROCESSING_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE * 20)
#define PREPROCESSING_TASK_PRIORITY            (configMAX_PRIORITIES - 2)

/* Interrupt priorities */
#define GPIO_INTERRUPT_PRIORITY             (6)


/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void main_task(void *pvParameters);
static void preprocessing_task(void *pvParameters);
static void inference_task(void *pvParameters);
static void timer_callbak(TimerHandle_t xTimer);

static int32_t init_leds(void);
static int32_t init_sensor(void);
static void xensiv_bgt60trxx_interrupt_handler(void* args, cyhal_gpio_event_t event);


/*******************************************************************************
* Global Variables
********************************************************************************/
static cyhal_spi_t spi_obj;
static xensiv_bgt60trxx_mtb_t bgt60_obj;
static uint16_t bgt60_buffer[NUM_SAMPLES_PER_FRAME] __attribute__((aligned(2)));

float32_t temp_frame[XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS][NUM_SAMPLES_PER_FRAME/XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS];
float32_t frame[NUM_SAMPLES_PER_FRAME];
cfloat32_t range[NUM_RANGE_BINS * XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME ];
cfloat32_t doppler[XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS][NUM_RANGE_BINS * NUM_DOPPLER_BINS ];
int counter = 0;
int max_prob_id = 0;
float max_prob = 0;
float prob = 0;

static TaskHandle_t main_task_handler;
static TaskHandle_t preprocessing_task_handler;
static TimerHandle_t timer_handler;
/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

#define MAX_COUNT UINT64_MAX
int count = 0u;
volatile uint64_t count_time=0;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It initializes BSP, creates FreeRTOS  
* main task and starts the scheduler.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
	#if defined(CY_DEVICE_SECURE)
        cyhal_wdt_t wdt_obj;
        /* Clear watchdog timer so that it doesn't trigger a reset */
        result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
        CY_ASSERT(CY_RSLT_SUCCESS == result);
        cyhal_wdt_free(&wdt_obj);
    #endif
    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
    	handle_error();
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    
	if (result != CY_RSLT_SUCCESS)
	{
		handle_error();
	}
#ifdef TARGET_CYSBSYSKIT_DEV_01

    /* Initialize the User LED */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

#endif

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "Radar Range Doppler Map"
           "****************** \r\n\n"
           );

    printf("Radar Raw Data Shape: (%d, %d, %d) \r\n", XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS, NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP);

    /* Create the RTOS task */
    if (xTaskCreate(main_task, MAIN_TASK_NAME, MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &main_task_handler) != pdPASS)
    {
        CY_ASSERT(0);
    }

    /* Start the FreeRTOS scheduler. */
    vTaskStartScheduler();

    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: main_task
********************************************************************************
* Summary:
* This is the main task.
*    1. Create RTOS task
*    2. In the infinite loop, fetch the radar data when the buffer is available
* Parameters:
*  void
*
* Return:
*  none
*
*******************************************************************************/
static __NO_RETURN void main_task(void *pvParameters)
{
    (void)pvParameters;

    timer_handler = xTimerCreate("timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, timer_callbak);    
    if (timer_handler == NULL)
    {
        CY_ASSERT(0);
    }

    if (xTimerStart(timer_handler, 0) != pdPASS)
    {
        CY_ASSERT(0);
    }

    if (xTaskCreate(preprocessing_task, PREPROCESSING_TASK_NAME, PREPROCESSING_TASK_STACK_SIZE, NULL, PREPROCESSING_TASK_PRIORITY, &preprocessing_task_handler) != pdPASS)
    {
        CY_ASSERT(0);
    }


    if (init_sensor() != 0)
    {
        CY_ASSERT(0);
    }

    if (init_leds () != 0)
    {
        CY_ASSERT(0);
    }

    // Initialize the model before calling any other functions.
    IMAI_init();


    if (xensiv_bgt60trxx_start_frame(&bgt60_obj.dev, true) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        CY_ASSERT(0);
    }

    for(;;)
    {
        /* Wait for the GPIO interrupt to indicate that another slice is available */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (xensiv_bgt60trxx_get_fifo_data(&bgt60_obj.dev,
                                           bgt60_buffer,
                                           NUM_SAMPLES_PER_FRAME) == XENSIV_BGT60TRXX_STATUS_OK)
        {
            /* Data preprocessing */
            uint16_t *bgt60_buffer_ptr = &bgt60_buffer[0];
            float32_t *frame_ptr = &frame[0];
            for (int32_t sample = 0; sample < NUM_SAMPLES_PER_FRAME; ++sample)
            {
            	*frame_ptr++ = ((float32_t)(*bgt60_buffer_ptr++) / 4095.0F);
            }
            /* Tell processing task to take over */
            xTaskNotifyGive(preprocessing_task_handler);
        }
    }
}

/*******************************************************************************
* Function Name: preprocessing_task
********************************************************************************
* Summary:
* This is the data preprocessing task.
*    1. Reorder fetch data
*    2. DopplerMap + ComplexToReal
*    3. Normalize into range [0, 1]
*    4. Reshape the processed frames to H x W x C to meet Tensorflow model input form
*
* Parameters:
*  void
*
* Return:
*  None
*
*******************************************************************************/
static __NO_RETURN void preprocessing_task(void *pvParameters)
{
    (void)pvParameters;

    for(;;)
    {
        /* Wait for frame data available to process */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Reorder the fetch radar data into Rx x Chirps x Samples
        for (int i = 0; i < XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS; i++)
		{
        	for (int j = 0; j < NUM_CHIRPS_PER_FRAME; j++)
        	{
        		for (int k = 0; k < NUM_SAMPLES_PER_CHIRP; k++)
        		{
        			temp_frame[i][ j * NUM_SAMPLES_PER_CHIRP + k] = frame[j * (NUM_SAMPLES_PER_CHIRP * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS) + k * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS + i];
        		}
        	}
		}

        // Process complex doppler per channel
        for(int k = 0; k < XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS; k++)
        {
            float32_t* frame_ptr = &temp_frame[k][0];
        	// Range FFT
        	if(ifx_range_fft_f32(frame_ptr, range, true, NULL, NUM_SAMPLES_PER_CHIRP, NUM_CHIRPS_PER_FRAME) != IFX_SENSOR_DSP_STATUS_OK)
        	{
        		printf("Range FFT failed\r\n");
				abort();
        	}
        	// Range Doppler
        	if(ifx_doppler_cfft_f32(range, doppler[k], false, NULL, NUM_RANGE_BINS, NUM_DOPPLER_BINS) != IFX_SENSOR_DSP_STATUS_OK)
			{
				printf("Range Doppler failed\r\n");
				abort();
			}
        	ifx_shift_cfft_f32(doppler[k], NUM_DOPPLER_BINS, NUM_RANGE_BINS);

        	frame_ptr += NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP;
        }


        // RDM of 3 Antennas is ready to be used.

        // Prepare data for enqueue
        float data_to_enqueue[XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS * NUM_DOPPLER_BINS * NUM_RANGE_BINS];
        int index = 0;
        for(int k = 0; k < XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS; k++)
        {
        	for (int i = 0; i < NUM_DOPPLER_BINS; i++)
        	{
        		for(int j = 0; j < NUM_RANGE_BINS; j++)
        		{
        			data_to_enqueue[index++] = cabs(doppler[k][i * NUM_DOPPLER_BINS + j]);
        		}
        	}
        }
        // Enqueue the data
        if (IMAI_enqueue(data_to_enqueue) != 0)
        {
        	printf("Failed to enqueue data\r\n");
        	abort();
        }

        // Dequeue the data
        float data_out[3];
        if (IMAI_dequeue(data_out) != 0)
        {
        	printf("Failed to dequeue data\r\n");
        	abort();
        }
        // Process the output data
        for (int i = 0; i < 3; i++)
        {
        	printf("Confidence for class %d: %f\n", i, data_out[i]);
        }

	}
}


/*******************************************************************************
* Function Name: init_sensor
********************************************************************************
* Summary:
* This function configures the SPI interface, initializes radar and interrupt
* service routine to indicate the availability of radar data.
*
* Parameters:
*  void
*
* Return:
*  Success or error
*
*******************************************************************************/
static int32_t init_sensor(void)
{
    if (cyhal_spi_init(&spi_obj,
                       PIN_XENSIV_BGT60TRXX_SPI_MOSI,
                       PIN_XENSIV_BGT60TRXX_SPI_MISO,
                       PIN_XENSIV_BGT60TRXX_SPI_SCLK,
                       NC,
                       NULL,
                       8,
                       CYHAL_SPI_MODE_00_MSB,
                       false) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: cyhal_spi_init failed\n");
        return -1;
    }

    /* Reduce drive strength to improve EMI */
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CY_GPIO_DRIVE_1_8);

    /* Set the data rate to 25 Mbps */
    if (cyhal_spi_set_frequency(&spi_obj, XENSIV_BGT60TRXX_SPI_FREQUENCY) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: cyhal_spi_set_frequency failed\n");
        return -1;
    }

    /* Enable LDO */
    if (cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_LDO_EN,
                        CYHAL_GPIO_DIR_OUTPUT,
                        CYHAL_GPIO_DRIVE_STRONG,
                        true) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: LDO_EN cyhal_gpio_init failed\n");
        return -1;
    }

    /* Wait LDO stable */
    (void)cyhal_system_delay_ms(5);

    if (xensiv_bgt60trxx_mtb_init(&bgt60_obj, 
                                  &spi_obj, 
                                  PIN_XENSIV_BGT60TRXX_SPI_CSN, 
                                  PIN_XENSIV_BGT60TRXX_RSTN, 
                                  register_list, 
                                  XENSIV_BGT60TRXX_CONF_NUM_REGS) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: xensiv_bgt60trxx_mtb_init failed\n");
        return -1;
    }

    if (xensiv_bgt60trxx_mtb_interrupt_init(&bgt60_obj,
                                            NUM_SAMPLES_PER_FRAME,
                                            PIN_XENSIV_BGT60TRXX_IRQ,
                                            GPIO_INTERRUPT_PRIORITY,
                                            xensiv_bgt60trxx_interrupt_handler,
                                            NULL) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: xensiv_bgt60trxx_mtb_interrupt_init failed\n");
        return -1;
    }

    return 0;
}


/*******************************************************************************
* Function Name: xensiv_bgt60trxx_interrupt_handler
********************************************************************************
* Summary:
* This is the interrupt handler to react on sensor indicating the availability
* of new data
*    1. Notifies main task on interrupt from sensor
*
* Parameters:
*  void
*
* Return:
*  none
*
*******************************************************************************/
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
static void xensiv_bgt60trxx_interrupt_handler(void *args, cyhal_gpio_event_t event)
#else
static void xensiv_bgt60trxx_interrupt_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(main_task_handler, &xHigherPriorityTaskWoken);

    /* Context switch needed? */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/*******************************************************************************
* Function Name: init_leds
********************************************************************************
* Summary:
* This function initializes the GPIOs for LEDs and set them to off state.
* Parameters:
*  void
*
* Return:
*  Success or error
*
*******************************************************************************/
static int32_t init_leds(void)
{

    if(cyhal_gpio_init(LED_RGB_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false)!= CY_RSLT_SUCCESS)
    {
        printf("ERROR: GPIO initialization for LED_RGB_RED failed\n");
        return -1;
    }

    if( cyhal_gpio_init(LED_RGB_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false)!= CY_RSLT_SUCCESS)
    {
        printf("ERROR: GPIO initialization for LED_RGB_GREEN failed\n");
        return -1;
    }

    if( cyhal_gpio_init(LED_RGB_BLUE, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false)!= CY_RSLT_SUCCESS)
    {
        printf("ERROR: GPIO initialization for LED_RGB_BLUE failed\n");
        return -1;
    }

    return 0;
}


/*******************************************************************************
* Function Name: timer_callbak
********************************************************************************
* Summary:
* This is the timer_callback which toggles the LED
*
* Parameters:
*  void
*
* Return:
*  none
*
*******************************************************************************/
static void timer_callbak(TimerHandle_t xTimer)
{
    (void)xTimer;

#ifdef TARGET_CYSBSYSKIT_DEV_01
    cyhal_gpio_toggle(CYBSP_USER_LED);
#endif
}

/* [] END OF FILE */
