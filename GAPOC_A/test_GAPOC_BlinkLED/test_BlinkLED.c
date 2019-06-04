/*
 * Copyright (c) 2019, GreenWaves Technologies
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of GreenWaves Technologies nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


// #############################################################################
// ##### DECLARATIONS  ########################################################

// ==  Handy "defines" for application, used locally   =====================
// <none>


// ====  Includes    ==========================================================

#include "GAPOC_BSP_General.h"

#include "hyperbus_api.h"

#ifdef __FREERTOS__
#include "FreeRTOS_util.h"
#else
#include "mbed_wait_api.h"
#endif


// ====  Application's Custom Types    ========================================
// <none>

    
// ==== Application's public global variables  ==============================
// <none>


// ==== Application's own global variables  ====================================
#ifdef __FREERTOS__
/* Utilities to control tasks. */
TaskHandle_t tasks[NBTASKS];
uint8_t taskSuspended;
#endif

// ==== Application's Function Prototypes    ===================================
void vTestBlinkLED(void *parameters);

// #############################################################################
// ##### MAIN APPLICATION  ###########################################

int main()
{
    printf("\nBlink LED TEST !\n");

    #ifdef __FREERTOS__
    #if configSUPPORT_DYNAMIC_ALLOCATION == 1
    BaseType_t xTask;
    TaskHandle_t xHandler0 = NULL;

    xTask = xTaskCreate( vTestBlinkLED, "TestBlinkLed", configMINIMAL_STACK_SIZE * 2,
                         NULL, tskIDLE_PRIORITY + 1, &xHandler0 );
    if( xTask != pdPASS )
    {
        printf("TestBlinkLED is NULL !\n");
        exit(0);
    }
    #endif //configSUPPORT_DYNAMIC_ALLOCATION

    tasks[0] = xHandler0;

    /* Start the kernel.  From here on, only tasks and interrupts will run. */
    printf("\nScheduler starts !\n");
    vTaskStartScheduler();

    /* Exit FreeRTOS */
    return 0;
    #else
    vTestBlinkLED(NULL);
    return 0;
    #endif
}


void vTestBlinkLED(void *parameters)
{
    DBG_PRINT("\nGAPOC LED Blink Test\n\n");
    DBG_PRINT("\n\n** DIP Switch position #6 must be closed (ON) to enable the on-board LED **\n");
    DBG_PRINT("and pin xx of Connector3 also bears the GPIO signal that controls the LED\n\n");    
      
    //  Initalize Board (GPIO direction and default level, supplies, etc.)       
    GAPOC_BSP_Board_Init();

    // Initialize HyperBus I/Os to limit consumption from Memory
    // Ultimately this should go into GAPOC_BSP_Board_Init();

    hyperbus_t hyperbus0;
    hyperbus_init(&hyperbus0, HYPERBUS_DQ0, HYPERBUS_DQ1, HYPERBUS_DQ2, HYPERBUS_DQ3,
                  HYPERBUS_DQ4, HYPERBUS_DQ5, HYPERBUS_DQ6, HYPERBUS_DQ7,
                  HYPERBUS_CLK, HYPERBUS_CLKN, HYPERBUS_RWDS, HYPERBUS_CSN0, HYPERBUS_CSN1);
                     

    // Initialize relevant GPIO as pure output, starting @high logic level
    GAPOC_GPIO_Init_Pure_Output_High( GAPOC_HEARTBEAT_LED );

    //  ---   Main Loop                  --------------------------------------------------------------------------------       
    
    #define LED_ON_TIME_ms   150
    #define LED_OFF_TIME_ms  850
    while(1)
    {
        #ifdef __FREERTOS__
        vTaskDelay( LED_ON_TIME_ms / portTICK_PERIOD_MS );
        #else
        wait( (float)(LED_ON_TIME_ms)/1000 );
        #endif
        GAPOC_GPIO_Toggle( GAPOC_HEARTBEAT_LED );

        #ifdef __FREERTOS__
        vTaskDelay( LED_OFF_TIME_ms / portTICK_PERIOD_MS );
        #else
        wait( (float)(LED_OFF_TIME_ms)/1000 );
        #endif
        GAPOC_GPIO_Toggle( GAPOC_HEARTBEAT_LED );        

    }

    #ifdef __FREERTOS__
    vTaskSuspend(NULL);
    #endif
}
// ## END OF FILE ##############################################################################


