/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//****************************************************************************
//
// main.c - MSP-EXP432P401R + Educational Boosterpack MkII - Accelerometer
//
//          Displays raw 14-bit ADC measurements for X, Y, and Z axis
//          on the colored LCD.
//
//          Tilting the boosterpack to the 4 different vertical orientations
//          also controls the LCD orientation accordingly.
//
//****************************************************************************

#include <ti/devices/msp432p4xx/inc/msp.h>
//#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
//#include <ti/grlib/grlib.h>
//#include "LcdDriver/Crystalfontz128x128_ST7735.h"
//#include <stdio.h>
//
///* Graphic library context */
//Graphics_Context g_sContext;
//
///* Graphic library display */
//Graphics_Display g_sDisplay;
//
//Graphics_Rectangle values_rectangle_heaven;
//Graphics_Rectangle values_rectangle_earth;
//
///* ADC results buffer */
//static uint16_t resultsBuffer[3];
//
//void drawHorizon(void);
//
///*
// * Main function
// */
//int main(void)
//{
//    /* Halting WDT and disabling master interrupts */
//    MAP_WDT_A_holdTimer();
//    MAP_Interrupt_disableMaster();
//
//    /* Initializes Clock System */
//    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
//
//    /* Initializes display */
//    Crystalfontz128x128_Init();
//
//    /* Set default screen orientation */
//    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
//
//    /* Initializes graphics context */
//    Graphics_initContext(&g_sContext, &g_sDisplay, &g_sCrystalfontz128x128_funcs);
//    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
//    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
//
//    /* Configures Pin 4.0, 4.2, and 6.1 as ADC input */
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);
//
//    /* Initializing ADC (ADCOSC/64/8) */
//    MAP_ADC14_enableModule();
//    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8,
//            0);
//
//    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM2 (A11, A13, A14)  with no repeat)
//         * with internal 2.5v reference */
//    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
//    MAP_ADC14_configureConversionMemory(ADC_MEM0,
//            ADC_VREFPOS_AVCC_VREFNEG_VSS,
//            ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);
//
//    MAP_ADC14_configureConversionMemory(ADC_MEM1,
//            ADC_VREFPOS_AVCC_VREFNEG_VSS,
//            ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);
//
//    MAP_ADC14_configureConversionMemory(ADC_MEM2,
//            ADC_VREFPOS_AVCC_VREFNEG_VSS,
//            ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);
//
//    /* Enabling the interrupt when a conversion on channel 2 (end of sequence)
//     *  is complete and enabling conversions */
//    MAP_ADC14_enableInterrupt(ADC_INT2);
//
//    /* Enabling Interrupts */
//    MAP_Interrupt_enableInterrupt(INT_ADC14);
//    MAP_Interrupt_enableMaster();
//
//    /* Setting up the sample timer to automatically step through the sequence
//     * convert.
//     */
//    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
//
//    /* Triggering the start of the sample */
//    MAP_ADC14_enableConversion();
//    MAP_ADC14_toggleConversionTrigger();
//
//    while(1)
//    {
//
//    }
//}
//
//
///*
//
// */
//void drawHorizon()
//{
//    values_rectangle_heaven.xMin = 0;
//    values_rectangle_heaven.xMax = 127;
//    values_rectangle_heaven.yMin = 0;
//    values_rectangle_heaven.yMax = (128*(resultsBuffer[2]-11462))/(4922-11462);
//    /*values_rectangle_heaven.yMax =
//            (resultsBuffer[0] < 8192) ?
//                    (y0 * (resultsBuffer[0] - 8192)) / (8192 - 6557) + y0 :
//                    (-1 * y0 * (resultsBuffer[0] - 8192)) / (9827 - 8192) + y0;
//*/
//    // Dibujo cielo azul
//    Graphics_fillRectangleOnDisplay(&g_sDisplay,
//                                    &values_rectangle_heaven,
//                                    0x001F);
//
//    values_rectangle_earth.xMin = 0;
//    values_rectangle_earth.xMax = 127;
//    values_rectangle_earth.yMin = values_rectangle_heaven.yMax;
//    values_rectangle_earth.yMax = 127;
//
//    // Dibujo tierra cafe
//    Graphics_fillRectangleOnDisplay(&g_sDisplay,
//                                    &values_rectangle_earth,
//                                    0x8208);
///*
//    // Extras
//    if (resultsBuffer[0] < 8192)
//    {
//        int y1 = (y0*(resultsBuffer[0]-8192))/(8192-6557)+y0;
//        int i;
//        for (i = 0; i < (y0-y1); i++)
//        {
//            //Extra 1.0
//            Graphics_drawHorizontalLineOnDisplay(
//                    &g_sDisplay,
//                    0,
//                    (128*(i+1) / (y0-y1)),
//                    (y1+i),
//                    0x804040);
//            //Extra 1.1
//            Graphics_drawHorizontalLineOnDisplay(
//                    &g_sDisplay,
//                    (128*(i+1) / (y0-y1))+1,
//                    127,
//                    (y1+i),
//                    0x0000FF);
//            // //Extra 2.0
//            Graphics_drawHorizontalLineOnDisplay(&g_sDisplay,
//                                                 63-(63)*(8192-resultsBuffer[0])/(8192-4922),
//                                                 63,
//                                                 63,
//                                                 0xFFFFFF);
//        }
//    }else{
//        int y2 = (-1*y0*(resultsBuffer[0]-8192))/(9827-8192)+y0;
//        int i;
//        for (i = 0; i < (y0-y2); i++)
//        {
//            //Extra 1.0
//            Graphics_drawHorizontalLineOnDisplay(
//                    &g_sDisplay,
//                    0,
//                    128-(128*(i+1)/(y0-y2)),
//                    (y2+i),
//                    0x0000FF);
//            //Extra 1.1
//            Graphics_drawHorizontalLineOnDisplay(
//                    &g_sDisplay,
//                    128-(128*(i+1)/(y0-y2))+1,
//                    127,
//                    (y2+i),
//                    0x804040);
//            //Extra 2.0
//            Graphics_drawHorizontalLineOnDisplay(&g_sDisplay,
//                                                      63,
//                                                      63+(127-63)*(resultsBuffer[0]-8192)/(11462-8192),
//                                                      63,
//                                                      0xFFFFFF);
//        }
//   }
//*/
//}
///* This interrupt is fired whenever a conversion is completed and placed in
// * ADC_MEM2. This signals the end of conversion and the results array is
// * grabbed and placed in resultsBuffer */
//void ADC14_IRQHandler(void)
//{
//    uint64_t status;
//
//    status = MAP_ADC14_getEnabledInterruptStatus();
//    MAP_ADC14_clearInterruptFlag(status);
//
//    /* ADC_MEM2 conversion completed */
//    if(status & ADC_INT2)
//    {
//        /* Store ADC14 conversion results */
//        resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
//        resultsBuffer[1] = ADC14_getResult(ADC_MEM1);
//        resultsBuffer[2] = ADC14_getResult(ADC_MEM2);
//
//        /*
//         * Draw accelerometer data on display and determine if orientation
//         * change thresholds are reached and redraw as necessary
//         */
//        if (resultsBuffer[1] < 8192) {
//                Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
//                drawHorizon();
//        }else{
//                Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_DOWN);
//                drawHorizon();
//        }
//    }
//}

//#include "msp.h"
#include "main.hpp"
#include "Scheduler.hpp"
#include "Task.hpp"
#include "LED.hpp"

// ##########################
// Global/Static declarations
// ##########################
uint8_t Task::m_u8NextTaskID = 0; // - Init task ID
volatile static uint64_t g_SystemTicks = 0; // - The system counter.
Scheduler g_MainScheduler; // - Instantiate a Scheduler

// #########################
//          MAIN
// #########################
void main(void)
{

    // - Instantiate two new Tasks
    LED BlueLED(BIT2);
    LED GreenLED(BIT1);
    // - Run the overall setup function for the system
    Setup();
    // - Attach the Tasks to the Scheduler;
    g_MainScheduler.attach(&BlueLED, 500);
    //g_MainScheduler.attach(&GreenLED, 300);
    // - Run the Setup for the scheduler and all tasks
    g_MainScheduler.setup();
    // - Main Loop
    while(1)
    {
        __wfe(); // Wait for Event
        if(g_SystemTicks != g_MainScheduler.m_u64ticks)
        {
            //- Only execute the tasks if one tick has passed.
            g_MainScheduler.m_u64ticks = g_SystemTicks;
            g_MainScheduler.run();
        }
    }
}

// **********************************
// Setup function for the application
// @input - none
// @output - none
// **********************************
void Setup(void)
{
    // ****************************
    //         DEVICE CONFIG
    // ****************************
    // - Disable WDT
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    // ****************************
    //         PORT CONFIG
    // ****************************
    // - P1.0 is connected to the Red LED
    // - This is the heart beat indicator.
    P1->DIR |= BIT0; //Red LED

    // ****************************
    //       TIMER CONFIG
    // ****************************
    // - Configure Timer32_1  with MCLK (3Mhz), Division by 1, Enable the interrupt, Periodic Mode
    // - Enable the interrupt in the NVIC
    // - Start the timer in UP mode.
    // - Re-enable interrupts
    TIMER32_1->LOAD = TIMER32_COUNT; //~1ms ---> a 3Mhz
    TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
    NVIC_SetPriority(T32_INT1_IRQn,1);
    NVIC_EnableIRQ(T32_INT1_IRQn);
    __enable_irq();

    return;
}

extern "C"
{
    // - Handle the Timer32 Interrupt
    void T32_INT1_IRQHandler(void)
    {
        TIMER32_1->INTCLR = 0U;
        P1->OUT ^= BIT0; // - Toggle the heart beat indicator (1ms)
        g_SystemTicks++;
        return;
    }
}

