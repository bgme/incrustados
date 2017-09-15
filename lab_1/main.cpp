/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
/******************************************************************************
 * MSP432 Empty Project
 *
 * Description: An empty project that uses DriverLib
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST               |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 * Author: Badilla Esteban, Mora Berni
*******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include <HAL_I2C.h>
#include <HAL_OPT3001.h>
/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <vector>

#define ON true

uint8_t lights 	= 4; /* 1:red 2:green 4:blue */
uint32_t blink_counter = 0U;
uint32_t ADC14Result = 0U;
bool init = true;

//Headers
void initialize(void);
void turn_on(uint8_t lights);
void set_config_outport(uint8_t lights);
void adc14_config(void);
void T32_INIT2_CONFIG(bool init);
void T32_INT1_INIT(void);
void BUTTON_CONFIG();
void button_toggle(void);

using namespace std;
int main(void)
{

	vector <uint16_t> samples;
	samples.push_back(0x0012); /* Add an element at the end*/

	uint16_t hola = samples.at(0);
	samples.erase(samples.begin()); /* Erase the first element*/

	samples.size();

    /* Halting WDT and disabling master interrupts */
    WDTCTL = WDTPW | WDTHOLD;                    /* Stop watchdog timer */

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);

    /* Initialize I2C communication */
    Init_I2C_GPIO();
    I2C_init();
    /* Initialize OPT3001 digital ambient light sensor */
    OPT3001_init();

    /* Ready to use */
    initialize(); /* Blink 3 times at the beginning. SRS-002 */

    set_config_outport(lights);

    turn_on(lights); /* Check at the beginning the intensity of light. SRS-003 */

    T32_INIT2_CONFIG(init);

//    adc14_config();
    BUTTON_CONFIG();
    while(1)
    {
        
    }
}



void initialize(void)
{
	init = true;
	P2->DIR |= BIT0|BIT1|BIT2;
	P2->OUT &= 0xF8;

	T32_INIT2_CONFIG(init);
	while (blink_counter < 6) {
		/* To blink three times */
	}
	TIMER32_2->CONTROL = 0; /* Disable the timer for saving energy */
	NVIC_DisableIRQ(T32_INT2_IRQn);
	init = false;
}



void turn_on(uint8_t lights) {
	unsigned long int lux = 0;
	/* Obtain lux value from OPT3001 */
	lux = OPT3001_getLux();
	if (lux < 15) {
		P2->OUT |= lights;
		T32_INT1_INIT();
	}

}

void button_toggle(void) {
	if (P2->OUT & 0x7) {/* Led is currently ON */
		P2->OUT &= 0xF8;
		TIMER32_1->CONTROL = 0; /* turn off the timer1*/
		NVIC_DisableIRQ(T32_INT1_IRQn);
	} else {/* Led is currently OFF */
		P2->OUT |= lights;
		T32_INT1_INIT();
	}
}

void set_config_outport(uint8_t lights)
{
	P2->DIR |= lights;
}

void adc14_config(void){
       P1->DIR = BIT0;
       P1->OUT = BIT0;
       // Set P4.3 for Analog input, disabling the I/O circuit.
       P4->SEL0 = BIT3;
       P4->SEL1 = BIT3;
       P4->DIR &= ~BIT3;

       //TIMER32_1->LOAD = 0x00B71B00; //~0.5s ---> a 48Mhz
       TIMER32_1->LOAD = 0x0000BB80; //~0.5s ---> a 48Mhz
       TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
       NVIC_SetPriority(T32_INT1_IRQn,1);
       NVIC_EnableIRQ(T32_INT1_IRQn);

       ADC14->CTL0 = ADC14_CTL0_PDIV_0 | ADC14_CTL0_SHS_0 | ADC14_CTL0_DIV_7 |
                     ADC14_CTL0_SSEL__MCLK | ADC14_CTL0_SHT0_1 | ADC14_CTL0_ON
                     | ADC14_CTL0_SHP;
       ADC14->MCTL[0] = ADC14_MCTLN_INCH_10 | ADC14_MCTLN_VRSEL_0;
       ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_ENC;
       ADC14->IER0 = ADC14_IER0_IE0;
       NVIC_SetPriority(ADC14_IRQn,1);
       NVIC_EnableIRQ(ADC14_IRQn);
}


void T32_INIT2_CONFIG(bool init){
	if(init){
		TIMER32_2->LOAD = 0x0016E360; /* ~500ms ---> a 3Mhz */
	}else{
		TIMER32_2->LOAD = 0x000B71B0; /* ~250ms ---> a 3Mhz */
	}
	TIMER32_2->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
	NVIC_SetPriority(T32_INT2_IRQn, 1);
	NVIC_EnableIRQ(T32_INT2_IRQn);
}

void T32_INT1_INIT(void){
	TIMER32_1->CONTROL = 0; /* turn off the timer, it would be used to reset the count */
	TIMER32_1->LOAD = 0x055D4A80; /* ~30s ---> a 3Mhz */
	TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE | TIMER32_CONTROL_ONESHOT ;
	NVIC_SetPriority(T32_INT1_IRQn, 1);
	NVIC_EnableIRQ(T32_INT1_IRQn);
}

void BUTTON_CONFIG(){
    /* Configuring P1.1 (switch) as input */
	//P1->DIR &= 0xFD;
	/* FIXME: pasar a registros */

    /* Configuring P5.1 as an input and enabling interrupts */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN1);
    MAP_Interrupt_enableInterrupt(INT_PORT5);
}


extern "C"
{
    void T32_INT1_IRQHandler(void)
    {
        __disable_irq();
        TIMER32_1->INTCLR = 0U;
        P2->OUT &= 0xF8;
        __enable_irq();
        return;
    }

    void ADC14_IRQHandler(void)
    {
        __disable_irq();
		ADC14Result = ADC14->MEM[0];
		ADC14->CLRIFGR0 = ADC14_CLRIFGR0_CLRIFG0;
        __enable_irq();
        return;
    }

void T32_INT2_IRQHandler(void) {
	__disable_irq();
	TIMER32_2->INTCLR = 0U;
	if (init) {
		P2->OUT ^= BIT0|BIT1|BIT2;
		blink_counter++;
	} else {
        /* P1->OUT ^= BIT0; */
        ADC14->CTL0 |= ADC14_CTL0_SC; /* Start */
	}
	__enable_irq();
	return;
}

/* GPIO ISR */
void PORT5_IRQHandler(void) {
	__disable_irq();
	uint32_t status = 0;
	status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);
	button_toggle();
	__enable_irq();
	return;
}

}

