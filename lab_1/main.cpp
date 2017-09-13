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
 * Author: 
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
#ifdef _WIN32
#include <Windows.h> //For using sleep function, I think.
#endif


int lux 	= 0;
int lights 	= 0;
int on_off 	= 0;


//Headers
void initialize(int lights);
void turn_on_off(int lights, int on_off);
void set_config_outport(int lights);


int main(void)
{
    /* Halting WDT and disabling master interrupts */
    WDTCTL = WDTPW | WDTHOLD;                    /* Stop watchdog timer */

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);

    /* Initialize I2C communication */
    Init_I2C_GPIO();
    I2C_init();
    /* Initialize OPT3001 digital ambient light sensor */
    OPT3001_init();

    /* Port 2 Out */
    set_config_outport(lights);

    while(1)
    {
        
    }
}


void initialize(int lights)
{
	int i;
	for (i=0; i<3; i++){
		switch(lights) {
		    case 1:  P2->OUT = BIT0;
            case 2:  P2->OUT = BIT0|BIT1;
		    case 3:  P2->OUT = BIT0|BIT1|BIT2;
		    default: P2->OUT = BIT0;
		}
	    //Sleep(250);
	    P2->OUT &= 0xF8;
	    //sleep(250);
	}
}

void turn_on_off(int lights, int on_off)
{
    unsigned long int lux = 0;
	switch(on_off) {
		case 0:
			P2->OUT &= 0xF8;
		default:
			/* Obtain lux value from OPT3001 */
			lux = OPT3001_getLux();
			if(lux < 15){
				switch(lights) {
				case 1:  P2->OUT = BIT0;
				case 2:  P2->OUT = BIT0|BIT1;
				case 3:  P2->OUT = BIT0|BIT1|BIT2;
				default: P2->OUT = BIT0;
				}
			}
	}
}

void set_config_outport(int lights)
{
	switch(lights) {
		case 3:  P2->DIR = BIT0|BIT1|BIT2;
		case 2:  P2->DIR = BIT0|BIT1;
		default: P2->DIR = BIT0;
	}
}

