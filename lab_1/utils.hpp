/* Author: Badilla Esteban (ebadilla10@gmail.com), Mora Berni (bgme171@gmail.com)*/

#ifndef UTILS_HPP_
#define UTILS_HPP_

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include <HAL_I2C.h>
#include <HAL_OPT3001.h>
/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <iostream>
#include <stdio.h>

using namespace std;

/* Global variables */
extern uint8_t blink_counter;
extern int ADC14Result;
extern bool init;
extern bool new_sample;

void INITIALIZE(void);
void TURN_ON();
void SET_CONFIG_PORT2();
void T32_INIT2_CONFIG(bool init);
void T32_INT1_INIT(void);
void BUTTON_CONFIG();
void BUTTON_TOGGLE(void);
void CONFIG_LIGHT_SENSOR(void);
void DEBOUNCE(void);
void CONFIG_ADC14(void);
void CONFIG_MICROPHONE(void);
void PROCESS_NEW_SAMPLE(uint32_t * samples_mic);
uint32_t AVERAGE_SAMPLES_MIC(uint32_t * samples_mic);

#endif /* UTILS_HPP_ */
