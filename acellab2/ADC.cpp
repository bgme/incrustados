/*
 * ADC.cpp
 *
 *  Created on: 24 oct. 2017
 *      Author: berni
 */

#include <ADC.hpp>

ADC::ADC(uint16_t i_BITN)
{

}

ADC::~ADC()
{
    // TODO Auto-generated destructor stub
}

uint8_t ADC::run()
{
    uint8_t status = NO_ERR;

    /* TODO: send a message to DRAW */
    /* Store ADC14 conversion results */
    *(this->ptr_MailBox) = ADC14_getResult(ADC_MEM0);
    *(this->ptr_MailBox+1)  = ADC14_getResult(ADC_MEM1);
    *(this->ptr_MailBox+2)  = ADC14_getResult(ADC_MEM2);

    *(this->run_flag + 1) = true; // Activate the flag for running the next task
    return status;
}

uint8_t ADC::setup()
{
    uint8_t status = NO_ERR;

    /* Configures Pin 4.0, 4.2, and 6.1 as ADC input */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8) */
    MAP_ADC14_enableModule();
   // MAP_ADC14_initModule(ADC_CLOCKSOURCE_ACLK, ADC_PREDIVIDER_64, ADC_DIVIDER_1,0);
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8,0);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM2 (A11, A13, A14)  with no repeat)
     * with internal 2.5v reference */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
    ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                        ADC_INPUT_A14,
                                        ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM1,
    ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                        ADC_INPUT_A13,
                                        ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM2,
    ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                        ADC_INPUT_A11,
                                        ADC_NONDIFFERENTIAL_INPUTS);

    /* Enabling the interrupt when a conversion on channel 2 (end of sequence)
     *  is complete and enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT2);

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    return status;
}

uint8_t ADC::getMessage()
{
    uint8_t status = NO_ERR;

    return status;
}
uint8_t ADC::putMessage()
{
    uint8_t status = NO_ERR;

    return status;
}
