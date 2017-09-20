/* Author: Badilla Esteban (ebadilla10@gmail.com), Mora Berni (bgme171@gmail.com)*/

#include "utils.hpp"
#include "profile.hpp"

uint8_t blink_counter = 6;
int ADC14Result = 0U;
bool init = true;
bool new_sample;

int main(void)
{
    /* Initializes the samples with a value approx at
     * the noise from the micro */
    uint32_t samples_mic[] = { [0 ... 19] = 100 };

    /* Halting WDT and disabling master interrupts */
    WDTCTL = WDTPW | WDTHOLD; /* Stop watchdog timer */

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);

    /*Config light sensor */
    CONFIG_LIGHT_SENSOR();

    /* Ready to use
     * Blink 3 times at the beginning. SRS-002 */
    INITIALIZE();

    /* Enables the output according to 1 2 or 3 lights */
    SET_CONFIG_PORT2();

    /* Check at the beginning the intensity of light. SRS-003 */
    TURN_ON();

    /* Configure the timer32_1 for 0.25 s interruptions*/
    init = false;
    T32_INIT2_CONFIG(init);

    /* Config the button for interruptions */
    BUTTON_CONFIG();

    CONFIG_ADC14();

    CONFIG_MICROPHONE();

    while (1)
    {
        if (new_sample)
            PROCESS_NEW_SAMPLE(samples_mic);
    }
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
    new_sample = true;
    __enable_irq();
    return;
}

void T32_INT2_IRQHandler(void)
{
    __disable_irq();
    TIMER32_2->INTCLR = 0U;
    if (init)
    {
        P2->OUT ^= BIT0 | BIT1 | BIT2;
        blink_counter--;
    }
    else
    {
        ADC14->CTL0 |= ADC14_CTL0_SC; /* Start */
    }
    __enable_irq();
    return;
}

/* GPIO ISR */
void PORT5_IRQHandler(void)
{
    __disable_irq();
    DEBOUNCE();
    uint32_t status = 0;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);
    BUTTON_TOGGLE();
    __enable_irq();
    return;
}

}
