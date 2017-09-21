/* Author: Badilla Esteban (ebadilla10@gmail.com), Mora Berni (bgme171@gmail.com)*/

#include "utils.hpp"
#include "profile.hpp"

/* -----------------------------------------------------
 *                  HELPER FUNCTIONS
 *  ---------------------------------------------------- */

void PROCESS_NEW_SAMPLE(uint32_t * samples_mic)
{
    for (int i = 0; i < 19; i++)
    {
        samples_mic[i] = samples_mic[i + 1];
    }

    if (ADC14Result > 8192)
        samples_mic[19] = ADC14Result - 8192;
    else
        samples_mic[19] = 8192 - ADC14Result;

    uint32_t avg_5sec = AVERAGE_SAMPLES_MIC(samples_mic);
    uint32_t threshold = avg_5sec * (noise_percent + 100) / 100;
    if (samples_mic[19] > threshold & samples_mic[18] > threshold
            & samples_mic[17] > threshold & samples_mic[16] > threshold)
    {
        TURN_ON();
    }
    new_sample = false;
}

uint32_t AVERAGE_SAMPLES_MIC(uint32_t * samples_mic)
{
    uint32_t sum = 0;
    for (int i = 0; i < 20; i++)
    {
        sum += samples_mic[i];
    }
    return sum / 20;
}

void CONFIG_LIGHT_SENSOR(void)
{
    /* Initialize I2C communication */
    /* Select I2C function for I2C_SCL, CLK at P6.5
     * and I2C_SDA, data at P6.4 */
    Init_I2C_GPIO();

    /* Initialize I2C module */
    I2C_init();

    /* Initialize OPT3001 digital ambient light sensor
     * Set slave address */
    OPT3001_init();
}

void CONFIG_MICROPHONE(void)
{
    /* Set P4.3 for Analog input, disabling the I/O circuit */
    P4->SEL0 = BIT3;
    P4->SEL1 = BIT3;
    P4->DIR &= ~BIT3;
}

void CONFIG_ADC14(void)
{
	/* Predivide by 1 */
    ADC14->CTL0 = ADC14_CTL0_PDIV_0 | ADC14_CTL0_SHS_0 | ADC14_CTL0_DIV_7
            |
            ADC14_CTL0_SSEL__MCLK | ADC14_CTL0_SHT0_1 | ADC14_CTL0_ON
            | ADC14_CTL0_SHP;
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_10 | ADC14_MCTLN_VRSEL_0;
    ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_ENC;
    ADC14->IER0 = ADC14_IER0_IE0;
    NVIC_SetPriority(ADC14_IRQn, 1);
    NVIC_EnableIRQ(ADC14_IRQn);
}

void INITIALIZE(void)
{
    init = true;
    P2->DIR |= BIT0 | BIT1 | BIT2;
    P2->OUT &= 0xF8;

    T32_INIT2_CONFIG(init);
    while (blink_counter > 0)
    {
        /* To blink three times */
    }
    TIMER32_2->CONTROL = 0; /* Disable the timer for saving energy */
    NVIC_DisableIRQ(T32_INT2_IRQn);
}

void TURN_ON()
{
    unsigned long int lux = 0;
    /* Obtain lux value from OPT3001 */
    lux = OPT3001_getLux();
    ;
    if (lux < min_lux)
    {
        P2->OUT |= lights;
        T32_INT1_INIT();
    }
}

void BUTTON_TOGGLE(void)
{
    if (P2->OUT & 0x7)
    {/* Led is currently ON */
        P2->OUT &= 0xF8;
        TIMER32_1->CONTROL = 0; /* turn off the timer1*/
        NVIC_DisableIRQ(T32_INT1_IRQn);
    }
    else
    {/* Led is currently OFF */
        P2->OUT |= lights;
        T32_INT1_INIT();
    }
}

void SET_CONFIG_PORT2()
{
    P2->DIR |= lights;
}

void T32_INIT2_CONFIG(bool init)
{
    if (init)
    {
        TIMER32_2->LOAD = 0x0016E360; /* ~500ms ---> a 3Mhz */
    }
    else
    {
        TIMER32_2->LOAD = 0x000B71B0; /* ~250ms ---> a 3Mhz */
    }
    TIMER32_2->CONTROL =
    TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE
            | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
    NVIC_SetPriority(T32_INT2_IRQn, 1);
    NVIC_EnableIRQ(T32_INT2_IRQn);
}

void T32_INT1_INIT(void)
{
    TIMER32_1->CONTROL = 0; /* turn off the timer, it would be used to reset the count */
    TIMER32_1->LOAD = 0x02AEA540; /* ~15s ---> a 3Mhz */
    TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0
            | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE
            | TIMER32_CONTROL_ONESHOT;
    NVIC_SetPriority(T32_INT1_IRQn, 1);
    NVIC_EnableIRQ(T32_INT1_IRQn);
}

void BUTTON_CONFIG()
{
    /* Configuring P5.1 (switch) as an input and enabling interrupts */
    // P5.1 as input pin default. P5DIR-> XXXX XX0X
    P5->REN = BIT1; // P5.1 PullUp-PullDown resistor
    P5->OUT = BIT1; //Input with pullup resistor
    P5->IFG &= 0xF2; //Clear the interrupt flag
    P5->IE |= BIT1; //Enable the interrupt flag
    P5->IES |= BIT1; //Interrupt high to low transition
    //Set Port5 interrupt
    NVIC_SetPriority(PORT5_IRQn, 1);
    NVIC_EnableIRQ(PORT5_IRQn);
}

void DEBOUNCE(void)
{
    do
    {
        for (int i = 0; i < 3000; i++)
        {
        }
    }
    while ((P5->IN & 0x02) == 0x00);
}

