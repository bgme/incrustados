/*
 Author: Badilla A, Esteban
 */

#include "snake_functions.h"

// Functions
void PORT1_IRQHandler(void);

// Variables for using in ISR

/* variable used as a register flag, 1 bit for each button indicating in which bouncing period we are*/
uint8_t debounce_flags = 0;

/* counter of ticks when the food has to change place */
uint16_t feed_expiration_counter = 0;

/* counter for doing the bouncing period of the start button larger */
uint8_t debounce_p1_counter = 0;

/* bool to select if the start button press is a start or a stop gaming */
bool first_time_bj = true;

/* bool indicating that a button was pressed and it have to wait for the bouncing period */
bool bounce_p1 = false; // start button
bool bounce_p3 = false; // level down button
bool bounce_p5 = false; // level up button

/*
 * Main function
 */
int main(void)
{
    /* Halting WDT and disabling master interrupts */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);

    INIT_GRAPHIC_MODULE();

    /* Set the conversion for the joystick */
    INIT_ADC_JOYSTICK_MODULE();

    /* Set the interruption for the buttons */
    BUTTONS_CONFIG();

    /* Set the initial window (configuration) */
    draw_text_initial_screen(level);
    draw_display_initial_screen(level);

    while (1)
    {
        MAP_PCM_gotoLPM0();
    }
}

/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    uint64_t status;
    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    /* ADC_MEM1 conversion completed */
    if (status & ADC_INT1)
    {
        /* Store ADC14 conversion results */
        resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
        resultsBuffer[1] = ADC14_getResult(ADC_MEM1);
        RECALCULATE_DIRECTION();
    }
}

/* GPIO ISR */
void PORT3_IRQHandler(void)
{
    __disable_irq();

    uint32_t status = 0;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);

    if (!bounce_p3)
    {
        bounce_p3 = true;
        debounce_flags = BIT3;
        T32_INT2_INIT(); /* wait 10ms */
        NVIC_DisableIRQ(PORT5_IRQn);
        if (level == 0){
            level = 4;
        }
        else{
            level--;
        }

        draw_display_initial_screen(level);

    }

    __enable_irq();
    return;
}

/* GPIO ISR, joystick */
void PORT1_IRQHandler(void)
{
    __disable_irq();
    uint32_t status = 0;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (!bounce_p1)
    {
        bounce_p1 = true;
        debounce_flags = BIT1;
        T32_INT2_INIT(); /* wait 10ms */
        if (first_time_bj) {
            bounce_p3 = true;
            bounce_p5 = true;
            first_time_bj = false;
            T32_INT1_INIT();
            GAME_INIT();
        }
        else{
            TIMER32_1->CONTROL = 0; /* turn off the timer, it would be used to reset the count */
            TIMER32_1->INTCLR = 0U;
            bounce_p3 = false;
            bounce_p5 = false;
            first_time_bj = true;
            level = 0;
            Graphics_clearDisplay(&g_sContext);
            draw_display_initial_screen(level);
        }
    }
    __enable_irq();
    return;
}

/* GPIO ISR */
void PORT5_IRQHandler(void)
{
    __disable_irq();
    uint32_t status = 0;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

    if (!bounce_p5)
    {
        bounce_p5 = true;
        debounce_flags = BIT5;
        T32_INT2_INIT(); /* wait 10ms */
        NVIC_DisableIRQ(PORT3_IRQn);
        level++;
        level = level % LEVELS;
        draw_display_initial_screen(level);
    }

    __enable_irq();
    return;
}

void T32_INT1_IRQHandler(void)
{
    __disable_irq();
    TIMER32_1->INTCLR = 0U;

    uint8_t current_tail[2] = { snake_array[2 * snake_size - 2], snake_array[2
            * snake_size - 1] };

    int i = 0;
    for (i = snake_size; i > 1; i--)
    {
        snake_array[2 * i - 2] = snake_array[2 * i - 4];
        snake_array[2 * i - 1] = snake_array[2 * i - 3];
    }

    switch (direction)
    {
    case RIGHT:
        snake_array[0] = (snake_array[0] + 1) % SCREEN_LOGICAL_SIZE;
        break;
    case LEFT:
        snake_array[0] = (snake_array[0]) ? snake_array[0] - 1 : 31;
        break;
    case UP:
        snake_array[1] = (snake_array[1]) ? snake_array[1] - 1 : 31;
        break;
    case DOWN:
        snake_array[1] = (snake_array[1] + 1) % SCREEN_LOGICAL_SIZE;
        ;
        break;
    default:
        break;
    }
    if ((feed_position[0] == snake_array[0])
            && (feed_position[1] == snake_array[1]))
    {
        snake_size++;
        feed_expiration_counter = 0; /* restart the count for feed expiration */
        // Update global variables to screen initial
        score = snake_size - 2;
        if (score > max_score)
            max_score = score;

        snake_array[2 * snake_size - 2] = current_tail[0];
        snake_array[2 * snake_size - 1] = current_tail[1];
        DRAW_SNAKE();
        GENERATE_FEED();
        DRAW_4x4_LCD_PIXEL(feed_position[0], feed_position[1],RED);
    }
    else
    {
        DRAW_4x4_LCD_PIXEL(current_tail[0], current_tail[1],GREEN);
        DRAW_SNAKE();
    }

    if (GAME_OVER())
        PORT1_IRQHandler();

    __enable_irq();
    return;
}

void T32_INT2_IRQHandler(void)
{
    __disable_irq();
    TIMER32_2->INTCLR = 0U;
    switch (debounce_flags)
    {
    case BIT3 :
        bounce_p3 = false;
        debounce_flags = 0;
        NVIC_EnableIRQ(PORT5_IRQn);
        break;
    case BIT1 :

        if (debounce_p1_counter >= 2){
            bounce_p1 = false;
            debounce_p1_counter = 0;
            debounce_flags = 0;
        }
        else {
            debounce_p1_counter++;
        }

        break;
    case BIT5 :
        bounce_p5 = false;
        debounce_flags = 0;
        NVIC_EnableIRQ(PORT3_IRQn);
        break;
    }
    if (!first_time_bj)
    {
        feed_expiration_counter++;
        if (feed_expiration_counter >= 200 / (level + 1))
        {
            feed_expiration_counter = 0;
            DRAW_4x4_LCD_PIXEL(feed_position[0], feed_position[1],GREEN);
            GENERATE_FEED();
            DRAW_4x4_LCD_PIXEL(feed_position[0], feed_position[1],RED);
        }
    }

    __enable_irq();
    return;
}
