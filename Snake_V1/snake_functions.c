/*
 * snake_functions.c
 *
 *  Created on: 14 dic. 2017
 *      Author: berni
 */

#include "snake_functions.h"


// Global variables

/* Graphic library context: Para usar la LCD para texto */
Graphics_Context g_sContext;

/* Graphic library display: Para usar la LCD para figuras */
Graphics_Display g_sDisplay;

/* ADC results buffer */
uint16_t resultsBuffer[2];

Graphics_Rectangle measures_rect = { 0 };

/* variable for tracking the selected level of the game */
uint8_t level = 0;

/* to storage the current storage of the player */
int score = 0;

/* storages the maximum score reached for a player*/
uint8_t max_score = 0;

/* storages the current direction of movement */
uint8_t direction = 0;

/* array whit the coordinates of all of the parts of the snake, necessary for checking colitions with itself */
int snake_array[2048] = { [0 ... 2047] = 32 };

/* array to storage the x and t coordinates of the food */
uint8_t feed_position[2];

/* indicates the size of the snake */
int snake_size = 2;

void draw_display_initial_screen(uint8_t level)
{
    Graphics_initContext(&g_sContext, &g_sDisplay,
                         &g_sCrystalfontz128x128_funcs);

    measures_rect.xMin = 13;
    measures_rect.yMin = 13;
    measures_rect.xMax = 114;
    measures_rect.yMax = 13 + 15 * (4 - level) - 1;
    Graphics_fillRectangleOnDisplay(&g_sDisplay, &measures_rect, 0xffff);
    measures_rect.yMin = 13 + 15 * (4 - level);
    measures_rect.yMax = 87;
    Graphics_fillRectangleOnDisplay(&g_sDisplay, &measures_rect, 0x0000);
    draw_text_initial_screen(level);
}

void draw_text_initial_screen(uint8_t level)
{
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128,
                         &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_ORANGE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    char string[15];
    sprintf(string, "Level: %1d", level + 1);
    Graphics_drawStringCentered(&g_sContext, (int8_t *) string, 8, 64, 93,
    OPAQUE_TEXT);
    sprintf(string, "Score: %2d", score);
    Graphics_drawStringCentered(&g_sContext, (int8_t *) string, 9, 64, 105,
    OPAQUE_TEXT);
    sprintf(string, "Max Score: %2d", max_score);
    Graphics_drawStringCentered(&g_sContext, (int8_t *) string, 13, 64, 117,
    OPAQUE_TEXT);

}

void T32_INT1_INIT(void)
{
//    level = level % 5;
    TIMER32_1->CONTROL = 0; /* turn off the timer, it would be used to reset the count */
    TIMER32_1->LOAD = FASTEST_LEVEL_FRAME_MILLISECOND_TIME * TICKS_PER_MILLISECOND + (4 - level) * (FASTEST_LEVEL_FRAME_MILLISECOND_TIME * TICKS_PER_MILLISECOND / 2); /* ~0.5s ---> a 48 MHz */
    TIMER32_1->CONTROL =
            TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0
                    | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE
                    | TIMER32_CONTROL_ENABLE;
    NVIC_SetPriority(T32_INT1_IRQn, 1);
    NVIC_EnableIRQ(T32_INT1_IRQn);
}

/* timer for debouncing */
void T32_INT2_INIT(void)
{
    TIMER32_2->CONTROL = 0; /* turn off the timer, it would be used to reset the count */
    TIMER32_2->LOAD = ONE_HUNDREAD_MILLISECONDS_TICKS; /* ~100ms ---> a 187.5 KHz */
    TIMER32_2->CONTROL =
            TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_2
                    | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE
                    | TIMER32_CONTROL_ENABLE;
    NVIC_SetPriority(T32_INT2_IRQn, 1);
    NVIC_EnableIRQ(T32_INT2_IRQn);
}

void INIT_GRAPHIC_MODULE(void){
    /* Initializes display */
       Crystalfontz128x128_Init();

       /* Set default screen orientation */
       Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

       /* Initializes graphics context */
       Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128,
                            &g_sCrystalfontz128x128_funcs);
       Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
       Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_ORANGE);
       GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
       Graphics_clearDisplay(&g_sContext);
}


void INIT_ADC_JOYSTICK_MODULE(void){
    /* Configures Pin 6.0 and 4.4 as ADC input */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ACLK/32) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ACLK, ADC_PREDIVIDER_32, ADC_DIVIDER_1, 0);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
     * with internal 2.5v reference */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
    ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                        ADC_INPUT_A15,
                                        ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM1,
                                        ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                        ADC_INPUT_A9,
                                        ADC_NONDIFFERENTIAL_INPUTS);

    /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
     *  is complete and enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT1);

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();
}

/* Configure the level up/down and start button as Key Wakeups interruptions */
void BUTTONS_CONFIG(void)
{
    /* Config the Level Up button */
    /* Configuring P5.1 (switch) as an input and enabling interrupts */
    /* P5.1 as input pin default. P5DIR-> XXXX XX0X */
    P5->REN |= BIT1; // P5.1 PullUp-PullDown resistor
    P5->OUT |= BIT1; //Input with pullup resistor
    P5->IFG &= 0xFD; //Clear the interrupt flag
    P5->IE |= BIT1; //Enable the interrupt flag
    P5->IES |= BIT1; //Interrupt high to low transition
    /* Set Port5 interrupt */
    NVIC_SetPriority(PORT5_IRQn, 1);
    NVIC_EnableIRQ(PORT5_IRQn);

    /* Config the Level Down button */
    /* Configuring P3.5 (switch) as an input and enabling interrupts */
    /* P3.5 as input pin default. P3DIR-> XX0X XXXX */
    P3->REN |= BIT5; // P3.5 PullUp-PullDown resistor
    P3->OUT |= BIT5; //Input with pullup resistor
    P3->IFG &= 0xDF; //Clear the interrupt flag
    P3->IE |= BIT5; //Enable the interrupt flag
    P3->IES |= BIT5; //Interrupt high to low transition
    //Set Port3 interrupt
    NVIC_SetPriority(PORT3_IRQn, 1);
    NVIC_EnableIRQ(PORT3_IRQn);

    /* Config the Start button */
    /* Configuring P1.4 (switch) as an input and enabling interrupts */
    /* P4.1 as input pin default. P4DIR-> XXXX XX0X */
    P1->REN |= BIT4; // P1.4 PullUp-PullDown resistor
    P1->OUT |= BIT4; //Input with pullup resistor
    P1->IFG &= 0xEF; //Clear the interrupt flag
    P1->IE |= BIT4; //Enable the interrupt flag
    P1->IES |= BIT4; //Interrupt high to low transition
    //Set Port1 interrupt
    NVIC_SetPriority(PORT1_IRQn, 1);
    NVIC_EnableIRQ(PORT1_IRQn);
}

/* Get the current direction of movement from the result of the joystick (after the ADC) */
void RECALCULATE_DIRECTION(void)
{
    if ((resultsBuffer[0] > 14336) && (direction != LEFT))
        direction = RIGHT;
    else if ((resultsBuffer[0] < 2048) && (direction != RIGHT))
        direction = LEFT;
    else if ((resultsBuffer[1] > 14336) && (direction != DOWN))
        direction = UP;
    else if ((resultsBuffer[1] < 2048) && (direction != UP)) // Not only else because when the button is free, direction do not should change
        direction = DOWN;
}

void GAME_INIT()
{
    snake_size = INITIAL_SNAKE_SIZE;
    score = 0;
    measures_rect.xMin = 0;
    measures_rect.yMin = 0;
    measures_rect.xMax = 127;
    measures_rect.yMax = 127;
    Graphics_fillRectangleOnDisplay(&g_sDisplay, &measures_rect, 0x07E0);
    int x_init = rand() % SCREEN_LOGICAL_SIZE; // returns a pseudo-random integer between 0 and RAND_MAX
    int y_init = rand() % SCREEN_LOGICAL_SIZE; // returns a pseudo-random integer between 0 and RAND_MAX
    direction = rand() % DIRECTIONS_MAX; // returns a pseudo-random integer between 0 and RAND_MAX

    snake_array[0] = x_init;
    snake_array[1] = y_init;

    switch (direction)
    {
    case RIGHT:
        snake_array[2] = (x_init + 1) % SCREEN_LOGICAL_SIZE;
        snake_array[3] = y_init;
        break;       // and exits the switch
    case LEFT:
        snake_array[2] = (x_init) ? ((x_init - 1) % SCREEN_LOGICAL_SIZE) : 31;
        snake_array[3] = y_init;
        break;       // and exits the switch
    case UP:
        snake_array[2] = x_init;
        snake_array[3] = (y_init) ? ((y_init - 1) % SCREEN_LOGICAL_SIZE) : 31;
        break;       // and exits the switch
    case DOWN:
        snake_array[2] = x_init;
        snake_array[3] = (y_init + 1) % 32;
        break;       // and exits the switch
    default:
        break;
    }

    DRAW_SNAKE();
    GENERATE_FEED();
    DRAW_4x4_LCD_PIXEL(feed_position[0], feed_position[1],RED);
}

void GENERATE_FEED()
{
    feed_position[0] = rand() % SCREEN_LOGICAL_SIZE;
    feed_position[1] = rand() % SCREEN_LOGICAL_SIZE;

    while (!CHECK_VALID_FEED(feed_position[0], feed_position[1]))
    {
        feed_position[0] = rand() % SCREEN_LOGICAL_SIZE;
        feed_position[1] = rand() % SCREEN_LOGICAL_SIZE;
    }
}

bool CHECK_VALID_FEED(uint8_t x, uint8_t y)
{
    int i = 0;
    for (i = 0; i < snake_size; i++)
    {
        if (snake_array[2 * i] == x && (snake_array[2 * i + 1] == y))
            return false;
    }
    return true;
}

void DRAW_SNAKE()
{
    int i = 0;
    for (i = 0; i < snake_size; i++)
    {
        DRAW_4x4_LCD_PIXEL(snake_array[2 * i], snake_array[2 * i + 1], BLACK);
    }
}

bool GAME_OVER()
{
    int i = 0;
    for (i = 1; i < snake_size; i++)
    {
        if ((snake_array[2 * i] == snake_array[0])
                && (snake_array[2 * i + 1] == snake_array[1]))
            return true;
    }
    return false;
}

void DRAW_4x4_LCD_PIXEL(uint8_t x, uint8_t y, uint16_t colour){
    measures_rect.xMin = x * 4;
       measures_rect.yMin = y * 4;
       measures_rect.xMax = x * 4 + 3;
       measures_rect.yMax = y * 4 + 3;
       Graphics_fillRectangleOnDisplay(&g_sDisplay, &measures_rect, colour);
}
