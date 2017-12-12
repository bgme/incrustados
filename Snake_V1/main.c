/*
Author: Badilla A, Esteban
*/
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <stdlib.h>

/* Graphic library context: Para usar la LCD para texto */
Graphics_Context g_sContext;

/* Graphic library display: Para usar la LCD para figuras */
Graphics_Display g_sDisplay;

/* ADC results buffer */
 static uint16_t resultsBuffer[2];


Graphics_Rectangle measures_rect = {0};

// Functions
void draw_display_initial_screen(uint8_t level);
void draw_text_initial_screen(uint8_t level);
void BUTTONS_CONFIG(void);
void DEBOUNCE_3(void);
void DEBOUNCE_4(void);
void DEBOUNCE_5(void);
void RECALCULATE_DIRECTION(void);
void GAME_INIT();
void DRAW_SNAKE_PIXEL(uint8_t x, uint8_t y);
void DRAW_FEED_PIXEL(uint8_t x, uint8_t y);
void DRAW_SNAKE();
void DRAW_ERASE_TAIL(uint8_t x, uint8_t y);
bool CHECK_VALID_FEED(uint8_t x, uint8_t y);
void GENERATE_FEED();
bool GAME_OVER();
void PORT4_IRQHandler(void);

// Global variables
uint8_t level = 0;
int score = 0;
uint8_t max_score = 0;

uint8_t direction = 0;
bool first_time_bj = true;
int snake_array[2048] = { [0 ... 2047] = 32 };
uint8_t feed_position[2];
int snake_size = 2;
uint8_t debounce_flags = 0;
uint16_t feed_expiration_counter = 0;
uint8_t debounce_p1_counter = 0;

bool bounce_p1 = false;
bool bounce_p3 = false;
bool bounce_p5 = false;

enum DIRECTIONS {
    RIGHT,
    LEFT,
    UP,
    DOWN,
    DIRECTIONS_MAX
};

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

    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_ORANGE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);

    /* Configures Pin 6.0 and 4.4 as ADC input */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
         * with internal 2.5v reference */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

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
    BUTTONS_CONFIG();
    draw_text_initial_screen(level);
    draw_display_initial_screen(level);
    //
    while(1)
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
    if(status & ADC_INT1)
    {
        /* Store ADC14 conversion results */
        resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
        resultsBuffer[1] = ADC14_getResult(ADC_MEM1);
        RECALCULATE_DIRECTION();
    }
}

void draw_display_initial_screen(uint8_t level)
{
    level = level % 5;
    Graphics_initContext(&g_sContext, &g_sDisplay, &g_sCrystalfontz128x128_funcs);

    measures_rect.xMin = 13;
    measures_rect.yMin = 13;
    measures_rect.xMax = 114;
    measures_rect.yMax = 13+15*(4-level)-1;
    Graphics_fillRectangleOnDisplay(&g_sDisplay,
                                    &measures_rect,
                                    0xffff);
    measures_rect.yMin = 13+15*(4-level);
    measures_rect.yMax = 87;
    Graphics_fillRectangleOnDisplay(&g_sDisplay,
                                    &measures_rect,
                                    0x0000);
    draw_text_initial_screen(level);
}

void draw_text_initial_screen(uint8_t level)
{
    level = level % 5;
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_ORANGE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    char string[15];
    sprintf(string, "Level: %1d", level);
    Graphics_drawStringCentered(&g_sContext,
                                     (int8_t *)string,
                                     8,
                                     64,
                                     93,
                                    OPAQUE_TEXT);
    sprintf(string, "Score: %2d", score);
    Graphics_drawStringCentered(&g_sContext,
                                     (int8_t *)string,
                                     9,
                                     64,
                                     105,
                                    OPAQUE_TEXT);
    sprintf(string, "Max Score: %2d", max_score);
    Graphics_drawStringCentered(&g_sContext,
                                     (int8_t *)string,
                                     13,
                                     64,
                                     117,
                                    OPAQUE_TEXT);

}

void BUTTONS_CONFIG()
{
    /* Configuring P5.1 (switch) as an input and enabling interrupts */
    // P5.1 as input pin default. P5DIR-> XXXX XX0X
    P5->REN |= BIT1; // P5.1 PullUp-PullDown resistor
    P5->OUT |= BIT1; //Input with pullup resistor
    P5->IFG &= 0xFD; //Clear the interrupt flag
    P5->IE |= BIT1; //Enable the interrupt flag
    P5->IES |= BIT1; //Interrupt high to low transition
    //Set Port5 interrupt
    NVIC_SetPriority(PORT5_IRQn, 1);
    NVIC_EnableIRQ(PORT5_IRQn);

    /* Configuring P3.5 (switch) as an input and enabling interrupts */
     // P3.5 as input pin default. P3DIR-> XX0X XXXX
     P3->REN |= BIT5; // P3.5 PullUp-PullDown resistor
     P3->OUT |= BIT5; //Input with pullup resistor
     P3->IFG &= 0xDF; //Clear the interrupt flag
     P3->IE |= BIT5; //Enable the interrupt flag
     P3->IES |= BIT5; //Interrupt high to low transition
     //Set Port3 interrupt
     NVIC_SetPriority(PORT3_IRQn, 1);
     NVIC_EnableIRQ(PORT3_IRQn);

     /* Configuring P1.4 (switch) as an input and enabling interrupts */
     // P4.1 as input pin default. P4DIR-> XXXX XX0X
     P1->REN |= BIT4; // P1.4 PullUp-PullDown resistor
     P1->OUT |= BIT4; //Input with pullup resistor
     P1->IFG &= 0xEF; //Clear the interrupt flag
     P1->IE |= BIT4; //Enable the interrupt flag
     P1->IES |= BIT4; //Interrupt high to low transition
     //Set Port1 interrupt
     NVIC_SetPriority(PORT1_IRQn, 1);
     NVIC_EnableIRQ(PORT1_IRQn);

}

void RECALCULATE_DIRECTION(void)
{
if((resultsBuffer[0] > 14336) && (direction != LEFT))
    direction = RIGHT;
else if((resultsBuffer[0] < 2048) && (direction != RIGHT))
    direction = LEFT;
else if((resultsBuffer[1] > 14336) && (direction != DOWN))
    direction = UP;
else if((resultsBuffer[1] < 2048) && (direction != UP)) // Not only else because when the button is free, direction do not should change
    direction = DOWN;
}

void GAME_INIT(){
    snake_size = 2;
    score = 0;
    measures_rect.xMin = 0;
    measures_rect.yMin = 0;
    measures_rect.xMax = 127;
    measures_rect.yMax = 127;
    Graphics_fillRectangleOnDisplay(&g_sDisplay,
                                    &measures_rect,
                                    0x07E0);
    int x_init = rand() % 32;                // returns a pseudo-random integer between 0 and RAND_MAX
    int y_init = rand() % 32;               // returns a pseudo-random integer between 0 and RAND_MAX
    direction  = rand() % DIRECTIONS_MAX;   // returns a pseudo-random integer between 0 and RAND_MAX

    snake_array[0] = x_init;
    snake_array[1] = y_init;

    switch(direction) {
        case RIGHT :
                snake_array[2] = (x_init+1) % 32;
                snake_array[3] = y_init;
                break;       // and exits the switch
        case LEFT :
                snake_array[2] = (x_init) ? ((x_init-1)%32 ) : 31;
                snake_array[3] = y_init;
                break;       // and exits the switch
        case UP :
                snake_array[2] = x_init;
                snake_array[3] = (y_init) ? ((y_init-1)%32 ): 31;
                break;       // and exits the switch
        case DOWN :
                snake_array[2] = x_init;
                snake_array[3] = (y_init+1) % 32;
                break;       // and exits the switch
        default:
                break;
    }

    DRAW_SNAKE();
    GENERATE_FEED();
    DRAW_FEED_PIXEL(feed_position[0], feed_position[1]);
}

void GENERATE_FEED(){
    feed_position[0] = rand() % 32;
    feed_position[1] = rand() % 32;

    while (!CHECK_VALID_FEED(feed_position[0], feed_position[1])){
        feed_position[0] = rand() % 32;
        feed_position[1] = rand() % 32;
    }
}

bool CHECK_VALID_FEED(uint8_t x, uint8_t y){
    int i=0;
    for ( i = 0; i < snake_size; i++ ){
        if(snake_array[2*i] == x && (snake_array[2*i+1] == y))
            return false;
    }
    return true;
}

void DRAW_SNAKE(){
    int i=0;
    for ( i = 0; i < snake_size; i++ ){
        DRAW_SNAKE_PIXEL(snake_array[2*i], snake_array[2*i+1]);
    }
}

bool GAME_OVER(){
    int i=0;
    for ( i = 1; i < snake_size; i++ ){
        if ( (snake_array[2*i] == snake_array[0]) && ( snake_array[2*i+1] == snake_array[1]) )
            return true;
    }
    return false;
}

void DRAW_ERASE_TAIL(uint8_t x, uint8_t y){
    measures_rect.xMin = x*4;
    measures_rect.yMin = y*4;
    measures_rect.xMax = x*4 + 3;
    measures_rect.yMax = y*4 + 3;
    Graphics_fillRectangleOnDisplay(&g_sDisplay,
                                    &measures_rect,
                                    0x07E0);
}

void DRAW_FEED_PIXEL(uint8_t x, uint8_t y){
    measures_rect.xMin = x*4;
    measures_rect.yMin = y*4;
    measures_rect.xMax = x*4 + 3;
    measures_rect.yMax = y*4 + 3;
    Graphics_fillRectangleOnDisplay(&g_sDisplay,
                                    &measures_rect,
                                    0xF800);
}

void DRAW_SNAKE_PIXEL(uint8_t x, uint8_t y){
    measures_rect.xMin = x*4;
    measures_rect.yMin = y*4;
    measures_rect.xMax = x*4 + 3;
    measures_rect.yMax = y*4 + 3;
    Graphics_fillRectangleOnDisplay(&g_sDisplay,
                                    &measures_rect,
                                    0x0000);
}

void T32_INT1_INIT(void)
{
    level = level % 5;
    TIMER32_1->CONTROL = 0; /* turn off the timer, it would be used to reset the count */
    TIMER32_1->LOAD = 25*48000 + (4-level)*(25*48000/2); /* ~0.5s ---> a 48 MHz */
    TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0
            | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
    NVIC_SetPriority(T32_INT1_IRQn, 1);
    NVIC_EnableIRQ(T32_INT1_IRQn);
}

/* timer for debounce */
void T32_INT2_INIT(void)
{
    TIMER32_2->CONTROL = 0; /* turn off the timer, it would be used to reset the count */
    TIMER32_2->LOAD = 1875*10; /* ~100ms ---> a 187.5 KHz */
    TIMER32_2->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_2
            | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
    NVIC_SetPriority(T32_INT2_IRQn, 1);
    NVIC_EnableIRQ(T32_INT2_IRQn);
}

void T32_INT1_IRQHandler(void)
{
    __disable_irq();
    TIMER32_1->INTCLR = 0U;

    uint8_t current_tail[2] = {snake_array[2*snake_size-2], snake_array[2*snake_size-1]};


    int i=0;
    for (i=snake_size; i > 1; i--){
        snake_array[2*i-2] = snake_array[2*i-4];
        snake_array[2*i-1] = snake_array[2*i-3];
    }

    switch (direction){
        case RIGHT:
            snake_array[0] = (snake_array[0] + 1) % 32;
            break;
        case LEFT:
            snake_array[0] = (snake_array[0]) ? snake_array[0]-1 : 31;
            break;
        case UP:
            snake_array[1] = (snake_array[1]) ? snake_array[1]-1 : 31;
            break;
        case DOWN:
            snake_array[1] = (snake_array[1] + 1) % 32;;
            break;
        default:
            break;
    }
    if( (feed_position[0] == snake_array[0]) && (feed_position[1] == snake_array[1]) ){
        snake_size++;
        feed_expiration_counter = 0; /* restart the count for feed expiration */
        // Update global variables to screen initial
        score = snake_size-2;
        if (score > max_score) max_score = score;

        snake_array[2*snake_size-2] = current_tail[0];
        snake_array[2*snake_size-1] = current_tail[1];
        DRAW_SNAKE();
        GENERATE_FEED();
        DRAW_FEED_PIXEL(feed_position[0], feed_position[1]);
    }else{
        DRAW_ERASE_TAIL(current_tail[0], current_tail[1]);
        DRAW_SNAKE();
    }


    if (GAME_OVER())
        PORT1_IRQHandler();

    __enable_irq();
    return;
}

/* GPIO ISR */
void PORT3_IRQHandler(void)
{
    __disable_irq();


    uint32_t status = 0;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);

    if(!bounce_p3){
    bounce_p3 = true;
    debounce_flags = BIT3;
    T32_INT2_INIT(); /* wait 10ms */
    if(level==0){
        level = 4;
    }else{
    level --;
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

    if(!bounce_p1){
        bounce_p1 = true;
    debounce_flags = BIT1;
        T32_INT2_INIT(); /* wait 10ms */

    if (first_time_bj){
        NVIC_DisableIRQ(PORT3_IRQn);
        NVIC_DisableIRQ(PORT5_IRQn);
        first_time_bj = false;
        T32_INT1_INIT();
        GAME_INIT();
    }else{
        TIMER32_1->CONTROL = 0; /* turn off the timer, it would be used to reset the count */
        TIMER32_1->INTCLR = 0U;
        NVIC_EnableIRQ(PORT3_IRQn);
        NVIC_EnableIRQ(PORT5_IRQn);
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

    if(!bounce_p5){
        bounce_p5 = true;
    debounce_flags = BIT5;
    T32_INT2_INIT(); /* wait 10ms */

    level ++;
    level = level % 5;
    draw_display_initial_screen(level);

    }

    __enable_irq();
    return;
}

void DEBOUNCE_3(void)
{
    int i;
    do
    {
        for (i = 0; i < 10000; i++)
        {
        }
    }
    while ((P3->IN & 0x20) == 0x00);
}

void DEBOUNCE_4(void)
{
    int i;
    do
    {
        for (i = 0; i < 10000; i++)
        {
        }
    }
    while ((P4->IN & 0x02) == 0x00);
}
void DEBOUNCE_5(void)
{
    int i;
    do
    {
        for (i = 0; i < 10000; i++)
        {
        }
    }
    while ((P5->IN & 0x02) == 0x00);
}

void T32_INT2_IRQHandler(void)
{
    __disable_irq();
    TIMER32_2->INTCLR = 0U;
    switch (debounce_flags){
    case BIT3:
        bounce_p3 = false;
        debounce_flags = 0;
        break;
    case BIT1:

        if(debounce_p1_counter >= 2){
        bounce_p1 = false;
        debounce_p1_counter=0;
        debounce_flags = 0;
        }else{
            debounce_p1_counter++;
        }

        break;
    case BIT5:
        bounce_p5 = false;
        debounce_flags = 0;
        break;
    }
 if(!first_time_bj){
    feed_expiration_counter ++;
    if(feed_expiration_counter >= 200/(level+1)){
        feed_expiration_counter = 0;
        DRAW_ERASE_TAIL(feed_position[0],feed_position[1]);
        GENERATE_FEED();
        DRAW_FEED_PIXEL(feed_position[0], feed_position[1]);
    }}



    __enable_irq();
    return;
}
