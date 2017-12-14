/*
 * snake_functions.h
 *
 *  Created on: 14 dic. 2017
 *      Author: berni
 */

#ifndef SNAKE_FUNCTIONS_H_
#define SNAKE_FUNCTIONS_H_

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <stdlib.h>

/* amount of levels */
#define LEVELS 5

#define INITIAL_SNAKE_SIZE 2

/* RGB value for painting green a pixel in the LCD */
#define GREEN 0x07E0

/* RGB value for painting red a pixel in the LCD */
#define RED 0xF800

/* RGB value for painting red a pixel in the LCD */
#define BLACK 0x000

/* The LCD screen is divided in a 32x32 logical grid */
#define SCREEN_LOGICAL_SIZE 32

/* Amount of clock cycles for a millisecond with a clock frequency of 48MHz*/
#define TICKS_PER_MILLISECOND 48000

/* The hardest level have a new frame every 25ms */
#define FASTEST_LEVEL_FRAME_MILLISECOND_TIME 25

/* Ticks for a 100ms period of time with a clock of 187.5 KHz */
#define ONE_HUNDREAD_MILLISECONDS_TICKS 18750

void draw_display_initial_screen(uint8_t level);
void draw_text_initial_screen(uint8_t level);
void BUTTONS_CONFIG(void);

/* Get the current direction with the result of the last ADC conversion */
void RECALCULATE_DIRECTION(void);

/* Initialize the variables of a game
 * snake size= 2
 * snake position = random
 * movement direction = random
 * food position = random */
void GAME_INIT();

/* Draw a 4x4 pixel of a specific colour */
void DRAW_4x4_LCD_PIXEL(uint8_t x, uint8_t y, uint16_t colour);
void DRAW_SNAKE();
bool CHECK_VALID_FEED(uint8_t x, uint8_t y);
void GENERATE_FEED();
bool GAME_OVER();
void INIT_GRAPHIC_MODULE(void);
void INIT_ADC_JOYSTICK_MODULE(void);
void T32_INT1_INIT(void);
void T32_INT2_INIT(void);

enum DIRECTIONS
{
    RIGHT,
    LEFT,
    UP,
    DOWN,
    DIRECTIONS_MAX
};

// Global variables

/* Graphic library context: Para usar la LCD para texto */
extern Graphics_Context g_sContext;

/* Graphic library display: Para usar la LCD para figuras */
extern Graphics_Display g_sDisplay;

/* ADC results buffer */
extern uint16_t resultsBuffer[];

extern Graphics_Rectangle measures_rect;

/* variable for tracking the selected level of the game */
extern uint8_t level;

/* to storage the current storage of the player */
extern int score;

/* storages the maximum score reached for a player*/
extern uint8_t max_score;

/* storages the current direction of movement */
extern uint8_t direction;

/* array whit the coordinates of all of the parts of the snake, necessary for checking colitions with itself */
extern int snake_array[];

/* array to storage the x and t coordinates of the food */
extern uint8_t feed_position[];

/* indicates the size of the snake */
extern int snake_size;

#endif /* SNAKE_FUNCTIONS_H_ */
