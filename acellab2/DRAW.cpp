/*
 * DRAW.cpp
 *
 *  Created on: 24 oct. 2017
 *      Author: berni
 */

#include <DRAW.hpp>

DRAW::DRAW(uint16_t i_BITN)
{
}

DRAW::~DRAW()
{
    // TODO Auto-generated destructor stub
}

uint8_t DRAW::run()
{
    uint8_t status = NO_ERR;

    DRAW::getMessage();

    if (resultsBuffer[1] < 8192)
    {
        Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
        drawHorizon();
    }
    else
    {
        Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_DOWN);
        drawHorizon();
    }
    return status;
}

uint8_t DRAW::setup()
{
    uint8_t status = NO_ERR;
    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sDisplay,
                         &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);

    return status;
}

void DRAW::drawHorizon(void)
{

    int y0 = (128 * (resultsBuffer[2] - 11462)) / (4922-11462);

    int y1 =
    (resultsBuffer[0] < 8192) ?
    (y0 * (resultsBuffer[0] - 6557)) / (8192 - 6557) :
    (-1 * y0 * (resultsBuffer[0] - 9827)) / (9827 - 8192);

    if((ID_function & 0x3) == 0){
    DRAW::rectangle_sky(y1);
    }else if((ID_function & 0x3) == 1){
    DRAW::rectangle_earth(y0, y1);
    }else if((ID_function & 0x3) == 2){
    DRAW::triangle_earth(y0, y1, resultsBuffer[0] );
    }else if((ID_function & 0x3) == 3){
    DRAW::triangle_sky(y0, y1, resultsBuffer[0] );
    }
    ID_function += 1;
}

uint8_t DRAW::getMessage()
{
    uint8_t status = NO_ERR;

    resultsBuffer[0] = *(this->ptr_MailBox + TASK1_ID);
    resultsBuffer[1] = *(this->ptr_MailBox + 256 + TASK1_ID);
    resultsBuffer[2] = *(this->ptr_MailBox + 512 + TASK1_ID);

    return status;
}
uint8_t DRAW::putMessage(uint8_t dst_task_id)
{
    uint8_t status = NO_ERR;

    return status;
}

void DRAW::rectangle_sky(int y1){

    values_rectangle_sky.xMin = 0;
    values_rectangle_sky.xMax = 127;
    values_rectangle_sky.yMin = 0;
    values_rectangle_sky.yMax = y1;

    // Dibujo cielo azul
    Graphics_fillRectangleOnDisplay(&g_sDisplay,
                                    &values_rectangle_sky,
                                    0x001F);

}
void DRAW::rectangle_earth(int y0, int y1){

    values_rectangle_earth.xMin = 0;
    values_rectangle_earth.xMax = 127;
    values_rectangle_earth.yMin = 2 * y0 -y1 ;
    values_rectangle_earth.yMax = 127;
    // Dibujo tierra cafe
    Graphics_fillRectangleOnDisplay(&g_sDisplay, &values_rectangle_earth,
                                        0x0000);
}


void DRAW::triangle_earth(int y0, int y1, uint16_t result)
{
    if (result < 8192)
    {
        for (int i = 0; i < (2 * (y0 - y1)); i++)
        {
            //brown triangle
            Graphics_drawHorizontalLineOnDisplay(
                    &g_sDisplay, 0, (128 * (i + 1) / (2 * (y0 - y1))) - 4,
                    (y1 + i), 0x000000);
        }
    }
    else
    {
        for (int i = 0; i < (2 * (y0 - y1)); i++)
        {
            //brown triangle
            Graphics_drawHorizontalLineOnDisplay(
                    &g_sDisplay,
                    128 - (128 * (i + 1) / (2 * (y0 - y1)) + 1) + 4, 127,
                    (y1 + i), 0x000000);
        }
    }
}

void DRAW::triangle_sky(int y0, int y1, uint16_t result)
{
    if (resultsBuffer[0] < 8192)
    {
        for (int i = 0; i < (2 * (y0 - y1)); i++)
        {
            // White line between horizon
            Graphics_drawHorizontalLineOnDisplay(
                    &g_sDisplay, (128 * (i + 1) / (2 * (y0 - y1))) - 3,
                    (128 * (i + 1) / (2 * (y0 - y1))), (y1 + i), 0xFFFFFF);
            //blue triangle
            Graphics_drawHorizontalLineOnDisplay(
                    &g_sDisplay, (128 * (i + 1) / (2 * (y0 - y1)) + 1), 127,
                    (y1 + i), 0x0000FF);
        }
    }
    else
    {
        for (int i = 0; i < (2 * (y0 - y1)); i++)
        {
            //blue triangle
            Graphics_drawHorizontalLineOnDisplay(
                    &g_sDisplay, 0, 128 - (128 * (i + 1) / (2 * (y0 - y1))),
                    (y1 + i), 0x0000FF);
            // White line between horizon
            Graphics_drawHorizontalLineOnDisplay(
                    &g_sDisplay, 128 - (128 * (i + 1) / (2 * (y0 - y1)) + 1),
                    128 - (128 * (i + 1) / (2 * (y0 - y1)) + 1) + 3, (y1 + i),
                    0xFFFFFF);
        }
    }
}
