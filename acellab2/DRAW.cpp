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

    resultsBuffer[0] =  *(this->ptr_MailBox);
    resultsBuffer[1] = *(this->ptr_MailBox+1);
    resultsBuffer[2] = *(this->ptr_MailBox+2);

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

    values_rectangle_heaven.xMin = 0;
    values_rectangle_heaven.xMax = 127;
    values_rectangle_heaven.yMin = 0;
    values_rectangle_heaven.yMax = (128 * (resultsBuffer[2] - 11462))
            / (4922 - 11462);
    /*values_rectangle_heaven.yMax =
     (resultsBuffer[0] < 8192) ?
     (y0 * (resultsBuffer[0] - 8192)) / (8192 - 6557) + y0 :
     (-1 * y0 * (resultsBuffer[0] - 8192)) / (9827 - 8192) + y0;
     */
    // Dibujo cielo azul
    Graphics_fillRectangleOnDisplay(&g_sDisplay, &values_rectangle_heaven,
                                    0x001F);

    values_rectangle_earth.xMin = 0;
    values_rectangle_earth.xMax = 127;
    values_rectangle_earth.yMin = values_rectangle_heaven.yMax;
    values_rectangle_earth.yMax = 127;

    // Dibujo tierra cafe
    Graphics_fillRectangleOnDisplay(&g_sDisplay, &values_rectangle_earth,
                                    0x8208);

    ///*
    //    // Extras
    //    if (resultsBuffer[0] < 8192)
    //    {
    //        int y1 = (y0*(resultsBuffer[0]-8192))/(8192-6557)+y0;
    //        int i;
    //        for (i = 0; i < (y0-y1); i++)
    //        {
    //            //Extra 1.0
    //            Graphics_drawHorizontalLineOnDisplay(
    //                    &g_sDisplay,
    //                    0,
    //                    (128*(i+1) / (y0-y1)),
    //                    (y1+i),
    //                    0x804040);
    //            //Extra 1.1
    //            Graphics_drawHorizontalLineOnDisplay(
    //                    &g_sDisplay,
    //                    (128*(i+1) / (y0-y1))+1,
    //                    127,
    //                    (y1+i),
    //                    0x0000FF);
    //            // //Extra 2.0
    //            Graphics_drawHorizontalLineOnDisplay(&g_sDisplay,
    //                                                 63-(63)*(8192-resultsBuffer[0])/(8192-4922),
    //                                                 63,
    //                                                 63,
    //                                                 0xFFFFFF);
    //        }
    //    }else{
    //        int y2 = (-1*y0*(resultsBuffer[0]-8192))/(9827-8192)+y0;
    //        int i;
    //        for (i = 0; i < (y0-y2); i++)
    //        {
    //            //Extra 1.0
    //            Graphics_drawHorizontalLineOnDisplay(
    //                    &g_sDisplay,
    //                    0,
    //                    128-(128*(i+1)/(y0-y2)),
    //                    (y2+i),
    //                    0x0000FF);
    //            //Extra 1.1
    //            Graphics_drawHorizontalLineOnDisplay(
    //                    &g_sDisplay,
    //                    128-(128*(i+1)/(y0-y2))+1,
    //                    127,
    //                    (y2+i),
    //                    0x804040);
    //            //Extra 2.0
    //            Graphics_drawHorizontalLineOnDisplay(&g_sDisplay,
    //                                                      63,
    //                                                      63+(127-63)*(resultsBuffer[0]-8192)/(11462-8192),
    //                                                      63,
    //                                                      0xFFFFFF);
    //        }
    //   }
    //*/
}

uint8_t DRAW::getMessage()
{
    uint8_t status = NO_ERR;

    return status;
}
uint8_t DRAW::putMessage()
{
    uint8_t status = NO_ERR;

    return status;
}
