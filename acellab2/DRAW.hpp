/*
 * DRAW.hpp
 *
 *  Created on: 24 oct. 2017
 *      Author: berni
 */

#ifndef DRAW_HPP_
#define DRAW_HPP_
#include "LcdDriver/Crystalfontz128x128_ST7735.hpp"
#include <ti/devices/msp432p4xx/inc/msp.h>
#include "Task.hpp"

class DRAW: public Task
{
public:
    DRAW(uint16_t i_BITN);
    virtual ~DRAW();
    virtual uint8_t run(void);
    virtual uint8_t setup(void);
    void drawHorizon(void);

    virtual uint8_t getMessage();
    virtual uint8_t putMessage();

    /* Graphic library context */
    Graphics_Context g_sContext;

    /* Graphic library display */
    Graphics_Display g_sDisplay;

    Graphics_Rectangle values_rectangle_heaven;
    Graphics_Rectangle values_rectangle_earth;

    /* ADC results buffer */
    uint16_t resultsBuffer[3];

protected:
private:
};

#endif /* DRAW_HPP_ */

