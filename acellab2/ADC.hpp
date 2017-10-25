/*
 * ADC.hpp
 *
 *  Created on: 24 oct. 2017
 *      Author: berni
 */

#ifndef ADC_HPP_
#define ADC_HPP_

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include <ti/devices/msp432p4xx/inc/msp.h>
#include "Task.hpp"

class ADC: public Task
{
public:
    ADC(uint16_t i_BITN);
    virtual ~ADC();
    virtual uint8_t run(void);
    virtual uint8_t setup(void);

    virtual uint8_t getMessage();
    virtual uint8_t putMessage();

protected:

private:
};

#endif /* ADC_HPP_ */
