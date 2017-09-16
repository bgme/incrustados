/* Author: Badilla Esteban, Mora Berni */
/* ebadilla10@gmail.com, correoberni@*/

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include <HAL_I2C.h>
#include <HAL_OPT3001.h>
/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <iostream>
#include <vector>
#include <stdio.h>
using namespace std;

#define ON true

uint8_t lights = 4; /* 1:red 2:green 4:blue */
uint32_t blink_counter = 0U;
uint32_t ADC14Result = 0U;
bool init = true;
bool new_sample;
vector<uint16_t> samples_mic (20, 100);


//Headers
void initialize(void);
void turn_on(uint8_t lights);
void set_config_outport(uint8_t lights);
void adc14_config(void);
void T32_INIT2_CONFIG(bool init);
void T32_INT1_INIT(void);
void BUTTON_CONFIG();
void button_toggle(void);
void CONFIG_LIGHT_SENSOR(void);
void DEBOUNCE(void);
void CONFIG_ADC14(void);
void CONFIG_MICROPHONE(void);
void PROCESS_NEW_SAMPLE(void);
int  AVERAGE_SAMPLES_MIC(vector<uint16_t>);

int main(void) {

	//vector <uint16_t> samples;
	//samples.push_back(0x0012); /* Add an element at the end*/

	//uint16_t hola = samples.at(0);
	//samples.erase(samples.begin()); /* Erase the first element*/

	//samples.size();

	/* Halting WDT and disabling master interrupts */
	WDTCTL = WDTPW | WDTHOLD; /* Stop watchdog timer */

	/* Initializes Clock System */
	MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);

	/*Config light sensor */
	CONFIG_LIGHT_SENSOR();

	/* Ready to use */
	initialize(); /* Blink 3 times at the beginning. SRS-002 */

	set_config_outport(lights);

	turn_on(lights); /* Check at the beginning the intensity of light. SRS-003 */

	T32_INIT2_CONFIG(init);

//    adc14_config();
	BUTTON_CONFIG();

	CONFIG_ADC14();
	CONFIG_MICROPHONE();
	while (1) {
		if(new_sample) PROCESS_NEW_SAMPLE();
	}
}

void PROCESS_NEW_SAMPLE(void) {
	int avg_5sec = AVERAGE_SAMPLES_MIC(samples_mic);
	if (samples_mic[19] > avg_5sec & samples_mic[18] > avg_5sec
			& samples_mic[17] > avg_5sec & samples_mic[16] > avg_5sec) {
		turn_on(lights);
	}
	new_sample = false;
}

int AVERAGE_SAMPLES_MIC(vector<uint16_t> vec_samples) {
	int sum = 0;
	for (int i = 0; i < vec_samples.size(); i++) {
		sum += vec_samples[i];
	}
	return vec_samples.empty() ? 0 : sum / vec_samples.size();
}

void CONFIG_LIGHT_SENSOR(void) {
	/* Initialize I2C communication */
	Init_I2C_GPIO();
	I2C_init();
	/* Initialize OPT3001 digital ambient light sensor */
	OPT3001_init();
}

void CONFIG_MICROPHONE(void){
	// Set P4.3 for Analog input, disabling the I/O circuit.
	P4->SEL0 = BIT3;
	P4->SEL1 = BIT3;
	P4->DIR &= ~BIT3;
}

void CONFIG_ADC14(void) {
	ADC14->CTL0 = ADC14_CTL0_PDIV_0 | ADC14_CTL0_SHS_0 | ADC14_CTL0_DIV_7
			| //predividido por 1,
			ADC14_CTL0_SSEL__MCLK | ADC14_CTL0_SHT0_1 | ADC14_CTL0_ON
			| ADC14_CTL0_SHP;
	ADC14->MCTL[0] = ADC14_MCTLN_INCH_10 | ADC14_MCTLN_VRSEL_0;
	ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_ENC;
	ADC14->IER0 = ADC14_IER0_IE0;
	NVIC_SetPriority(ADC14_IRQn, 1);
	NVIC_EnableIRQ(ADC14_IRQn);
}

void initialize(void) {
	init = true;
	P2->DIR |= BIT0 | BIT1 | BIT2;
	P2->OUT &= 0xF8;

	T32_INIT2_CONFIG(init);
	while (blink_counter < 6) {
		/* To blink three times */
	}
	TIMER32_2->CONTROL = 0; /* Disable the timer for saving energy */
	NVIC_DisableIRQ(T32_INT2_IRQn);
	init = false;
}

void turn_on(uint8_t lights) {
	unsigned long int lux = 0;
	/* Obtain lux value from OPT3001 */
	lux = OPT3001_getLux();
	if (lux < 15) {
		P2->OUT |= lights;
		T32_INT1_INIT();
	}
}

void button_toggle(void) {
	if (P2->OUT & 0x7) {/* Led is currently ON */
		P2->OUT &= 0xF8;
		TIMER32_1->CONTROL = 0; /* turn off the timer1*/
		NVIC_DisableIRQ(T32_INT1_IRQn);
	} else {/* Led is currently OFF */
		P2->OUT |= lights;
		T32_INT1_INIT();
	}
}

void set_config_outport(uint8_t lights) {
	P2->DIR |= lights;
}
/*
void adc14_config(void) {
	P1->DIR = BIT0;
	P1->OUT = BIT0;
	// Set P4.3 for Analog input, disabling the I/O circuit.
	P4->SEL0 = BIT3;
	P4->SEL1 = BIT3;
	P4->DIR &= ~BIT3;

	//TIMER32_1->LOAD = 0x00B71B00; //~0.5s ---> a 48Mhz
	TIMER32_1->LOAD = 0x0000BB80; //~0.5s ---> a 48Mhz
	TIMER32_1->CONTROL =
	TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE
			| TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
	NVIC_SetPriority(T32_INT1_IRQn, 1);
	NVIC_EnableIRQ(T32_INT1_IRQn);

	ADC14->CTL0 = ADC14_CTL0_PDIV_0 | ADC14_CTL0_SHS_0 | ADC14_CTL0_DIV_7 |
	ADC14_CTL0_SSEL__MCLK | ADC14_CTL0_SHT0_1 | ADC14_CTL0_ON | ADC14_CTL0_SHP;
	ADC14->MCTL[0] = ADC14_MCTLN_INCH_10 | ADC14_MCTLN_VRSEL_0;
	ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_ENC;
	ADC14->IER0 = ADC14_IER0_IE0;
	NVIC_SetPriority(ADC14_IRQn, 1);
	NVIC_EnableIRQ(ADC14_IRQn);
}
*/
void T32_INIT2_CONFIG(bool init) {
	if (init) {
		TIMER32_2->LOAD = 0x0016E360; /* ~500ms ---> a 3Mhz */
	} else {
		TIMER32_2->LOAD = 0x000B71B0; /* ~250ms ---> a 3Mhz */
	}
	TIMER32_2->CONTROL =
	TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE
			| TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
	NVIC_SetPriority(T32_INT2_IRQn, 1);
	NVIC_EnableIRQ(T32_INT2_IRQn);
}

void T32_INT1_INIT(void) {
	TIMER32_1->CONTROL = 0; /* turn off the timer, it would be used to reset the count */
	TIMER32_1->LOAD = 0x055D4A80; /* ~30s ---> a 3Mhz */
	TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0
			| TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE
			| TIMER32_CONTROL_ONESHOT;
	NVIC_SetPriority(T32_INT1_IRQn, 1);
	NVIC_EnableIRQ(T32_INT1_IRQn);
}

void BUTTON_CONFIG() {
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

void DEBOUNCE(void) {
	do {
		for (int i = 0; i < 3000; i++) {
		}
	} while ((P5->IN & 0x02) == 0x00);
}

extern "C" {
void T32_INT1_IRQHandler(void) {
	__disable_irq();
	TIMER32_1->INTCLR = 0U;
	P2->OUT &= 0xF8;
	__enable_irq();
	return;
}

void ADC14_IRQHandler(void) {
	__disable_irq();
	ADC14Result = ADC14->MEM[0];
	ADC14->CLRIFGR0 = ADC14_CLRIFGR0_CLRIFG0;
	new_sample = true;
	__enable_irq();
	return;
}

void T32_INT2_IRQHandler(void) {
	__disable_irq();
	TIMER32_2->INTCLR = 0U;
	if (init) {
		P2->OUT ^= BIT0 | BIT1 | BIT2;
		blink_counter++;
	} else {
		/* P1->OUT ^= BIT0; */
		ADC14->CTL0 |= ADC14_CTL0_SC; /* Start */
	}
	__enable_irq();
	return;
}

/* GPIO ISR */
void PORT5_IRQHandler(void) {
	__disable_irq();
	DEBOUNCE();
	uint32_t status = 0;
	status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);
	button_toggle();
	__enable_irq();
	return;
}

}
