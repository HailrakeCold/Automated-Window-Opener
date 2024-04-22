
#include "stm32l552xx.h"
#include "stdio.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) & 1      ) //Checks the bit number <idx> -- 0 means clear; !0 means set.

// Helping functions
void setClks();
void BTNinit();
void LPUART1init(void);
void LPUART1write(int c);
int  LPUART1read(void);
void myprint(char msg[]);
void delayMs(int n);

void openWindow();
void closeWindow();
void ADCinit();

volatile char windowState;		// window state 'o' or 'c'
volatile int idlePeriod;		// time between temp checks in seconds
volatile int extPeriod;			// time for actuator to extend in ms
volatile int holdPeriod;		// time hold open after BTN in seconds

int main (void) {

	// init basics
	setClks();
	LPUART1init();

	// init function vars
	windowState = 'c';
	idlePeriod = 1;		// idle time in s
	extPeriod = 5000;	// actuator extension time in s
	holdPeriod = 5;		// BTN hold time in s

	int maxTemp = 75;	// upper temp thresh
	int minTemp = 70;	// lower temp thresh


	// ADC1/16 	- PB1
	// BTN 		- PA0
	// OPEN GRN - PA1
	// CLOSE BLU - PA2

	// enable GPIOA clk
	RCC->AHB2ENR |= 1 << 0;

	// set PA1 and PA2 to output, pull down
	GPIOA->MODER &= ~(0b111111);
	GPIOA->MODER |=  (0b0101 << 2);
	GPIOA->ODR &= ~(0b11 << 1);

	// set PA0 btn interrupt
	BTNinit();

	// PB1 - already in analog reset state

	// init ADC
	ADCinit();


	// set known state
	closeWindow();
	windowState = 'c';

	while (1) {
		char  txt [20];

		// read data from thermistor
		ADC1->CR |= (1 << 2);	// ADSTART
		while((ADC1->ISR & (1<<3)) == 0);		// wait for EOS flag
		int adc_val = ADC1->DR;					// read DR for ADC1_IN5
		while((ADC1->CR & (1<<2)) != 0);		// wait for ADSTART to fall

		// VREF=3.3V, step size is 3.3V/(2^12)=3.3/4096
		float voltage = adc_val*(3.3/4096);
		float tempC = (voltage - 0.3) / 0.0195;	// convert voltage to temp Celsius
		float tempF = (tempC * (1.8)) + 32;		// convert temp Celsius to Fahrenheit

		// print temp over UART and serial
		sprintf(txt, "$%.02f;",  tempF);
		myprint(txt);


		// if temp is over thresh and closed, open
		if ((tempF > maxTemp) && windowState == 'c') {
			openWindow();
			windowState = 'o';
		}

		// if temp is under thresh and open, close
		if ((tempF < minTemp) && windowState == 'o') {
			closeWindow();
			windowState = 'c';
		}

		// delay 1s
		delayMs(idlePeriod * 1000);

	}







}

void EXTI0_IRQHandler(){
	// check window state, signal opposite
	if (windowState == 'o') {
		closeWindow();
		windowState = 'c';
	} else {
		openWindow();
		windowState = 'o';
	}

	// delay 5s
	delayMs(holdPeriod * 1000);

	SysTick->CTRL = 0x5;    // re-enable timer for resuming delay

	// 10- Clear pending Interrupt
	EXTI->RPR1 |= 1<<0; // Cleared by writing 1 to it!
						 // Use RPR1 when trigger by Rising edge
}

void openWindow() {
	// send open signal for 5s
	GPIOA->ODR &= ~(0b11 << 1);
	GPIOA->ODR |=  (0b01 << 1);

	delayMs(extPeriod);

	GPIOA->ODR &= ~(0b11 << 1);
}

void closeWindow() {
	// send open signal for 5s
	GPIOA->ODR &= ~(0b11 << 1);
	GPIOA->ODR |=  (0b10 << 1);

	delayMs(extPeriod);

	GPIOA->ODR &= ~(0b11 << 1);
}


void BTNinit(){
	// set PA0 to input, pull down
	GPIOA->MODER &= ~(0b11);
	GPIOA->PUPDR &= ~(0b11);
	GPIOA->PUPDR |=  (0b10);

	/**********************************************
	 * EXTI CONFIG (keeping for future reference) *
	 **********************************************/

	// To use EXTI you need to enable SYSCFG
	RCC->APB2ENR   |= 1;  // Enable Clock to SYSCFG & EXTI
	// Select Input Using EXTI-->CRx
	EXTI->EXTICR[0] = (0x0)<<0;  // Select PA0

		//  REG        |  31-24   | 23-16  |  15-8   |    7-0    |
		//-------------+----------+--------+---------+-----------+
		// EXTICR[0]   |  GPIOx3  | GPIOx2 |  GPIOx1 |  GPIOx0   |
		// EXTICR[1]   |  GPIOx7  | GPIOx6 |  GPIOx5 |  GPIOx4   |
		// EXTICR[2]   |  GPIOx11 | GPIOx10|  GPIOx9 |  GPIOx8   |
		// EXTICR[3]   |  GPIOx15 | GPIOx14|  GPIOx13|  GPIOx12  |
		//-------------+----------+--------+---------+-----------+
		//  MUXSEL:   0 1 2 3 4 5 6 7
		//  PORT:     A B C D E F G H

	//	Select Trigger type (Failing or Raising Edge)
	EXTI->RTSR1    |= 1<<0;     // Trigger on rising edge of PA0
								 // Use RTSR1 register for rising edge

	//	Disable Interrupt Mask
	EXTI->IMR1     |= 1<<0;     // Interrupt mask disable for PA0

	// enable interrupt for push button
	NVIC_SetPriority(EXTI0_IRQn, 0); // Priority for EXTI0
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void ADCinit() {
	// Enable ADC Clock
	bitset(RCC->AHB2ENR, 13);  // Enable ADC clock
	RCC->CCIPR1 |=0x3<<28;     // Route SYSCLK (HCLK) to ADC

	// Turn on ADC Voltage Regulator
	bitclear(ADC1->CR, 29);  // Get out of deep power down mode
	bitset(ADC1->CR, 28);

	// Wait for the voltage regulator to stabilize
	delayMs(10);

	// Set up ADC1
	ADC1->SQR1 = (0x10<<6)|(0);			// 1 channel to read, channel 16
	ADC1->CR   |= 1;            		// Enable ADC
	while((ADC1->ISR & 0b1) == 0);		// wait for ADCRDY
}

void setClks(){
	RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
	RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
	RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
	RCC->CCIPR1   &= ~(0x400);
	RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
	RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}
void LPUART1init() {

	// enable clocks
//	bitset(RCC->APB1ENR1, 28);  // Enable Clock to PWR Interface
	bitset(RCC->AHB2ENR,   6);  // Enable Clock to GPIOG
//	bitset(RCC->APB1ENR2,  0);  // Enable Clock to LPUART
//	bitset(RCC->CCIPR1,   11);  // Select the high speed internal (HSI) oscillator as the clock to LPUART1 (16MHz)
//	bitclear(RCC->CCIPR1, 10);  //
//	bitset(RCC->CR, 8);         // HSI16 clock enable

	// enable power going to port G
	bitset(PWR->CR2, 9);        // Enable GPIOG power

	// config GPIOG
	//set GPIOG.7 to AF
	bitset(GPIOG->MODER,    15);  // Setting 0b10 in pin 7 two bit mode cfgs
	bitclear(GPIOG->MODER,  14);

	bitset(GPIOG->AFR[0],   31);  // Programming 0b1000
	bitclear(GPIOG->AFR[0], 30);
	bitclear(GPIOG->AFR[0], 29);
	bitclear(GPIOG->AFR[0], 28);

	//set GPIOG.8 to AF
	bitset(GPIOG->MODER,    17);  // Setting 0b10 in pin 8 two bit mode cfgs
	bitclear(GPIOG->MODER,  16);

	bitset(  GPIOG->AFR[1], 3);  // Programming 0b1000
	bitclear(GPIOG->AFR[1], 2);
	bitclear(GPIOG->AFR[1], 1);
	bitclear(GPIOG->AFR[1], 0);

	LPUART1-> PRESC = 0;

	// set baud rate and enable TX and RX LPUART
	// BRR = 256*16000000/115200 =
	LPUART1->BRR = 35555;
	LPUART1->CR1 = 0xD | (1<<5); // Enable Receive Data Not Empty Interrupt (RXNEIE)

}

void myprint (char msg[]){
    uint8_t idx=0;
    while(msg[idx]!='\0' ){
    	LPUART1write(msg[idx++]);
    }
}

/* Write a character to LPUART1 */
void LPUART1write (int ch) {
    while (!(LPUART1->ISR & 0x0080)); // wait until Tx buffer empty
    LPUART1->TDR = (ch & 0xFF);
}

/* Read a character from LPUART1 */
int LPUART1read(void) {
    while (!(LPUART1->ISR & 0x0020)) {}   // wait until char arrives
    return LPUART1->RDR;
}

void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}











