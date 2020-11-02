#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"

#include "stdio.h"
#include "stdlib.h"
#include "time.h"

const int time_constant = 160000;		//constant to adjust speed of blinks globally.  160000 gives about 1 second
const int rand_divisor = 0x3fffffff;

bool combination[16], user_input[16];

void setup_hardware()
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();
}

void setup_LCD()
{
	//turn on clock gating for SLCD, B, C, D, and E
	SIM->SCGC5 |= 1 << 10 | 1 << 11 | 1 << 12 | 1 << 13 | 1 << 19;
	LCD->GCR |= 1 << 15; // disable padsafe (active low)
	LCD->GCR &= ~(1 << 7); // disable LCD driver

	//i commented these out because PCR MUX defaults to ALT0 right?

	LCD->GCR &= ~(1 << 31); // disable internal VR
	LCD->GCR |= 1 << 23; // Sets the charge pump to supply voltage to LCD
	LCD->GCR |= 0x3 << 21; // CPSEL = 1, so we set this to 11 for lowest clock speed
	LCD->GCR &= ~(1 << 17); // clear voltage supply control. drive from VDD
	LCD->GCR &= ~(1 << 14); // no interrupt req
	LCD->GCR &= ~(3 << 13); // Divide 32kHz clk by 1
	LCD->GCR &= ~(1 << 11); // using alt1 clk
	LCD->GCR &= ~(1 << 10); // standard frame rate
	LCD->GCR &= ~(1 << 9); // enable LCD in doze mode
	LCD->GCR &= ~(1 << 8); // enable LCD in stop mode
	LCD->GCR |= 1 << 6; // using alt1 clk source (32kHz)
	LCD->GCR |= 1 << 5; // refresh clks as fast as possible so prescaler is set to 1
	LCD->GCR |= 0x3; // we have 4 pins for front / back plane, so duty cycle is / 4. we have 4 phases here
	LCD->AR &= ~(1 << 7); // disables LCD blinking
	LCD->AR &= ~(1 << 6); // normal display mode
	LCD->AR &= ~(1 << 5); // setting this clears all segments in LCD
	LCD->AR &= ~(1 << 3); // blank display during blink period
	LCD->AR &= ~(0x7); // LCD blink freq = LCD clk / 2^(12 + BRATE). i set BRATE to 0 here
	LCD->FDCR = 0x00000000;
	LCD->FDSR = 0x0000;

	// enabling each pin for LCD (12). the non COM0-4 pins will be set as frontplane by default
	// PEN[0] for lower 32 bits, PEN[1] for upper 32 bits
	LCD->PEN[0] |= 1 << 7 | 1 << 8 | 1 << 10 | 1 << 11 | 1 << 17 | 1 << 18 | 1 << 19;
	LCD->PEN[1] |= 1 << 5 | 1 << 6 | 1 << 8 | 1 << 20 | 1 << 21; // PEN[1] for upper 32 bits

	// set COM0-4 pins as backplane. frontplane pins will always intersect w/ one COM pin to turn on
	LCD->BPEN[0] |= 1 << 18 | 1 << 19;
	LCD->BPEN[1] |= 1 << 8 | 1 << 20;

	// setting the waveform registers. these registers are modified to display segmends of the LCD
	// WF8B[] accesses pin 0-63 and stores 8 bits
	// WF[16] stores 32 bits
	// MSB controls phase H, LSB controls phase A
	LCD->WF[0] = 0x00000000; // pins 0-3 are disabled
	LCD->WF[1] = 0x00000000; // pins 4-7 are disabled
	LCD->WF[2] = 0x00000000; // pins 8-11 are disabled
	LCD->WF[3] = 0x00000000; // pins 12-15 are disabled
	LCD->WF[4] = 0x44880000; // pin 18 (COM3) active on phase d, pin 19 (COM2) active on phase c
	LCD->WF[5] = 0x00000000; // pins 20-23 are disabled
	LCD->WF[6] = 0x00000000; // pins 24-27 are disabled
	LCD->WF[7] = 0x00000000; // pins 28-31 are disabled
	LCD->WF[8] = 0x00000000; // pins 32-35 are disabled
	LCD->WF[9] = 0x00000000; // pins 36-39 are disabled
	LCD->WF[10] = 0x00000011; // pin 40 (COM0) is active on phase A
	LCD->WF[11] = 0x00000000; // pins 44-47 are disabled
	LCD->WF[12] = 0x00000000; // pins 48-51 are disabled
	LCD->WF[13] = 0x00000022; // pin 52 (COM1) is active on phase B
	LCD->WF[14] = 0x00000000; // pins 56-59 are disabled
	LCD->WF[15] = 0x00000000; // pins 60-63 are disabled
	LCD->GCR &= ~(1 << 15); // Enable padsafe (active low)
	LCD->GCR |= (1 << 7); // LCD driver enable
}
/* KEY
 * pin 1 - 40 (COM0)
 * pin 2 - 52 (COM1)
 * pin 3 - 19 (COM2)
 * pin 4 - 18 (COM3)
 * pin 5 - 37
 * pin 6 - 17
 * pin 7 - 7
 * pin 8 - 8
 * pin 9 - 53
 * pin 10 - 38
 * pin 11 - 10
 * pin 12 - 11
  */

void setup_LEDs()	//PTE29 RED LED || PTD5 GREEN LED
{
	SIM->SCGC5 |= (0x3 << 12);

	PORTD->PCR[5] &= ~0x700;			//pin D control register
	PORTD->PCR[5] |= 0x700 & (1 << 8);

	PORTE->PCR[29] &= ~0x700;			//pin E control register
	PORTE->PCR[29] |= 0x700 & (1 << 8);

	GPIOD->PDDR |= (1 << 5);
	GPIOE->PDDR |= (1 << 29);

	GPIOE->PDOR |= (1 << 29);
	GPIOD->PDOR |= (1 << 5);
}

void setup_buttons()
{
	//SIM->SCGC5 done in LED setup

	PORTC->PCR[3] &= ~0x703;
	PORTC->PCR[12] &= ~0x703;
	PORTC->PCR[3] |= 0x703 & ((1 << 8) | 0x03);
	PORTC->PCR[12] |= 0x703 & ((1 << 8) | 0x03);
	GPIOC->PDDR &= !(0x201 << 3); 						//direction to input
}

void blink_LED(float time, bool LED_sel)					//hold LED on for given time in seconds
{
	if(LED_sel)		//LED_sel: 0 = green, 1 = red
	{
		GPIOE->PDOR &= ~(1 << 29);
	}
	else
	{
		GPIOD->PDOR &= ~(1 << 5);
	}

	int i = 0;
	while(i < (time_constant * time))	//busy while loop
	{
		i++;
	}

	GPIOE->PDOR |= (1 << 29);
	GPIOD->PDOR |= (1 << 5);
}

void pause(float time)
{
	int i = 0;
	while(i < (time_constant * time))	//busy while loop
	{
		i++;
	}
}

void generate_comb(int seed)
{
	srand(seed);

	for(int i = 0; i < 16; i++)
	{
		combination[i] = rand() / rand_divisor;
	}
}

void print_comb()
{
	for (int i = 0; i < 16; i++)
	{
		printf("%d", combination[i]);
	}
}

void display_comb(int round)
{
	for (int i = 0; i < round; i++)
	{
		blink_LED(.75, combination[i]);
		pause(.25);
	}
}

bool check_input(int round)
{
	for(int i = 0; i < round; i++)
	{
		if(!(combination[i] == user_input[i]))
		{
			return 0;
		}
	}
	return 1;
}

void display_round_LCD(int round)
{
	LCD->WF8B[10] = 0;
	LCD->WF8B[11] = 0;
	LCD->WF8B[38] = 0;

	switch(round)
	{
	case 1:
		LCD->WF8B[11] |= 0x6;
		break;
	case 2:
		LCD->WF8B[10] |= 0x7;
		LCD->WF8B[11] |= 0xC;
		break;
	case 3:
		LCD->WF8B[10] |= 0xF;
		LCD->WF8B[11] |= 0x8;
		break;
	case 4:
		LCD->WF8B[10] |= 0xE;
		LCD->WF8B[11] |= 0x2;
		break;
	case 5:
		LCD->WF8B[10] |= 0xD;
		LCD->WF8B[11] |= 0xA;
		break;
	case 6:
		LCD->WF8B[10] |= 0xC;
		LCD->WF8B[11] |= 0xE;
		break;
	case 7:
		LCD->WF8B[10] |= 0xB;
		break;
	case 8:
		LCD->WF8B[10] |= 0xF;
		LCD->WF8B[11] |= 0xE;
		break;
	case 9:
		LCD->WF8B[10] |= 0xC;
		LCD->WF8B[11] |= 0xE;
		break;
	case 10:
		LCD->WF8B[38] |= 0x6;
		LCD->WF8B[10] |= 0xB;
		LCD->WF8B[11] |= 0xE;
		break;
	case 11:
		LCD->WF8B[38] |= 0x6;
		LCD->WF8B[11] |= 0x6;
		break;
	case 12:
		LCD->WF8B[38] |= 0x6;
		LCD->WF8B[10] |= 0x7;
		LCD->WF8B[11] |= 0xC;
		break;
	case 13:
		LCD->WF8B[38] |= 0x6;
		LCD->WF8B[10] |= 0x5;
		LCD->WF8B[11] |= 0xE;
		break;
	case 14:
		LCD->WF8B[38] |= 0x6;
		LCD->WF8B[10] |= 0xC;
		LCD->WF8B[11] |= 0x6;
		break;
	case 15:
		LCD->WF8B[38] |= 0x6;
		LCD->WF8B[10] |= 0xD;
		LCD->WF8B[11] |= 0xA;
		break;
	case 16:
		LCD->WF8B[38] |= 0x6;
		LCD->WF8B[10] |= 0xF;
		LCD->WF8B[11] |= 0x2;
		break;
	case -1:
		LCD->WF8B[11] |= 0xE;
		LCD->WF8B[53] |= 0xB;
		LCD->WF8B[38] |= 0xE;
		LCD->WF8B[7] |= 0xD;
		LCD->WF8B[8] |= 0xA;
		LCD->WF8B[37] |= 0x5;
		LCD->WF8B[17] |= 0xE;
		break;
	}
}

int main()
{
	setup_hardware();
	setup_LEDs();
	setup_LCD();
	setup_buttons();

	int seed = 0;

	while((GPIOC->PDIR & (1 << 3)) && (GPIOC->PDIR & (1 << 12)))		//while both are not pressed
	{
		seed++;
	}

	generate_comb(seed);
	print_comb();

	/* KEY
	 * pin 1 - 40 (COM0)
	 * pin 2 - 52 (COM1)
	 * pin 3 - 19 (COM2)
	 * pin 4 - 18 (COM3)
	 * pin 5 - 37
	 * pin 6 - 17
	 * pin 7 - 7
	 * pin 8 - 8
	 * pin 9 - 53
	 * pin 10 - 38
	 * pin 11 - 10
	 * pin 12 - 11
	  */

	//LCD->WF8B[53] |= 0xC;
	//LCD->WF8B[38] |= 0x6;
	int round = 1;

	while(round < 16)	//game loop
	{
		display_round_LCD(round);

		display_comb(round);

		for(int i = 0; i < round; i++)
		{
			while(1)
			{
				if(!(GPIOC->PDIR & (1 << 3)))			//right
				{
					GPIOD->PDOR &= ~(1 << 5);
					user_input[i] = 0;

					while(!(GPIOC->PDIR & (1 << 3)))	//while held down, wait
					{
						__asm volatile ("nop");			//do nothing
					}

					GPIOD->PDOR |= (1 << 5);
					break;
				}
				else if(!(GPIOC->PDIR & (1 << 12)))		//left
				{
					GPIOE->PDOR &= ~(1 << 29);
					user_input[i] = 1;

					while(!(GPIOC->PDIR & (1 << 12)))	//while held down, wait
					{
						__asm volatile ("nop");			//do nothing
					}

					GPIOE->PDOR |= (1 << 29);
					break;
				}
			}
		}

		if(!check_input(round))
		{
			while(1)
			{
				display_round_LCD(-1);
			}
		}

		pause(1);

		round++;
	}

	return 0;
}
