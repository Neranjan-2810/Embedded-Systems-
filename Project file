#include "TM4C123.h"
#include "TM4C123GH6PM.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

uint32_t Measure_distance(void);
void Timer0ACapture_init(void);
void Delay_MicroSecond(int time);

void Delay(unsigned long counter);
void UART5_init(void);
void UART5_Transmitter(unsigned char data);
void printstring(char *str);
char mesg[12];
float voltage;

//UltraSonic
uint32_t time; /*stores pulse on time */
uint32_t distance; /* stores measured distance value */
char mesg1[20];  /* string format of distance value */

#define DELAY_TIME 1000000 // Define delay time in microseconds



void Delay(unsigned long counter); // used to add delay
void UART5_init(void); // Initialize UART5 module for HC-05
void Bluetooth_Write_String(char *str); // Transmit a string to HC-05 over Tx5 pin 

int main(void)
{
	    unsigned int adc_value;
	   UART5_init();
    /* Enable Clock to ADC0 and GPIO pins*/
    SYSCTL->RCGCGPIO |= (1<<4);   /* Enable Clock to GPIOE or PE3/AN0 */
    SYSCTL->RCGCADC |= (1<<0);    /* AD0 clock enable*/
    
    /* initialize PE3 for AIN0 input  */
    GPIOE->AFSEL |= (1<<3);       /* enable alternate function */
    GPIOE->DEN &= ~(1<<3);        /* disable digital function */
    GPIOE->AMSEL |= (1<<3);       /* enable analog function */
   
    /* initialize sample sequencer3 */
    ADC0->ACTSS &= ~(1<<3);        /* disable SS3 during configuration */
    ADC0->EMUX &= ~0xF000;    /* software trigger conversion */
    ADC0->SSMUX3 = 0;         /* get input from channel 0 */
    ADC0->SSCTL3 |= (1<<1)|(1<<2);        /* take one sample at a time, set flag at 1st sample */
    ADC0->ACTSS |= (1<<3);         /* enable ADC0 sequencer 3 */
	
	Timer0ACapture_init();  /*initialize Timer0A in edge edge time */
	UART5_init(); // call HC05_init() to initialze UART5 of TM4C123GH6PM

	while(1)
	{
		ADC0->PSSI |= (1<<3);        /* Enable SS3 conversion or start sampling data from AN0 */
    while((ADC0->RIS & 8) == 0) ;   /* Wait untill sample conversion completed*/
    adc_value = ADC0->SSFIFO3; /* read adc coversion result from SS3 FIFO*/
    ADC0->ISC = 8;          /* clear coversion clear flag bit*/
		
		voltage = (adc_value * 0.0008 );
		sprintf(mesg, "\r\n Potentiometer Value (Voltage) = %0.3f Volts", voltage);
		
		Bluetooth_Write_String(mesg); // Send "Hello!" message

		Delay(500000); // Delay for 5 seconds
		
		//UltraSonic
		time = Measure_distance(); /* take pulse duration measurement */ 
		distance = (time * 10625)/10000000; /* convert pulse duration into distance */
		sprintf(mesg1, "\r\n Ultrasonic Value (Distance) = %d cm", distance); /*convert float type distance data into string */
		Bluetooth_Write_String(mesg1); /*transmit data */
		Delay(500000);
	}
}







/* This function captures consecutive rising and falling edges of a periodic signal */
/* from Timer Block 0 Timer A and returns the time difference (the period of the signal). */
uint32_t Measure_distance(void)
{
    int lastEdge, thisEdge;
	
	  /* Given 10us trigger pulse */
	  GPIOA->DATA &= ~(1<<4); /* make trigger  pin high */
	  Delay_MicroSecond(10); /*10 seconds delay */
	  GPIOA->DATA |= (1<<4); /* make trigger  pin high */
	  Delay_MicroSecond(10); /*10 seconds delay */
	  GPIOA->DATA &= ~(1<<4); /* make trigger  pin low */

 	while(1)
	{
    TIMER0->ICR = 4;            /* clear timer0A capture flag */
    while((TIMER0->RIS & 4) == 0) ;    /* wait till captured */
	  if(GPIOB->DATA & (1<<6)) /*check if rising edge occurs */
		{
    lastEdge = TIMER0->TAR;     /* save the timestamp */
		/* detect falling edge */
    TIMER0->ICR = 4;            /* clear timer0A capture flag */
    while((TIMER0->RIS & 4) == 0) ;    /* wait till captured */
    thisEdge = TIMER0->TAR;     /* save the timestamp */
		return (thisEdge - lastEdge); /* return the time difference */
		}
	}
}

/* Timer0A initialization function */
/* Initialize Timer0A in input-edge time mode with up-count mode */
void Timer0ACapture_init(void)
{
    SYSCTL->RCGCTIMER |= 1;     /* enable clock to Timer Block 0 */
    SYSCTL->RCGCGPIO |= 2;      /* enable clock to PORTB */
    
    GPIOB->DIR &= ~0x40;        /* make PB6 an input pin */
    GPIOB->DEN |= 0x40;         /* make PB6 as digital pin */
    GPIOB->AFSEL |= 0x40;       /* use PB6 alternate function */
    GPIOB->PCTL &= ~0x0F000000;  /* configure PB6 for T0CCP0 */
    GPIOB->PCTL |= 0x07000000;
    
	  /* PB2 as a digital output signal to provide trigger signal */
	  SYSCTL->RCGCGPIO |= 1;      /* enable clock to PORTA */
	  GPIOA->DIR |=(1<<4);         /* set PB2 as a digial output pin */
	  GPIOA->DEN |=(1<<4);         /* make PB2 as digital pin */

    TIMER0->CTL &= ~1;          /* disable timer0A during setup */
    TIMER0->CFG = 4;            /* 16-bit timer mode */
    TIMER0->TAMR = 0x17;        /* up-count, edge-time, capture mode */
    TIMER0->CTL |=0x0C;        /* capture the rising edge */
    TIMER0->CTL |= (1<<0);           /* enable timer0A */
}



/* Create one microsecond second delay using Timer block 1 and sub timer A */

void Delay_MicroSecond(int time)
{
    int i;
    SYSCTL->RCGCTIMER |= 2;     /* enable clock to Timer Block 1 */
    TIMER1->CTL = 0;            /* disable Timer before initialization */
    TIMER1->CFG = 0x04;         /* 16-bit option */ 
    TIMER1->TAMR = 0x02;        /* periodic mode and down-counter */
    TIMER1->TAILR = 16 - 1;  /* TimerA interval load value reg */
    TIMER1->ICR = 0x1;          /* clear the TimerA timeout flag */
    TIMER1->CTL |= 0x01;        /* enable Timer A after initialization */

    for(i = 0; i < time; i++)
    {
        while ((TIMER1->RIS & 0x1) == 0) ;      /* wait for TimerA timeout flag */
        TIMER1->ICR = 0x1;      /* clear the TimerA timeout flag */
    }
}















void UART5_init(void)
{
	SYSCTL->RCGCUART |= 0x20;  /* enable clock to UART5 */
    SYSCTL->RCGCGPIO |= 0x10;  /* enable clock to PORTE for PE4/Rx and RE5/Tx */
    Delay(1);
    /* UART0 initialization */
    UART5->CTL = 0;         /* UART5 module disbable */
    UART5->IBRD = 104;      /* for 9600 baud rate, integer = 104 */
    UART5->FBRD = 11;       /* for 9600 baud rate, fractional = 11*/
    UART5->CC = 0;          /*select system clock*/
    UART5->LCRH = 0x60;     /* data lenght 8-bit, not parity bit, no FIFO */
    UART5->CTL = 0x301;     /* Enable UART5 module, Rx and Tx */

    /* UART5 TX5 and RX5 use PE4 and PE5. Configure them digital and enable alternate function */
    GPIOE->DEN = 0x30;      /* set PE4 and PE5 as digital */
    GPIOE->AFSEL = 0x30;    /* Use PE4,PE5 alternate function */
    GPIOE->AMSEL = 0;    /* Turn off analg function*/
    GPIOE->PCTL = 0x00110000;     /* configure PE4 and PE5 for UART */
} 

void Bluetooth_Write(unsigned char data)
{
    // Wait until the transmit buffer is not full
    while ((UART5->FR & (1 << 5)) != 0)
    {
        // Wait for the transmit buffer to be empty
    }

    // Write the data to the transmit buffer
    UART5->DR = data;
}

void Bluetooth_Write_String(char *str)
{
  while(*str)
	{
		Bluetooth_Write(*(str++));
	}
}

void Delay(unsigned long counter)
{
	unsigned long i = 0;
	
	for(i=0; i< counter; i++);
}
