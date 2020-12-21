//SYSC 3310 - Project
//Maged Mohammed

#include "msp.h"

uint8_t led_state = 1;

void init_uart(void)
{
	// Configure UART pins
	P1SEL0 |= (BIT2 | BIT3);              // set 2-UART pin as secondary function
	P1SEL1 &= ~(BIT2 | BIT3);             // set 2-UART pin as secondary function

	// Configure UART
	EUSCI_A0->CTLW0 |= (UCSSEL_1 | UCSWRST);	// Put eUSCI in reset

	// Baud Rate calculation
	// N = 32768 / 1200 = 27.306
	// Fractional portion = 0.306
	// OS16 = 1, 
	// UCBRx = INT(N/16) = INT(1.706) = 1
	// UCBRFx = INT([(N/16) – INT(N/16)] × 16) = 11 
	// User's Guide Table 21-4: UCBRSx = 0x25
	EUSCI_A0->BRW = 1;
	EUSCI_A0->MCTLW = 0x0000;
	EUSCI_A0->MCTLW |= UCOS16;
	EUSCI_A0->MCTLW |= 0x00B0;
	EUSCI_A0->MCTLW |= 0x2500;
	EUSCI_A0->CTLW0 &= ~(UCSWRST);
}

void init_buttons_leds(void)
{
	// Config GPIO for Port 1 (pin 0,1 and 4) and 2 (pin 0,1 and 2)
	P1SEL0 &= (uint8_t)(~(BIT4 | BIT1 | BIT0));
	P1SEL1 &= (uint8_t)(~(BIT4 | BIT1 | BIT0));
	P2SEL0 &= (uint8_t)(~(BIT2 | BIT1 | BIT0));
	P2SEL1 &= (uint8_t)(~(BIT2 | BIT1 | BIT0));
	
	P1DIR &= (uint8_t) (~(BIT4 | BIT1)); 		// Port 1 pin 1 and 4 config as input
	P1DIR |= (uint8_t) BIT0; 					// Port 1 pin //Port 1 pin 0 as output
	P2DIR |= (uint8_t) ((BIT2 | BIT1 | BIT0));	// Port 2 pin 0,1,2 as output
	
	// Config Pin states
	P1OUT &= (uint8_t)(~BIT0); 
	P2OUT &= (uint8_t)(~(BIT2 | BIT1 | BIT0));
	P1OUT |= (uint8_t)(BIT4 | BIT1); 
	
	// Config pull up resistor for Port 1 pin 1 and 4
	P1REN |= (uint8_t)(BIT4 | BIT1);
}

void init_interupts(void)
{
	EUSCI_A0->IFG &= ~UCRXIFG; 		// Reset interupt flag
	EUSCI_A0->IE  |= UCRXIE; 		// Enable interupts when RX buffer is full 
	
	// Device interrupt configuration
	P1IES |=  (uint8_t)(BIT4 | BIT1);
	P1IFG &=  (uint8_t)(~BIT4 | BIT1);
	P1IE |= (uint8_t)(BIT4 | BIT1);
	
	// Port 1 NVIC configuration
	NVIC_SetPriority(PORT1_IRQn, 2);
	NVIC_ClearPendingIRQ(PORT1_IRQn);
	NVIC_EnableIRQ(PORT1_IRQn);

	// UART NVIC configuration
	NVIC_SetPriority(EUSCIA0_IRQn, 2);
	NVIC_ClearPendingIRQ(EUSCIA0_IRQn);
	NVIC_EnableIRQ(EUSCIA0_IRQn);

	// Enable global interrupt
	__ASM("CPSIE I"); 
}

void update_led_state(uint8_t state)
{
	//Update LED's based on inputed state
	switch (state)
	{
    case 1:
			led_state = 1;
		//Both LED's off
      P1OUT &= (uint8_t) ~BIT0;
			P2OUT &= (uint8_t) ~BIT0;	
			break;
		
    case 2:
			led_state = 2;
			//P1 LED on only
      P1OUT |= (uint8_t) BIT0;
			P2OUT &= (uint8_t) ~BIT0;	
			break;
		
    case 3:
			led_state = 3;
			//P2 LED on only
      P1OUT &= (uint8_t) ~BIT0;
			P2OUT |= (uint8_t) BIT0;	
			break;
		
    case 4:
			led_state = 4;
			//Both LED's on
      P1OUT |= (uint8_t) BIT0;
			P2OUT |= (uint8_t) BIT0;
			break;
		
    default:
			//Any other state do nothing
			break;
	}
}

void increment_status(void){
	if(led_state >= 4) //State equal or greater than 4
	{
		//Reset state to 1
		led_state = 1;
	} 
	else 
	{
		//Otherwise increase state
		led_state++;
	}
	//Update state with new value
	update_led_state(led_state);
}

void decrement_status(void){
	if(led_state <= 1) //State equal or less than 1 
	{
		//Reset state to 4
		led_state = 4;
	} 
	else 
	{
		//Otherwise decrease state
		led_state--;
	}
	//Update state with new value
	update_led_state(led_state);
}

void transmit(uint8_t message)
{
	while (EUSCI_A0->STATW & UCBUSY); //Wait for bus to clear
	EUSCI_A0->TXBUF = message;
}

void EUSCIA0_IRQHandler(void)
{
	if (EUSCI_A0->IFG & UCRXIFG) // Check if interupt is from RX IFG
	{
		EUSCI_A0->IFG &= ~UCRXIFG; // Reset interupt flag
	
		// Check if the TX buffer is empty first
		while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));

		//Decode received character from the RX buffer 
		switch(EUSCI_A0->RXBUF) 
		{
			case 0x41: //A char, increment state
				increment_status();
				transmit(led_state + '0');
				break;
			case 0x61: //a char, increment state 
				increment_status();
				transmit(led_state + '0');
				break;
			case 0x44: //D char, decrement state  
				decrement_status();
				transmit(led_state + '0');
				break;
			case 0x64: //d char, decrement  state
				decrement_status();
				transmit(led_state + '0');
				break;
			case 0x53: //S char, return state  
				transmit(led_state + '0');
				break;
			case 0x73: //s char, return state
				transmit(led_state + '0');
				break;
			default: //Any other char return F char indicating fail 
				transmit('F');
		}
	}
}

void PORT1_IRQHandler(void) 
{	
	if ((P1IFG & (uint8_t) BIT1) != 0) // Check if button 1
	{
		P1IFG &= ~(uint8_t) BIT1; // clear flag (ack)
		
		increment_status();
		transmit(led_state + '0');
	} 
	else if ((P1IFG & (uint8_t) BIT4) != 0) // Check if button 4
	{
		P1IFG &= ~(uint8_t) BIT4; // clear flag (ack)
		
		decrement_status();
		transmit(led_state + '0');
	}
}

int main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;   // Stop watchdog timer
	
	init_uart();
	init_buttons_leds();
	init_interupts();

	while(1); // Do nothing
}
