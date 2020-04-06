/*	IVAN CORTES
 * 	DATE: 2020-03-16
 * 	CLASS: ECE5721
 * 	FINAL PROJECT: LCD
 */
#include <stdio.h>
#include <MKL25Z4.h>
#include "fsl_debug_console.h"

#define MASK(x)  (1UL << (x))
#define UART_OVERSAMPLE_RATE (16)
#define SYS_CLOCK (48e6)

void delay(unsigned int length_ms)
{
    SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK;  // Make sure clock is enabled
    LPTMR0->CSR = 0;                     // Reset LPTMR settings
    LPTMR0->CMR = length_ms;             // Set compare value (in ms)

    // Use 1kHz LPO with no prescaler
    LPTMR0->PSR = LPTMR_PSR_PCS(1) | LPTMR_PSR_PBYP_MASK;

    // Start the timer and wait for it to reach the compare value
    LPTMR0->CSR = LPTMR_CSR_TEN_MASK;
    while (!(LPTMR0->CSR & LPTMR_CSR_TCF_MASK))
        ;

    LPTMR0->CSR = 0;                     // Turn off timer
}

void Init_UART0(uint32_t baud_rate) {
	uint16_t sbr;
	//enable clock gating for UART0 and PortA
	SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	//Make sure transmitter and receiver are disabled before init
	UART0->C2 &= ~UART0_C2_TE_MASK & ~UART0_C2_RE_MASK;
	//Set UART clock to 48 MHz clock
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);
	SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
	//Set pins to UART0 Rx and Tx
	PORTA->PCR[1] = PORT_PCR_ISF_MASK | PORT_PCR_MUX(2); //Rx
	PORTA->PCR[2] = PORT_PCR_ISF_MASK | PORT_PCR_MUX(2); //Tx
	//Set Baud rate and oversampling ratio
	sbr = (uint16_t)((SYS_CLOCK)/(baud_rate * UART_OVERSAMPLE_RATE));
	UART0->BDH &= ~UART0_BDH_SBR_MASK;
	UART0->BDH |= UART0_BDH_SBR(sbr>>8);
	UART0->BDL = UART0_BDL_SBR(sbr);
	UART0->C4 |= UART0_C4_OSR(UART_OVERSAMPLE_RATE-1);
	//Disable interrupts for Rx active edge and LIN break detect, select one stop bit
	UART0->BDH |= UART0_BDH_RXEDGIE(0) | UART0_BDH_SBNS(0) | UART0_BDH_LBKDIE(0);
	// Don't enable loopback mode, use 8 data bit mode, don't use parity
	UART0->C1 = UART0_C1_LOOPS(0) | UART0_C1_M(0) | UART0_C1_PE(0);
	//Don't invert transmit data, do enable interrupts for errors
	UART0->C3 = UART0_C3_TXINV(0) | UART0_C3_ORIE(1) | UART0_C3_NEIE(1) | UART0_C3_FEIE(1) | UART0_C3_PEIE(1);
	//Clear error flags
	UART0->S1 = UART0_S1_OR(1) | UART0_S1_NF(1) | UART0_S1_FE(1) | UART0_S1_PF(1);
	//Send LSB first, do not invert received data
	UART0->S2 = UART0_S2_MSBF(0) | UART0_S2_RXINV(0);
	//Enable UART transmitter and receiver
	UART0->C2 |= UART0_C2_TE(1) | UART0_C2_RE(1);
}

void UART0_Transmit_Poll(uint8_t data){
	while (!(UART0->S1 & UART0_S1_TDRE_MASK))
		;
	UART0->D = data;
}

uint8_t UART0_Receive_Poll(void) {
	while (!(UART0->S1 & UART0_S1_RDRF_MASK))
		;
	return UART0->D;
}

void main(void){
	uint8_t c;
	BOARD_InitBootClocks();
	Init_UART0(9600);

	UART0_Transmit_Poll(0x7C);
	UART0_Transmit_Poll(0x80);  //backlight Off
	UART0_Transmit_Poll(0xFE);
	UART0_Transmit_Poll(0x01);  //clear LCD
	delay(1000);
	UART0_Transmit_Poll(0x7C);
	UART0_Transmit_Poll(0x9D);  //backlight ON
	delay(1000);
	UART0_Transmit_Poll('H');
	UART0_Transmit_Poll('E');
	UART0_Transmit_Poll('A');
	UART0_Transmit_Poll('R');
	UART0_Transmit_Poll('T');
	UART0_Transmit_Poll(' ');
	UART0_Transmit_Poll('R');
	UART0_Transmit_Poll('A');
	UART0_Transmit_Poll('T');
	UART0_Transmit_Poll('E');
	UART0_Transmit_Poll(':');
	UART0_Transmit_Poll('9');
	UART0_Transmit_Poll('5');


	while (1){

		//uint8_t c = UART0_Receive_Poll();
		//UART0_Transmit_Poll(64);
		//PRINTF("data recieved: 0x%x\r\n", c);
	}
}
