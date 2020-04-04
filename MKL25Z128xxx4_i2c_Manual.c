/*  Standard C Included Files */
#include <string.h>
/*  SDK Included Files */
#include "board.h"
#include "fsl_debug_console.h"
//#include "fsl_i2c.c"
#include "fsl_i2c.h"
#include "pin_mux.h"
#include <MKL25Z4.h>
/*******************************************************************************
 * Definitions*/
#define MASK(x)  (1UL << (x))
// RST PIN
#define RSTN_PIN (8) 		// Define on Port C
// MFIO PIN
#define MFIO_PIN (9)       	// Define on Port C
#define I2C_M_START I2C1->C1 |= I2C_C1_MST_MASK
#define I2C_M_STOP I2C1->C1 &= ~I2C_C1_MST_MASK
#define I2C_M_RSTART I2C1->C1 |= I2C_C1_RSTA_MASK
#define I2C_TRAN I2C1->C1 |= I2C_C1_TX_MASK
#define I2C_REC I2C1->C1 &= ~I2C_C1_TX_MASK
#define BUSY_ACK while(I2C1->S & 0x01)
#define TRANS_COMP while(!(I2C1->S & 0x80))
#define I2C_WAIT while((I2C1->S & I2C_S_IICIF_MASK)==0){} \
					I2C1->S |= I2C_S_IICIF_MASK
#define NACK I2C1->C1 |= I2C_C1_TXAK_MASK
#define ACK I2C1->C1 &= ~I2C_C1_TXAK_MASK
////////////////////////////////////////////////////////////////////////////////
// MAX3010 PULSE OXIMETER AND HARTH RATE SENSOR
// MAX30101EFD
// I2C READ ADDRESS
//#define I2C_MAX3010_READ_ADDRESS (0xAB)
//I2C WRITE ADDRESS
//#define I2C_MAX3010_WRITE_ADDRESS (0xAA)
////////////////////////////////////////////////////////////////////////////////
#define GY521_SLAVE_ADDRESS (0xD0)
#define WHO_AM_I (0x75)
#define INT_PIN_CFG (0x37)
/*
 *****************************************************************************
#define EXAMPLE_I2C_MASTER_BASEADDR I2C0
#define I2C_MASTER_CLK_SRC I2C0_CLK_SRC
#define I2C_MASTER_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)
#define I2C_MASTER_SLAVE_ADDR_7BIT 0x7EU
#define I2C_BAUDRATE 100000U
#define I2C_DATA_LENGTH 1U
////////////////////////////////////////////////////////////////////////////////
// MAX32644 BIOMETRIC HUB
// MAX32664GWEA
// I2C ADDRESS
#define I2C_MAX32644_ADDRESS 0x55



 //Code
 //
*/
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
//Sequence the MAX32664 to enter application mode
void Init_MAX32664(void)
{
		PRINTF("Initializing MAX32664...\r\n");
		// Enable Clock to port C
		SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
		// Make  pins GPIO
		PORTC->PCR[RSTN_PIN] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[RSTN_PIN] |= PORT_PCR_MUX(1);
		PORTC->PCR[MFIO_PIN] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[MFIO_PIN] |= PORT_PCR_MUX(1);
		//set bits to outputs
		PTC->PDDR |= MASK(RSTN_PIN) | MASK(MFIO_PIN);
		PRINTF("Setting the RSTN pin to low for 10ms\r\n");
		//Set the RSTN pin to low for 10ms
		PTC->PCOR = MASK(RSTN_PIN);
		delay(5);
		PRINTF("Setting the MFIO pin to high\r\n");
		//While RSTN is low, set the MFIO pin to high
		PTC->PSOR = MASK(MFIO_PIN);
		delay(5);
		PRINTF("Setting the RSTN pin to high\r\n");
		//After the 10ms has elapsed, set the RSTN pin to high. (MFIO pin should be set at least 1ms before RSTN pin is set to high
		PTC->PSOR = MASK(RSTN_PIN);
		//After an additional 50ms has elapsed, the MAX32664 is in application mode and the application performs its initialization of the application software
		delay(50);
		PRINTF("MAX32664 is in application mode, congratulations\r\n");
		delay(1000);
		PRINTF("MAX32664 is ready to accept I2C commands\r\n");
}

void Init_i2c(void){
	PRINTF("I2C Initialization...\r\n");
	//clock i2c peripheral and portE
	SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	//set pins to I2C function
	PORTC->PCR[1] |= PORT_PCR_MUX(2);
	PORTC->PCR[2] |= PORT_PCR_MUX(2);
	//set to 400k baud
	//baud = bus freq/ (scl_div*mul)
	//24MHz/400KHz = 60; icr=0x11 sets scl div to 56
	I2C1->F = I2C_F_ICR(0x11) | I2C_F_MULT(0);
	//enable i2c and set to master mode
	I2C1->C1 |= (I2C_C1_IICEN_MASK);
	//select high drive mode
	I2C1->C2 |= (I2C_C2_HDRS_MASK);
	PRINTF("I2C Initialization completed\r\n");
}

uint8_t i2c_write_byte(uint8_t slaveAddress, uint8_t slaveRegister, uint8_t data){
	I2C_TRAN; /*set to transmit mode */
	I2C_M_START; /*send start */
	I2C1->D = slaveAddress; /*send dev address */
	I2C_WAIT; /*wait for ack */
	I2C1->D = slaveRegister; /*send write address */
	I2C_WAIT;
	I2C1->D = data; /*send data */
	I2C_WAIT;
	I2C_M_STOP;
}

uint8_t i2c_read_byte(uint8_t slaveAddress, uint8_t slaveRegister){
	uint8_t data;
	I2C_TRAN; /*set to transmit mode */
	I2C_M_START; /*send start */
	I2C1->D = slaveAddress; /*send dev address */
	I2C_WAIT; /*wait for completion */
	I2C1->D = slaveRegister; /*send read address */
	I2C_WAIT; /*wait for completion */
	I2C_M_RSTART; /*repeated start */
	I2C1->D = (slaveAddress|0x1); /*send dev address (read) */
	I2C_WAIT; /*wait for completion */
	I2C_REC; /*set to recieve mode */
	NACK; /*set NACK after read*/
	data = I2C1->D; /*dummy read */
	I2C_WAIT; /*wait for completion */
	I2C_M_STOP; /*send stop */
	data = I2C1->D; /*read data */
	return data;
}

int main(void){
	uint8_t intPinValue = 0x25;
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

	Init_MAX32664();
	Init_i2c();
	PRINTF("Start with the Fun....!!\r\n");
	PRINTF("Read WHO_AM_I Register value\r\n");
	PRINTF("WHO_AM_I value is: 0x%x\r\n",i2c_read_byte(GY521_SLAVE_ADDRESS, WHO_AM_I));
	PRINTF("Write value to a register\r\n");
	PRINTF("Value of written to INT_PIN_CFG Register 0x%x is: 0x%x\r\n",INT_PIN_CFG, intPinValue);
	i2c_write_byte(0xD0, INT_PIN_CFG, intPinValue);
	delay(3000);
	PRINTF("INT_PIN_CFG Value read is: 0x%x\r\n",i2c_read_byte(GY521_SLAVE_ADDRESS, INT_PIN_CFG));

	while(1){
		;
			}
}
