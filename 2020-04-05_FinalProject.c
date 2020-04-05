/*  Standard C Included Files */
#include <string.h>
/*  SDK Included Files */
#include "board.h"
#include "fsl_debug_console.h"
//#include "fsl_i2c.c"
#include "fsl_i2c.h"
#include "pin_mux.h"
#include <MKL25Z4.h>
#include "MAX32664.h"
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
// MAX32644 BIOMETRIC HUB
// MAX32664GWEA
//I2C WRITE ADDRESS
#define I2C_MAX32664_SLAVE_ADDRESS (0xAA)
#define SET_OUTPUT_MODE (0x10)
////////////////////////////////////////////////////////////////////////////////
 //CODE
//Delay Function
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
		delay(2000);
		PRINTF("MAX32664 is ready to accept I2C commands\r\n");
}
// I2C Initialization Code
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
// Write single byte
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

// Read single byte
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
//Write Multiple Bytes
uint8_t i2c_write_bytes(uint8_t slaveAddress, uint8_t familyByte, uint8_t indexByte ,uint8_t *data2, uint8_t data_count){
	uint8_t data1;
	uint8_t dummy;
	int8_t num_bytes_write = 0;
	I2C_TRAN; 								/*set to transmit mode*/
	I2C_M_START; 							/*send start*/
	I2C1->D = slaveAddress;					/*send device address (write)*/
	I2C_WAIT;								/*wait for completion*/
	I2C1->D = familyByte;					/*send register address */
	I2C_WAIT;								/*wait for completion*/
	I2C1->D = indexByte;					/*send register address */
	I2C_WAIT;								/*wait for completion*/
	do{
		ACK;								/*tell HW to send ACK after read*/
		I2C1->D = data2[num_bytes_write++]; 	/*read data*/
		I2C_WAIT;							/*wait for completion*/
		}while (num_bytes_write < data_count-2);
	I2C_M_STOP;
	delay(20);
	I2C_M_START; 							/*repeated start*/
	I2C1->D = slaveAddress|0x01; 			/*send device address (read)*/
	I2C_WAIT;								/*wait for completion*/
	I2C_REC;								/*set to receive mode*/
	ACK;									/*tell HW to send ACK after read*/
	dummy = I2C1->D;						/*dummy read to start I2C read*/
	I2C_WAIT;								/*wait for completion*/
	NACK;									/*tell HW to send NACK after read*/
	data1 = I2C1->D;						/*read data*/
	I2C_WAIT;								/*wait for completion*/
	I2C_M_STOP;								/*send stop*/
	return data1;
}
//Read Multiple Bytes
uint8_t i2c_read_bytes(uint8_t slaveAddress, uint8_t familyByte, uint8_t indexByte ,uint8_t extraByte ,uint8_t *data, uint8_t data_count){
	uint8_t dummy;
	int8_t num_bytes_read = 0;
	I2C_TRAN; 								/*set to transmit mode*/
	I2C_M_START; 							/*send start*/
	I2C1->D = slaveAddress;					/*send device address (write)*/
	I2C_WAIT;								/*wait for completion*/
	I2C1->D = familyByte;					/*send register address */
	I2C_WAIT;								/*wait for completion*/
	I2C1->D = indexByte;					/*send register address */
	I2C_WAIT;								/*wait for completion*/
	I2C1->D = extraByte;					/*send register address */
	I2C_WAIT;								/*wait for completion*/
	I2C_M_STOP;
	delay(10);
	I2C_M_START; 							/*repeated start*/
	I2C1->D = slaveAddress|0x01; 			/*send device address (read)*/
	I2C_WAIT;								/*wait for completion*/
	I2C_REC;								/*set to receive mode*/
	ACK;									/*tell HW to send ACK after read*/
	dummy = I2C1->D;						/*dummy read to start I2C read*/
	I2C_WAIT;								/*wait for completion*/
	do{
		ACK;								/*tell HW to send ACK after read*/
		data[num_bytes_read++] = I2C1->D; 	/*read data*/
		I2C_WAIT;							/*wait for completion*/
	}while (num_bytes_read < data_count-2);
	NACK;									/*tell HW to send NACK after read*/
	data[num_bytes_read++] = I2C1->D;		/*read data*/
	I2C_WAIT;								/*wait for completion*/
	I2C_M_STOP;								/*send stop*/
}
//Main Function Code
int main(void){
	//uint8_t intPinValue = 0x25;    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

	Init_MAX32664();
	Init_i2c();
	PRINTF("\r\nIt is time to interface with this little guy!!!!\r\n");
	PRINTF("Read device mode\r\n");
	uint16_t numBytes = 2;
	uint8_t data[numBytes];
	uint8_t indexByte = 0x00;
	uint8_t extraByte = 0x00;
	i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,DEVICE_MODE,indexByte,extraByte,data, numBytes);
	PRINTF("Status Byte: 0x%x\r\n", data[0]);
	if(data[0] == 0x00){
			    PRINTF("No error\r\n");
			 }else {
			    	PRINTF("Error\r\n");
			  }
	PRINTF("Response: 0x%x\r\n", data[1]);
	    if (data[1] == 0x00){
		    PRINTF("Device is in application operating mode");
	    }else if (data[1] == 0x08){
	    		PRINTF("Device is in bootloader operating mode");
	    }
	PRINTF("\r\n\r\n");
//////
	PRINTF("Read the hub sensor version\r\n");
	numBytes = 4;
	//data[numBytes];
	indexByte = 0x03;
	extraByte = 0x00;
	i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,IDENTITY,indexByte,extraByte,data, numBytes);
	PRINTF("Status Byte: 0x%x\r\n", data[0]);
	if(data[0] == 0x00){
			    PRINTF("No error\r\n");
			 }else {
			    	PRINTF("Error\r\n");
			  }
	PRINTF("Response: 0x%x, 0x%x, 0x%x\r\n", data[1], data[2], data[3]);
	    if ((data[1] == 0x0a) & (data[2] == 0x01) & (data[3] == 0x00)){
		    PRINTF("Version is 10.1.0");
		    }else{
		    		PRINTF("Unknown version");
		    }
		    PRINTF("\r\n\r\n");
 //////
	PRINTF("Get the MAX30101 register attributes\r\n");
	numBytes = 4;
	//data[numBytes];
	indexByte = 0x03;
	extraByte = 0x00;
	i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,GET_ATTRIBUTES_OF_THE_AFE,indexByte,extraByte,data, numBytes);
	PRINTF("Status Byte: 0x%x\r\n", data[0]);
		if(data[0] == 0x00){
		    PRINTF("No error\r\n");
		 }else {
		    	PRINTF("Error\r\n");
		  }
	PRINTF("Response: 0x%x, 0x%x\r\n",data[1], data[2]);
	PRINTF("\r\n\r\n");
//
	PRINTF("Read all the MAX30101 registers\r\n");
		uint16_t numBytes55 = 256;
		uint8_t data55[numBytes55];
		uint8_t indexByte55 = 0x03;
		uint8_t extraByte55 = 0x00;
		i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,DUMP_REGISTERS,indexByte55,extraByte55,data55, numBytes55);
		for (uint32_t i = 0U; i < numBytes55; i++)
			    {
			        if (i % 8 == 0)
			        {
			            PRINTF("\r\n");
			        }
			        PRINTF("0x%2x  ", data55[i]);
			    }
			    PRINTF("\r\n\r\n");

	   /* numBytes = 1;
	    uint8_t data2[] = {0x03};
	    indexByte = 0x00;
	    uint8_t ivan = 0x05;
	    ivan = i2c_write_bytes(I2C_MAX32664_SLAVE_ADDRESS,SET_OUTPUT_MODE,indexByte,data2, numBytes);
	    PRINTF("Status Register is: 0x%x\r\n", ivan);*/
	    PRINTF("End of Code here\r\n\r\n");
	while(1){
		;
			}
}
