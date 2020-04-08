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
#include <stdio.h>
/*******************************************************************************
/*******************************************************************************
 * Definitions*/
#define UART_OVERSAMPLE_RATE (16)
#define SYS_CLOCK (48e6)

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
//Arrays
uint8_t data[256];


////////////////////////////////////////////////////////////////////////////////
 //CODE

//UART
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
//Write Multiple Bytes
uint8_t i2c_write_bytes(uint8_t slaveAddress, uint8_t familyByte, uint8_t indexByte ,uint8_t *data2, uint8_t data_count){
	uint8_t data1;
	uint8_t dummy;
	uint8_t num_bytes_write = 0;
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
	delay(200);
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
	uint8_t num_bytes_read = 0;
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
	delay(200);
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
	//uint8_t intPinValue = 0x25;
	BOARD_InitPins();
    BOARD_BootClockRUN();
    //BOARD_InitDebugConsole();

	Init_UART0(9600);
	UART0_Transmit_Poll(0x7C);
	UART0_Transmit_Poll(0x80);  //backlight Off
	UART0_Transmit_Poll(0xFE);
	UART0_Transmit_Poll(0x01);  //clear LCD
	delay(1000);
	UART0_Transmit_Poll(0x7C);
	UART0_Transmit_Poll(0x9D);  //backlight ON
	delay(1000);
	//int i = 1983;
	//uint8_t  data001[20];
	//sprintf(data001, "%d", i);
	char str_ivan[32]= "Display         Initialized";

	//uint8_t  data001[20];
	//sprintf(data001, "%c", i);

	UART0_Transmit_Poll(str_ivan[0]);
	UART0_Transmit_Poll(str_ivan[1]);
	UART0_Transmit_Poll(str_ivan[2]);
	UART0_Transmit_Poll(str_ivan[3]);
	UART0_Transmit_Poll(str_ivan[4]);
	UART0_Transmit_Poll(str_ivan[5]);
	UART0_Transmit_Poll(str_ivan[6]);
	UART0_Transmit_Poll(str_ivan[7]);
	UART0_Transmit_Poll(str_ivan[8]);
	UART0_Transmit_Poll(str_ivan[9]);
	UART0_Transmit_Poll(str_ivan[10]);
	UART0_Transmit_Poll(str_ivan[11]);
	UART0_Transmit_Poll(str_ivan[12]);
	UART0_Transmit_Poll(str_ivan[13]);
	UART0_Transmit_Poll(str_ivan[14]);
	UART0_Transmit_Poll(str_ivan[15]);
	UART0_Transmit_Poll(str_ivan[16]);
	UART0_Transmit_Poll(str_ivan[17]);
	UART0_Transmit_Poll(str_ivan[18]);
	UART0_Transmit_Poll(str_ivan[19]);
	UART0_Transmit_Poll(str_ivan[20]);
	UART0_Transmit_Poll(str_ivan[21]);
	UART0_Transmit_Poll(str_ivan[22]);
	UART0_Transmit_Poll(str_ivan[23]);
	UART0_Transmit_Poll(str_ivan[24]);
	UART0_Transmit_Poll(str_ivan[25]);
	UART0_Transmit_Poll(str_ivan[26]);

	Init_MAX32664();
	Init_i2c();
	PRINTF("\r\nIt is time to interface with this little guy!!!!\r\n");
	PRINTF("Read device mode\r\n");
	uint8_t indexByte = 0x00;
	uint8_t extraByte = 0x00;
	i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,DEVICE_MODE,indexByte,extraByte,data,2);
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
	i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,IDENTITY,0x03,0x00,data,4);
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
	i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,GET_ATTRIBUTES_OF_THE_AFE,0x03,0x00,data,4);
	PRINTF("Status Byte: 0x%x\r\n", data[0]);
		if(data[0] == 0x00){
		    PRINTF("No error\r\n");
		 }else {
		    	PRINTF("Error\r\n");
		  }
	PRINTF("Response: 0x%x, 0x%x\r\n",data[1], data[2]);
	PRINTF("\r\n\r\n");
//
	/*PRINTF("Read all the MAX30101 registers\r\n");
		//uint8_t data55[numBytes55];
		uint8_t data55[255];
		i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,DUMP_REGISTERS,0x03,0x00,data55,255);
		PRINTF("Status Byte: 0x%x\r\n", data55[0]);
		if(data55[0] == 0x00){
				    PRINTF("No error\r\n");
				 }else {
				    	PRINTF("Error\r\n");
				  }
		PRINTF("0x%x, 0x%x, 0x%x, 0x%x\r\n", data55[0],data55[1],data55[2],data55[3]);
		PRINTF("0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n", data55[4],data55[5],data55[6],data55[7],data55[8]);
		PRINTF("0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n", data55[9],data55[10],data55[11],data55[12],data55[13]);
		PRINTF("0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n", data55[14],data55[15],data55[16],data55[17],data55[18]);
		PRINTF("0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n", data55[19],data55[20],data55[21],data55[22],data55[23]);
		PRINTF("0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n", data55[24],data55[25],data55[26],data55[27],data55[28]);*/
	/*for (uint8_t i = 0; i < 255; i++)
			    {
			        if (i % 8 == 0)
			        {
			            PRINTF("\r\n");
			        }
			        PRINTF("0x%x  ", data55[i]);
			    }*/
		//PRINTF("value of register 7 is:0x%x", data55[13]);
			    //PRINTF("\r\n\r\n");

			    //
	PRINTF("Read the MAX30101 register 7\r\n");
	i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,READ_REGISTER,0x03,0x07,data,2);
	PRINTF("Status Byte: 0x%x\r\n", data[0]);
	if(data[0] == 0x00){
		PRINTF("No error\r\n");
	}else {
		PRINTF("Error\r\n");
		}
	PRINTF("Response: 0x%x, 0x%x\r\n",data[0], data[1]);
	PRINTF("\r\n\r\n");

	    PRINTF("Set output mode to sensor and algorithm data\r\n");
	    uint8_t data2[] = {0x03};
	    uint8_t ivan = 0x05;
	    ivan = i2c_write_bytes(I2C_MAX32664_SLAVE_ADDRESS,SET_OUTPUT_MODE,0x00,data2, 1);
	    PRINTF("Status Byte is: 0x%x\r\n", ivan);
	    if(ivan == 0x00){
	    		    PRINTF("No error\r\n");
	    		 }else {
	    		    	PRINTF("Error\r\n");
	    		  }

	    PRINTF("Set FIFO threshold to 0x0F. Increase or decrease this value if you want more or fewer samples per interrupt\r\n");
	    uint8_t data3[] = {0x0F};
	    ivan = 0x05;
	    ivan = i2c_write_bytes(I2C_MAX32664_SLAVE_ADDRESS,SET_OUTPUT_MODE,0x01,data3, 1);
	    PRINTF("Status Byte is: 0x%x\r\n", ivan);
	    if(ivan == 0x00){
	    	    		    PRINTF("No error\r\n");
	    	    		 }else {
	    	    		    	PRINTF("Error\r\n");
	    	    		  }

	    	    PRINTF("Enable AGC algorithm\r\n");
	    	    	    	    uint8_t data4[] = {0x01};
	    	    	    	    ivan = 0x05;
	    	    	    	    ivan = i2c_write_bytes(I2C_MAX32664_SLAVE_ADDRESS,ALGORITHM_MODE_ENABLE,0x00,data4, 1);
	    	    	    	    PRINTF("Status Byte is: 0x%x\r\n", ivan);
	    	    	    	    if(ivan == 0x00){
	    	    	    	    		    PRINTF("No error\r\n");
	    	    	    	    		 }else {
	    	    	    	    		    	PRINTF("Error\r\n");
	    	    	    	    		  }
	    	    	    	    PRINTF("Enable the MAX30101 sensor\r\n");
	    	    	    	    	    	    	    	    uint8_t data5[] = {0x01};
	    	    	    	    	    	    	    	   ivan = 0x05;
	    	    	    	    	    	    	    	    ivan = i2c_write_bytes(I2C_MAX32664_SLAVE_ADDRESS,SENSOR_MODE_ENABLE,0x03,data5, 1);
	    	    	    	    	    	    	    	    PRINTF("Status Byte is: 0x%x\r\n", ivan);
	    	    	    	    	    	    	    	    if(ivan == 0x00){
	    	    	    	    	    	    	    	    		    PRINTF("No error\r\n");
	    	    	    	    	    	    	    	    		 }else {
	    	    	    	    	    	    	    	    		    	PRINTF("Error\r\n");
	    	    	    	    	    	    	    	    		  }


		PRINTF("Enable MaximFast algorithm mode 1\r\n");
	    uint8_t data6[] = {0x01};
	    ivan = 0x05;
		ivan = i2c_write_bytes(I2C_MAX32664_SLAVE_ADDRESS,ALGORITHM_MODE_ENABLE,0x02,data6, 1);
		PRINTF("Status Byte is: 0x%x\r\n", ivan);
		if(ivan == 0x00){
			   PRINTF("No error\r\n");
		}else {
		PRINTF("Error\r\n");
		}

	    PRINTF("Read the sensor hub status\r\n");
	    uint8_t data001[5];
		i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,READ_SENSOR_HUB_STATUS,0x00,0x00,data001,3);
		PRINTF("Status Byte: 0x%x\r\n", data001[0]);
					    		if(data001[0] == 0x00){
					    		    PRINTF("No error\r\n");
					    		 }else {
					    		    	PRINTF("Error\r\n");
					    		  }
					    	PRINTF("Response: 0x%x, 0x%x\r\n",data001[0], data001[1]);
					    	PRINTF("\r\n\r\n");

					    	PRINTF("Get the number of samples in the FIFO\r\n");
					    	 uint8_t data002[5];
					    			i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,READ_OUTPUT_FIFO,0x00,0x00,data002,3);
					    			PRINTF("Status Byte: 0x%x\r\n", data002[0]);
					    						    		if(data002[0] == 0x00){
					    						    		    PRINTF("No error\r\n");
					    						    		 }else {
					    						    		    	PRINTF("Error\r\n");
					    						    		  }
					    						    	PRINTF("Response: 0x%x, 0x%x\r\n",data002[0], data002[1]);
					    						    	PRINTF("\r\n\r\n");

		PRINTF("Read the data stored in the FIFO\r\n");
		uint8_t data003[255];
		i2c_read_bytes(I2C_MAX32664_SLAVE_ADDRESS,READ_OUTPUT_FIFO,0x01,0x00,data003,255);
		PRINTF("Status Byte: 0x%x\r\n", data003[0]);
		if(data003[0] == 0x00){
		PRINTF("No error\r\n");
		}else {
		PRINTF("Error\r\n");
		}
		PRINTF("\r\n\r\n");
		//PRINTF("MaximFast State Machine Status Codes:\r\n0: No object detected\r\n1: Object detected\r\n2: Object other than finger detected\r\n3: Finger detected\r\n 0x%x\r\n", data003[37]);
				if (data003[37] == 0x00){
					PRINTF("No Object Detected\r\n");
				}else if(data003[37] == 0x01){
					PRINTF("Object Detected\r\n");
				}else if(data003[37] == 0x02){
					PRINTF("Object other than finger detected\r\n");
				}else if(data003[37] == 0x03){
					PRINTF("Finger Detected\r\n");
				}
		PRINTF("Patient Results:\r\n");
		//("IR COUNTS: 0x%x, 0x%x, 0x%x\r\n", data003[1],data003[2],data003[3]);
		//PRINTF("RED COUNTS: 0x%x, 0x%x, 0x%x\r\n", data003[4],data003[5],data003[6]);
		//PRINTF("LED3 COUNTS: 0x%x, 0x%x, 0x%x\r\n", data003[7],data003[8],data003[9]);
		//PRINTF("LED4 COUNTS: 0x%x, 0x%x, 0x%x\r\n", data003[10],data003[11],data003[12]);
		//PRINTF("X accelerometer: 0x%x, 0x%x\r\n", data003[13],data003[14]);
		//PRINTF("Y accelerometer: 0x%x, 0x%x\r\n", data003[15],data003[16]);
		//PRINTF("Z accelerometer: 0x%x, 0x%x\r\n", data003[17],data003[18]);
		//PRINTF("Heart Rate: 0x%x, 0x%x\r\n", data003[19],data003[20]);
		uint16_t num = ((data003[19] << 8) + data003[20]);
		float hrVal= num/10;
		PRINTF("Heart Rate:%.f\ Bpm\r\n", hrVal);
		UART0_Transmit_Poll(0xFE);  //clear LCD
		UART0_Transmit_Poll(0x01);  //clear LCD
		delay(1000);
		//int i = 1983;
		//uint8_t  data501[20];
		//sprintf(data001, "%d", i);
		char str_ivan1[10]= "HR:";
		//uint8_t  data001[20];
		//sprintf(data001, "%c", i);
		UART0_Transmit_Poll(str_ivan1[0]);
		UART0_Transmit_Poll(str_ivan1[1]);
		UART0_Transmit_Poll(str_ivan1[2]);

		//int i = 1983;
		uint8_t  data501[20];
		sprintf(data501, "%.1f", hrVal);
		UART0_Transmit_Poll(data501[0]);
		UART0_Transmit_Poll(data501[1]);
		UART0_Transmit_Poll(data501[2]);
		UART0_Transmit_Poll(data501[3]);
		UART0_Transmit_Poll(' ');
		UART0_Transmit_Poll('b');
		UART0_Transmit_Poll('p');
		UART0_Transmit_Poll('m');
		UART0_Transmit_Poll(' ');
		UART0_Transmit_Poll(' ');
		UART0_Transmit_Poll(' ');
		UART0_Transmit_Poll(' ');
		UART0_Transmit_Poll(' ');

		//PRINTF("Confidence Level: 0x%x\r\n", data003[21]);
		float confidencePer = (data003[21])*0.39215;
		PRINTF("Confidence Level Decimal: %.f\ %%\r\n", confidencePer);
		//PRINTF("SpO2 value (0-100%): 0x%x, 0x%x\r\n", data003[22],data003[23]);
		uint16_t num2 = ((data003[22] << 8) + data003[23]);
		float hrVal1= num2/10;
		PRINTF("SpO2 value (0-100%):%.f\ %%\r\n", hrVal1);
		char str_ivan2[10]= "SpO2:";
			//uint8_t  data001[20];
			//sprintf(data001, "%c", i);
			UART0_Transmit_Poll(str_ivan2[0]);
			UART0_Transmit_Poll(str_ivan2[1]);
			UART0_Transmit_Poll(str_ivan2[2]);
			UART0_Transmit_Poll(str_ivan2[3]);
			UART0_Transmit_Poll(str_ivan2[4]);

			//int i = 1983;
			uint8_t  data502[20];
			sprintf(data502, "%.1f", hrVal1);
			UART0_Transmit_Poll(data502[0]);
			UART0_Transmit_Poll(data502[1]);
			UART0_Transmit_Poll(data502[2]);
			UART0_Transmit_Poll(data502[3]);
			UART0_Transmit_Poll('%');
		/*
		PRINTF("0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n", data003[1],data003[2],data003[3],data003[4],data003[5],data003[6],data003[7],data003[8],data003[9]);
		PRINTF("0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n", data003[10],data003[11],data003[12],data003[13],data003[14],data003[15],data003[16],data003[17],data003[18]);
		*/
		PRINTF("\r\n\r\n");
	    	    	    	    PRINTF("\r\n\r\n");
	    PRINTF("End of Code here\r\n\r\n");
	while(1){
		;
			}
}

