/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*  SDK Included Files */
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* I2C source clock */
#define ACCEL_I2C_CLK_SRC I2C1_CLK_SRC
#define ACCEL_I2C_CLK_FREQ CLOCK_GetFreq(I2C1_CLK_SRC)
#define I2C_BAUDRATE 100000U

#define I2C_RELEASE_SDA_PORT PORTC
#define I2C_RELEASE_SCL_PORT PORTC
#define I2C_RELEASE_SDA_GPIO GPIOC
#define I2C_RELEASE_SDA_PIN 2U
#define I2C_RELEASE_SCL_GPIO GPIOC
#define I2C_RELEASE_SCL_PIN 1U
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE 100000U
#define FXOS8700_WHOAMI 0x00U
#define MMA8451_WHOAMI 0x00U
#define ACCEL_STATUS 0x00U
#define ACCEL_XYZ_DATA_CFG 0x0EU
#define ACCEL_CTRL_REG1 0x2AU
/* FXOS8700 and MMA8451 have the same who_am_i register address. */
#define ACCEL_WHOAMI_REG 0x02U
#define ACCEL_READ_TIMES 10U

#define MASK(x)  (1UL << (x))
// RST PIN
#define RSTN_PIN (8) 		// Define on Port C
// MFIO PIN
#define MFIO_PIN (9)       	// Define on Port C
#define I2C_DATA_LENGTH 3U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static bool I2C_ReadAccelWhoAmI(void);
static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*  FXOS8700 and MMA8451 device address */
//const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};
const uint8_t g_accel_address[] = {0xAAU, 0xABU, 0x1CU, 0x1DU};
i2c_master_handle_t g_m_handle;

uint8_t g_accel_addr_found = 0x00;

volatile bool completionFlag = false;
volatile bool nakFlag = false;
uint8_t g_master_buff[I2C_DATA_LENGTH];

i2c_master_handle_t g_m_handle;
/*******************************************************************************
 * Code
 ******************************************************************************/

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
		delay(1500);
		PRINTF("MAX32664 is ready to accept I2C commands\r\n");
}

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA low */
    for (i = 0; i < 9; i++)
    {
        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
    {
        nakFlag = true;
    }
}



/*!
 * @brief Main function
 */
int main(void)
{
	i2c_master_config_t masterConfig;
	    uint32_t sourceClock;
	    i2c_master_transfer_t masterXfer;
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    BOARD_InitDebugConsole();
    Init_MAX32664();

    PRINTF("\r\nWriting Data..\r\n");

    PRINTF("Master will send data :");
    uint8_t g_master_buff[] = {0x00U, 0x04U, 0x08U};

        for (uint32_t i = 0U; i < I2C_DATA_LENGTH; i++)
        {
            if (i % 8 == 0)
            {
                PRINTF("\r\n");
            }
            PRINTF("0x%2x  ", g_master_buff[i]);
        }
        PRINTF("\r\n\r\n");
        I2C_MasterGetDefaultConfig(&masterConfig);
        masterConfig.baudRate_Bps = I2C_BAUDRATE;

        sourceClock = ACCEL_I2C_CLK_FREQ;

        I2C_MasterInit(I2C1, &masterConfig, sourceClock);

        memset(&g_m_handle, 0, sizeof(g_m_handle));
        memset(&masterXfer, 0, sizeof(masterXfer));

        masterXfer.slaveAddress = 0xAA;
        masterXfer.direction = kI2C_Write;
        masterXfer.subaddress = 0;
        masterXfer.subaddressSize = 0;
        masterXfer.data = g_master_buff;
        masterXfer.dataSize = I2C_DATA_LENGTH;
        masterXfer.flags = kI2C_TransferDefaultFlag;

        PRINTF("Values: 0x%x", I2C_MasterTransferBlocking(I2C1, &masterXfer));

    //I2C_WriteAccelReg(I2C1,0xAA, 0x02, 0x00);

    PRINTF("\r\nReading 0xAB, 0x00\r\n");





    PRINTF("\r\nEnd of I2C .\r\n");
    while (1)
    {
    }
}
