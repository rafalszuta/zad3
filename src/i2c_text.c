/*
 * @brief I2C bus master example using the ROM interrupt API
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* I2C master handle and memory for ROM API */
static I2C_HANDLE_T *i2cHandleMaster;

/* Use a buffer size larger than the expected return value of
   i2c_get_mem_size() for the static I2C handle type */
static uint32_t i2cMasterHandleMEM[0x20];

/* 100kbps I2C bit-rate */
#define I2C_BITRATE             (100000)

/** 7-bit and 10-bit I2C addresses */
#define I2C_ADDR_7BIT_1     (0x20 << 1) //i2c expander

#define GPA_REG 0x00
#define GPB_REG 0x10

#define I2C_RD_CMD_BIT      (1)

static volatile int intErrCode;

static volatile unsigned int ledValues[] = {0x31, 0x03, 0xE3, 0x61};  //POLE
#define SHUTDOWN 10
#define DELAY_TIME 10000

/* SysTick rate in Hz */
#define TICKRATE_HZ (400)

/* Current state for LED control via I2C cases */
static volatile int state = 0;
static volatile int ledValue = 0;
static volatile uint8_t displayNumber = 0;
static volatile bool ledState = false;
#define LED0 0x01
#define LED1 0x02
#define LED2 0x04
#define LED3 0x08

static volatile uint8_t displays[] = { LED0, LED1, LED2, LED3 };

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Converts given value @a val into a hexadecimal string
 * and stores the result to @a dest with leading zeros
 * RETURN: Number of hexdigits excluding leading zeros
 */
static int Hex2Str(char *dest, uint32_t val)
{
	int i, ret = 0;
	for (i = 0; i < sizeof(val) * 2; i ++) {
		int idx = val & 0xF;
		dest[7 - i] = "0123456789ABCDEF"[idx];
		val >>= 4;
		if (idx)
			ret = i;
	}
	return ret + 1;
}

/* Prints a hexadecimal value with given string in front */
/* Using printf might cause text section overflow */
static void Print_Val(const char *str, uint32_t val)
{
	char buf[9];
	int ret;
	buf[8] = 0;
	DEBUGSTR(str);
	ret = Hex2Str(buf, val);
	DEBUGSTR(&buf[8 - ret]);
	DEBUGSTR("\r\n");
}

/* Initializes pin muxing for I2C interface - note that SystemInit() may
   already setup your pin muxing at system startup */
static void Init_I2C_PinMux(void)
{
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Connect the I2C_SDA and I2C_SCL signals to port pins(P0.10, P0.11) */
	Chip_SWM_MovablePinAssign(SWM_I2C_SDA_IO, 10);
	Chip_SWM_MovablePinAssign(SWM_I2C_SCL_IO, 11);

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}

/* Turn on LED to indicate an error */
static void errorI2C(void)
{
	Board_LED_Set(0, true);
	while (1) {}
}

/* Setup I2C handle and parameters */
static void setupI2CMaster()
{
	/* Enable I2C clock and reset I2C peripheral - the boot ROM does not
	   do this */
	Chip_I2C_Init(LPC_I2C);

	/* Perform a sanity check on the storage allocation */
	if (LPC_I2CD_API->i2c_get_mem_size() > sizeof(i2cMasterHandleMEM)) {
		/* Example only: this should never happen and probably isn't needed for
		   most I2C code. */
		errorI2C();
	}

	/* Setup the I2C handle */
	i2cHandleMaster = LPC_I2CD_API->i2c_setup(LPC_I2C_BASE, i2cMasterHandleMEM);
	if (i2cHandleMaster == NULL) {
		errorI2C();
	}

	/* Set I2C bitrate */
	if (LPC_I2CD_API->i2c_set_bitrate(i2cHandleMaster, Chip_Clock_GetSystemClockRate(),
									  I2C_BITRATE) != LPC_OK) {
		errorI2C();
	}
}

/* I2C interrupt callback, called on completion of I2C operation when in
   interrupt mode. Called in interrupt context. */
static void cbI2CComplete(uint32_t err_code, uint32_t n)
{
	intErrCode = (int) err_code;
}

/* Master transmit in interrupt mode */
static void sendI2CMaster(uint16_t AddressI2C, bool ledStateOut, uint8_t regAddr)
{
	uint8_t SendData[10];
	I2C_PARAM_T param;
	I2C_RESULT_T result;
	ErrorCode_t error_code;
	int index = 0;

	SendData[index++] = (uint8_t) AddressI2C;
	SendData[index++] = (uint8_t) regAddr;			/* I2C device regAddr */
	SendData[index++] = (uint8_t) 0x00;

	SendData[index++] = (uint8_t) ledValues[ledValue];
	SendData[index++] = (uint8_t) displayNumber;

	/* Setup I2C parameters for number of bytes with stop - appears as follows on bus:
	   Start - address7 or address10upper - ack
	   (10 bits addressing only) address10lower - ack
	   value 1 - ack
	   value 2 - ack - stop */
	param.num_bytes_send    = index;
	param.buffer_ptr_send   = &SendData[0];
	param.num_bytes_rec     = 0;
	param.stop_flag         = 1;
	param.func_pt           = cbI2CComplete;

	/* Set timeout (much) greater than the transfer length */
	LPC_I2CD_API->i2c_set_timeout(i2cHandleMaster, 100000);

	/* Do master write transfer */
	intErrCode = -1;

	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_transmit_intr(i2cHandleMaster, &param, &result);

	/* Sleep until transfer is complete, but allow IRQ to wake system
	   to handle I2C IRQ */
	while (intErrCode == -1) {
		__WFI();
	}

	/* Cast saved error code from callback */
	error_code = (ErrorCode_t) intErrCode;

//	/* Completed without erors? */
	if (error_code != LPC_OK) {
		/* Likely cause is NAK */
		Print_Val("i2c_master_transmit error code : 0x", error_code);
		errorI2C();
	}

	/* Note results are only valid when there are no errors */
}

static void showNumberOnDisplay(uint8_t number, uint8_t display)
{
	ledValue = number;
	displayNumber = display;
	/* Set LED state on slave device */
//	sendI2CMaster(I2C_ADDR_7BIT_1, ledState, GPB_REG);
	sendI2CMaster(I2C_ADDR_7BIT_1, ledState, GPA_REG);
	__WFI();
}

static void showNumber(int number)
{
	int display = 0;
	while(number > 0 && display < 4)
	{
		showNumberOnDisplay(number % 10, displays[display++]);
		showNumberOnDisplay(SHUTDOWN, displays[display - 1]);
		number /= 10;
	}
}
/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	I2C interrupt handler
 * @return	Nothing
 */
void I2C_IRQHandler(void)
{
	/* Call I2C ISR function in ROM with the I2C handle */
	LPC_I2CD_API->i2c_isr_handler(i2cHandleMaster);
}

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	static int ticks = 0;

	ticks++;
	if (ticks > TICKRATE_HZ) {
		ticks = 0;
		state ^= 1;	/* Toggle the state of bit 1 */
	}
}



/**
 * @brief		Function delay
 * @param[in]	delay - delay time
 */
static void delay(uint32_t delay) {
	while (delay--) {
	}
}




/**
 * @brief	Main routine for I2C example
 * @return	Function should not exit
 */
int main(void)
{
	int lastState = -1;

	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();

	Board_LED_Set(0, false);

	/* Setup I2C pin muxing */
	Init_I2C_PinMux();

	/* Allocate I2C handle, setup I2C rate, and initialize I2C
	   clocking */
	setupI2CMaster();

	/* Enable the interrupt for the I2C */
	NVIC_EnableIRQ(I2C_IRQn);

	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);



	/* Toggle LED on other board via I2C */
	while (1) {


		for (int i = 0; i < 4; i++) {
					ledValue = i;
					switch (i) {
					case 0:
						displayNumber = LED0;
						break;
					case 1:
						displayNumber = LED1;
						break;
					case 2:
						displayNumber = LED2;
						break;
					case 3:
						displayNumber = LED3;
						break;

					default:
						break;

					}
					sendI2CMaster(I2C_ADDR_7BIT_1, ledState, GPB_REG);
					sendI2CMaster(I2C_ADDR_7BIT_1, ledState, GPA_REG);
					delay(DELAY_TIME);

				}


		/* Handle states */
		switch (state) {
		case 0:
			/* Toggle LED state value */
			ledState = (ledState == 0);
			break;

		case 1:
		default:
			break;
		}

		lastState = state;

		/* Match this board's LED to other boards state */
		Board_LED_Set(0, ledState);
	}
}
