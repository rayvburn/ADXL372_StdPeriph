/*
 * acc_adxl372.c
 *
 *  Created on: 16.08.2018
 *      Author: user
 */

#include "ACC_ADXL372.h"

// static ADXL372 "class" variables for interrupt handling
// static struct 	ADXL372_AccelTriplet ADXL372_accel_triplet = {0,0,0};
// static uint8_t 	ADXL372_triplet_ready = 0;
static int16_t 	ADXL372_manual_z_offset = 8;

void ADXL372_Setup() {

	ADXL372_SetRegister_MEASURE();
	ADXL372_SetRegister_TIMING();
	ADXL372_SetRegister_FIFO_CTL();
	ADXL372_SetRegister_FIFO_SAMPLES();
	#ifdef ADXL372_ENABLE_INT
		ADXL372_InterruptsInit();
		ADXL372_SetRegister_INT2_MAP();
		ADXL372_SetRegister_INT1_MAP();
	#endif
	ADXL372_SetRegister_HPF();
	ADXL372_SetAxisOffsets();

	// Note that order of setup procedures isn't arbitrary
	ADXL372_SetRegister_POWER_CTL();
	/* This is the same as above, but above seems to create same
	 * kind of g threshold that triggers measurements. Why? */
	ADXL372_Write(ADXL372_POWER_CTL, 0x1F);

}

uint8_t ADXL372_ReadAccTriplet(struct ADXL372_AccelTriplet *triplet, uint8_t source) {

	uint8_t x_data_h = 0;
	uint8_t x_data_l = 0;
	uint8_t y_data_h = 0;
	uint8_t y_data_l = 0;
	uint8_t z_data_h = 0;
	uint8_t z_data_l = 0;

	switch ( source ) {

	case(ADXL372_READ_DIRECT):

		ADXL372_CSLineTake();
		// burst transfer, auto-increment of adress
		SPI_TransmitReceive(ADXL372_AddressToSPIData(ADXL372_XDATA_H, ADXL372_READ_MASK)); // ignore received data during transmit
		x_data_h = SPI_TransmitReceive(0xAA);
		x_data_l = SPI_TransmitReceive(0xAA);
		y_data_h = SPI_TransmitReceive(0xAA);
		y_data_l = SPI_TransmitReceive(0xAA);
		z_data_h = SPI_TransmitReceive(0xAA);
		z_data_l = SPI_TransmitReceive(0xAA);
		ADXL372_CSLineLeave();
		break;

	case(ADXL372_READ_FIFO):

		// burst transfer, auto-increment of register address not present (read from FIFO)
		if ( ADXL372_CheckFIFOSamplesNumber() > 3 ) {	// there must be at least 1 sample left in the FIFO after reading

			/* Note that first sample read from FIFO could not be connected with X axis - data order is not guaranteed in this read mode */
			/* In my case there is always the following order: Y, Z, X */
			ADXL372_CSLineTake();
			SPI_TransmitReceive(ADXL372_AddressToSPIData(ADXL372_FIFO_DATA, ADXL372_READ_MASK)); // ignore received data during transmit
			#ifdef ADXL372_FIFO_CHANGE_ORDER
				y_data_h = SPI_TransmitReceive(0xAA);
				y_data_l = SPI_TransmitReceive(0xAA);
			#else
				x_data_h = SPI_TransmitReceive(0xAA);
				x_data_l = SPI_TransmitReceive(0xAA);
			#endif
			ADXL372_CSLineLeave();

			ADXL372_CSLineTake();
			SPI_TransmitReceive(ADXL372_AddressToSPIData(ADXL372_FIFO_DATA, ADXL372_READ_MASK)); // ignore received data during transmit
			#ifdef ADXL372_FIFO_CHANGE_ORDER
				z_data_h = SPI_TransmitReceive(0xAA);
				z_data_l = SPI_TransmitReceive(0xAA);
			#else
				y_data_h = SPI_TransmitReceive(0xAA);
				y_data_l = SPI_TransmitReceive(0xAA);
			#endif
			ADXL372_CSLineLeave();

			ADXL372_CSLineTake();
			SPI_TransmitReceive(ADXL372_AddressToSPIData(ADXL372_FIFO_DATA, ADXL372_READ_MASK)); // ignore received data during transmit
			#ifdef ADXL372_FIFO_CHANGE_ORDER
				x_data_h = SPI_TransmitReceive(0xAA);
				x_data_l = SPI_TransmitReceive(0xAA);
			#else
				z_data_h = SPI_TransmitReceive(0xAA);
				z_data_l = SPI_TransmitReceive(0xAA);
			#endif
			ADXL372_CSLineLeave();

		} else {
			return ADXL372_READ_ERROR;
		}
		break;
	}

	triplet->x = ADXL372_ConvertFrom2Complement(&x_data_h, &x_data_l);
	triplet->y = ADXL372_ConvertFrom2Complement(&y_data_h, &y_data_l);
	triplet->z = ADXL372_ConvertFrom2Complement(&z_data_h, &z_data_l) + ADXL372_manual_z_offset;

	return ADXL372_READ_SUCCESS;

}

struct ADXL372_AccelTripletG ADXL372_ConvertAccTripletToG(const struct ADXL372_AccelTriplet *triplet) {

	struct ADXL372_AccelTripletG triplet_g = {0.0, 0.0, 0.0};
	triplet_g.x = triplet->x * 100.0 / 1000.0;
	triplet_g.y = triplet->y * 100.0 / 1000.0;
	triplet_g.z = triplet->z * 100.0 / 1000.0;
	return (triplet_g);

}

int16_t ADXL372_ConvertFrom2Complement(uint8_t *high_part, uint8_t *low_part) {

	int16_t axis_data = 0;
	axis_data = (*high_part << 4);		// shift H part - 8MSBs to allow storing 4 LSBs
	axis_data &= 0x0FF0;				// make sure L part is zeroed
	*low_part = (*low_part  >> 4);		// shift L part to delete reserved bits
	axis_data |= *low_part;

	if ( axis_data & 0x800 ) { 			// 2s complement data, MSB indicates the sign
		axis_data = ~(axis_data);
		axis_data &= 0x0FFF;			// zeroing higest part
		axis_data += 1;
		axis_data = -axis_data;
	}
	return axis_data;

}

void 	ADXL372_SetAxisOffsets() {

	// 4 MSBs in offset registers are reserved, read-only
	// description - Figure 36, p. 25

	// x axis
	ADXL372_Write(ADXL372_OFFSET_X, 0x0E);
	// y axis
	ADXL372_Write(ADXL372_OFFSET_Y, 0x00);
	// z axis
	ADXL372_Write(ADXL372_OFFSET_Z, 0x07);

}

uint16_t ADXL372_CheckFIFOSamplesNumber() {

	uint8_t msb_part = ADXL372_Read(ADXL372_FIFO_ENTRIES);
	uint8_t msb_mask = 0x03; // only 2 LSB are significant here
	msb_part &= msb_mask;
	uint8_t lsb_part = ADXL372_Read(ADXL372_FIFO_ENTRIES2);
	uint16_t total = 0;
	total = msb_part;
	total = total << 8;
	return (total |= lsb_part);

}


void ADXL372_InterruptsInit() {

	GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// INT1
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &gpio);

	// INT2
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &gpio);

	// same NVIC for both interrupts
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = EXTI15_10_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0x00;
	nvic.NVIC_IRQChannelSubPriority = 0x00;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	// EXTI INT1
	EXTI_InitTypeDef interrupt;
	EXTI_StructInit(&interrupt);
	interrupt.EXTI_Line = EXTI_Line11;
	interrupt.EXTI_Mode = EXTI_Mode_Interrupt;
	interrupt.EXTI_Trigger = EXTI_Trigger_Falling;
	interrupt.EXTI_LineCmd = ENABLE;
	EXTI_Init(&interrupt);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);

	// EXTI INT2
	interrupt.EXTI_Line = EXTI_Line10;
	interrupt.EXTI_Mode = EXTI_Mode_Interrupt;
	interrupt.EXTI_Trigger = EXTI_Trigger_Falling;
	interrupt.EXTI_LineCmd = ENABLE;
	EXTI_Init(&interrupt);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);

}

void EXTI15_10_IRQHandler() {

	// INT1 Interrupt handler
	if (EXTI_GetITStatus(EXTI_Line11)) {
		EXTI_ClearITPendingBit(EXTI_Line11);
		if ( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0 ) {
			// INT1 Event occured!
		}
	}

	// INT2 Interrupt handler
	if (EXTI_GetITStatus(EXTI_Line10)) {
		EXTI_ClearITPendingBit(EXTI_Line10);
		if ( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 0 ) {
			// INT2 Event occured!
		}
	}

}

void 	ADXL372_SetRegister_POWER_CTL() {

#define ADXL372_I2C_HSM_EN 			0 // i2c speed
// reserved, defined in header		0
#define ADXL372_INSTANT_ON_THRESH	0 // low - not used
#define ADXL372_FILTER_SETTLE		1 // 16 ms (ideal when HPF, LPF disabled)
#define ADXL372_LPF_DISABLE			1 // disable
#define ADXL372_HPF_DISABLE			1 // disable
#define ADXL372_MODE1				1
#define ADXL372_MODE0				1 // full bandwidth

	struct ADXL372_Register reg;
	reg.b7 = ADXL372_I2C_HSM_EN;
	reg.b6 = ADXL372_RESERVED_BIT;
	reg.b5 = ADXL372_INSTANT_ON_THRESH;
	reg.b4 = ADXL372_FILTER_SETTLE;
	reg.b3 = ADXL372_LPF_DISABLE;
	reg.b2 = ADXL372_HPF_DISABLE;
	reg.b1 = ADXL372_MODE1;
	reg.b0 = ADXL372_MODE0;
	ADXL372_SetRegister(ADXL372_POWER_CTL, &reg);

}

void 	ADXL372_SetRegister_MEASURE() {

#define ADXL372_USER_OR_DISABLE 0 // Overange disable
#define ADXL372_AUTOSLEEP 		0 // Autosleep
#define ADXL372_LINKLOOP1		0 // Link/Loop Activity Processing
#define ADXL372_LINKLOOP0		0 //
#define ADXL372_LOW_NOISE		0 // 0 -> normal noise level and 1 -> ~1/3 the normal noise level
#define ADXL372_BANDWIDTH2		1 // LPF disabled but it's BW set to 3200 Hz
#define ADXL372_BANDWIDTH1		0
#define ADXL372_BANDWIDTH0		0

	struct ADXL372_Register reg;
	reg.b7 = ADXL372_USER_OR_DISABLE;
	reg.b6 = ADXL372_AUTOSLEEP;
	reg.b5 = ADXL372_LINKLOOP1;
	reg.b4 = ADXL372_LINKLOOP0;
	reg.b3 = ADXL372_LOW_NOISE;
	reg.b2 = ADXL372_BANDWIDTH2;
	reg.b1 = ADXL372_BANDWIDTH1;
	reg.b0 = ADXL372_BANDWIDTH0;
	ADXL372_SetRegister(ADXL372_MEASURE, &reg);

}
void 	ADXL372_SetRegister_TIMING() {

#define ADXL372_ODR2 			1 // Output data rate 6400 Hz
#define ADXL372_ODR1 			0
#define ADXL372_ODR0			0
#define ADXL372_WAKEUP_RATE2	0 // Timer Rate for Wake-Up Mode
#define ADXL372_WAKEUP_RATE1	0
#define ADXL372_WAKEUP_RATE0	0 // disabled, but set to 52 ms
#define ADXL372_EXT_CLK			0 // Enable external clock
#define ADXL372_EXT_SYNC		0 // Enable external trigger

	struct ADXL372_Register reg;
	reg.b7 = ADXL372_ODR2;
	reg.b6 = ADXL372_ODR1;
	reg.b5 = ADXL372_ODR0;
	reg.b4 = ADXL372_WAKEUP_RATE2;
	reg.b3 = ADXL372_WAKEUP_RATE1;
	reg.b2 = ADXL372_WAKEUP_RATE0;
	reg.b1 = ADXL372_EXT_CLK;
	reg.b0 = ADXL372_EXT_SYNC;
	ADXL372_SetRegister(ADXL372_TIMING, &reg);

}
void 	ADXL372_SetRegister_INT2_MAP() {

#define ADXL372_INT2_LOW 		1 // Configures INT2 for active low operation
#define ADXL372_AWAKE_INT2 		0 // Map awake interrupt onto INT2
#define ADXL372_ACT2_INT2		0 // Map Activity 2 (motion warning) interrupt onto INT2
#define ADXL372_INACT_INT2		0 // Map inactivity interrupt onto INT2
#define ADXL372_FIFO_OVR_INT2	0 // Map FIFO_OVERRUN interrupt onto INT2
#define ADXL372_FIFO_FULL_INT2	1 // Map FIFO_FULL interrupt onto INT2
#define ADXL372_FIFO_RDY_INT2	0 // Map FIFO_READY interrupt onto INT2
#define ADXL372_DATA_RDY_INT2	0 // Map data ready interrupt onto INT2

	struct ADXL372_Register reg;
	reg.b7 = ADXL372_INT2_LOW;
	reg.b6 = ADXL372_AWAKE_INT2;
	reg.b5 = ADXL372_ACT2_INT2;
	reg.b4 = ADXL372_INACT_INT2;
	reg.b3 = ADXL372_FIFO_OVR_INT2;
	reg.b2 = ADXL372_FIFO_FULL_INT2;
	reg.b1 = ADXL372_FIFO_RDY_INT2;
	reg.b0 = ADXL372_DATA_RDY_INT2;
	ADXL372_SetRegister(ADXL372_INT2_MAP, &reg);

}
void 	ADXL372_SetRegister_INT1_MAP() {

#define ADXL372_INT1_LOW 		1 // Configures INT1 for active low operation
#define ADXL372_AWAKE_INT1 		0 // Map awake interrupt onto INT1
#define ADXL372_ACT_INT1		0 // Map activity interrupt onto INT1
#define ADXL372_INACT_INT1		0 // Map inactivity interrupt onto INT1
#define ADXL372_FIFO_OVR_INT1	0 // Map FIFO_OVERRUN interrupt onto INT1
#define ADXL372_FIFO_FULL_INT1	0 // Map FIFO_FULL interrupt onto INT1
#define ADXL372_FIFO_RDY_INT1	1 // Map FIFO_READY interrupt onto INT1
#define ADXL372_DATA_RDY_INT1	0 // Map data ready interrupt onto INT1

	struct ADXL372_Register reg;
	reg.b7 = ADXL372_INT1_LOW;
	reg.b6 = ADXL372_AWAKE_INT1;
	reg.b5 = ADXL372_ACT_INT1;
	reg.b4 = ADXL372_INACT_INT1;
	reg.b3 = ADXL372_FIFO_OVR_INT1;
	reg.b2 = ADXL372_FIFO_FULL_INT1;
	reg.b1 = ADXL372_FIFO_RDY_INT1;
	reg.b0 = ADXL372_DATA_RDY_INT1;
	ADXL372_SetRegister(ADXL372_INT1_MAP, &reg);

}
void 	ADXL372_SetRegister_FIFO_CTL() {

	/* FIFO modes must be configured while in standby mode */
	/* at least one sample set must be left in the FIFO after every read */

// reserved, defined in header
// reserved, defined in header
#define ADXL372_FIFO_FORMAT2		0 // Specifies which data is stored in the FIFO buffer
#define ADXL372_FIFO_FORMAT1		0
#define ADXL372_FIFO_FORMAT0		0
#define ADXL372_FIFO_MODE1			0 // Specifies FIFO operating mode
#define ADXL372_FIFO_MODE0			1 // Stream Mode
#define ADXL372_FIFO_SAMPLES_MSB	0 // Watermark number of FIFO samples that triggers a FIFO_FULL condition when reached

	struct ADXL372_Register reg;
	reg.b7 = ADXL372_RESERVED_BIT;
	reg.b6 = ADXL372_RESERVED_BIT;
	reg.b5 = ADXL372_FIFO_FORMAT2;
	reg.b4 = ADXL372_FIFO_FORMAT1;
	reg.b3 = ADXL372_FIFO_FORMAT0;
	reg.b2 = ADXL372_FIFO_MODE1;
	reg.b1 = ADXL372_FIFO_MODE0;
	reg.b0 = ADXL372_FIFO_SAMPLES_MSB;
	ADXL372_SetRegister(ADXL372_FIFO_CTL, &reg);

}
void 	ADXL372_SetRegister_FIFO_SAMPLES() {

	/*
	 * FIFO Samples. Watermark number of FIFO samples that triggers a FIFO_FULL condition when reached
	 * 8 LSBs - FIFO_SAMPLES[7:0]
	 */

#define ADXL372_FIFO_SAMPLES_7_0		0xFF

	ADXL372_Write(ADXL372_FIFO_SAMPLES, ADXL372_FIFO_SAMPLES_7_0);

}
void 	ADXL372_SetRegister_HPF() {

// 6 MSBs reserved
#define ADXL372_HPF_CORNER1 		0 // High-Pass Filter Corner Frequency Selection
#define ADXL372_HPF_CORNER0 		0

	struct ADXL372_Register reg;
	reg.b7 = ADXL372_RESERVED_BIT;
	reg.b6 = ADXL372_RESERVED_BIT;
	reg.b5 = ADXL372_RESERVED_BIT;
	reg.b4 = ADXL372_RESERVED_BIT;
	reg.b3 = ADXL372_RESERVED_BIT;
	reg.b2 = ADXL372_RESERVED_BIT;
	reg.b1 = ADXL372_HPF_CORNER1;
	reg.b0 = ADXL372_HPF_CORNER0;
	ADXL372_SetRegister(ADXL372_HPF, &reg);

}

void 	ADXL372_SetRegister(uint8_t address, struct ADXL372_Register *value) {
	uint8_t reg = 0;
	reg |= value->b7;
	reg = reg << 1;
	reg |= value->b6;
	reg = reg << 1;
	reg |= value->b5;
	reg = reg << 1;
	reg |= value->b4;
	reg = reg << 1;
	reg |= value->b3;
	reg = reg << 1;
	reg |= value->b2;
	reg = reg << 1;
	reg |= value->b1;
	reg = reg << 1;
	reg |= value->b0;
	reg = reg << 1;
	ADXL372_Write(address, reg);
}

void ADXL372_Write(uint8_t address, uint8_t value) {
	ADXL372_CSLineTake();
	SPI_TransmitReceive(ADXL372_AddressToSPIData(address, ADXL372_WRITE_MASK)); // ignore received data
	SPI_TransmitReceive(value);  												// ignore received data
	ADXL372_CSLineLeave();
	return;
}

uint8_t	ADXL372_Read(uint8_t address) {
	ADXL372_CSLineTake();
	SPI_TransmitReceive(ADXL372_AddressToSPIData(address, ADXL372_READ_MASK)); // ignore received data during transmit
	uint8_t reg = SPI_TransmitReceive(0xAA);
	ADXL372_CSLineLeave();
	return reg;
}

uint8_t ADXL372_AddressToSPIData(uint8_t address, uint8_t rw_mask) {
	address = address << 1;
	return (address |= rw_mask);
}

inline void ADXL372_CSLineTake() {
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}

inline void ADXL372_CSLineLeave() {
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

uint8_t ADXL372_GetDeviceID() {
	return (ADXL372_Read(ADXL372_DEVID_AD));
}

uint8_t ADXL372_GetPartID() {
	return (ADXL372_Read(ADXL372_PARTID));
}

uint8_t ADXL372_GetRevID() {
	return (ADXL372_Read(ADXL372_REVID));
}

uint8_t ADXL372_GetDevIDMst() {
	return (ADXL372_Read(ADXL372_DEVID_MST));
}

uint8_t ADXL372_GetRegisterContent(uint8_t address) {
	return (ADXL372_Read(address));
}

uint8_t ADXL372_SelfTest() {

	/* 1. Place the device into measurement mode.
	 * 2. Make sure the low-pass activity filter is enabled.
	 * 3. Assert self test by setting the ST bit in the SELF_TEST register (Register0x40). */

	// 1.
	ADXL372_CSLineTake();
	ADXL372_Write(ADXL372_POWER_CTL, 0x8F);

	// 2.
	int reg = ADXL372_Read(ADXL372_POWER_CTL);
	if ( (reg & 0x08) ) {
		return 0xFF;						// error
	}

	// 3.
	ADXL372_Write(ADXL372_SELF_TEST, 0x01);	// initiates self test

	for ( int i = 0; i < 2000000; i++) { } 	// necessary delay

	// Read the self test status bits, ST_DONE and USER_ST,  after approximately 300 ms to check the pass or fail condition.

	reg = ADXL372_Read(ADXL372_SELF_TEST);
	ADXL372_CSLineLeave();
	return reg;

}
