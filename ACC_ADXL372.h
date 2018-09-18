/*
 * acc_adxl372.h
 *
 *  Created on: 16.08.2018
 *      Author: user
 */

#ifndef ACC_ADXL372_H_
#define ACC_ADXL372_H_

#include "stm32f10x.h"
#include <stdint.h>
#include <math.h>
#include "SPI.h"

// --------------------------------------------------------------------------------------------

/* REGISTER ADDRESSES */
#define ADXL372_DEVID_AD 		0x00 // Analog Devices, Inc., ID
#define ADXL372_DEVID_MST 		0x01 // Analog Devices MEMS ID
#define ADXL372_PARTID			0x02 // device ID
#define ADXL372_REVID			0x03 // mask revision ID
#define ADXL372_STATUS			0x04 // bits that describe various conditions
#define ADXL372_STATUS2			0x05 //

#define ADXL372_FIFO_ENTRIES2	0x06 // indicate the number of valid data samples present in the FIFO buffer
#define ADXL372_FIFO_ENTRIES	0x07 // indicate the number of valid data samples present in the FIFO buffer

#define ADXL372_XDATA_H			0x08 // contains the x-axis acceleration data (8 MSB)
#define ADXL372_XDATA_L			0x09 // contains the x-axis acceleration data (4 LSB)
#define ADXL372_YDATA_H			0x0A // contains the y-axis acceleration data (8 MSB)
#define ADXL372_YDATA_L			0x0B // contains the y-axis acceleration data (4 LSB)
#define ADXL372_ZDATA_H			0x0C // contains the z-axis acceleration data (8 MSB)
#define ADXL372_ZDATA_L			0x0D // contains the z-axis acceleration data (4 LSB)

#define ADXL372_OFFSET_X		0x20 // offset trim register - x-axis
#define ADXL372_OFFSET_Y		0x21 // offset trim register - y-axis
#define ADXL372_OFFSET_Z		0x22 // offset trim register - z-axis

// inactivity, threshold registers ommited here

#define ADXL372_HPF				0x38 // specifies parameters for the internal high-pass filter
#define ADXL372_FIFO_SAMPLES	0x39 // number of samples to store in the FIFO (8 LSB)
#define ADXL372_FIFO_CTL		0x3A // operating parameters for the FIFO

#define ADXL372_INT1_MAP		0x3B // configures the INT1 interrupt pin
#define ADXL372_INT2_MAP		0x3C // configures the INT1 interrupt pin

#define ADXL372_TIMING			0x3D // timing parameters: ODR and external timing triggers
#define ADXL372_MEASURE			0x3E // controls several measurement settings

#define ADXL372_POWER_CTL		0x3F
#define ADXL372_SELF_TEST		0x40 // self test feature
#define ADXL372_RESET			0x41 // proper value puts device into standby mode
#define ADXL372_FIFO_DATA		0x42 // accesses data stored in the FIFO

#define ADXL372_RESERVED_BIT 	0 // reserved bit, read only

// there seems to be a mistake in the datasheet (R/W inverted logic?), fig. 40-43, p.28
#define ADXL372_READ_MASK		1
#define ADXL372_WRITE_MASK		0

// interrupts
// #define ADXL372_ENABLE_INT 		// deleting the comment will cause mapping events to interrupt pins of the sensor

// read source
#define ADXL372_READ_FIFO		0 	// reading from fifo does not work ATM
#define ADXL372_READ_DIRECT		1

#define ADXL372_READ_ERROR		0
#define ADXL372_READ_SUCCESS	1

#define ADXL372_FIFO_CHANGE_ORDER	// comment this line if fifo samples order is wrong

// --------------------------------------------------------------------------------------------

struct ADXL372_AccelTriplet {
	int16_t x;
	int16_t y;
	int16_t z;
};

struct ADXL372_AccelTripletG {
	float x;
	float y;
	float z;
};

struct ADXL372_Register {
	uint8_t b7;
	uint8_t b6;
	uint8_t b5;
	uint8_t b4;
	uint8_t b3;
	uint8_t b2;
	uint8_t b1;
	uint8_t b0;
};

// --------------------------------------------------------------------------------------------

extern void		ADXL372_Setup();
extern void 	ADXL372_InterruptsInit();
extern uint8_t 	ADXL372_GetDeviceID();
extern uint8_t 	ADXL372_GetPartID();
extern uint8_t 	ADXL372_GetRevID();
extern uint8_t 	ADXL372_GetDevIDMst();
extern uint8_t 	ADXL372_GetRegisterContent(uint8_t address);

extern uint16_t	ADXL372_CheckFIFOSamplesNumber();
extern int16_t 	ADXL372_ConvertFrom2Complement(uint8_t *high_part, uint8_t *low_part);

extern uint8_t 	ADXL372_ReadAccTriplet(struct ADXL372_AccelTriplet *triplet, uint8_t source);
extern uint8_t 	ADXL372_GetLastTriplet(struct ADXL372_AccelTriplet *triplet);
extern struct ADXL372_AccelTripletG	ADXL372_ConvertAccTripletToG(const struct ADXL372_AccelTriplet *triplet);

// SPI specific functions
extern void		ADXL372_Write(uint8_t address, uint8_t value);
extern uint8_t	ADXL372_Read(uint8_t address);
extern uint8_t 	ADXL372_AddressToSPIData(uint8_t address, uint8_t rw_mask);
extern void 	ADXL372_CSLineTake();
extern void		ADXL372_CSLineLeave();

// register setup functions
extern void 	ADXL372_SetRegister(uint8_t address, struct ADXL372_Register *value);
extern void		ADXL372_SetAxisOffsets();
extern void 	ADXL372_SetRegister_POWER_CTL();
extern void 	ADXL372_SetRegister_MEASURE();
extern void 	ADXL372_SetRegister_TIMING();
extern void 	ADXL372_SetRegister_INT2_MAP();
extern void 	ADXL372_SetRegister_INT1_MAP();
extern void 	ADXL372_SetRegister_FIFO_CTL();
extern void 	ADXL372_SetRegister_FIFO_SAMPLES();
extern void 	ADXL372_SetRegister_HPF();

extern uint8_t 	ADXL372_SelfTest(); 	// NOT TESTED!

#endif /* ACC_ADXL372_H_ */
