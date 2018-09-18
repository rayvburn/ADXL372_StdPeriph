/*
 * SPI.h
 *
 *  Created on: 16.08.2018
 *      Author: user
 */

#ifndef SPI_H_
#define SPI_H_

extern void 	SPI_Config();
extern void 	SPI_Write(uint8_t dane);
extern uint8_t 	SPI_Read();
extern uint8_t 	SPI_TransmitReceive();

#endif /* SPI_H_ */
