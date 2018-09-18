/*
 * SPI.c
 *
 *  Created on: 16.08.2018
 *      Author: user
 */

#include "stm32f10x.h"

void SPI_Config() {

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // port clock


	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7; // SCK, MOSI
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_6; // MISO
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_4; // CS
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &gpio);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);

	SPI_InitTypeDef spi;
	SPI_StructInit(&spi);
	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_NSS  = SPI_NSS_Soft;

	// Configure SPI1 in Mode 0:
	//     CPOL = 0 --> clock is low when idle
	//     CPHA = 0 --> data is sampled at the first edge

	spi.SPI_CPOL = SPI_CPOL_Low;
	spi.SPI_CPHA = SPI_CPHA_1Edge;
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; //SPI_BaudRatePrescaler_2; // and was OK
	spi.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI1, &spi);

	SPI_Cmd(SPI1, ENABLE);

}

void SPI_Write(uint8_t data) {

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, data);

}

uint8_t SPI_Read() {

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	uint8_t data = SPI_I2S_ReceiveData(SPI1);
	return data;

}

uint8_t SPI_TransmitReceive(uint8_t byte) {

	// wait for empty transmit buffer
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, byte);

	// wait for data in receive buffer
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SPI1);

}
