/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the Naze32Pro Flight Control Board

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

// SPI2
// SCK  PB13
// MISO PB14
// MOSI PB15

///////////////////////////////////////////////////////////////////////////////
// SPI Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define SPI2_GPIO             GPIOB
#define SPI2_SCK_PIN          GPIO_Pin_13
#define SPI2_SCK_PIN_SOURCE   GPIO_PinSource13
#define SPI2_SCK_CLK          RCC_AHBPeriph_GPIOB
#define SPI2_MISO_PIN         GPIO_Pin_14
#define SPI2_MISO_PIN_SOURCE  GPIO_PinSource14
#define SPI2_MISO_CLK         RCC_AHBPeriph_GPIOB
#define SPI2_MOSI_PIN         GPIO_Pin_15
#define SPI2_MOSI_PIN_SOURCE  GPIO_PinSource15
#define SPI2_MOSI_CLK         RCC_AHBPeriph_GPIOB

static volatile uint16_t spi2ErrorCount = 0;

///////////////////////////////////////////////////////////////////////////////
// SPI Initialize
///////////////////////////////////////////////////////////////////////////////

void spiInit(SPI_TypeDef *SPIx)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    ///////////////////////////////////

    if (SPIx == SPI2)
    {
    	RCC_AHBPeriphClockCmd(SPI2_SCK_CLK | SPI2_MISO_CLK | SPI2_MOSI_CLK, ENABLE);

        GPIO_PinAFConfig(SPI2_GPIO, SPI2_SCK_PIN_SOURCE,  GPIO_AF_5);
	    GPIO_PinAFConfig(SPI2_GPIO, SPI2_MISO_PIN_SOURCE, GPIO_AF_5);
	    GPIO_PinAFConfig(SPI2_GPIO, SPI2_MOSI_PIN_SOURCE, GPIO_AF_5);

	    // Init pins
        GPIO_InitStructure.GPIO_Pin   = SPI2_SCK_PIN | SPI2_MISO_PIN | SPI2_MOSI_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(SPI2_GPIO, &GPIO_InitStructure);

        ///////////////////////////////

        GPIO_InitStructure.GPIO_Pin   = EEPROM_CS_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

		GPIO_Init(EEPROM_CS_GPIO, &GPIO_InitStructure);

		DISABLE_EEPROM;

		///////////////////////////////

        GPIO_InitStructure.GPIO_Pin   = HMC5983_CS_PIN;
	  //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

		GPIO_Init(HMC5983_CS_GPIO, &GPIO_InitStructure);

		DISABLE_HMC5983;

		///////////////////////////////

        GPIO_InitStructure.GPIO_Pin   = MPU6000_CS_PIN;
	  //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

		GPIO_Init(MPU6000_CS_GPIO, &GPIO_InitStructure);

		DISABLE_MPU6000;

		///////////////////////////////

        GPIO_InitStructure.GPIO_Pin   = MS5611_CS_PIN;
	  //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

		GPIO_Init(MS5611_CS_GPIO, &GPIO_InitStructure);

		DISABLE_MS5611;

		///////////////////////////////

		SPI_I2S_DeInit(SPI2);

        SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
        SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
        SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;
        SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
        SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  // 36/2 = 18 MHz SPI Clock
        SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
        SPI_InitStructure.SPI_CRCPolynomial     = 7;

        SPI_Init(SPI2, &SPI_InitStructure);

        SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);

        SPI_Cmd(SPI2, ENABLE);
    }

}

///////////////////////////////////////////////////////////////////////////////
// SPI Timeout Callback
///////////////////////////////////////////////////////////////////////////////

uint32_t spiTimeoutUserCallback(SPI_TypeDef *SPIx)
{
	return spi2ErrorCount;
}

///////////////////////////////////////////////////////////////////////////////
// SPI Transfer
///////////////////////////////////////////////////////////////////////////////

uint8_t spiTransfer(SPI_TypeDef *SPIx, uint8_t data)
{
    uint16_t spiTimeout;

    spiTimeout = 0x1000;
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
      if ((spiTimeout--) == 0) return spiTimeoutUserCallback(SPIx);

    SPI_SendData8(SPIx, data);

    spiTimeout = 0x1000;
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
      if ((spiTimeout--) == 0) return spiTimeoutUserCallback(SPIx);

    return((uint8_t)SPI_ReceiveData8(SPIx));
}

///////////////////////////////////////////////////////////////////////////////
// Set SPI Divisor
///////////////////////////////////////////////////////////////////////////////

void setSPIdivisor(SPI_TypeDef *SPIx, uint16_t data)
{
    #define BR_CLEAR_MASK 0xFFC7

	uint16_t tempRegister;

    SPI_Cmd(SPIx, DISABLE);

	tempRegister = SPIx->CR1;

	switch (data)
	{
	case 2:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_2;
	    break;

	case 4:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_4;
	    break;

	case 8:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_8;
	    break;

	case 16:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_16;
	    break;

	case 32:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_32;
	    break;

	case 64:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_64;
	    break;

	case 128:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_128;
	    break;

	case 256:
		tempRegister &= BR_CLEAR_MASK;
	    tempRegister |= SPI_BaudRatePrescaler_256;
	    break;
	}

	SPIx->CR1 = tempRegister;

	SPI_Cmd(SPIx, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// Get SPI Error Count
///////////////////////////////////////////////////////////////////////////////

uint16_t spiGetErrorCounter(SPI_TypeDef *SPIx)
{
    return spi2ErrorCount;
}

///////////////////////////////////////////////////////////////////////////////
