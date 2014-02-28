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

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/

///////////////////////////////////////////////////////////////////////////////
// UART2 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define UART2_TX_PIN        GPIO_Pin_3
#define UART2_RX_PIN        GPIO_Pin_15
#define UART2_TX_GPIO       GPIOB
#define UART2_RX_GPIO       GPIOA
#define UART2_TX_PINSOURCE  GPIO_PinSource3
#define UART2_RX_PINSOURCE  GPIO_PinSource15

#define UART2_BUFFER_SIZE    2048

// Receive buffer, circular DMA
volatile uint8_t rx2Buffer[UART2_BUFFER_SIZE];
uint32_t rx2DMAPos = 0;

volatile uint8_t  tx2Buffer[UART2_BUFFER_SIZE];
volatile uint16_t tx2BufferTail = 0;
volatile uint16_t tx2BufferHead = 0;

volatile uint8_t  tx2DmaEnabled = false;

///////////////////////////////////////////////////////////////////////////////
// UART2 Transmit via DMA
///////////////////////////////////////////////////////////////////////////////

static void uart2TxDMA(void)
{
	if ((tx2DmaEnabled == true) || (tx2BufferHead == tx2BufferTail))  // Ignore call if already active or no new data in buffer
        return;

	DMA1_Channel7->CMAR = (uint32_t)&tx2Buffer[tx2BufferTail];

    if (tx2BufferHead > tx2BufferTail)
    {
    	DMA1_Channel7->CNDTR = tx2BufferHead - tx2BufferTail;
	    tx2BufferTail = tx2BufferHead;
    }
    else
    {
    	DMA1_Channel7->CNDTR = UART2_BUFFER_SIZE - tx2BufferTail;
	    tx2BufferTail = 0;
    }

    tx2DmaEnabled = true;

    DMA_Cmd(DMA1_Channel7, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// UART2 TX Complete Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void DMA1_Channel7_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA1_IT_TC7);
    DMA_Cmd(DMA1_Channel7, DISABLE);

    tx2DmaEnabled = false;

    uart2TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// GPS Initialization
///////////////////////////////////////////////////////////////////////////////

void gpsInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    GPIO_InitStructure.GPIO_Pin   = UART2_TX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_PinAFConfig(UART2_TX_GPIO, UART2_TX_PINSOURCE, GPIO_AF_7);

    GPIO_Init(UART2_TX_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = UART2_RX_PIN;
  //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_PinAFConfig(UART2_RX_GPIO, UART2_RX_PINSOURCE, GPIO_AF_7);

    GPIO_Init(UART2_RX_GPIO, &GPIO_InitStructure);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate            = eepromConfig.gpsBaudRate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART2, &USART_InitStructure);

    // Receive DMA into a circular buffer

    DMA_DeInit(DMA1_Channel6);

    DMA_InitStructure.DMA_BufferSize         = UART2_BUFFER_SIZE;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)rx2Buffer;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->RDR;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;

    DMA_Init(DMA1_Channel6, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel6, ENABLE);

    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

    rx2DMAPos = DMA_GetCurrDataCounter(DMA1_Channel6);

    // Transmit DMA
    DMA_DeInit(DMA1_Channel7);

    DMA_InitStructure.DMA_BufferSize         = UART2_BUFFER_SIZE;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)tx2Buffer;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART2->TDR;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;

    DMA_Init(DMA1_Channel7, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);

    DMA1_Channel7->CNDTR = 0;

    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(USART2, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// GPS Available
///////////////////////////////////////////////////////////////////////////////

uint16_t gpsAvailable(void)
{
    return (DMA_GetCurrDataCounter(DMA1_Channel6) != rx2DMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// GPS Read
///////////////////////////////////////////////////////////////////////////////

uint8_t gpsRead(void)
{
    uint8_t ch;

    ch = rx2Buffer[UART2_BUFFER_SIZE - rx2DMAPos];
    // go back around the buffer
    if (--rx2DMAPos == 0)
	    rx2DMAPos = UART2_BUFFER_SIZE;

    return ch;
}

///////////////////////////////////////////////////////////////////////////////
// GPS Read Poll
///////////////////////////////////////////////////////////////////////////////

uint8_t gpsReadPoll(void)
{
    while (!gpsAvailable()); // wait for some bytes
    return gpsRead();
}

///////////////////////////////////////////////////////////////////////////////
// GPS Write
///////////////////////////////////////////////////////////////////////////////

void gpsWrite(uint8_t ch)
{
    tx2Buffer[tx2BufferHead] = ch;
    tx2BufferHead = (tx2BufferHead + 1) % UART2_BUFFER_SIZE;

    uart2TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// GPS Print
///////////////////////////////////////////////////////////////////////////////

void gpsPrint(char *str)
{
    while (*str)
    {
    	tx2Buffer[tx2BufferHead] = *str++;
    	tx2BufferHead = (tx2BufferHead + 1) % UART2_BUFFER_SIZE;
    }

    uart2TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
