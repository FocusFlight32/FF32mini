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
    Copyright � 2011  Bill Nesbitt
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
// UART2 Initialization
///////////////////////////////////////////////////////////////////////////////

enum { expandEvr = 0 };

void uart2ListenerCB(evr_t e)
{
    if (expandEvr)
        uart2PrintF("EVR-%s %8.3fs %s (%04X)\r\n", evrToSeverityStr(e.evr), (float)e.time/1000., evrToStr(e.evr), e.reason);
    else
        uart2PrintF("EVR:%08X %04X %04X\r\n", e.time, e.evr, e.reason);
}

///////////////////////////////////////

void uart2Init(void)
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

    USART_InitStructure.USART_BaudRate            = 9600;
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

    evrRegisterListener(uart2ListenerCB);
}

///////////////////////////////////////////////////////////////////////////////
// UART2 Available
///////////////////////////////////////////////////////////////////////////////

uint32_t uart2Available(void)
{
    return (DMA_GetCurrDataCounter(DMA1_Channel6) != rx2DMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// UART2 Clear Buffer
///////////////////////////////////////////////////////////////////////////////

void uart2ClearBuffer(void)
{
    rx2DMAPos = DMA_GetCurrDataCounter(DMA1_Channel6);
}

///////////////////////////////////////////////////////////////////////////////
// UART2 Number of Characters Available
///////////////////////////////////////////////////////////////////////////////

uint16_t uart2NumCharsAvailable(void)
{
	int32_t number;

	number = rx2DMAPos - DMA_GetCurrDataCounter(DMA1_Channel6);

	if (number >= 0)
	    return (uint16_t)number;
	else
	    return (uint16_t)(UART2_BUFFER_SIZE + number);
}

///////////////////////////////////////////////////////////////////////////////
// UART2 Read
///////////////////////////////////////////////////////////////////////////////

uint8_t uart2Read(void)
{
    uint8_t ch;

    ch = rx2Buffer[UART2_BUFFER_SIZE - rx2DMAPos];
    // go back around the buffer
    if (--rx2DMAPos == 0)
	    rx2DMAPos = UART2_BUFFER_SIZE;

    return ch;
}

///////////////////////////////////////////////////////////////////////////////
// UART2 Read Poll
///////////////////////////////////////////////////////////////////////////////

uint8_t uart2ReadPoll(void)
{
    while (!uart2Available()); // wait for some bytes
    return uart2Read();
}

///////////////////////////////////////////////////////////////////////////////
// UART2 Write
///////////////////////////////////////////////////////////////////////////////

void uart2Write(uint8_t ch)
{
    tx2Buffer[tx2BufferHead] = ch;
    tx2BufferHead = (tx2BufferHead + 1) % UART2_BUFFER_SIZE;

    uart2TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// UART2 Print
///////////////////////////////////////////////////////////////////////////////

void uart2Print(char *str)
{
    while (*str)
    {
    	tx2Buffer[tx2BufferHead] = *str++;
    	tx2BufferHead = (tx2BufferHead + 1) % UART2_BUFFER_SIZE;
    }

    uart2TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// UART2 Print Formatted - Print formatted string to UART2
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void uart2PrintF(const char * fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start (vlist, fmt);

	vsnprintf(buf, sizeof(buf), fmt, vlist);
	uart2Print(buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
// UART2 Print Binary String
///////////////////////////////////////////////////////////////////////////////

void uart2PrintBinary(uint8_t *buf, uint16_t length)
{
    uint16_t i;

   for (i = 0; i < length; i++)
    {
    	tx2Buffer[tx2BufferHead] = buf[i];
    	tx2BufferHead = (tx2BufferHead + 1) % UART2_BUFFER_SIZE;
    }

	uart2TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
