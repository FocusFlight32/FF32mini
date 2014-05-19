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
// UART1 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define UART1_TX_PIN        GPIO_Pin_9
#define UART1_RX_PIN        GPIO_Pin_10
#define UART1_TX_GPIO       GPIOA
#define UART1_RX_GPIO       GPIOA
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10

#define UART1_BUFFER_SIZE    2048

// Receive buffer, circular DMA
volatile uint8_t rx1Buffer[UART1_BUFFER_SIZE];
uint32_t rx1DMAPos = 0;

volatile uint8_t  tx1Buffer[UART1_BUFFER_SIZE];
volatile uint16_t tx1BufferTail = 0;
volatile uint16_t tx1BufferHead = 0;

volatile uint8_t  tx1DmaEnabled = false;

///////////////////////////////////////////////////////////////////////////////
// UART1 Transmit via DMA
///////////////////////////////////////////////////////////////////////////////

static void uart1TxDMA(void)
{
	if ((tx1DmaEnabled == true) || (tx1BufferHead == tx1BufferTail))  // Ignore call if already active or no new data in buffer
        return;

	DMA1_Channel4->CMAR = (uint32_t)&tx1Buffer[tx1BufferTail];

    if (tx1BufferHead > tx1BufferTail)
    {
    	DMA1_Channel4->CNDTR = tx1BufferHead - tx1BufferTail;
	    tx1BufferTail = tx1BufferHead;
    }
    else
    {
    	DMA1_Channel4->CNDTR = UART1_BUFFER_SIZE - tx1BufferTail;
	    tx1BufferTail = 0;
    }

    tx1DmaEnabled = true;

    DMA_Cmd(DMA1_Channel4, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// UART1 TX Complete Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void DMA1_Channel4_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA1_IT_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);

    tx1DmaEnabled = false;

    uart1TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// UART1 Initialization
///////////////////////////////////////////////////////////////////////////////

enum { expandEvr = 0 };

void uart1ListenerCB(evr_t e)
{
    if (expandEvr)
        uart1PrintF("EVR-%s %8.3fs %s (%04X)\r\n", evrToSeverityStr(e.evr), (float)e.time/1000., evrToStr(e.evr), e.reason);
    else
        uart1PrintF("EVR:%08X %04X %04X\r\n", e.time, e.evr, e.reason);
}

///////////////////////////////////////

void uart1Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    GPIO_InitStructure.GPIO_Pin   = UART1_TX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_PinAFConfig(UART1_TX_GPIO, UART1_TX_PINSOURCE, GPIO_AF_7);

    GPIO_Init(UART1_TX_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = UART1_RX_PIN;
  //GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_PinAFConfig(UART1_RX_GPIO, UART1_RX_PINSOURCE, GPIO_AF_7);

    GPIO_Init(UART1_RX_GPIO, &GPIO_InitStructure);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART1, &USART_InitStructure);

    // Receive DMA into a circular buffer

    DMA_DeInit(DMA1_Channel5);

    DMA_InitStructure.DMA_BufferSize         = UART1_BUFFER_SIZE;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)rx1Buffer;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->RDR;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;

    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel5, ENABLE);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    rx1DMAPos = DMA_GetCurrDataCounter(DMA1_Channel5);

    // Transmit DMA
    DMA_DeInit(DMA1_Channel4);

    DMA_InitStructure.DMA_BufferSize         = UART1_BUFFER_SIZE;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)tx1Buffer;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART1->TDR;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;

    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

    DMA1_Channel4->CNDTR = 0;

    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(USART1, ENABLE);

    evrRegisterListener(uart1ListenerCB);
}

///////////////////////////////////////////////////////////////////////////////
// UART1 Available
///////////////////////////////////////////////////////////////////////////////

uint32_t uart1Available(void)
{
    return (DMA_GetCurrDataCounter(DMA1_Channel5) != rx1DMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// UART1 Clear Buffer
///////////////////////////////////////////////////////////////////////////////

void uart1ClearBuffer(void)
{
    rx1DMAPos = DMA_GetCurrDataCounter(DMA1_Channel5);
}

///////////////////////////////////////////////////////////////////////////////
// UART1 Number of Characters Available
///////////////////////////////////////////////////////////////////////////////

uint16_t uart1NumCharsAvailable(void)
{
	int32_t number;

	number = rx1DMAPos - DMA_GetCurrDataCounter(DMA1_Channel5);

	if (number >= 0)
	    return (uint16_t)number;
	else
	    return (uint16_t)(UART1_BUFFER_SIZE + number);
}

///////////////////////////////////////////////////////////////////////////////
// UART1 Read
///////////////////////////////////////////////////////////////////////////////

uint8_t uart1Read(void)
{
    uint8_t ch;

    ch = rx1Buffer[UART1_BUFFER_SIZE - rx1DMAPos];
    // go back around the buffer
    if (--rx1DMAPos == 0)
	    rx1DMAPos = UART1_BUFFER_SIZE;

    return ch;
}

///////////////////////////////////////////////////////////////////////////////
// UART1 Read Poll
///////////////////////////////////////////////////////////////////////////////

uint8_t uart1ReadPoll(void)
{
    while (!uart1Available()); // wait for some bytes
    return uart1Read();
}

///////////////////////////////////////////////////////////////////////////////
// UART1 Write
///////////////////////////////////////////////////////////////////////////////

void uart1Write(uint8_t ch)
{
    tx1Buffer[tx1BufferHead] = ch;
    tx1BufferHead = (tx1BufferHead + 1) % UART1_BUFFER_SIZE;

    uart1TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// UART1 Print
///////////////////////////////////////////////////////////////////////////////

void uart1Print(char *str)
{
    while (*str)
    {
    	tx1Buffer[tx1BufferHead] = *str++;
    	tx1BufferHead = (tx1BufferHead + 1) % UART1_BUFFER_SIZE;
    }

    uart1TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// UART1 Print Formatted - Print formatted string to UART1
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void uart1PrintF(const char * fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start (vlist, fmt);

	vsnprintf(buf, sizeof(buf), fmt, vlist);
	uart1Print(buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
// UART1 Print Binary String
///////////////////////////////////////////////////////////////////////////////

void uart1PrintBinary(uint8_t *buf, uint16_t length)
{
    uint16_t i;

   for (i = 0; i < length; i++)
    {
    	tx1Buffer[tx1BufferHead] = buf[i];
    	tx1BufferHead = (tx1BufferHead + 1) % UART1_BUFFER_SIZE;
    }

	uart1TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
