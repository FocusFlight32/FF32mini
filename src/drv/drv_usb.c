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

#define USB_TIMEOUT  50

///////////////////////////////////////////////////////////////////////////////

enum { expandEvr = 1 };

void usbListenerCB(evr_t e)
{
    if (expandEvr)
        usbPrintF("EVR-%s %8.3fs %s (%04X)\r\n", evrToSeverityStr(e.evr), (float)e.time/1000., evrToStr(e.evr), e.reason);
    else
        usbPrintF("EVR:%08X %04X %04X\r\n", e.time, e.evr, e.reason);
}

///////////////////////////////////////////////////////////////////////////////
// USB Initialization
///////////////////////////////////////////////////////////////////////////////

void usbInit(void)
{
	Set_System();
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();

	evrRegisterListener(usbListenerCB);
}

///////////////////////////////////////////////////////////////////////////////
// USB Available
///////////////////////////////////////////////////////////////////////////////

uint32_t usbAvailable(void)
{
    return receiveLength;
}

///////////////////////////////////////////////////////////////////////////////
// USB Read
///////////////////////////////////////////////////////////////////////////////

uint8_t usbRead(void)
{
    uint8_t buf[1];

    uint32_t rxed = 0;

    while (rxed < 1)
    {
        rxed += CDC_Receive_DATA((uint8_t*)buf + rxed, 1 - rxed);
    }

    return buf[0];
}

///////////////////////////////////////////////////////////////////////////////
// USB Print
///////////////////////////////////////////////////////////////////////////////

void usbPrint(char *str)
{
    uint32_t len;
    uint32_t oldTxed;
    uint32_t start;
    uint32_t txed;

    if (!(usbIsConnected() && usbIsConfigured()) || !str)
    {
	    return;
	}

	len     = strlen(str);

	txed    = 0;
	oldTxed = 0;

	start   = millis();

	while (txed < len && (millis() - start < USB_TIMEOUT))
	{
	    txed += CDC_Send_DATA((uint8_t*)str + txed, len - txed);

	    if (oldTxed != txed)
	    {
	        start = millis();
	    }

	    oldTxed = txed;
	}
}

///////////////////////////////////////////////////////////////////////////////
// USB Print Formatted - Print formatted string to USB VCP
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void usbPrintF(const char * fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start (vlist, fmt);

	vsnprintf(buf, sizeof(buf) - 1, fmt, vlist);
	usbPrint(buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
// USB Print Binary String
///////////////////////////////////////////////////////////////////////////////

void usbPrintBinary(uint8_t *buf, uint16_t length)
{
    uint32_t oldTxed;
    uint32_t start;
    uint32_t txed;

    if (!(usbIsConnected() && usbIsConfigured()) || !buf)
    {
	    return;
	}

	txed    = 0;
	oldTxed = 0;

	start   = millis();

	while (txed < length && (millis() - start < USB_TIMEOUT))
	{
	    txed += CDC_Send_DATA((uint8_t*)buf + txed, length - txed);

	    if (oldTxed != txed)
	    {
	        start = millis();
	    }

	    oldTxed = txed;
	}
}

///////////////////////////////////////////////////////////////////////////////
// Check USB Active
///////////////////////////////////////////////////////////////////////////////

void checkUsbActive(void)
{
	if (usbIsConfigured())
	{
		cliPortAvailable       = &usbAvailable;
		cliPortPrint           = &usbPrint;
		cliPortPrintF          = &usbPrintF;
		cliPortRead            = &usbRead;

		mavlinkPortPrintBinary = &usbPrintBinary;
	}
	else
	{
		cliPortAvailable       = &uart1Available;
		cliPortPrint           = &uart1Print;
		cliPortPrintF          = &uart1PrintF;
		cliPortRead            = &uart1Read;

		mavlinkPortPrintBinary = &uart1PrintBinary;
	}
}

///////////////////////////////////////////////////////////////////////////////
