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

#pragma once

///////////////////////////////////////////////////////////////////////////////
// MS5611 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define MS5611_SPI          SPI2

#define MS5611_CS_GPIO      GPIOC
#define MS5611_CS_PIN       GPIO_Pin_13

#define ENABLE_MS5611       GPIO_ResetBits(MS5611_CS_GPIO, MS5611_CS_PIN)

#define DISABLE_MS5611      GPIO_SetBits(MS5611_CS_GPIO,   MS5611_CS_PIN)

///////////////////////////////////////

extern uint32andUint8_t d1;

extern uint32_t d1Value;

extern uint32andUint8_t d2;

extern uint32_t d2Value;

extern int32_t ms5611Temperature;

extern uint8_t newPressureReading;

extern uint8_t newTemperatureReading;

///////////////////////////////////////////////////////////////////////////////
// Read Temperature Request Pressure
///////////////////////////////////////////////////////////////////////////////

void readTemperatureRequestPressure(void);

///////////////////////////////////////////////////////////////////////////////
// Read Pressure Request Pressure
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestPressure(void);

///////////////////////////////////////////////////////////////////////////////
// Read Pressure Request Temperature
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestTemperature(void);

///////////////////////////////////////////////////////////////////////////////
// Calculate Temperature
///////////////////////////////////////////////////////////////////////////////

void calculateTemperature(void);

///////////////////////////////////////////////////////////////////////////////
// Calculate Pressure Altitude
///////////////////////////////////////////////////////////////////////////////

void calculatePressureAltitude(void);

///////////////////////////////////////////////////////////////////////////////
// Pressure Initialization
///////////////////////////////////////////////////////////////////////////////

void initPressure(void);

///////////////////////////////////////////////////////////////////////////////
