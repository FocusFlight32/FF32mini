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

#define HMC5983_ADDRESS 0x1E

#define HMC5983_CONFIG_REG_A     0x00
#define HMC5983_CONFIG_REG_B     0x01
#define HMC5983_MODE_REG         0x02
#define HMC5983_DATA_X_MSB_REG   0x03
#define HMC5983_STATUS_REG       0x09
#define HMC5983_TEMP_OUT_MSB_REG 0x31

///////////////////////////////////////////////////////////////////////////////

#define TS                               0x80  // Temperature compensation enable

#define SENSOR_CONFIG                    0x70  // 8 Sample average, 15 Hz

#define NORMAL_MEASUREMENT_CONFIGURATION 0x00

#define POSITIVE_BIAS_CONFIGURATION      0x01

///////////////////////////////////////////////////////////////////////////////

//#define SENSOR_GAIN 0x00  // +/- 0.88 Ga
#define SENSOR_GAIN 0x20        // +/- 1.3  Ga (default)
//#define SENSOR_GAIN 0x40  // +/- 1.9  Ga
//#define SENSOR_GAIN 0x60  // +/- 2.5  Ga
//#define SENSOR_GAIN 0x80  // +/- 4.0  Ga
//#define SENSOR_GAIN 0xA0  // +/- 4.7  Ga
//#define SENSOR_GAIN 0xC0  // +/- 5.6  Ga
//#define SENSOR_GAIN 0xE0  // +/- 8.1  Ga

///////////////////////////////////////////////////////////////////////////////

#define OP_MODE_CONTINUOUS 0x00 // Continuous conversion
#define OP_MODE_SINGLE     0x01 // Single conversion

#define STATUS_RDY         0x01 // Data Ready

///////////////////////////////////////////////////////////////////////////////

float magScaleFactor[3];

uint8_t magDataUpdate = false;

uint8_t newMagData = false;

int16andUint8_t rawMag[3];

///////////////////////////////////////////////////////////////////////////////
// Read Magnetometer
///////////////////////////////////////////////////////////////////////////////

uint8_t readMag()
{
    setSPIdivisor(HMC5983_SPI, 8);  // 4.5 MHz SPI clock

    ENABLE_HMC5983;
                             spiTransfer(HMC5983_SPI, HMC5983_DATA_X_MSB_REG + 0x80 + 0x40);
    rawMag[XAXIS].bytes[1] = spiTransfer(HMC5983_SPI, 0x00);
    rawMag[XAXIS].bytes[0] = spiTransfer(HMC5983_SPI, 0x00);
    rawMag[ZAXIS].bytes[1] = spiTransfer(HMC5983_SPI, 0x00);
    rawMag[ZAXIS].bytes[0] = spiTransfer(HMC5983_SPI, 0x00);
    rawMag[YAXIS].bytes[1] = spiTransfer(HMC5983_SPI, 0x00);
    rawMag[YAXIS].bytes[0] = spiTransfer(HMC5983_SPI, 0x00);
    DISABLE_HMC5983;

    // check for valid data
	if (rawMag[XAXIS].value == -4096 || rawMag[YAXIS].value == -4096 || rawMag[ZAXIS].value == -4096)
	    return false;
	else
	    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Initialize Magnetometer
///////////////////////////////////////////////////////////////////////////////

void initMag()
{
    uint8_t hmc5983Status = 0;
    uint8_t i;

    setSPIdivisor(HMC5983_SPI, 8);  // 4.5 MHz SPI clock

    ENABLE_HMC5983;
    spiTransfer(HMC5983_SPI, HMC5983_CONFIG_REG_A);
    spiTransfer(HMC5983_SPI, TS | SENSOR_CONFIG | POSITIVE_BIAS_CONFIGURATION);
    DISABLE_HMC5983;

    delay(50);

    ENABLE_HMC5983;
    spiTransfer(HMC5983_SPI, HMC5983_CONFIG_REG_B);
    spiTransfer(HMC5983_SPI, SENSOR_GAIN);
    DISABLE_HMC5983;

    delay(20);

    magScaleFactor[XAXIS] = 0.0f;
    magScaleFactor[YAXIS] = 0.0f;
    magScaleFactor[ZAXIS] = 0.0f;

    for (i = 0; i < 10; i++)
    {
    	ENABLE_HMC5983;
        spiTransfer(HMC5983_SPI, HMC5983_MODE_REG);
        spiTransfer(HMC5983_SPI, OP_MODE_SINGLE);
        DISABLE_HMC5983;

        delay(20);

        while ((hmc5983Status && STATUS_RDY) == 0x00)
        {
            ENABLE_HMC5983;
            spiTransfer(HMC5983_SPI, HMC5983_STATUS_REG + 0x80);
            hmc5983Status = spiTransfer(HMC5983_SPI, 0x00);
            DISABLE_HMC5983;
       }

        readMag();

        magScaleFactor[XAXIS] += (1.16f * 1090.0f) / (float)rawMag[XAXIS].value;
        magScaleFactor[YAXIS] += (1.16f * 1090.0f) / (float)rawMag[YAXIS].value;
        magScaleFactor[ZAXIS] += (1.08f * 1090.0f) / (float)rawMag[ZAXIS].value;
    }

    magScaleFactor[XAXIS] = fabs(magScaleFactor[XAXIS] / 10.0f);
    magScaleFactor[YAXIS] = fabs(magScaleFactor[YAXIS] / 10.0f);
    magScaleFactor[ZAXIS] = fabs(magScaleFactor[ZAXIS] / 10.0f);

    ENABLE_HMC5983;
    spiTransfer(HMC5983_SPI, HMC5983_CONFIG_REG_A);
    spiTransfer(HMC5983_SPI, TS | SENSOR_CONFIG | NORMAL_MEASUREMENT_CONFIGURATION);
    DISABLE_HMC5983;

    delay(50);

    ENABLE_HMC5983;
    spiTransfer(HMC5983_SPI, HMC5983_MODE_REG);
    spiTransfer(HMC5983_SPI, OP_MODE_CONTINUOUS);
    DISABLE_HMC5983;

    delay(20);

    readMag();

    delay(20);
}

///////////////////////////////////////////////////////////////////////////////
