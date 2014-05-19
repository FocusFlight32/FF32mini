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
// EEPROM Defines
////////////////////////////////////////////////////////////////////////////////

#define EEPROM_SPI          SPI2

#define EEPROM_CS_GPIO      GPIOB
#define EEPROM_CS_PIN       GPIO_Pin_2
#define EEPROM_CS_GPIO_CLK  RCC_AHBPeriph_GPIOB

#define ENABLE_EEPROM       GPIO_ResetBits(EEPROM_CS_GPIO, EEPROM_CS_PIN)

#define DISABLE_EEPROM      GPIO_SetBits(EEPROM_CS_GPIO,   EEPROM_CS_PIN)

///////////////////////////////////////////////////////////////////////////////
// EEPROM Variables
///////////////////////////////////////////////////////////////////////////////

extern const char rcChannelLetters[8];

void parseRcChannels(const char *input);

///////////////////////////////////////////////////////////////////////////////

enum crcFlags { CRC_HistoryBad = 1 };

///////////////////////////////////////

typedef struct sensorConfig_t
{
    uint8_t version;

    float accelBiasMPU[3];          // Bias for MPU60x0 Accel
    float accelScaleFactorMPU[3];   // Scale factor for MPU60x0 Accel

    float accelTCBiasSlope[3];
    float accelTCBiasIntercept[3];

    float gyroTCBiasSlope[3];
    float gyroTCBiasIntercept[3];

    float magBias[3];

    float accelCutoff;

    float KpAcc;

    float KiAcc;

    float KpMag;

    float KiMag;

    float compFilterA;

    float compFilterB;

    uint8_t dlpfSetting;

    ///////////////////////////////////

    uint8_t batteryCells;
    float   voltageMonitorScale;
    float   voltageMonitorBias;

    float   batteryLow;
    float   batteryVeryLow;
    float   batteryMaxLow;

    ///////////////////////////////////

    uint8_t gpsVelocityHoldOnly;
    uint8_t verticalVelocityHoldOnly;

    ///////////////////////////////////

    uint8_t  CRCFlags;
    uint32_t CRCAtEnd[1];

} sensorConfig_t;

////////////////////////////////////////

extern sensorConfig_t sensorConfig;

///////////////////////////////////////////////////////////////////////////////

typedef struct systemConfig_t
{
	uint8_t version;

	float rollAndPitchRateScaling;

    float yawRateScaling;

    float attitudeScaling;

    float nDotEdotScaling;

    float hDotScaling;

    ///////////////////////////////////

    uint8_t receiverType;

    uint8_t slaveSpektrum;

    uint8_t rcMap[8];

    uint16_t escPwmRate;
    uint16_t servoPwmRate;

    float midCommand;
    float minCheck;
    float maxCheck;
    float minThrottle;
    float maxThrottle;

    ///////////////////////////////////

    uint8_t  mixerConfiguration;
    float    yawDirection;

    uint16_t triYawServoPwmRate;
    float    triYawServoMin;
    float    triYawServoMid;
    float    triYawServoMax;
    float    triCopterYawCmd500HzLowPassTau;

    uint8_t  freeMixMotors;

    float    freeMix[6][3];

    ///////////////////////////////////

    PIDdata_t PID[NUMBER_OF_PIDS];

    ///////////////////////////////////

    uint8_t armCount;
    uint8_t disarmCount;

    ///////////////////////////////////

    uint16_t activeTelemetry;

    uint8_t  mavlinkEnabled;

    ///////////////////////////////////

    uint8_t  CRCFlags;
    uint32_t CRCAtEnd[1];

} systemConfig_t;

////////////////////////////////////////

extern systemConfig_t systemConfig;

///////////////////////////////////////////////////////////////////////////////
// Read Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void readSensorEEPROM(void);

///////////////////////////////////////////////////////////////////////////////
// Write Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void writeSensorEEPROM(void);

///////////////////////////////////////////////////////////////////////////////
// Check Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void checkSensorEEPROM(bool eepromReset);

///////////////////////////////////////////////////////////////////////////////
// Read System EEPROM
///////////////////////////////////////////////////////////////////////////////

void readSystemEEPROM(void);

///////////////////////////////////////////////////////////////////////////////
// Write System EEPROM
///////////////////////////////////////////////////////////////////////////////

void writeSystemEEPROM(void);

///////////////////////////////////////////////////////////////////////////////
// Check System EEPROM
///////////////////////////////////////////////////////////////////////////////

void checkSystemEEPROM(bool eepromReset);

///////////////////////////////////////////////////////////////////////////////

