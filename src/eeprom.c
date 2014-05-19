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
// EEPROM Defines
///////////////////////////////////////////////////////////////////////////////

#define WRITE_ENABLE                    0x06
#define WRITE_DISABLE                   0x04
#define READ_STATUS_REGISTER            0x05
#define WRITE_STATUS_REGISTER           0x01
#define READ_DATA                       0x03
#define FAST_READ                       0x0B
#define PAGE_PROGRAM_256_BYTES          0x02
#define SECTOR_ERASE_64KB               0xD8
#define CHIP_ERASE                      0xC7

///////////////////////////////////////

// Sensor data stored in 1st 64KB Sector, 0x000000

#define SENSOR_EEPROM_ADDR  0x000000

// System data stored in 2nd 64KB Sector, 0x010000

#define SYSTEM_EEPROM_ADDR  0x010000

///////////////////////////////////////////////////////////////////////////////
// EEPROM Variables
///////////////////////////////////////////////////////////////////////////////

static uint8_t sensorVersion = 1;
static uint8_t systemVersion = 1;

sensorConfig_t sensorConfig;
systemConfig_t systemConfig;

uint32andUint8_t sensorConfigAddr;
uint32andUint8_t systemConfigaddr;

const char rcChannelLetters[] = "AERT1234";

///////////////////////////////////////////////////////////////////////////////
// Parse RC Channels
///////////////////////////////////////////////////////////////////////////////

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++)
    {
        s = strchr(rcChannelLetters, *c);
        if (s)
            systemConfig.rcMap[s - rcChannelLetters] = c - input;
    }
}

///////////////////////////////////////////////////////////////////////////////
// EEPROM Busy
///////////////////////////////////////////////////////////////////////////////

void eepromBusy(void)
{
	uint8_t busy = 0x01;

	ENABLE_EEPROM;

    spiTransfer(EEPROM_SPI, READ_STATUS_REGISTER);

    while (busy == 0x01)
        busy = spiTransfer(EEPROM_SPI, 0x00) & 0x01;

    DISABLE_EEPROM;

    delayMicroseconds(2);
}

///////////////////////////////////////////////////////////////////////////////
// Write Enable
///////////////////////////////////////////////////////////////////////////////

void writeEnable(void)
{
	ENABLE_EEPROM;

    spiTransfer(EEPROM_SPI, WRITE_ENABLE);

    DISABLE_EEPROM;

    delayMicroseconds(2);
}

///////////////////////////////////////////////////////////////////////////////
// Read Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void readSensorEEPROM(void)
{
	uint8_t* start = (uint8_t*)(&sensorConfig);
	uint8_t* end   = (uint8_t*)(&sensorConfig + 1);

	///////////////////////////////////

	setSPIdivisor(EEPROM_SPI, 2);  // 18 MHz SPI clock

	sensorConfigAddr.value = SENSOR_EEPROM_ADDR;

	///////////////////////////////////

	ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[2]);
	spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[1]);
	spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[0]);

	while (start < end)
		*start++ = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	///////////////////////////////////

    if ( crcCheckVal != crc32B((uint32_t*)(&sensorConfig),       // CRC32B[sensorConfig CRC32B[sensorConfig]]
                               (uint32_t*)(&sensorConfig + 1)))
    {
        evrPush(EVR_SensorCRCFail,0);
        sensorConfig.CRCFlags |= CRC_HistoryBad;
    }
    else if ( sensorConfig.CRCFlags & CRC_HistoryBad )
    {
        evrPush(EVR_ConfigBadSensorHistory,0);
    }

    ///////////////////////////////////

    accConfidenceDecay = 1.0f / sqrt(sensorConfig.accelCutoff);
}

///////////////////////////////////////////////////////////////////////////////
// Write Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void writeSensorEEPROM(void)
{
    uint16_t byteCount;
    uint8_t  pageIndex;

    uint8_t* start = (uint8_t*)(&sensorConfig);
	uint8_t* end   = (uint8_t*)(&sensorConfig + 1);

	///////////////////////////////////

    if (sensorConfig.CRCFlags & CRC_HistoryBad)
        evrPush(EVR_ConfigBadSensorHistory,0);

    sensorConfig.CRCAtEnd[0] = crc32B((uint32_t*)(&sensorConfig),                  // CRC32B[sensorConfig]
                                      (uint32_t*)(&sensorConfig.CRCAtEnd));

    ///////////////////////////////////

    setSPIdivisor(EEPROM_SPI, 2);  // 18 MHz SPI clock

    sensorConfigAddr.value = SENSOR_EEPROM_ADDR;

    ///////////////////////////////////

    // Sector Erase

    writeEnable();

    ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, SECTOR_ERASE_64KB);

	spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[2]);
	spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[1]);
	spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[0]);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	eepromBusy();

    ///////////////////////////////////

    // Program Page(s)

	for (pageIndex = 0; pageIndex < ((sizeof(sensorConfig) / 256) + 1); pageIndex++)
    {
        writeEnable();

        ENABLE_EEPROM;

        spiTransfer(EEPROM_SPI, PAGE_PROGRAM_256_BYTES);

		spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[2]);
		spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[1]);
		spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[0]);

	    for (byteCount = 0; byteCount < 256; byteCount++)
	    {
			spiTransfer(EEPROM_SPI, *start++);

			if (start >= end)
			    break;
		}

		DISABLE_EEPROM;

		delayMicroseconds(2);

		eepromBusy();

		sensorConfigAddr.value += 0x0100;
    }

	readSensorEEPROM();
}

///////////////////////////////////////////////////////////////////////////////
// Check Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void checkSensorEEPROM(bool eepromReset)
{
    uint8_t version;

    sensorConfigAddr.value = SENSOR_EEPROM_ADDR;

    ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[2]);
	spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[1]);
	spiTransfer(EEPROM_SPI, sensorConfigAddr.bytes[0]);

	version = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

    ///////////////////////////////////

    if (eepromReset || version != sensorVersion)
    {
		// Default settings
        sensorConfig.version = sensorVersion;

	    ///////////////////////////////

        sensorConfig.accelBiasMPU[XAXIS] = 0.0f;
        sensorConfig.accelBiasMPU[YAXIS] = 0.0f;
        sensorConfig.accelBiasMPU[ZAXIS] = 0.0f;

        ///////////////////////////////

        sensorConfig.accelScaleFactorMPU[XAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)
        sensorConfig.accelScaleFactorMPU[YAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)
        sensorConfig.accelScaleFactorMPU[ZAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)

	    ///////////////////////////////

        sensorConfig.accelTCBiasSlope[XAXIS] = 0.0f;
        sensorConfig.accelTCBiasSlope[YAXIS] = 0.0f;
        sensorConfig.accelTCBiasSlope[ZAXIS] = 0.0f;

        ///////////////////////////////

        sensorConfig.accelTCBiasIntercept[XAXIS] = 0.0f;
        sensorConfig.accelTCBiasIntercept[YAXIS] = 0.0f;
        sensorConfig.accelTCBiasIntercept[ZAXIS] = 0.0f;

        ///////////////////////////////

        sensorConfig.gyroTCBiasSlope[ROLL ] = 0.0f;
        sensorConfig.gyroTCBiasSlope[PITCH] = 0.0f;
        sensorConfig.gyroTCBiasSlope[YAW  ] = 0.0f;

	    ///////////////////////////////

	    sensorConfig.gyroTCBiasIntercept[ROLL ] = 0.0f;
	    sensorConfig.gyroTCBiasIntercept[PITCH] = 0.0f;
	    sensorConfig.gyroTCBiasIntercept[YAW  ] = 0.0f;

	    ///////////////////////////////

	    sensorConfig.magBias[XAXIS] = 0.0f;
	    sensorConfig.magBias[YAXIS] = 0.0f;
	    sensorConfig.magBias[ZAXIS] = 0.0f;

		///////////////////////////////

		sensorConfig.accelCutoff = 1.0f;

		///////////////////////////////

	    sensorConfig.KpAcc = 5.0f;    // proportional gain governs rate of convergence to accelerometer
	    sensorConfig.KiAcc = 0.0f;    // integral gain governs rate of convergence of gyroscope biases
	    sensorConfig.KpMag = 5.0f;    // proportional gain governs rate of convergence to magnetometer
	    sensorConfig.KiMag = 0.0f;    // integral gain governs rate of convergence of gyroscope biases

	    ///////////////////////////////

	    sensorConfig.compFilterA =  2.000f;
	    sensorConfig.compFilterB =  1.000f;

	    ///////////////////////////////////

	    sensorConfig.dlpfSetting = BITS_DLPF_CFG_98HZ;

		///////////////////////////////////

        sensorConfig.batteryCells             = 3;
		sensorConfig.voltageMonitorScale      = 11.0f / 1.0f;
		sensorConfig.voltageMonitorBias       = 0.0f;

		sensorConfig.batteryLow               = 3.30f;
        sensorConfig.batteryVeryLow           = 3.20f;
        sensorConfig.batteryMaxLow            = 3.10f;

        ///////////////////////////////////

        sensorConfig.gpsVelocityHoldOnly      = true;
        sensorConfig.verticalVelocityHoldOnly = true;

        ///////////////////////////////////

	    sensorConfig.CRCFlags = 0;

	    ///////////////////////////////////

	    writeSensorEEPROM();
	}
}

///////////////////////////////////////////////////////////////////////////////
// Read System EEPROM
///////////////////////////////////////////////////////////////////////////////

void readSystemEEPROM(void)
{
	uint8_t* start = (uint8_t*)(&systemConfig);
	uint8_t* end   = (uint8_t*)(&systemConfig + 1);

	///////////////////////////////////

	setSPIdivisor(EEPROM_SPI, 2);  // 18 MHz SPI clock

	systemConfigaddr.value = SYSTEM_EEPROM_ADDR;

	///////////////////////////////////

	ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[2]);
	spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[1]);
	spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[0]);

	while (start < end)
		*start++ = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	///////////////////////////////////

    if ( crcCheckVal != crc32B((uint32_t*)(&systemConfig),       // CRC32B[systemConfig CRC32B[systemConfig]]
                               (uint32_t*)(&systemConfig + 1)))
    {
        evrPush(EVR_SystemCRCFail,0);
        systemConfig.CRCFlags |= CRC_HistoryBad;
    }
    else if ( systemConfig.CRCFlags & CRC_HistoryBad )
    {
        evrPush(EVR_ConfigBadSystemHistory,0);
    }

    ///////////////////////////////////

    if (systemConfig.yawDirection >= 0)
	    systemConfig.yawDirection =  1.0f;
	else
        systemConfig.yawDirection = -1.0f;
}

///////////////////////////////////////////////////////////////////////////////
// Write System EEPROM
///////////////////////////////////////////////////////////////////////////////

void writeSystemEEPROM(void)

{
    uint16_t byteCount;
    uint8_t  pageIndex;

    uint8_t* start = (uint8_t*)(&systemConfig);
	uint8_t* end   = (uint8_t*)(&systemConfig + 1);

	///////////////////////////////////

	// there's no reason to write these values to EEPROM, they'll just be noise
    zeroPIDintegralError();
    zeroPIDstates();

    if (systemConfig.CRCFlags & CRC_HistoryBad)
        evrPush(EVR_ConfigBadSystemHistory,0);

    systemConfig.CRCAtEnd[0] = crc32B((uint32_t*)(&systemConfig),                  // CRC32B[systemConfig]
                                      (uint32_t*)(&systemConfig.CRCAtEnd));

    ///////////////////////////////////

    setSPIdivisor(EEPROM_SPI, 2);  // 18 MHz SPI clock

    systemConfigaddr.value = SYSTEM_EEPROM_ADDR;

    ///////////////////////////////////

    // Sector Erase

    writeEnable();

    ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, SECTOR_ERASE_64KB);

	spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[2]);
	spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[1]);
	spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[0]);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	eepromBusy();

    ///////////////////////////////////

    // Program Page(s)

	for (pageIndex = 0; pageIndex < ((sizeof(systemConfig) / 256) + 1); pageIndex++)
    {
        writeEnable();

        ENABLE_EEPROM;

        spiTransfer(EEPROM_SPI, PAGE_PROGRAM_256_BYTES);

		spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[2]);
		spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[1]);
		spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[0]);

	    for (byteCount = 0; byteCount < 256; byteCount++)
	    {
			spiTransfer(EEPROM_SPI, *start++);

			if (start >= end)
			    break;
		}

		DISABLE_EEPROM;

		delayMicroseconds(2);

		eepromBusy();

		systemConfigaddr.value += 0x0100;
    }

	readSystemEEPROM();
}

///////////////////////////////////////////////////////////////////////////////
// Check System EEPROM
///////////////////////////////////////////////////////////////////////////////

void checkSystemEEPROM(bool eepromReset)
{
    uint8_t version;

    systemConfigaddr.value = SYSTEM_EEPROM_ADDR;

    ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[2]);
	spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[1]);
	spiTransfer(EEPROM_SPI, systemConfigaddr.bytes[0]);

	version = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

    ///////////////////////////////////

    if (eepromReset || version != systemVersion)
    {
		// Default settings
        systemConfig.version = systemVersion;

	    ///////////////////////////////////

	    systemConfig.rollAndPitchRateScaling = 100.0 / 180000.0 * PI;  // Stick to rate scaling for 100 DPS

        systemConfig.yawRateScaling          = 100.0 / 180000.0 * PI;  // Stick to rate scaling for 100 DPS

        systemConfig.attitudeScaling         = 60.0  / 180000.0 * PI;  // Stick to att scaling for 60 degrees

        systemConfig.nDotEdotScaling         = 0.009f;                 // Stick to nDot/eDot scaling (9 mps)/(1000 RX PWM Steps) = 0.009

        systemConfig.hDotScaling             = 0.003f;                 // Stick to hDot scaling (3 mps)/(1000 RX PWM Steps) = 0.003

        ///////////////////////////////////

	    systemConfig.receiverType  = SPEKTRUM;

        systemConfig.slaveSpektrum = false;

	    parseRcChannels("TAER2134");

	    systemConfig.escPwmRate   = 450;
        systemConfig.servoPwmRate = 50;

        ///////////////////////////////////

        systemConfig.mixerConfiguration = MIXERTYPE_TRI;
        systemConfig.yawDirection = 1.0f;

        systemConfig.triYawServoPwmRate             = 50;
        systemConfig.triYawServoMin                 = 2000.0f;
        systemConfig.triYawServoMid                 = 3000.0f;
        systemConfig.triYawServoMax                 = 4000.0f;
        systemConfig.triCopterYawCmd500HzLowPassTau = 0.05f;

        // Free Mix Defaults to Quad X
		systemConfig.freeMixMotors        = 4;

		systemConfig.freeMix[0][ROLL ]    =  1.0f;
		systemConfig.freeMix[0][PITCH]    = -1.0f;
		systemConfig.freeMix[0][YAW  ]    = -1.0f;

		systemConfig.freeMix[1][ROLL ]    = -1.0f;
		systemConfig.freeMix[1][PITCH]    = -1.0f;
		systemConfig.freeMix[1][YAW  ]    =  1.0f;

		systemConfig.freeMix[2][ROLL ]    = -1.0f;
		systemConfig.freeMix[2][PITCH]    =  1.0f;
		systemConfig.freeMix[2][YAW  ]    = -1.0f;

		systemConfig.freeMix[3][ROLL ]    =  1.0f;
		systemConfig.freeMix[3][PITCH]    =  1.0f;
		systemConfig.freeMix[3][YAW  ]    =  1.0f;

		systemConfig.freeMix[4][ROLL ]    =  0.0f;
		systemConfig.freeMix[4][PITCH]    =  0.0f;
		systemConfig.freeMix[4][YAW  ]    =  0.0f;

		systemConfig.freeMix[5][ROLL ]    =  0.0f;
		systemConfig.freeMix[5][PITCH]    =  0.0f;
        systemConfig.freeMix[5][YAW  ]    =  0.0f;

        ///////////////////////////////////

        systemConfig.midCommand   = 3000.0f;
        systemConfig.minCheck     = (float)(MINCOMMAND + 200);
        systemConfig.maxCheck     = (float)(MAXCOMMAND - 200);
        systemConfig.minThrottle  = (float)(MINCOMMAND + 200);
        systemConfig.maxThrottle  = (float)(MAXCOMMAND);

        ///////////////////////////////////

        systemConfig.PID[ROLL_RATE_PID].B               =   1.0f;
        systemConfig.PID[ROLL_RATE_PID].P               = 250.0f;
        systemConfig.PID[ROLL_RATE_PID].I               = 100.0f;
        systemConfig.PID[ROLL_RATE_PID].D               =   0.0f;
        systemConfig.PID[ROLL_RATE_PID].iTerm           =   0.0f;
        systemConfig.PID[ROLL_RATE_PID].windupGuard     = 100.0f;  // PWMs
        systemConfig.PID[ROLL_RATE_PID].lastDcalcValue  =   0.0f;
        systemConfig.PID[ROLL_RATE_PID].lastDterm       =   0.0f;
        systemConfig.PID[ROLL_RATE_PID].lastLastDterm   =   0.0f;
        systemConfig.PID[ROLL_RATE_PID].dErrorCalc      =   D_ERROR;
        systemConfig.PID[ROLL_RATE_PID].type            =   OTHER;

        systemConfig.PID[PITCH_RATE_PID].B              =   1.0f;
        systemConfig.PID[PITCH_RATE_PID].P              = 250.0f;
        systemConfig.PID[PITCH_RATE_PID].I              = 100.0f;
        systemConfig.PID[PITCH_RATE_PID].D              =   0.0f;
        systemConfig.PID[PITCH_RATE_PID].iTerm          =   0.0f;
        systemConfig.PID[PITCH_RATE_PID].windupGuard    = 100.0f;  // PWMs
        systemConfig.PID[PITCH_RATE_PID].lastDcalcValue =   0.0f;
        systemConfig.PID[PITCH_RATE_PID].lastDterm      =   0.0f;
        systemConfig.PID[PITCH_RATE_PID].lastLastDterm  =   0.0f;
        systemConfig.PID[PITCH_RATE_PID].dErrorCalc     =   D_ERROR;
        systemConfig.PID[PITCH_RATE_PID].type           =   OTHER;

        systemConfig.PID[YAW_RATE_PID].B                =   1.0f;
        systemConfig.PID[YAW_RATE_PID].P                = 350.0f;
        systemConfig.PID[YAW_RATE_PID].I                = 100.0f;
        systemConfig.PID[YAW_RATE_PID].D                =   0.0f;
        systemConfig.PID[YAW_RATE_PID].iTerm            =   0.0f;
        systemConfig.PID[YAW_RATE_PID].windupGuard      = 100.0f;  // PWMs
        systemConfig.PID[YAW_RATE_PID].lastDcalcValue   =   0.0f;
        systemConfig.PID[YAW_RATE_PID].lastDterm        =   0.0f;
        systemConfig.PID[YAW_RATE_PID].lastLastDterm    =   0.0f;
        systemConfig.PID[YAW_RATE_PID].dErrorCalc       =   D_ERROR;
        systemConfig.PID[YAW_RATE_PID].type             =   OTHER;

        systemConfig.PID[ROLL_ATT_PID].B                =   1.0f;
        systemConfig.PID[ROLL_ATT_PID].P                =   2.0f;
        systemConfig.PID[ROLL_ATT_PID].I                =   0.0f;
        systemConfig.PID[ROLL_ATT_PID].D                =   0.0f;
        systemConfig.PID[ROLL_ATT_PID].iTerm            =   0.0f;
        systemConfig.PID[ROLL_ATT_PID].windupGuard      =   0.5f;  // radians/sec
        systemConfig.PID[ROLL_ATT_PID].lastDcalcValue   =   0.0f;
        systemConfig.PID[ROLL_ATT_PID].lastDterm        =   0.0f;
        systemConfig.PID[ROLL_ATT_PID].lastLastDterm    =   0.0f;
        systemConfig.PID[ROLL_ATT_PID].dErrorCalc       =   D_ERROR;
        systemConfig.PID[ROLL_ATT_PID].type             =   ANGULAR;

        systemConfig.PID[PITCH_ATT_PID].B               =   1.0f;
        systemConfig.PID[PITCH_ATT_PID].P               =   2.0f;
        systemConfig.PID[PITCH_ATT_PID].I               =   0.0f;
        systemConfig.PID[PITCH_ATT_PID].D               =   0.0f;
        systemConfig.PID[PITCH_ATT_PID].iTerm           =   0.0f;
        systemConfig.PID[PITCH_ATT_PID].windupGuard     =   0.5f;  // radians/sec
        systemConfig.PID[PITCH_ATT_PID].lastDcalcValue  =   0.0f;
        systemConfig.PID[PITCH_ATT_PID].lastDterm       =   0.0f;
        systemConfig.PID[PITCH_ATT_PID].lastLastDterm   =   0.0f;
        systemConfig.PID[PITCH_ATT_PID].dErrorCalc      =   D_ERROR;
        systemConfig.PID[PITCH_ATT_PID].type            =   ANGULAR;

        systemConfig.PID[HEADING_PID].B                 =   1.0f;
        systemConfig.PID[HEADING_PID].P                 =   3.0f;
        systemConfig.PID[HEADING_PID].I                 =   0.0f;
        systemConfig.PID[HEADING_PID].D                 =   0.0f;
        systemConfig.PID[HEADING_PID].iTerm             =   0.0f;
        systemConfig.PID[HEADING_PID].windupGuard       =   0.5f;  // radians/sec
        systemConfig.PID[HEADING_PID].lastDcalcValue    =   0.0f;
        systemConfig.PID[HEADING_PID].lastDterm         =   0.0f;
        systemConfig.PID[HEADING_PID].lastLastDterm     =   0.0f;
        systemConfig.PID[HEADING_PID].dErrorCalc        =   D_ERROR;
        systemConfig.PID[HEADING_PID].type              =   ANGULAR;

        systemConfig.PID[NDOT_PID].B                    =   1.0f;
        systemConfig.PID[NDOT_PID].P                    =   3.0f;
        systemConfig.PID[NDOT_PID].I                    =   0.0f;
        systemConfig.PID[NDOT_PID].D                    =   0.0f;
        systemConfig.PID[NDOT_PID].iTerm                =   0.0f;
        systemConfig.PID[NDOT_PID].windupGuard          =   0.5f;
        systemConfig.PID[NDOT_PID].lastDcalcValue       =   0.0f;
        systemConfig.PID[NDOT_PID].lastDterm            =   0.0f;
        systemConfig.PID[NDOT_PID].lastLastDterm        =   0.0f;
        systemConfig.PID[NDOT_PID].dErrorCalc           =   D_ERROR;
        systemConfig.PID[NDOT_PID].type                 =   OTHER;

        systemConfig.PID[EDOT_PID].B                    =   1.0f;
        systemConfig.PID[EDOT_PID].P                    =   3.0f;
        systemConfig.PID[EDOT_PID].I                    =   0.0f;
        systemConfig.PID[EDOT_PID].D                    =   0.0f;
        systemConfig.PID[EDOT_PID].iTerm                =   0.0f;
        systemConfig.PID[EDOT_PID].windupGuard          =   0.5f;
        systemConfig.PID[EDOT_PID].lastDcalcValue       =   0.0f;
        systemConfig.PID[EDOT_PID].lastDterm            =   0.0f;
        systemConfig.PID[EDOT_PID].lastLastDterm        =   0.0f;
        systemConfig.PID[EDOT_PID].dErrorCalc           =   D_ERROR;
        systemConfig.PID[EDOT_PID].type                 =   OTHER;

        systemConfig.PID[HDOT_PID].B                    =   1.0f;
        systemConfig.PID[HDOT_PID].P                    =   2.0f;
        systemConfig.PID[HDOT_PID].I                    =   0.0f;
        systemConfig.PID[HDOT_PID].D                    =   0.0f;
        systemConfig.PID[HDOT_PID].iTerm                =   0.0f;
        systemConfig.PID[HDOT_PID].windupGuard          =   5.0f;
        systemConfig.PID[HDOT_PID].lastDcalcValue       =   0.0f;
        systemConfig.PID[HDOT_PID].lastDterm            =   0.0f;
        systemConfig.PID[HDOT_PID].lastLastDterm        =   0.0f;
        systemConfig.PID[HDOT_PID].dErrorCalc           =   D_ERROR;
        systemConfig.PID[HDOT_PID].type                 =   OTHER;

        systemConfig.PID[N_PID].B                       =   1.0f;
        systemConfig.PID[N_PID].P                       =   3.0f;
        systemConfig.PID[N_PID].I                       =   0.0f;
        systemConfig.PID[N_PID].D                       =   0.0f;
        systemConfig.PID[N_PID].iTerm                   =   0.0f;
        systemConfig.PID[N_PID].windupGuard             =   0.5f;
        systemConfig.PID[N_PID].lastDcalcValue          =   0.0f;
        systemConfig.PID[N_PID].lastDterm               =   0.0f;
        systemConfig.PID[N_PID].lastLastDterm           =   0.0f;
        systemConfig.PID[N_PID].dErrorCalc              =   D_ERROR;
        systemConfig.PID[N_PID].type                    =   OTHER;

        systemConfig.PID[E_PID].B                       =   1.0f;
        systemConfig.PID[E_PID].P                       =   3.0f;
        systemConfig.PID[E_PID].I                       =   0.0f;
        systemConfig.PID[E_PID].D                       =   0.0f;
        systemConfig.PID[E_PID].iTerm                   =   0.0f;
        systemConfig.PID[E_PID].windupGuard             =   0.5f;
        systemConfig.PID[E_PID].lastDcalcValue          =   0.0f;
        systemConfig.PID[E_PID].lastDterm               =   0.0f;
        systemConfig.PID[E_PID].lastLastDterm           =   0.0f;
        systemConfig.PID[E_PID].dErrorCalc              =   D_ERROR;
        systemConfig.PID[E_PID].type                    =   OTHER;

        systemConfig.PID[H_PID].B                       =   1.0f;
        systemConfig.PID[H_PID].P                       =   2.0f;
        systemConfig.PID[H_PID].I                       =   0.0f;
        systemConfig.PID[H_PID].D                       =   0.0f;
        systemConfig.PID[H_PID].iTerm                   =   0.0f;
        systemConfig.PID[H_PID].windupGuard             =   5.0f;
        systemConfig.PID[H_PID].lastDcalcValue          =   0.0f;
        systemConfig.PID[H_PID].lastDterm               =   0.0f;
        systemConfig.PID[H_PID].lastLastDterm           =   0.0f;
        systemConfig.PID[H_PID].dErrorCalc              =   D_ERROR;
        systemConfig.PID[H_PID].type                    =   OTHER;

        ///////////////////////////////////

        systemConfig.armCount                 =  50;
		systemConfig.disarmCount              =  0;

		///////////////////////////////////

		systemConfig.activeTelemetry          =  0;
		systemConfig.mavlinkEnabled           =  false;

		///////////////////////////////////

		systemConfig.CRCFlags = 0;

		///////////////////////////////////

	    writeSystemEEPROM();
	}
}

///////////////////////////////////////////////////////////////////////////////
