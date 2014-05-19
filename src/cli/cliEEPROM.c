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
// EEPROM CLI
///////////////////////////////////////////////////////////////////////////////

#define LINE_LENGTH 32

#define TIMEOUT 100     // mSec

///////////////////////////////////////

int min(int a, int b)
{
    return a < b ? a : b;
}

///////////////////////////////////////

int8_t parse_hex(char c)
{
    if ('0' <= c && c <= '9')
        return c - '0';
    if ('a' <= c && c <= 'f')
        return c - 'a' + 0x0A;
    if ('A' <= c && c <= 'F')
        return c - 'A' + 0x0A;
    return -1;
}

///////////////////////////////////////

void cliPrintSensorEEPROM(void)
{
    uint32_t old_crc = sensorConfig.CRCAtEnd[0];

    uint8_t *by = (uint8_t*)&sensorConfig;

    int i, j;

    sensorConfig.CRCAtEnd[0] = crc32B((uint32_t*)(&sensorConfig),                  // CRC32B[sensorConfig]
                                      (uint32_t*)(&sensorConfig.CRCAtEnd));

    if (sensorConfig.CRCFlags & CRC_HistoryBad)
      evrPush(EVR_ConfigBadHistory, 0);

    for (i = 0; i < ceil((float)sizeof(sensorConfig) / LINE_LENGTH); i++)
    {
        for (j = 0; j < min(LINE_LENGTH, sizeof(sensorConfig) - LINE_LENGTH * i); j++)
            cliPortPrintF("%02X", by[i * LINE_LENGTH + j]);

        cliPortPrint("\r\n");
    }

    sensorConfig.CRCAtEnd[0] = old_crc;
}

///////////////////////////////////////

void cliPrintSystemConfig(void)
{
    uint32_t old_crc = systemConfig.CRCAtEnd[0];

    uint8_t *by = (uint8_t*)&systemConfig;

    int i, j;

     systemConfig.CRCAtEnd[0] = crc32B((uint32_t*)(&systemConfig),                  // CRC32B[systemConfig]
                                       (uint32_t*)(&systemConfig.CRCAtEnd));

    if (systemConfig.CRCFlags & CRC_HistoryBad)
      evrPush(EVR_ConfigBadHistory, 0);

    for (i = 0; i < ceil((float)sizeof(systemConfig) / LINE_LENGTH); i++)
    {
        for (j = 0; j < min(LINE_LENGTH, sizeof(systemConfig) - LINE_LENGTH * i); j++)
            cliPortPrintF("%02X", by[i * LINE_LENGTH + j]);

        cliPortPrint("\r\n");
    }

    systemConfig.CRCAtEnd[0] = old_crc;
}

///////////////////////////////////////

void eepromCLI()
{
	char c;

	sensorConfig_t sensorRam;

	systemConfig_t systemRam;

	uint8_t  eepromQuery = 'x';

	uint8_t  *p;

	uint8_t  *end;

	uint8_t  secondNibble;

	uint8_t  validQuery  = false;

	uint16_t i;

	uint32_t c1, c2;

    uint32_t size;

	uint32_t time;

	uint32_t charsEncountered;

	///////////////////////////////////////////////////////////////////////////////

    cliBusy = true;

    cliPortPrint("\nEntering EEPROM CLI....\n\r\n");

    while(true)
    {
        cliPortPrint("EEPROM CLI -> ");

        while ((cliPortAvailable() == false) && (validQuery == false));

        if (validQuery == false)
            eepromQuery = cliPortRead();

        cliPortPrint("\r\n");

        switch(eepromQuery)
        {
            ///////////////////////////

            case 'a': // config struct data
                c1 = sensorConfig.CRCAtEnd[0];

                c2 = crc32B((uint32_t*)(&sensorConfig),                  // CRC32B[sensorConfig]
                            (uint32_t*)(&sensorConfig.CRCAtEnd));

                cliPortPrintF("Sensor EEPROM structure information:\r\n");
                cliPortPrintF("Version          : %d\r\n", sensorConfig.version);
                cliPortPrintF("Size             : %d\r\n", sizeof(sensorConfig));
                cliPortPrintF("CRC on last read : %08X\r\n", c1);
                cliPortPrintF("Current CRC      : %08X\r\n", c2);

                if ( c1 != c2 )
                    cliPortPrintF("  CRCs differ. Current Sensor Config has not yet been saved.\r\n");

                cliPortPrintF("CRC Flags :\r\n");
                cliPortPrintF("  History Bad    : %s\r\n", sensorConfig.CRCFlags & CRC_HistoryBad ? "true" : "false" );

                validQuery = false;
                break;

            ///////////////////////////

            case 'b': // Write out to Console in Hex.  (RAM -> console)
                // we assume the flyer is not in the air, so that this is ok;

                cliPortPrintF("\r\n");

                cliPrintSensorEEPROM();

                cliPortPrintF("\r\n");

                if (crcCheckVal != crc32B((uint32_t*)(&sensorConfig),       // CRC32B[sensorConfig CRC32B[sensorConfig]]
                                          (uint32_t*)(&sensorConfig + 1)))
                {
                    cliPortPrint("NOTE: in-memory sensor config CRC invalid; there have probably been\r\n");
                    cliPortPrint("      changes to sensor config since the last write to flash/eeprom.\r\n");
                }

                validQuery = false;
                break;

            ///////////////////////////

            case 'c': // Read Sensor Config -> RAM
                cliPortPrint("Re-reading Sensor EEPROM.\r\n");
                readSensorEEPROM();

                validQuery = false;
                break;

            ///////////////////////////

            case 'd': // Read Console -> Sensor RAM
                charsEncountered = 0;

                secondNibble = 0;

                size = sizeof(sensorConfig);

                time = millis();

                p = (uint8_t*)&sensorRam;

                end = (uint8_t*)(&sensorRam + 1);

                ///////////////////////

                cliPortPrintF("Ready to read in sensor config. Expecting %d (0x%03X) bytes as %d\r\n", size, size, size * 2);

                cliPortPrintF("hexadecimal characters, optionally separated by [ \\n\\r_].\r\n");

                cliPortPrintF("Times out if no character is received for %dms\r\n", TIMEOUT);

                memset(p, 0, end - p);

                while (p < end)
                {
                    while (!cliPortAvailable() && millis() - time < TIMEOUT) {}
                    time = millis();

                    c = cliPortAvailable() ? cliPortRead() : '\0';

                    int8_t hex = parse_hex(c);

                    int ignore = c == ' ' || c == '\n' || c == '\r' || c == '_' ? true : false;

                    if (c != '\0') // assume the person isn't sending null chars
                        charsEncountered++;

                    if (ignore)
                        continue;

                    if (hex == -1)
                        break;

                    *p |= secondNibble ? hex : hex << 4;

                    p += secondNibble;

                    secondNibble ^= 1;
                }

                if (c == 0)
                {
                    cliPortPrintF("Did not receive enough hex chars! (got %d, expected %d)\r\n",
                        (p - (uint8_t*)&sensorRam) * 2 + secondNibble, size * 2);
                }
                else if (p < end || secondNibble)
                {
                    cliPortPrintF("Invalid character found at position %d: '%c' (0x%02x)",
                        charsEncountered, c, c);
                }
                else if (crcCheckVal != crc32B((uint32_t*)(&sensorConfig),       // CRC32B[sensorConfig CRC32B[sensorConfig]]
                                               (uint32_t*)(&sensorConfig + 1)))
                {
                    cliPortPrintF("CRC mismatch! Not writing to in-memory config.\r\n");
                    cliPortPrintF("Here's what was received:\n\r\n");
                    cliPrintSensorEEPROM();
                }
                else
                {
                    // check to see if the newly received sytem config
                    // actually differs from what's in-memory

                    for (i = 0; i < size; i++)
                        if (((uint8_t*)&sensorRam)[i] != ((uint8_t*)&sensorConfig)[i])
                            break;

                    if (i == size)
                    {
                        cliPortPrintF("NOTE: Uploaded Sensor Config was Identical to RAM Config.\r\n");
                    }
                    else
                    {
                        sensorConfig = sensorRam;
                        cliPortPrintF("Sensor RAM Config updated!\r\n");
                        cliPortPrintF("NOTE: Sensor Config not written to EEPROM; use 'w' to do so.\r\n");
                    }

                }

                // eat the next 100ms (or whatever Timeout is) of characters,
                // in case the person pasted too much by mistake or something

                time = millis();

                while (millis() - time < TIMEOUT)
                    if (cliPortAvailable())
                        cliPortRead();

                validQuery = false;
                break;

            ///////////////////////////

            case 'h': // Clear Bad Sensor History Flag
                cliPortPrintF("Clearing Bad Sensor History Flag.\r\n");
                sensorConfig.CRCFlags &= ~CRC_HistoryBad;
                validQuery = false;
                break;

            ///////////////////////////

            case 'v': // Reset Sensor EEPROM Parameters
                cliPortPrint( "\nSensor EEPROM Parameters Reset....(not rebooting)\r\n" );
                checkSensorEEPROM(true);
                validQuery = false;
                break;

            ///////////////////////////

            case 'w': // Write to Sensor EEPROM
                cliPortPrint("\nWriting Sensor EEPROM Parameters....\r\n");
                writeSensorEEPROM();

                validQuery = false;
                break;

            ///////////////////////////

            case 'x': // exit EEPROM CLI
                cliPortPrint("\nExiting EEPROM CLI....\n\r\n");
                cliBusy = false;
                return;
                break;

            ///////////////////////////

            case 'A': // config struct data
                c1 = systemConfig.CRCAtEnd[0];

                zeroPIDintegralError();
                zeroPIDstates();

                c2 = crc32B((uint32_t*)(&systemConfig),                  // CRC32B[systemConfig]
                            (uint32_t*)(&systemConfig.CRCAtEnd));

                cliPortPrintF("System EEPROM structure information:\r\n");
                cliPortPrintF("Version          : %d\r\n", systemConfig.version);
                cliPortPrintF("Size             : %d\r\n", sizeof(systemConfig));
                cliPortPrintF("CRC on last read : %08X\r\n", c1);
                cliPortPrintF("Current CRC      : %08X\r\n", c2);

                if ( c1 != c2 )
                    cliPortPrintF("  CRCs differ. Current SystemConfig has not yet been saved.\r\n");

                cliPortPrintF("CRC Flags :\r\n");
                cliPortPrintF("  History Bad    : %s\r\n", systemConfig.CRCFlags & CRC_HistoryBad ? "true" : "false" );

                validQuery = false;
                break;

            ///////////////////////////

            case 'B': // Write out to Console in Hex.  (RAM -> console)
                // we assume the flyer is not in the air, so that this is ok;

                // these change randomly when not in flight and can mistakenly
                // make one think that the in-memory eeprom struct has changed
                zeroPIDintegralError();
                zeroPIDstates();

                cliPortPrintF("\r\n");

                cliPrintSystemConfig();

                cliPortPrintF("\r\n");

                if (crcCheckVal != crc32B((uint32_t*)(&systemConfig),       // CRC32B[systemConfig CRC32B[systemConfig]]
                                          (uint32_t*)(&systemConfig + 1)))
                {
                    cliPortPrint("NOTE: in-memory system config CRC invalid; there have probably been\r\n");
                    cliPortPrint("      changes to system config since the last write to flash/eeprom.\r\n");
                }

                validQuery = false;
                break;

            ///////////////////////////

            case 'C': // Read System Config -> RAM
                cliPortPrint("Re-reading System EEPROM.\r\n");
                readSystemEEPROM();

                validQuery = false;
                break;

            ///////////////////////////

            case 'D': // Read Console -> System RAM
            	charsEncountered = 0;

            	secondNibble = 0;

            	size = sizeof(systemConfig);

                time = millis();

                p = (uint8_t*)&systemRam;

                end = (uint8_t*)(&systemRam + 1);

                ///////////////////////

                cliPortPrintF("Ready to read in system config. Expecting %d (0x%03X) bytes as %d\r\n", size, size, size * 2);

                cliPortPrintF("hexadecimal characters, optionally separated by [ \\n\\r_].\r\n");

                cliPortPrintF("Times out if no character is received for %dms\r\n", TIMEOUT);

                memset(p, 0, end - p);

                while (p < end)
                {
                    while (!cliPortAvailable() && millis() - time < TIMEOUT) {}
                    time = millis();

                    c = cliPortAvailable() ? cliPortRead() : '\0';

                    int8_t hex = parse_hex(c);

                    int ignore = c == ' ' || c == '\n' || c == '\r' || c == '_' ? true : false;

                    if (c != '\0') // assume the person isn't sending null chars
                        charsEncountered++;

                    if (ignore)
                        continue;

                    if (hex == -1)
                        break;

                    *p |= secondNibble ? hex : hex << 4;

                    p += secondNibble;

                    secondNibble ^= 1;
                }

                if (c == 0)
                {
                    cliPortPrintF("Did not receive enough hex chars! (got %d, expected %d)\r\n",
                        (p - (uint8_t*)&systemRam) * 2 + secondNibble, size * 2);
                }
                else if (p < end || secondNibble)
                {
                    cliPortPrintF("Invalid character found at position %d: '%c' (0x%02x)",
                        charsEncountered, c, c);
                }
                else if (crcCheckVal != crc32B((uint32_t*)(&systemConfig),       // CRC32B[systemConfig CRC32B[systemConfig]]
                                               (uint32_t*)(&systemConfig + 1)))
                {
                    cliPortPrintF("CRC mismatch! Not writing to in-memory config.\r\n");
                    cliPortPrintF("Here's what was received:\n\r\n");
                    cliPrintSystemConfig();
                }
                else
                {
                    // check to see if the newly received sytem config
                    // actually differs from what's in-memory
                    zeroPIDintegralError();
                    zeroPIDstates();

                    for (i = 0; i < size; i++)
                        if (((uint8_t*)&systemRam)[i] != ((uint8_t*)&systemConfig)[i])
                            break;

                    if (i == size)
                    {
                        cliPortPrintF("NOTE: Uploaded System Config was Identical to RAM Config.\r\n");
                    }
                    else
                    {
                        systemConfig = systemRam;
                        cliPortPrintF("System RAM Config updated!\r\n");
                        cliPortPrintF("NOTE: System Config not written to EEPROM; use 'W' to do so.\r\n");
                    }

                }

                // eat the next 100ms (or whatever Timeout is) of characters,
                // in case the person pasted too much by mistake or something

                time = millis();

                while (millis() - time < TIMEOUT)
                    if (cliPortAvailable())
                        cliPortRead();

                validQuery = false;
                break;

            ///////////////////////////

            case 'H': // Clear Bad System History Flag
                cliPortPrintF("Clearing Bad System History Flag.\r\n");
                systemConfig.CRCFlags &= ~CRC_HistoryBad;
                validQuery = false;
                break;

            ///////////////////////////

            case 'V': // Reset System EEPROM Parameters
                cliPortPrint( "\nSystem EEPROM Parameters Reset....(not rebooting)\r\n" );
                checkSystemEEPROM(true);

                validQuery = false;
                break;

            ///////////////////////////

            case 'W': // Write out to System EEPROM
                cliPortPrint("\nWriting System EEPROM Parameters....\r\n");
                writeSystemEEPROM();

                validQuery = false;
                break;

            ///////////////////////////

            case '?':
            //                0         1         2         3         4         5         6         7
            //                01234567890123456789012345678901234567890123456789012345678901234567890123456789
                cliPortPrintF("\r\n");
                cliPortPrintF("'a' Display Sensor Config Information     'A' Display System Config Information\r\n");
                cliPortPrintF("'b' Write Sensor Config -> Console        'B' Write System Config - > Console\r\n");
                cliPortPrintF("'c' Read Sensor Config -> RAM             'C' Read System Config -> RAM\r\n");
                cliPortPrintF("'d' Read Console -> Sensor RAM            'D' Read Console -> System RAM\r\n");
                cliPortPrintF("'h' Clear System CRC Bad History flag     'H' Clear System CRC Bad History flag\r\n");
                cliPortPrintF("'v' Reset Sensor Config to Default        'V' Reset System Config to Default\r\n");
                cliPortPrintF("'w' Write Sensor Config -> EEPROM         'W' Write System Config -> EEPROM\r\n");
                cliPortPrintF("'x' Exit EEPROM CLI                       '?' Command Summary\r\n");
                cliPortPrintF("\r\n");
                break;

            ///////////////////////////
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
