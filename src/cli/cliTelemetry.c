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
// Telemetry CLI
///////////////////////////////////////////////////////////////////////////////

void telemetryCLI()
{
    uint8_t  telemetryQuery = 'x';
    uint8_t  validQuery = false;

    cliBusy = true;

    cliPortPrint("\nEntering Telemetry CLI....\n\r\n");

    while(true)
    {
        cliPortPrint("Telemetry CLI -> ");

            while ((cliPortAvailable() == false) && (validQuery == false));

	    if (validQuery == false)
		telemetryQuery = cliPortRead();

	    cliPortPrint("\r\n");

	    switch(telemetryQuery)

	    {
            ///////////////////////////

            case 'a': // Telemetry Configuration
                cliPortPrint("\nTelemetry Configuration:\r\n");

                cliPortPrint("    Telemetry Set 1: ");
                cliPortPrintF("%s\r\n", systemConfig.activeTelemetry == 1 ?   "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 2: ");
                cliPortPrintF("%s\r\n", systemConfig.activeTelemetry == 2 ?   "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 3: ");
                cliPortPrintF("%s\r\n", systemConfig.activeTelemetry == 4 ?   "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 4: ");
                cliPortPrintF("%s\r\n", systemConfig.activeTelemetry == 8 ?   "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 5: ");
                cliPortPrintF("%s\r\n", systemConfig.activeTelemetry == 16 ?  "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 6: ");
                cliPortPrintF("%s\r\n", systemConfig.activeTelemetry == 32 ?  "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 7: ");
                cliPortPrintF("%s\r\n", systemConfig.activeTelemetry == 64 ?  "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 8: ");
                cliPortPrintF("%s\r\n", systemConfig.activeTelemetry == 128 ? "  Active" : "Inactive");

                cliPortPrint("    MavLink:         ");
                cliPortPrintF("%s\r\n", systemConfig.mavlinkEnabled == true ? " Enabled\r\n" : "Disabled\r\n");

                validQuery = false;
                break;

            ///////////////////////////

            case 'b': // Turn all Telemetry Off
                systemConfig.activeTelemetry = 0;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'c': // Toggle Telemetry Set 1 State
                systemConfig.activeTelemetry = 1;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'd': // Toggle Telemetry Set 2 State
                systemConfig.activeTelemetry = 2;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'e': // Toggle Telemetry Set 3 State
                systemConfig.activeTelemetry = 4;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'f': // Toggle Telemetry Set 4 State
                systemConfig.activeTelemetry = 8;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'g': // Toggle Telemetry Set 5 State
                systemConfig.activeTelemetry = 16;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'h': // Toggle Telemetry Set 6 State
                systemConfig.activeTelemetry = 32;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'i': // Toggle Telemetry Set 7 State
                systemConfig.activeTelemetry = 64;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'j': // Toggle Telemetry Set 8 State
                systemConfig.activeTelemetry = 128;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case 'k': // Toggle MavLink Enable/Disable
                if (systemConfig.mavlinkEnabled == false)
                {
					systemConfig.mavlinkEnabled = true;
                    systemConfig.activeTelemetry = 0x0000;
				}
                else
                {
                    systemConfig.mavlinkEnabled = false;
				}

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

			case 'x':
			    cliPortPrint("\nExiting Telemetry CLI....\n\r\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'W': // Write System EEPROM Parameters
                cliPortPrint("\nWriting System EEPROM Parameters....\n\r\n");
                writeSystemEEPROM();

                validQuery = false;
                break;

            ///////////////////////////

			case '?':
			   	cliPortPrint("\r\n");
			   	cliPortPrint("'a' Telemetry Configuration Data\r\n");
   		        cliPortPrint("'b' Turn all Telemetry Off\r\n");
			   	cliPortPrint("'c' Toggle Telemetry Set 1 State\r\n");
			   	cliPortPrint("'d' Toggle Telemetry Set 2 State\r\n");
			   	cliPortPrint("'e' Toggle Telemetry Set 3 State\r\n");
			   	cliPortPrint("'f' Toggle Telemetry Set 4 State\r\n");
   		        cliPortPrint("'g' Toggle Telemetry Set 5 State\r\n");
   		        cliPortPrint("'h' Toggle Telemetry Set 6 State\r\n");
   		        cliPortPrint("'i' Toggle Telemetry Set 7 State\r\n");
   		        cliPortPrint("'j' Toggle Telemetry Set 8 State\r\n");
   		        cliPortPrint("'k' Toggle MavLink Enabled/Disabled\r\n");
   		        cliPortPrint("                                           'W' Write System EEPROM Parameters\r\n");
   		        cliPortPrint("'x' Exit Telemetry CLI                     '?' Command Summary\r\n");
   		        cliPortPrint("\r\n");
	    	    break;

	    	///////////////////////////
	    }
	}
}

///////////////////////////////////////////////////////////////////////////////
