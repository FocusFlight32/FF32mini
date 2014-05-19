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

uint8_t escCalibrating = false;
char    temp;

///////////////////////////////////////////////////////////////////////////////
// ESC Calibration
///////////////////////////////////////////////////////////////////////////////

void escCalibration(void)
{
    escCalibrating = true;

    armed = false;

    cliPortPrint("\nESC Calibration:\n\r\n");
    cliPortPrint("!!!! CAUTION - Remove all propellers and disconnect !!!!\r\n");
    cliPortPrint("!!!! flight battery before proceeding any further   !!!!\n\r\n");
    cliPortPrint("Type 'Y' to continue, anything other character exits\n\r\n");

    while (cliPortAvailable() == false);
    temp = cliPortRead();
    if (temp != 'Y')
    {
    	cliPortPrint("ESC Calibration Canceled!!\n\r\n");
    	escCalibrating = false;
    	return;
    }

    ///////////////////////////////////

    cliPortPrint("For ESC Calibration:\r\n");
    cliPortPrint("  Enter 'h' for Max Command....\r\n");
    cliPortPrint("  Enter 'm' for Mid Command....\r\n");
    cliPortPrint("  Enter 'l' for Min Command....\r\n");
    cliPortPrint("  Enter 'x' to exit....\n\r\n");
    cliPortPrint("For Motor Order Verification:\r\n");
    cliPortPrint("  Enter '0' to turn off all motors....\r\n");
    cliPortPrint("  Enter '1' to turn on Motor1....\r\n");
    cliPortPrint("  Enter '2' to turn on Motor2....\r\n");
    cliPortPrint("  Enter '3' to turn on Motor3....\r\n");
    cliPortPrint("  Enter '4' to turn on Motor4....\r\n");
    cliPortPrint("  Enter '5' to turn on Motor5....\r\n");
    cliPortPrint("  Enter '6' to turn on Motor6....\n\r\n");

    ///////////////////////////////////

    while(true)
    {
		while (cliPortAvailable() == false);

		temp = cliPortRead();

		switch (temp)
		{
			case 'h':
			    cliPortPrint("Applying Max Command....\n\r\n");
			    writeAllMotors(systemConfig.maxThrottle);
			    break;

			case 'm':
			    cliPortPrint("Applying Mid Command....\n\r\n");
			    writeAllMotors(systemConfig.midCommand);
			    break;

			case 'l':
			    cliPortPrint("Applying Min Command....\n\r\n");
			    writeAllMotors(MINCOMMAND);
			    break;

			case 'x':
			    cliPortPrint("Applying Min Command, Exiting Calibration....\n\r\n");
			    writeAllMotors(MINCOMMAND);
			    escCalibrating = false;
			    return;
			    break;

			case '0':
			    cliPortPrint("Motors at  Min Command....\n\r\n");
			    writeAllMotors(MINCOMMAND);
			    break;

			case '1':
				cliPortPrint("Motor1 at Min Throttle....\n\r\n");
				pwmEscWrite(0, systemConfig.minThrottle);
				break;

			case '2':
				cliPortPrint("Motor2 at Min Throttle....\n\r\n");
				pwmEscWrite(1, systemConfig.minThrottle);
				break;

			case '3':
				cliPortPrint("Motor3 at Min Throttle....\n\r\n");
				pwmEscWrite(2, systemConfig.minThrottle);
				break;

			case '4':
				cliPortPrint("Motor4 at Min Throttle....\n\r\n");
				pwmEscWrite(3, systemConfig.minThrottle);
				break;

			case '5':
				cliPortPrint("Motor5 at Min Throttle....\n\r\n");
				pwmEscWrite(4, systemConfig.minThrottle);
				break;

			case '6':
				cliPortPrint("Motor6 at Min Throttle....\n\r\n");
				pwmEscWrite(5, systemConfig.minThrottle);
				break;

			case '?':
			    cliPortPrint("For ESC Calibration:\r\n");
			    cliPortPrint("  Enter 'h' for Max Command....\r\n");
			    cliPortPrint("  Enter 'm' for Mid Command....\r\n");
			    cliPortPrint("  Enter 'l' for Min Command....\r\n");
			    cliPortPrint("  Enter 'x' to exit....\n\r\n");
			    cliPortPrint("For Motor Order Verification:\r\n");
			    cliPortPrint("  Enter '0' to turn off all motors....\r\n");
			    cliPortPrint("  Enter '1' to turn on Motor1....\r\n");
			    cliPortPrint("  Enter '2' to turn on Motor2....\r\n");
			    cliPortPrint("  Enter '3' to turn on Motor3....\r\n");
			    cliPortPrint("  Enter '4' to turn on Motor4....\r\n");
			    cliPortPrint("  Enter '5' to turn on Motor5....\r\n");
			    cliPortPrint("  Enter '6' to turn on Motor6....\n\r\n");

				break;

		}
	}
}

///////////////////////////////////////////////////////////////////////////////
