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
// Mixer CLI
///////////////////////////////////////////////////////////////////////////////

void mixerCLI()
{
    float    tempFloat;

    uint8_t  index;
    uint8_t  rows, columns;

    uint8_t  mixerQuery = 'x';
    uint8_t  validQuery = false;

    cliBusy = true;

    cliPortPrint("\nEntering Mixer CLI....\n\r\n");

    while(true)
    {
        cliPortPrint("Mixer CLI -> ");

		while ((cliPortAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    mixerQuery = cliPortRead();

		cliPortPrint("\r\n");

		switch(mixerQuery)
		{
            ///////////////////////////

            case 'a': // Mixer Configuration
                cliPortPrint("\nMixer Configuration:  ");
                switch (systemConfig.mixerConfiguration)
                {
                    case MIXERTYPE_TRI:
                        cliPortPrint("   MIXERTYPE TRI\r\n");
                        break;

                    case MIXERTYPE_QUADX:
                        cliPortPrint("MIXERTYPE QUAD X\r\n");
                        break;

                    case MIXERTYPE_HEX6X:
                        cliPortPrint(" MIXERTYPE HEX X\r\n");
                        break;

                    case MIXERTYPE_FREE:
                        cliPortPrint("  MIXERTYPE FREE\r\n");
                        break;
                }

                cliPortPrintF("Number of Motors:                    %1d\r\n",  numberMotor);
                cliPortPrintF("ESC PWM Rate:                      %3ld\r\n",   systemConfig.escPwmRate);
                cliPortPrintF("Servo PWM Rate:                    %3ld\n\r\n", systemConfig.servoPwmRate);

                if (systemConfig.yawDirection == 1.0f)
                	cliPortPrintF("Yaw Direction:                  Normal\n\r\n");
                else if (systemConfig.yawDirection == -1.0f)
                	cliPortPrintF("Yaw Direction:                 Reverse\n\r\n");
                else
                	cliPortPrintF("Yaw Direction:               Undefined\n\r\n");

                if (systemConfig.mixerConfiguration == MIXERTYPE_TRI)
                {
					cliPortPrintF("TriCopter Yaw Servo PWM Rate:      %3ld\r\n",   systemConfig.triYawServoPwmRate);
                    cliPortPrintF("TriCopter Yaw Servo Min PWM:      %4ld\r\n",    (uint16_t)systemConfig.triYawServoMin);
                    cliPortPrintF("TriCopter Yaw Servo Mid PWM:      %4ld\r\n",    (uint16_t)systemConfig.triYawServoMid);
                    cliPortPrintF("TriCopter Yaw Servo Max PWM:      %4ld\r\n",    (uint16_t)systemConfig.triYawServoMax);
                    cliPortPrintF("Tricopter Yaw Cmd Time Constant:  %5.3f\n\r\n", systemConfig.triCopterYawCmd500HzLowPassTau);
			    }

        	    if (systemConfig.mixerConfiguration == MIXERTYPE_FREE)
                {
					cliPortPrintF("\nNumber of Free Mixer Motors:  %1d\n         Roll    Pitch   Yaw\n\r\n", systemConfig.freeMixMotors);

        	        for ( index = 0; index < systemConfig.freeMixMotors; index++ )
        	        {
        	    	    cliPortPrintF("Motor%1d  %6.3f  %6.3f  %6.3f\r\n", index,
        	    			                                             systemConfig.freeMix[index][ROLL ],
        	    			                                             systemConfig.freeMix[index][PITCH],
        	    			                                             systemConfig.freeMix[index][YAW  ]);
        	        }

        	        cliPortPrint("\r\n");
			    }

                validQuery = false;
                break;

            ///////////////////////////

			case 'x':
			    cliPortPrint("\nExiting Mixer CLI....\n\r\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Read Mixer Configuration
                systemConfig.mixerConfiguration = (uint8_t)readFloatCLI();

                initMixer();
                pwmEscInit();

        	    mixerQuery = 'a';
                validQuery = true;
		        break;

            ///////////////////////////

            case 'B': // Read ESC and Servo PWM Update Rates
                systemConfig.escPwmRate   = (uint16_t)readFloatCLI();
                systemConfig.servoPwmRate = (uint16_t)readFloatCLI();

                pwmEscInit();
                pwmServoInit();

                mixerQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'D': // Read yaw direction
                tempFloat = readFloatCLI();
                if (tempFloat >= 0.0)
                    tempFloat = 1.0;
                else
                	tempFloat = -1.0;

                systemConfig.yawDirection = tempFloat;

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // Read TriCopter Servo PWM Rate
            	if (systemConfig.mixerConfiguration == MIXERTYPE_TRI)
               	{
               		systemConfig.triYawServoPwmRate = (uint16_t)readFloatCLI();

                    pwmEscInit();
               	}
                else
                {
                   	tempFloat = readFloatCLI();

                   	cliPortPrintF("\nTriCopter Mixing not Selected....\n\r\n");
                }

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'F': // Read TriCopter Servo Min Point
               	if (systemConfig.mixerConfiguration == MIXERTYPE_TRI)
               	{
               		systemConfig.triYawServoMin = readFloatCLI();

               		pwmEscWrite(5, (uint16_t)systemConfig.triYawServoMin);
                }
                else
                {
                   	tempFloat = readFloatCLI();

                    cliPortPrintF("\nTriCopter Mixing not Selected....\n\r\n");
                }

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'G': // Read TriCopter Servo Mid Point
                if (systemConfig.mixerConfiguration == MIXERTYPE_TRI)
                {
                    systemConfig.triYawServoMid = readFloatCLI();

                    pwmEscWrite(5, (uint16_t)systemConfig.triYawServoMid);
                }
                else
                {
                   	tempFloat = readFloatCLI();

                   	cliPortPrintF("\nTriCopter Mixing not Selected....\n\r\n");
                }

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'H': // Read TriCopter Servo Max Point
                if (systemConfig.mixerConfiguration == MIXERTYPE_TRI)
                {
                    systemConfig.triYawServoMax = readFloatCLI();

                    pwmEscWrite(5, (uint16_t)systemConfig.triYawServoMax);
                }
                else
                {
                    tempFloat = readFloatCLI();

                    cliPortPrintF("\nTriCopter Mixing not Selected....\n\r\n");
                }

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'I': // Read TriCopter Yaw Command Time Constant
                if (systemConfig.mixerConfiguration == MIXERTYPE_TRI)
                {
                	systemConfig.triCopterYawCmd500HzLowPassTau = readFloatCLI();

                	initFirstOrderFilter();
                }
                else
                {
                    tempFloat = readFloatCLI();

                    cliPortPrintF("\nTriCopter Mixing not Selected....\n\r\n");
                }

                mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'J': // Read Free Mix Motor Number
           	    if (systemConfig.mixerConfiguration == MIXERTYPE_FREE)
                {
                	systemConfig.freeMixMotors = (uint8_t)readFloatCLI();
           	        initMixer();
				}
				else
				{
					tempFloat = readFloatCLI();

					cliPortPrintF("\nFree Mix not Selected....\n\r\n");
                }

           	    mixerQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'K': // Read Free Mix Matrix Element
                if (systemConfig.mixerConfiguration == MIXERTYPE_FREE)
                {
                	rows    = (uint8_t)readFloatCLI() - 1;
                    columns = (uint8_t)readFloatCLI() - 1;

                    systemConfig.freeMix[rows][columns] = readFloatCLI();
				}
				else
				{
					tempFloat = readFloatCLI();
					tempFloat = readFloatCLI();
					tempFloat = readFloatCLI();

					cliPortPrintF("\nFree Mix not Selected....\n\r\n");
                }

           	    mixerQuery = 'a';
                validQuery = true;
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
			   	cliPortPrint("'a' Mixer Configuration Data               'A' Set Mixer Configuration              A0 thru 3, see ff32_Naze32Pro.h\r\n");
   		        cliPortPrint("                                           'B' Set PWM Rates                        BESC;Servo\r\n");
   		        cliPortPrint("                                           'D' Set Yaw Direction                    D1 or D-1\r\n");

   		        if (systemConfig.mixerConfiguration == MIXERTYPE_TRI)
   		    	{
   		        	cliPortPrint("                                           'E' Set TriCopter Servo PWM Rate         ERate\r\n");
   		        	cliPortPrint("                                           'F' Set TriCopter Servo Min Point        FMin\r\n");
			   	    cliPortPrint("                                           'G' Set TriCopter Servo Mid Point        GMid\r\n");
			   	    cliPortPrint("                                           'H' Set TriCopter Servo Max Point        HMax\r\n");
			   	    cliPortPrint("                                           'I' Set TriCopter Yaw Cmd Time Constant  ITimeConstant\r\n");
   		    	}

   		        if (systemConfig.mixerConfiguration == MIXERTYPE_FREE)
   		    	{
   		        	cliPortPrint("                                           'J' Set Number of FreeMix Motors         JNumb\r\n");
   		        	cliPortPrint("                                           'K' Set FreeMix Matrix Element           KRow;Col;Value\r\n");
			   	}

   		        cliPortPrint("                                           'W' Write EEPROM Parameters\r\n");
   		        cliPortPrint("'x' Exit Mixer CLI                         '?' Command Summary\r\n");
   		        cliPortPrint("\r\n");
	    	    break;

	    	///////////////////////////
	    }
	}
}

///////////////////////////////////////////////////////////////////////////////
