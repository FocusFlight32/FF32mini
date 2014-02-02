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
// Receiver CLI
///////////////////////////////////////////////////////////////////////////////

void receiverCLI()
{
    char     rcOrderString[9];
    float    tempFloat;
    uint8_t  index;
    uint8_t  receiverQuery = 'x';
    uint8_t  validQuery    = false;

    NVIC_InitTypeDef  NVIC_InitStructure;

    cliBusy = true;

    cliPrint("\nEntering Receiver CLI....\n\n");

    while(true)
    {
        cliPrint("Receiver CLI -> ");

		while ((cliAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    receiverQuery = cliRead();

		cliPrint("\n");

		switch(receiverQuery)
		{
            ///////////////////////////

            case 'a': // Receiver Configuration
                cliPrint("\nReceiver Type:                  ");
                switch(eepromConfig.receiverType)
                {
                    case PPM:
                        cliPrint("PPM\n");
                        break;
                    case SPEKTRUM:
                        cliPrint("Spektrum\n");
                        break;
		        }

                cliPrint("Current RC Channel Assignment:  ");
                for (index = 0; index < 8; index++)
                    rcOrderString[eepromConfig.rcMap[index]] = rcChannelLetters[index];

                rcOrderString[index] = '\0';

                cliPrint(rcOrderString);  cliPrint("\n");

                cliPrintF("Secondary Spektrum:             ");

                if ((eepromConfig.slaveSpektrum == true) && false)  // HJI Inhibit Slave Spektrum on Naze32 Pro
                    cliPrintF("Installed\n");
                else
                    cliPrintF("Uninstalled\n");

                cliPrintF("Mid Command:                    %4ld\n",   (uint16_t)eepromConfig.midCommand);
				cliPrintF("Min Check:                      %4ld\n",   (uint16_t)eepromConfig.minCheck);
				cliPrintF("Max Check:                      %4ld\n",   (uint16_t)eepromConfig.maxCheck);
				cliPrintF("Min Throttle:                   %4ld\n",   (uint16_t)eepromConfig.minThrottle);
				cliPrintF("Max Thottle:                    %4ld\n\n", (uint16_t)eepromConfig.maxThrottle);

				tempFloat = eepromConfig.rollAndPitchRateScaling * 180000.0 / PI;
				cliPrintF("Max Roll and Pitch Rate Cmd:    %6.2f DPS\n", tempFloat);

				tempFloat = eepromConfig.yawRateScaling * 180000.0 / PI;
				cliPrintF("Max Yaw Rate Cmd:               %6.2f DPS\n", tempFloat);

				tempFloat = eepromConfig.attitudeScaling * 180000.0 / PI;
                cliPrintF("Max Attitude Cmd:               %6.2f Degrees\n\n", tempFloat);

				cliPrintF("Arm Delay Count:                %3d Frames\n",   eepromConfig.armCount);
				cliPrintF("Disarm Delay Count:             %3d Frames\n\n", eepromConfig.disarmCount);

				validQuery = false;
				break;

            ///////////////////////////

			case 'x':
			    cliPrint("\nExiting Receiver CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Toggle PPM/Spektrum Satellite Receiver
            	if (eepromConfig.receiverType == PPM)
                {
                    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_CC_IRQn;
                    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
                    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
                    NVIC_InitStructure.NVIC_IRQChannelCmd                = DISABLE;

                    NVIC_Init(&NVIC_InitStructure);

                	TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
                	eepromConfig.receiverType = SPEKTRUM;
                    spektrumInit();
                }
                else
                {
                	NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_TRG_COM_TIM17_IRQn;
                    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
                    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
                    NVIC_InitStructure.NVIC_IRQChannelCmd                = DISABLE;

                    NVIC_Init(&NVIC_InitStructure);

                    TIM_ITConfig(TIM17, TIM_IT_Update, DISABLE);
                  	eepromConfig.receiverType = PPM;
                    ppmRxInit();
                }

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'B': // Read RC Control Order
                readStringCLI( rcOrderString, 8 );
                parseRcChannels( rcOrderString );

          	    receiverQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // Toggle Slave Spektrum State
                // HJI Inhibit Slave Spektrum on Naze32 Pro if (eepromConfig.slaveSpektrum == true)
                    eepromConfig.slaveSpektrum = false;
                // HJI Inhibit Slave Spektrum on Naze32 Pro else
                // HJI Inhibit Slave Spektrum on Naze32 Pro	    eepromConfig.slaveSpektrum = true;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // Read RC Control Points
                eepromConfig.midCommand   = readFloatCLI();
    	        eepromConfig.minCheck     = readFloatCLI();
    		    eepromConfig.maxCheck     = readFloatCLI();
    		    eepromConfig.minThrottle  = readFloatCLI();
    		    eepromConfig.maxThrottle  = readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // Read Arm/Disarm Counts
                eepromConfig.armCount    = (uint8_t)readFloatCLI();
        	    eepromConfig.disarmCount = (uint8_t)readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'F': // Read Max Rate Value
                eepromConfig.rollAndPitchRateScaling = readFloatCLI() / 180000.0f * PI;
                eepromConfig.yawRateScaling          = readFloatCLI() / 180000.0f * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'G': // Read Max Attitude Value
                eepromConfig.attitudeScaling = readFloatCLI() / 180000.0f * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                cliPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

			///////////////////////////

			case '?':
			   	cliPrint("\n");
			   	cliPrint("'a' Receiver Configuration Data            'A' Toggle PPM/Spektrum Receiver\n");
   		        cliPrint("                                           'B' Set RC Control Order                 BTAER1234\n");
			   	cliPrint("                                           'C' Toggle Slave Spektrum State\n");
			   	cliPrint("                                           'D' Set RC Control Points                DmidCmd;minChk;maxChk;minThrot;maxThrot\n");
			   	cliPrint("                                           'E' Set Arm/Disarm Counts                EarmCount;disarmCount\n");
			   	cliPrint("                                           'F' 'F' Set Maximum Rate Commands        FRP;Y RP = Roll/Pitch, Y = Yaw\n");
			   	cliPrint("                                           'G' Set Maximum Attitude Command\n");
			   	cliPrint("                                           'W' Write EEPROM Parameters\n");
			   	cliPrint("'x' Exit Receiver CLI                      '?' Command Summary\n");
			   	cliPrint("\n");
	    	    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
