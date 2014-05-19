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

uint32_t (*cliPortAvailable)(void);

uint8_t  (*cliPortRead)(void);

void     (*cliPortPrint)(char *str);

void     (*cliPortPrintF)(const char * fmt, ...);

///////////////////////////////////////

uint8_t cliBusy = false;

static volatile uint8_t cliQuery        = 'x';
static volatile uint8_t validCliCommand = false;

uint8_t gpsDataType = 0;

///////////////////////////////////////////////////////////////////////////////
// Read Character String from CLI
///////////////////////////////////////////////////////////////////////////////

char *readStringCLI(char *data, uint8_t length)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;

    do
    {
        if (cliPortAvailable() == false)
        {
            delay(10);
            timeout++;
        }
        else
        {
            data[index] = cliPortRead();
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < length));

    data[index] = '\0';

    return data;
}

///////////////////////////////////////////////////////////////////////////////
// Read Float from CLI
///////////////////////////////////////////////////////////////////////////////

float readFloatCLI(void)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;
    char    data[13] = "";

    do
    {
        if (cliPortAvailable() == false)
        {
            delay(10);
            timeout++;
        }
        else
        {
            data[index] = cliPortRead();
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));

    data[index] = '\0';

    return stringToFloat(data);
}

///////////////////////////////////////////////////////////////////////////////
// Read PID Values from CLI
///////////////////////////////////////////////////////////////////////////////

void readCliPID(unsigned char PIDid)
{
  struct PIDdata* pid = &systemConfig.PID[PIDid];

  pid->B              = readFloatCLI();
  pid->P              = readFloatCLI();
  pid->I              = readFloatCLI();
  pid->D              = readFloatCLI();
  pid->windupGuard    = readFloatCLI();
  pid->iTerm          = 0.0f;
  pid->lastDcalcValue = 0.0f;
  pid->lastDterm      = 0.0f;
  pid->lastLastDterm  = 0.0f;
  pid->dErrorCalc     =(uint8_t)readFloatCLI();
}

///////////////////////////////////////////////////////////////////////////////
// CLI Communication
///////////////////////////////////////////////////////////////////////////////

void cliCom(void)
{
	// HJI uint8_t  index;
	uint16_t index;

	char mvlkToggleString[5] = { 0, 0, 0, 0, 0 };

    if ((cliPortAvailable() && !validCliCommand))
    {
		cliQuery = cliPortRead();

        if (cliQuery == '#')                       // Check to see if we should toggle mavlink msg state
        {
	    	while (cliPortAvailable == false);

        	readStringCLI(mvlkToggleString, 5);

            if ((mvlkToggleString[0] == '#') &&
            	(mvlkToggleString[1] == '#') &&
                (mvlkToggleString[2] == '#') &&
                (mvlkToggleString[3] == '#'))
	    	{
	    	    if (systemConfig.mavlinkEnabled == false)
	    	    {
	    	 	    systemConfig.mavlinkEnabled  = true;
	    		    systemConfig.activeTelemetry = 0x0000;
	    		}
	    		else
	    		{
	    		    systemConfig.mavlinkEnabled = false;
	    	    }

	    	    if (mvlkToggleString[4] == 'W')
	    	    {
	                cliPortPrint("\nWriting EEPROM Parameters....\r\n");
	                writeSystemEEPROM();
	    	    }
	    	}
	    }
	}

	validCliCommand = false;

    if ((systemConfig.mavlinkEnabled == false) && (cliQuery != '#'))
    {
        switch (cliQuery)
        {
            ///////////////////////////////

            case 'a': // Rate PIDs
                cliPortPrintF("\nRoll Rate PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n", systemConfig.PID[ROLL_RATE_PID].B,
                                		                                                   systemConfig.PID[ROLL_RATE_PID].P,
                    		                                                               systemConfig.PID[ROLL_RATE_PID].I,
                    		                                                               systemConfig.PID[ROLL_RATE_PID].D,
                    		                                                               systemConfig.PID[ROLL_RATE_PID].windupGuard,
                    		                                                               systemConfig.PID[ROLL_RATE_PID].dErrorCalc ? "Error" : "State");

                cliPortPrintF("Pitch Rate PID: %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n",   systemConfig.PID[PITCH_RATE_PID].B,
                                		                                                   systemConfig.PID[PITCH_RATE_PID].P,
                    		                                                               systemConfig.PID[PITCH_RATE_PID].I,
                    		                                                               systemConfig.PID[PITCH_RATE_PID].D,
                    		                                                               systemConfig.PID[PITCH_RATE_PID].windupGuard,
                    		                                                               systemConfig.PID[PITCH_RATE_PID].dErrorCalc ? "Error" : "State");

                cliPortPrintF("Yaw Rate PID:   %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n",   systemConfig.PID[YAW_RATE_PID].B,
                                 		                                                   systemConfig.PID[YAW_RATE_PID].P,
                    		                                                               systemConfig.PID[YAW_RATE_PID].I,
                    		                                                               systemConfig.PID[YAW_RATE_PID].D,
                    		                                                               systemConfig.PID[YAW_RATE_PID].windupGuard,
                    		                                                               systemConfig.PID[YAW_RATE_PID].dErrorCalc ? "Error" : "State");
                cliQuery = 'x';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'b': // Attitude PIDs
                cliPortPrintF("\nRoll Attitude PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n", systemConfig.PID[ROLL_ATT_PID].B,
                  		                                                                       systemConfig.PID[ROLL_ATT_PID].P,
                   		                                                                       systemConfig.PID[ROLL_ATT_PID].I,
                   		                                                                       systemConfig.PID[ROLL_ATT_PID].D,
                   		                                                                       systemConfig.PID[ROLL_ATT_PID].windupGuard,
                   		                                                                       systemConfig.PID[ROLL_ATT_PID].dErrorCalc ? "Error" : "State");

                cliPortPrintF("Pitch Attitude PID: %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n",   systemConfig.PID[PITCH_ATT_PID].B,
                   		                                                                       systemConfig.PID[PITCH_ATT_PID].P,
                   		                                                                       systemConfig.PID[PITCH_ATT_PID].I,
                   		                                                                       systemConfig.PID[PITCH_ATT_PID].D,
                   		                                                                       systemConfig.PID[PITCH_ATT_PID].windupGuard,
                   		                                                                       systemConfig.PID[PITCH_ATT_PID].dErrorCalc ? "Error" : "State");

                cliPortPrintF("Heading PID:        %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n",   systemConfig.PID[HEADING_PID].B,
                   		                                                                       systemConfig.PID[HEADING_PID].P,
                   		                                                                       systemConfig.PID[HEADING_PID].I,
                   		                                                                       systemConfig.PID[HEADING_PID].D,
                   		                                                                       systemConfig.PID[HEADING_PID].windupGuard,
                   		                                                                       systemConfig.PID[HEADING_PID].dErrorCalc ? "Error" : "State");
                cliQuery = 'x';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'c': // Velocity PIDs
                cliPortPrintF("\nnDot PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n", systemConfig.PID[NDOT_PID].B,
                   		                                                              systemConfig.PID[NDOT_PID].P,
                   		                                                              systemConfig.PID[NDOT_PID].I,
                   		                                                              systemConfig.PID[NDOT_PID].D,
                   		                                                              systemConfig.PID[NDOT_PID].windupGuard,
                   		                                                              systemConfig.PID[NDOT_PID].dErrorCalc ? "Error" : "State");

                cliPortPrintF("eDot PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n",   systemConfig.PID[EDOT_PID].B,
                   		                                                              systemConfig.PID[EDOT_PID].P,
                   		                                                              systemConfig.PID[EDOT_PID].I,
                   		                                                              systemConfig.PID[EDOT_PID].D,
                   		                                                              systemConfig.PID[EDOT_PID].windupGuard,
                   		                                                              systemConfig.PID[EDOT_PID].dErrorCalc ? "Error" : "State");

                cliPortPrintF("hDot PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n",   systemConfig.PID[HDOT_PID].B,
                   		                                                              systemConfig.PID[HDOT_PID].P,
                   		                                                              systemConfig.PID[HDOT_PID].I,
                   		                                                              systemConfig.PID[HDOT_PID].D,
                   		                                                              systemConfig.PID[HDOT_PID].windupGuard,
                   		                                                              systemConfig.PID[HDOT_PID].dErrorCalc ? "Error" : "State");
                cliQuery = 'x';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'd': // Position PIDs
                cliPortPrintF("\nN PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n", systemConfig.PID[N_PID].B,
                   		                                                           systemConfig.PID[N_PID].P,
                   		                                                           systemConfig.PID[N_PID].I,
                   		                                                           systemConfig.PID[N_PID].D,
                   		                                                           systemConfig.PID[N_PID].windupGuard,
                   		                                                           systemConfig.PID[N_PID].dErrorCalc ? "Error" : "State");

                cliPortPrintF("E PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n",   systemConfig.PID[E_PID].B,
                   		                                                           systemConfig.PID[E_PID].P,
                   		                                                           systemConfig.PID[E_PID].I,
                   		                                                           systemConfig.PID[E_PID].D,
                   		                                                           systemConfig.PID[E_PID].windupGuard,
                   		                                                           systemConfig.PID[E_PID].dErrorCalc ? "Error" : "State");

                cliPortPrintF("h PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\r\n",   systemConfig.PID[H_PID].B,
                   		                                                           systemConfig.PID[H_PID].P,
                   		                                                           systemConfig.PID[H_PID].I,
                   		                                                           systemConfig.PID[H_PID].D,
                   		                                                           systemConfig.PID[H_PID].windupGuard,
                   		                                                           systemConfig.PID[H_PID].dErrorCalc ? "Error" : "State");
                cliQuery = 'x';
                validCliCommand = false;
              	break;

            ///////////////////////////////

            case 'e': // Loop Delta Times
               	cliPortPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\r\n", deltaTime1000Hz,
                   		                                                    deltaTime500Hz,
                   		                                                    deltaTime100Hz,
                   		                                                    deltaTime50Hz,
                   		                                                    deltaTime10Hz,
                   		                                                    deltaTime5Hz,
                   		                                                    deltaTime1Hz);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'f': // Loop Execution Times
               	cliPortPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\r\n", executionTime1000Hz,
               	        			                                        executionTime500Hz,
               	        			                                        executionTime100Hz,
               	        			                                        executionTime50Hz,
               	        			                                        executionTime10Hz,
               	        			                                        executionTime5Hz,
               	        			                                        executionTime1Hz);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'g': // 500 Hz Accels
            	cliPortPrintF("%9.4f, %9.4f, %9.4f\r\n", sensors.accel500Hz[XAXIS],
            			                               sensors.accel500Hz[YAXIS],
            			                               sensors.accel500Hz[ZAXIS]);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'h': // 100 hz Earth Axis Accels
            	cliPortPrintF("%9.4f, %9.4f, %9.4f\r\n", earthAxisAccels[XAXIS],
            			                               earthAxisAccels[YAXIS],
            			                               earthAxisAccels[ZAXIS]);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'i': // 500 hz Gyros
            	cliPortPrintF("%9.4f, %9.4f, %9.4f, %9.4f\r\n", sensors.gyro500Hz[ROLL ] * R2D,
            			                                      sensors.gyro500Hz[PITCH] * R2D,
            					                              sensors.gyro500Hz[YAW  ] * R2D,
            					                              mpu6000Temperature);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'j': // 10 Hz Mag Data
            	cliPortPrintF("%9.4f, %9.4f, %9.4f\r\n", sensors.mag10Hz[XAXIS],
            			                               sensors.mag10Hz[YAXIS],
            			                               sensors.mag10Hz[ZAXIS]);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'k': // Vertical Axis Variables
            	cliPortPrintF("%9.4f, %9.4f, %9.4f, %9.4f, %4ld, %9.4f\r\n", earthAxisAccels[ZAXIS],
            			                                                   sensors.pressureAlt50Hz,
            					                                           hDotEstimate,
            					                                           hEstimate,
            					                                           ms5611Temperature,
            					                                           aglRead());
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'l': // Attitudes
            	cliPortPrintF("%9.4f, %9.4f, %9.4f\r\n", sensors.attitude500Hz[ROLL ] * R2D,
            			                               sensors.attitude500Hz[PITCH] * R2D,
            			                               sensors.attitude500Hz[YAW  ] * R2D);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'm': // Axis PIDs
            	cliPortPrintF("%9.4f, %9.4f, %9.4f\r\n", axisPID[ROLL ],
               			                               axisPID[PITCH],
               			                               axisPID[YAW  ]);
               	validCliCommand = false;
               	break;

            ///////////////////////////////

            case 'n': // GPS Data
               	switch (gpsDataType)
               	{
               	    ///////////////////////

               	    case 0:
               	        cliPortPrintF("%12ld, %12ld, %12ld, %12ld, %12ld, %12ld, %4d, %4d\r\n", gps.latitude,
               			                                                                      gps.longitude,
               			                                                                      gps.hMSL,
               			                                                                      gps.velN,
               			                                                                      gps.velE,
               			                                                                      gps.velD,
               			                                                                      gps.fix,
               			                                                                      gps.numSats);
               	        break;

               	    ///////////////////////

               	    case 1:
               	    	cliPortPrintF("%3d: ", gps.numCh);

               	    	for (index = 0; index < gps.numCh; index++)
               	    	    cliPortPrintF("%3d  ", gps.chn[index]);

               	    	cliPortPrint("\r\n");

               	    	break;

               	    ///////////////////////

               	    case 2:
               	    	cliPortPrintF("%3d: ", gps.numCh);

               	    	for (index = 0; index < gps.numCh; index++)
               	    		cliPortPrintF("%3d  ", gps.svid[index]);

               	    	cliPortPrint("\r\n");

               	    	break;

               	    ///////////////////////

               	    case 3:
               	    	cliPortPrintF("%3d: ", gps.numCh);

               	    	for (index = 0; index < gps.numCh; index++)
               	    		cliPortPrintF("%3d  ", gps.cno[index]);

               	    	cliPortPrint("\r\n");

               	    	break;

               	    ///////////////////////
               	}

               	validCliCommand = false;
                break;

            ///////////////////////////////

            case 'o':
                cliPortPrintF("%9.4f\r\n", batteryVoltage);

                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'p': // Primary Spektrum Raw Data
            	cliPortPrintF("%04X, %04X, %04X, %04X, %04X, %04X, %04X, %04X, %04X, %04X\r\n", primarySpektrumState.lostFrameCnt,
            			                                                                      primarySpektrumState.rcAvailable,
            			                                                                      primarySpektrumState.values[0],
            			                                                                      primarySpektrumState.values[1],
            			                                                                      primarySpektrumState.values[2],
            			                                                                      primarySpektrumState.values[3],
            			                                                                      primarySpektrumState.values[4],
            			                                                                      primarySpektrumState.values[5],
            			                                                                      primarySpektrumState.values[6],
            			                                                                      primarySpektrumState.values[7]);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'q': // Not Used
                cliQuery = 'x';
               	validCliCommand = false;
               	break;

            ///////////////////////////////

            case 'r':
            	if (flightMode == RATE)
            		cliPortPrint("Flight Mode = RATE      ");
            	else if (flightMode == ATTITUDE)
            		cliPortPrint("Flight Mode = ATTITUDE  ");
            	else if (flightMode == GPS)
            		cliPortPrint("Flight Mode = GPS       ");

            	if (headingHoldEngaged == true)
            	    cliPortPrint("Heading Hold = ENGAGED     ");
            	else
            	    cliPortPrint("Heading Hold = DISENGAGED  ");

            	cliPortPrint("Alt Hold = ");

                switch (verticalModeState)
            	{
            		case ALT_DISENGAGED_THROTTLE_ACTIVE:
		                cliPortPrint("Alt Disenaged Throttle Active\r\n");

            		    break;

            		case ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT:
		                cliPortPrint("Alt Hold Fixed at Engagement Alt\r\n");

            		    break;

            		case ALT_HOLD_AT_REFERENCE_ALTITUDE:
		                cliPortPrint("Alt Hold at Reference Alt\r\n");

            		    break;

            		case VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY:
		                cliPortPrint("V Velocity Hold at Reference Vel\r\n");

            		    break;

            		case ALT_DISENGAGED_THROTTLE_INACTIVE:
            		    cliPortPrint("Alt Disengaged Throttle Inactive\r\n");

            		    break;
                }

            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 's': // Raw Receiver Commands
                if ((systemConfig.receiverType == SPEKTRUM) && (maxChannelNum > 0))
                {
		    		for (index = 0; index < maxChannelNum - 1; index++)
                         cliPortPrintF("%4ld, ", spektrumBuf[index]);

                    cliPortPrintF("%4ld\r\n", spektrumBuf[maxChannelNum - 1]);
                }
                else if ((systemConfig.receiverType == SPEKTRUM) && (maxChannelNum == 0))
                    cliPortPrint("Invalid Number of Spektrum Channels....\r\n");
		        else
		        {
		    		for (index = 0; index < 7; index++)
                        cliPortPrintF("%4i, ", ppmRxRead(index));

                    cliPortPrintF("%4i\r\n", ppmRxRead(7));
                }

            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 't': // Processed Receiver Commands
                for (index = 0; index < 7; index++)
                    cliPortPrintF("%8.2f, ", rxCommand[index]);

                cliPortPrintF("%8.2f\r\n", rxCommand[7]);

                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'u': // Command in Detent Discretes
            	cliPortPrintF("%s, ", commandInDetent[ROLL ] ? " true" : "false");
            	cliPortPrintF("%s, ", commandInDetent[PITCH] ? " true" : "false");
            	cliPortPrintF("%s\r\n", commandInDetent[YAW  ] ? " true" : "false");

                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'v': // ESC PWM Outputs
            	cliPortPrintF("%4ld, ", TIM2->CCR1 );
            	cliPortPrintF("%4ld, ", TIM2->CCR2 );
                cliPortPrintF("%4ld, ", TIM15->CCR1);
            	cliPortPrintF("%4ld, ", TIM15->CCR2);
            	cliPortPrintF("%4ld, ", TIM3->CCR1 );
            	cliPortPrintF("%4ld\r\n", TIM3->CCR2 );

            	validCliCommand = false;
                break;

            ///////////////////////////////

            case 'w': // Servo PWM Outputs
            	cliPortPrintF("%4ld, ", TIM4->CCR1);
            	cliPortPrintF("%4ld, ", TIM4->CCR2);
            	cliPortPrintF("%4ld, ", TIM4->CCR3);
            	cliPortPrintF("%4ld\r\n", TIM4->CCR4);

                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'x':
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'y': // ESC Calibration
            	escCalibration();

            	cliQuery = 'x';
            	break;

            ///////////////////////////////

            case 'z':
                cliPortPrintF("%5.2f, %5.2f\r\n", voltageMonitor(),
                		                        adcChannel());
                break;

            ///////////////////////////////

            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////

            ///////////////////////////////

            case 'A': // Read Roll Rate PID Values
                readCliPID(ROLL_RATE_PID);
                cliPortPrint( "\nRoll Rate PID Received....\r\n" );

            	cliQuery = 'a';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'B': // Read Pitch Rate PID Values
                readCliPID(PITCH_RATE_PID);
                cliPortPrint( "\nPitch Rate PID Received....\r\n" );

            	cliQuery = 'a';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'C': // Read Yaw Rate PID Values
                readCliPID(YAW_RATE_PID);
                cliPortPrint( "\nYaw Rate PID Received....\r\n" );

            	cliQuery = 'a';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'D': // Read Roll Attitude PID Values
                readCliPID(ROLL_ATT_PID);
                cliPortPrint( "\nRoll Attitude PID Received....\r\n" );

            	cliQuery = 'b';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'E': // Read Pitch Attitude PID Values
                readCliPID(PITCH_ATT_PID);
                cliPortPrint( "\nPitch Attitude PID Received....\r\n" );

            	cliQuery = 'b';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'F': // Read Heading Hold PID Values
                readCliPID(HEADING_PID);
                cliPortPrint( "\nHeading PID Received....\r\n" );

            	cliQuery = 'b';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'G': // Read nDot PID Values
                readCliPID(NDOT_PID);
                cliPortPrint( "\nnDot PID Received....\r\n" );

            	cliQuery = 'c';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'H': // Read eDot PID Values
                readCliPID(EDOT_PID);
                cliPortPrint( "\neDot PID Received....\r\n" );

                cliQuery = 'c';
              	validCliCommand = false;
              	break;

            ///////////////////////////////

            case 'I': // Read hDot PID Values
                readCliPID(HDOT_PID);
                cliPortPrint( "\nhDot PID Received....\r\n" );

              	cliQuery = 'c';
              	validCliCommand = false;
              	break;

       	    ///////////////////////////////

            case 'J': // Read n PID Values
                readCliPID(N_PID);
                cliPortPrint( "\nn PID Received....\r\n" );

                cliQuery = 'd';
                validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'K': // Read e PID Values
                readCliPID(E_PID);
                cliPortPrint( "\ne PID Received....\r\n" );

                cliQuery = 'd';
                validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'L': // Read h PID Values
                readCliPID(H_PID);
                cliPortPrint( "\nh PID Received....\r\n" );

                cliQuery = 'd';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'N': // Mixer CLI
                mixerCLI();

                cliQuery = 'x';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'O': // Receiver CLI
                receiverCLI();

                cliQuery = 'x';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'P': // Sensor CLI
               	sensorCLI();

               	cliQuery = 'x';
               	validCliCommand = false;
               	break;

            ///////////////////////////////

            case 'Q': // GPS Data Selection
            	gpsDataType = (uint8_t)readFloatCLI();

            	cliPortPrint("\r\n");

                cliQuery = 'n';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'R': // Reset to Bootloader
            	cliPortPrint("Entering Bootloader....\n\r\n");
            	delay(100);
            	systemReset(true);
            	break;

            ///////////////////////////////

            case 'S': // Reset System
            	cliPortPrint("\nSystem Reseting....\n\r\n");
            	delay(100);
            	systemReset(false);
            	break;

            ///////////////////////////////

            case 'T': // Telemetry CLI
                telemetryCLI();

                cliQuery = 'x';
             	validCliCommand = false;
             	break;

            ///////////////////////////////

            case 'U': // EEPROM CLI
                eepromCLI();

                cliQuery = 'x';
             	validCliCommand = false;
             	break;

            ///////////////////////////////

            case 'V': // Write Sensor EEPROM Parameters
                cliPortPrint("\nWriting Sensor EEPROM Parameters....\n\r\n");
                writeSensorEEPROM();

                cliQuery = 'x';
             	validCliCommand = false;
             	break;

            ///////////////////////////////

            case 'W': // Write System EEPROM Parameters
                cliPortPrint("\nWriting System EEPROM Parameters....\n\r\n");
                writeSystemEEPROM();

                cliQuery = 'x';
             	validCliCommand = false;
             	break;

            ///////////////////////////////

            case 'X': // Not Used
                cliQuery = 'x';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'Y': // Not Used
                computeGeoMagElements();

                cliQuery = 'x';
                break;

            ///////////////////////////////

            case 'Z': // Not Used

                cliQuery = 'x';
                break;

            ///////////////////////////////

            case '?': // Command Summary
            	cliBusy = true;

            	cliPortPrint("\r\n");
   		        cliPortPrint("'a' Rate PIDs                              'A' Set Roll Rate PID Data   AB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("'b' Attitude PIDs                          'B' Set Pitch Rate PID Data  BB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("'c' Velocity PIDs                          'C' Set Yaw Rate PID Data    CB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("'d' Position PIDs                          'D' Set Roll Att PID Data    DB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("'e' Loop Delta Times                       'E' Set Pitch Att PID Data   EB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("'f' Loop Execution Times                   'F' Set Hdg Hold PID Data    FB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("'g' 500 Hz Accels                          'G' Set nDot PID Data        GB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("'h' 100 Hz Earth Axis Accels               'H' Set eDot PID Data        HB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("'i' 500 Hz Gyros                           'I' Set hDot PID Data        IB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("'j' 10 hz Mag Data                         'J' Set n PID Data           JB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("'k' Vertical Axis Variable                 'K' Set e PID Data           KB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("'l' Attitudes                              'L' Set h PID Data           LB;P;I;D;windupGuard;dErrorCalc\r\n");
   		        cliPortPrint("\r\n");

   		        cliPortPrint("Press space bar for more, or enter a command....\r\n");

   		        while (cliPortAvailable() == false);

   		        cliQuery = cliPortRead();

   		        if (cliQuery != ' ')
   		        {
   		            validCliCommand = true;
   		            cliBusy = false;
   		        	return;
   		        }

   		        cliPortPrint("\r\n");
   		        cliPortPrint("'m' Axis PIDs                              'M' Not Used\r\n");
   		        cliPortPrint("'n' GPS Data                               'N' Mixer CLI\r\n");
   		        cliPortPrint("'o' Battery Voltage                        'O' Receiver CLI\r\n");
   		        cliPortPrint("'p' Primary Spektrum Raw Data              'P' Sensor CLI\r\n");
   		        cliPortPrint("'q' Not Used                               'Q' GPS Data Selection\r\n");
   		        cliPortPrint("'r' Mode States                            'R' Reset and Enter Bootloader\r\n");
   		        cliPortPrint("'s' Raw Receiver Commands                  'S' Reset\r\n");
   		        cliPortPrint("'t' Processed Receiver Commands            'T' Telemetry CLI\r\n");
   		        cliPortPrint("'u' Command In Detent Discretes            'U' EEPROM CLI\r\n");
   		        cliPortPrint("'v' Motor PWM Outputs                      'V' Write Sensor EEPROM Parameters\r\n");
   		        cliPortPrint("'w' Servo PWM Outputs                      'W' Write System EEPROM Parameters\r\n");
   		        cliPortPrint("'x' Terminate Serial Communication         'X' Not Used\r\n");
   		        cliPortPrint("\r\n");

   		        cliPortPrint("Press space bar for more, or enter a command....\r\n");

   		        while (cliPortAvailable() == false);

   		        cliQuery = cliPortRead();

   		        if (cliQuery != ' ')
   		        {
   		        	validCliCommand = true;
   		        	cliBusy = false;
   		        	return;
   		        }

   		        cliPortPrint("\r\n");
   		        cliPortPrint("'y' ESC Calibration                        'Y' Not Used\r\n");
   		        cliPortPrint("'z' ADC Values                             'Z' Not Used\r\n");
   		        cliPortPrint("                                           '?' Command Summary\r\n");
   		        cliPortPrint("\r\n");

  		        cliQuery = 'x';
  		        cliBusy = false;
   		        break;

                ///////////////////////////////
		}
    }
}

///////////////////////////////////////////////////////////////////////////////
