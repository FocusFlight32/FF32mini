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
// Sensor CLI
///////////////////////////////////////////////////////////////////////////////

void sensorCLI()
{
    uint8_t  sensorQuery = 'x';
    uint8_t  tempInt;
    uint8_t  validQuery  = false;

    cliBusy = true;

    cliPortPrint("\nEntering Sensor CLI....\n\r\n");

    while(true)
    {
        cliPortPrint("Sensor CLI -> ");

		while ((cliPortAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    sensorQuery = cliPortRead();

		cliPortPrint("\r\n");

		switch(sensorQuery)
		{
            ///////////////////////////

            case 'a': // Sensor Data
               	cliPortPrintF("MPU Accel Bias:            %9.3f, %9.3f, %9.3f\r\n", sensorConfig.accelBiasMPU[XAXIS],
			                                                		              sensorConfig.accelBiasMPU[YAXIS],
			                                                		              sensorConfig.accelBiasMPU[ZAXIS]);
			    cliPortPrintF("MPU Accel Scale Factor:    %9.7f, %9.7f, %9.7f\r\n", sensorConfig.accelScaleFactorMPU[XAXIS],
							                                                      sensorConfig.accelScaleFactorMPU[YAXIS],
			                                                		              sensorConfig.accelScaleFactorMPU[ZAXIS]);
            	cliPortPrintF("Accel Temp Comp Slope:     %9.4f, %9.4f, %9.4f\r\n", sensorConfig.accelTCBiasSlope[XAXIS],
            	                                   		                          sensorConfig.accelTCBiasSlope[YAXIS],
            	                                   		                          sensorConfig.accelTCBiasSlope[ZAXIS]);
            	cliPortPrintF("Accel Temp Comp Bias:      %9.4f, %9.4f, %9.4f\r\n", sensorConfig.accelTCBiasIntercept[XAXIS],
            	                                   		                          sensorConfig.accelTCBiasIntercept[YAXIS],
            	                                                		          sensorConfig.accelTCBiasIntercept[ZAXIS]);
            	cliPortPrintF("Gyro Temp Comp Slope:      %9.4f, %9.4f, %9.4f\r\n", sensorConfig.gyroTCBiasSlope[ROLL ],
            	                                                                  sensorConfig.gyroTCBiasSlope[PITCH],
            	                                                                  sensorConfig.gyroTCBiasSlope[YAW  ]);
            	cliPortPrintF("Gyro Temp Comp Intercept:  %9.4f, %9.4f, %9.4f\r\n", sensorConfig.gyroTCBiasIntercept[ROLL ],
            	                                                                  sensorConfig.gyroTCBiasIntercept[PITCH],
            	                                                                  sensorConfig.gyroTCBiasIntercept[YAW  ]);
            	cliPortPrintF("Mag Bias:                  %9.4f, %9.4f, %9.4f\r\n", sensorConfig.magBias[XAXIS],
                                                		                          sensorConfig.magBias[YAXIS],
                                                		                          sensorConfig.magBias[ZAXIS]);
                cliPortPrintF("Accel One G:               %9.4f\r\n",   accelOneG);
                cliPortPrintF("Accel Cutoff:              %9.4f\r\n",   sensorConfig.accelCutoff);
                cliPortPrintF("KpAcc (MARG):              %9.4f\r\n",   sensorConfig.KpAcc);
                cliPortPrintF("KiAcc (MARG):              %9.4f\r\n",   sensorConfig.KiAcc);
                cliPortPrintF("KpMag (MARG):              %9.4f\r\n",   sensorConfig.KpMag);
                cliPortPrintF("KiMag (MARG):              %9.4f\r\n",   sensorConfig.KiMag);
                cliPortPrintF("hdot est/h est Comp Fil A: %9.4f\r\n",   sensorConfig.compFilterA);
                cliPortPrintF("hdot est/h est Comp Fil B: %9.4f\r\n",   sensorConfig.compFilterB);

                cliPortPrint("MPU6000 DLPF:                 ");
                switch(sensorConfig.dlpfSetting)
                {
                    case DLPF_256HZ:
                        cliPortPrint("256 Hz\r\n");
                        break;
                    case DLPF_188HZ:
                        cliPortPrint("188 Hz\r\n");
                        break;
                    case DLPF_98HZ:
                        cliPortPrint("98 Hz\r\n");
                        break;
                    case DLPF_42HZ:
                        cliPortPrint("42 Hz\r\n");
                        break;
                }

                cliPortPrintF("Voltage Monitor Scale:     %9.4f\n\r\n",  sensorConfig.voltageMonitorScale);
                cliPortPrintF("Voltage Monitor Bias:      %9.4f\r\n",    sensorConfig.voltageMonitorBias);
                cliPortPrintF("Number of Battery Cells:      %1d\n\r\n", sensorConfig.batteryCells);

                cliPortPrintF("Battery Low Setpoint:      %4.2f volts\r\n",   sensorConfig.batteryLow);
                cliPortPrintF("Battery Very Low Setpoint: %4.2f volts\r\n",   sensorConfig.batteryVeryLow);
                cliPortPrintF("Battery Max Low Setpoint:  %4.2f volts\n\r\n", sensorConfig.batteryMaxLow);

                if (sensorConfig.gpsVelocityHoldOnly)
                	cliPortPrint("GPS Velocity Hold Only\r\n");
                else
                	cliPortPrint("GPS Velocity and Position Hold\r\n");

                if (sensorConfig.verticalVelocityHoldOnly)
                	cliPortPrint("Vertical Velocity Hold Only\n\r\n");
                else
                	cliPortPrint("Vertical Velocity and Altitude Hold\n\r\n");

                validQuery = false;
                break;

            ///////////////////////////

            case 'b': // MPU6000 Calibration
                mpu6000Calibration();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'c': // Magnetometer Calibration
                magCalibration();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'd': // Accel Bias and Scale Factor Calibration
                accelCalibrationMPU();

                sensorQuery = 'a';
                validQuery = true;
                break;

             ///////////////////////////

            case 'g': // Toggle GPS Velocity Hold Only
                if (sensorConfig.gpsVelocityHoldOnly)
                	sensorConfig.gpsVelocityHoldOnly = false;
                else
                	sensorConfig.gpsVelocityHoldOnly = true;

                sensorQuery = 'a';
                validQuery = true;
                break;

           ///////////////////////////

            case 'v': // Toggle Vertical Velocity Hold Only
                if (sensorConfig.verticalVelocityHoldOnly)
                	sensorConfig.verticalVelocityHoldOnly = false;
                else
                	sensorConfig.verticalVelocityHoldOnly = true;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'x':
			    cliPortPrint("\nExiting Sensor CLI....\n\r\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Set MPU6000 Digital Low Pass Filter
                tempInt = (uint8_t)readFloatCLI();

                switch(tempInt)
                {
                    case DLPF_256HZ:
                        sensorConfig.dlpfSetting = BITS_DLPF_CFG_256HZ;
                        break;

                    case DLPF_188HZ:
                    	sensorConfig.dlpfSetting = BITS_DLPF_CFG_188HZ;
                    	break;

                    case DLPF_98HZ:
                    	sensorConfig.dlpfSetting = BITS_DLPF_CFG_98HZ;
                    	break;

                    case DLPF_42HZ:
                    	sensorConfig.dlpfSetting = BITS_DLPF_CFG_42HZ;
                     	break;
                }

                setSPIdivisor(MPU6000_SPI, 64);  // 0.65625 MHz SPI clock

                GPIO_ResetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN);
			    spiTransfer(MPU6000_SPI, MPU6000_CONFIG);
			    spiTransfer(MPU6000_SPI, sensorConfig.dlpfSetting);
			    GPIO_SetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN);

                setSPIdivisor(MPU6000_SPI, 2);  // 21 MHz SPI clock (within 20 +/- 10%)

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'B': // Accel Cutoff
                sensorConfig.accelCutoff = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // kpAcc, kiAcc
                sensorConfig.KpAcc = readFloatCLI();
                sensorConfig.KiAcc = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // kpMag, kiMag
                sensorConfig.KpMag = readFloatCLI();
                sensorConfig.KiMag = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // h dot est/h est Comp Filter A/B
                sensorConfig.compFilterA = readFloatCLI();
                sensorConfig.compFilterB = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'M': // Set Voltage Monitor Trip Points
                sensorConfig.batteryLow     = readFloatCLI();
                sensorConfig.batteryVeryLow = readFloatCLI();
                sensorConfig.batteryMaxLow  = readFloatCLI();

                thresholds[BATTERY_LOW].value      = sensorConfig.batteryLow;
                thresholds[BATTERY_VERY_LOW].value = sensorConfig.batteryVeryLow;
                thresholds[BATTRY_MAX_LOW].value   = sensorConfig.batteryMaxLow;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'V': // Set Voltage Monitor Parameters
                sensorConfig.voltageMonitorScale = readFloatCLI();
                sensorConfig.voltageMonitorBias  = readFloatCLI();
                sensorConfig.batteryCells        = (uint8_t)readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write Sensor EEPROM Parameters
                cliPortPrint("\nWriting Sensor EEPROM Parameters....\n\r\n");
                writeSensorEEPROM();

                validQuery = false;
                break;

			///////////////////////////

			case '?':
			   	cliPortPrint("\r\n");
			   	cliPortPrint("'a' Display Sensor Data                    'A' Set MPU6000 DLPF                     A0 thru 3\r\n");
			   	cliPortPrint("'b' MPU6000 Calibration                    'B' Set Accel Cutoff                     BAccelCutoff\r\n");
			   	cliPortPrint("'c' Magnetometer Calibration               'C' Set kpAcc/kiAcc                      CKpAcc;KiAcc\r\n");
			   	cliPortPrint("'d' Accel Bias and SF Calibration          'D' Set kpMag/kiMag                      DKpMag;KiMag\r\n");
			   	cliPortPrint("                                           'E' Set h dot est/h est Comp Filter A/B  EA;B\r\n");
			   	cliPortPrint("'g' Toggle GPS Velocity Hold Only          'M' Set Voltage Monitor Trip Points      Mlow;veryLow;maxLow\r\n");
			   	cliPortPrint("'v' Toggle Vertical Velocity Hold Only     'V' Set Voltage Monitor Parameters       Vscale;bias;cells\r\n");
			    cliPortPrint("                                           'W' Write EEPROM Parameters\r\n");
			    cliPortPrint("'x' Exit Sensor CLI                        '?' Command Summary\r\n");
			    cliPortPrint("\r\n");
	    	    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
