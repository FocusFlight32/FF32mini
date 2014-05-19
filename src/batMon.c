/**
  \file       batMon.c
  \brief      Battery Monitoring.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for Focused Flight 32.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

thresholds_t thresholds[3];

uint8_t batConnectedFirstPass = false;

uint8_t batMonLowWarning          = 0;
uint8_t batMonLowWarningTriggered = false;

uint8_t batMonVeryLowWarning          = 0;
uint8_t batMonVeryLowWarningTriggered = false;

uint8_t batMonMaxLowWarningTriggered = false;

uint8_t lastArmed = false;

uint8_t batteryNumCells = 3;

float   batteryVoltage;

void batMonLow(void);
void batMonVeryLow(void);
void batMonMaxLow(void);

enum
{
  thresholdThreshold = 20, /* 2 second at 10Hz. */
  thresholdsNUM      = sizeof(thresholds) / sizeof(thresholds_t),
};

/* Exp Filter = LPF time const = 0.1 sampletime */
static const float alpha = 1.0f / ( 1.0f + 0.1f );
static float v_bat_ave   = 0.0f;
static int thresholdCount[thresholdsNUM]; /* Will be inited to zero */

///////////////////////////////////////////////////////////////////////////////

void measureBattery(void)
{
    batteryVoltage = voltageMonitor() * VOLTS_PER_BIT * (sensorConfig.voltageMonitorScale) + sensorConfig.voltageMonitorBias;
}

///////////////////////////////////////////////////////////////////////////////

void batteryInit(void)
{
    thresholds[BATTERY_LOW].value      = sensorConfig.batteryLow;
    thresholds[BATTERY_VERY_LOW].value = sensorConfig.batteryVeryLow;
    thresholds[BATTRY_MAX_LOW].value   = sensorConfig.batteryMaxLow;

    thresholds[BATTERY_LOW].func      = batMonLow;
    thresholds[BATTERY_VERY_LOW].func = batMonVeryLow;
    thresholds[BATTRY_MAX_LOW].func   = batMonMaxLow;

    measureBattery();

    if (sensorConfig.batteryCells == 0)
        batteryNumCells = batteryVoltage / 3;
    else
        batteryNumCells = sensorConfig.batteryCells;
}

///////////////////////////////////////////////////////////////////////////////
/*
  \brief  battery Monitor Tick function.
 */
void batMonTick(void)
{
    float v;
    int i;

    measureBattery();
    v = batteryVoltage / (float)batteryNumCells;

    if ((v > thresholds[2].value) || (batConnectedFirstPass == true))  // There is a battery connected
    {
        if (batConnectedFirstPass == false)
        {
        	v_bat_ave = v;
           	batConnectedFirstPass = true;
        }

        v_bat_ave = alpha * v_bat_ave + (1.0f - alpha) * v;

        for ( i = 0 ; i < thresholdsNUM; ++i )
            if (v_bat_ave < thresholds[i].value)
            {
                if ( thresholdCount[i] < thresholdThreshold )
                    if ( ++thresholdCount[i] == thresholdThreshold )
                        thresholds[i].func();
            }
        else if ( thresholdCount[i] > 0 )
            --thresholdCount[i];
    }

    if ((armed == false) && (lastArmed == true))
    {
		if (batMonLowWarningTriggered == true)
		{
			batMonLowWarningTriggered = false;
			batMonLowWarning          = 0;
		}

		if (batMonVeryLowWarningTriggered == true)
		{
			batMonVeryLowWarningTriggered = false;
			batMonVeryLowWarning          = 0;
		}

		if (batMonMaxLowWarningTriggered == true)
		{
			batMonMaxLowWarningTriggered = false;
		}

		BEEP_OFF;
	}

	lastArmed = armed;
}

///////////////////////////////////////////////////////////////////////////////

void batMonLow(void)
{
    // Need to do slow beeping here, push back in telem to flash controller
    // lights, etc.

    if (batMonLowWarningTriggered == false)
    {
    	evrPush(EVR_BatLow, (int)(v_bat_ave * 1000.0f));

    	batMonLowWarning          = 10;
    	batMonLowWarningTriggered = true;
    }
}

///////////////////////////////////////////////////////////////////////////////

void batMonVeryLow(void)
{
    // Need to do fast beeping here, push back in telem to flash controller
    // lights, etc.
    // User needs to descend now ...

    if (batMonVeryLowWarningTriggered == false)
    {
    	evrPush(EVR_BatVeryLow, (int)(v_bat_ave * 1000.0f));

        batMonVeryLowWarning          = 50;
        batMonVeryLowWarningTriggered = true;
    }
}

///////////////////////////////////////////////////////////////////////////////

void batMonMaxLow(void)
{
    // User isn't listening flyer needs to auto-descend now ....
    // Maybe do something more interesting like auto-descent or hover-hold.

    if (batMonMaxLowWarningTriggered == false)
    {
		evrPush(EVR_BatMaxLow, (int)(v_bat_ave * 1000.0f));

		BEEP_ON;

		batMonMaxLowWarningTriggered = true;
	}
}

///////////////////////////////////////////////////////////////////////////////
