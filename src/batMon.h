/**
  \file       batMon.h
  \brief      Battery Monitoring.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for Focused Flight 32.
*/

///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////

extern volatile uint8_t batMonLowWarning;
extern volatile uint8_t batMonVeryLowWarning;

extern uint8_t batteryNumCells;

extern float batteryVoltage;

///////////////////////////////////////////////////////////////////////////////

void batMonTick(void);

///////////////////////////////////////////////////////////////////////////////

void measureBattery(void);

///////////////////////////////////////////////////////////////////////////////

void batteryInit(void);

///////////////////////////////////////////////////////////////////////////////
